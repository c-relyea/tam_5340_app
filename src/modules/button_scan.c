/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* Pin scanner for locating an unknown push button.
 *
 * The tam_board devicetree inherits its five button nodes from the nRF5340
 * Audio DK, so the pins there describe the DK's hardware and not necessarily
 * this board's. This module brackets the problem from the other side: it
 * configures every GPIO not already claimed by a peripheral as an input with
 * a known pull, and reports every pin that reads against that pull.
 *
 * Detection is on absolute level rather than on edges, so a button that is
 * already held down when the scan starts still shows up. An unconnected pin
 * follows its pull; a pin shorted to the other rail by a pressed button does
 * not, and that disagreement is the signal.
 *
 * Two passes alternate so the direction of the button does not have to be
 * known in advance: the pull-up pass finds a button that shorts to ground,
 * the pull-down pass finds one that shorts to the rail. A pin driven by
 * external hardware will also read against one of the pulls, but it does so
 * whether or not the button is pressed -- hold and release a few times and
 * the button is whichever pin comes and goes.
 */

#include "button_scan.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(button_scan, CONFIG_MODULE_BUTTON_SCAN_LOG_LEVEL);

/* Pins already owned by something else on tam_board. Reconfiguring any of
 * these as a GPIO input would disconnect a live peripheral, so they are
 * skipped. Derived from tam_board_nrf5340_cpuapp_common.dtsi and its pinctrl.
 */
#define P0_RESERVED_MSK                                                                            \
	(BIT(1) |  /* cs47l63 gpio9 */                                                             \
	 BIT(9) | BIT(10) | BIT(11) | BIT(12) | BIT(13) | /* i2s0 */                               \
	 BIT(17) | /* cs47l63 reset */                                                             \
	 BIT(19) | /* cs47l63 irq */                                                               \
	 BIT(20) | /* spi4 cs */                                                                   \
	 BIT(21) | BIT(22)) /* spi4 miso/mosi */

#define P1_RESERVED_MSK                                                                            \
	(BIT(2) | BIT(3) |  /* i2c1 sda/scl */                                                     \
	 BIT(4) |  /* spi4 sck */                                                                  \
	 BIT(5) |  /* CHG_TERM */                                                                  \
	 BIT(7))   /* PWR_EN hog */

/* P1 only bonds pins 0..15 on the nRF5340. */
#define P1_PIN_CNT 16
#define P0_PIN_CNT 32

/* Time for the pulls to charge the pin capacitance before sampling. */
#define SETTLE_MS 5

enum scan_pass {
	PASS_PULL_UP,
	PASS_PULL_DOWN,
	PASS_CNT,
};

static const struct {
	gpio_flags_t flags;
	const char *name;
	int expect; /**< Level an unconnected pin settles to under this pull. */
} passes[PASS_CNT] = {
	[PASS_PULL_UP] = {GPIO_PULL_UP, "pull-up  ", 1},
	[PASS_PULL_DOWN] = {GPIO_PULL_DOWN, "pull-down", 0},
};

struct scan_port {
	const struct device *dev;
	const char *name;
	uint32_t reserved_msk;
	uint8_t pin_cnt;
	/* Pins reading against the pull, per pass, as of the last cycle. */
	uint32_t prev_active[PASS_CNT];
};

static struct scan_port ports[] = {
	{.name = "P0", .reserved_msk = P0_RESERVED_MSK, .pin_cnt = P0_PIN_CNT},
	{.name = "P1", .reserved_msk = P1_RESERVED_MSK, .pin_cnt = P1_PIN_CNT},
};

/* Latest result, kept in RAM so it survives the console. RTT on this board
 * stops delivering partway into a long run, and a debugger can read this
 * over SWD with the board left running.
 */
struct button_scan_state button_scan_last;

static bool pin_is_scannable(const struct scan_port *port, uint8_t pin)
{
	return (port->reserved_msk & BIT(pin)) == 0;
}

/* Render a pin mask as "03 14 27" into buf. */
static void mask_to_str(uint32_t msk, uint8_t pin_cnt, char *buf, size_t buf_len)
{
	size_t off = 0;

	buf[0] = '\0';

	for (uint8_t pin = 0; pin < pin_cnt && off + 4 < buf_len; pin++) {
		if (msk & BIT(pin)) {
			off += snprintk(&buf[off], buf_len - off, "%02u ", pin);
		}
	}
}

/* Apply one pull to every scannable pin of one port and return the mask of
 * pins that read against it.
 */
static uint32_t port_sample(struct scan_port *port, enum scan_pass pass)
{
	uint32_t active = 0;

	for (uint8_t pin = 0; pin < port->pin_cnt; pin++) {
		int ret;

		if (!pin_is_scannable(port, pin)) {
			continue;
		}

		ret = gpio_pin_configure(port->dev, pin, GPIO_INPUT | passes[pass].flags);
		if (ret) {
			/* Not fatal: a pin the SoC will not hand over is simply
			 * one fewer candidate. Drop it for the rest of the run.
			 */
			LOG_DBG("%s.%02u not configurable: %d", port->name, pin, ret);
			port->reserved_msk |= BIT(pin);
		}
	}

	k_sleep(K_MSEC(SETTLE_MS));

	for (uint8_t pin = 0; pin < port->pin_cnt; pin++) {
		int level;

		if (!pin_is_scannable(port, pin)) {
			continue;
		}

		level = gpio_pin_get_raw(port->dev, pin);
		if (level >= 0 && level != passes[pass].expect) {
			active |= BIT(pin);
		}
	}

	return active;
}

/* One full sweep of both pulls across both ports. Logs only when the set of
 * held pins changes, so holding a button produces one line rather than a
 * flood that RTT cannot keep up with.
 */
static void scan_cycle(bool force_log)
{
	char buf[96];

	for (enum scan_pass pass = 0; pass < PASS_CNT; pass++) {
		for (size_t p = 0; p < ARRAY_SIZE(ports); p++) {
			struct scan_port *port = &ports[p];
			uint32_t active = port_sample(port, pass);

			if (!force_log && active == port->prev_active[pass]) {
				continue;
			}

			port->prev_active[pass] = active;
			button_scan_last.active[pass][p] = active;
			button_scan_last.t_ms = k_uptime_get_32();
			button_scan_last.change_cnt++;

			if (active == 0) {
				LOG_INF("%s %s: (none)", passes[pass].name, port->name);
				continue;
			}

			mask_to_str(active, port->pin_cnt, buf, sizeof(buf));
			LOG_INF("%s %s: held %s", passes[pass].name, port->name, buf);
		}
	}
}

/* Leave every scanned pin disconnected so the scan does not keep pulls on
 * pins the application may want to use afterwards.
 */
static void scan_release(void)
{
	for (size_t p = 0; p < ARRAY_SIZE(ports); p++) {
		struct scan_port *port = &ports[p];

		for (uint8_t pin = 0; pin < port->pin_cnt; pin++) {
			if (!pin_is_scannable(port, pin)) {
				continue;
			}

			(void)gpio_pin_configure(port->dev, pin, GPIO_DISCONNECTED);
		}
	}
}

static int ports_bind(void)
{
	ports[0].dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	ports[1].dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	for (size_t p = 0; p < ARRAY_SIZE(ports); p++) {
		if (!device_is_ready(ports[p].dev)) {
			LOG_ERR("%s not ready", ports[p].name);
			return -ENODEV;
		}
	}

	return 0;
}

/* Focused probe of a single known pin.
 *
 * The board schematic puts SW1 on net SWITCH -> nRF5340 pin 34 = P0.00, with
 * a 100K pull-up to 1V8 and the switch closing to ground. If that pin does
 * not follow the internal pulls, the three readings below say why:
 *
 *   no-pull / pull-up / pull-down
 *   0 / 1 / 0   pin is healthy and floating -- switch open, 1V8 pull absent
 *   1 / 1 / 0   pin is healthy, external pull-up present, switch open
 *   0 / 0 / 0   pin is held to ground -- switch closed, shorted, or the pin
 *               is not acting as a GPIO at all
 *   x / 1 / 0   follows the pulls, so the pin works; press should force 0
 */
static void focus_probe(void)
{
	static const struct {
		gpio_flags_t flags;
		const char *name;
	} modes[] = {
		{0, "none"},
		{GPIO_PULL_UP, "up"},
		{GPIO_PULL_DOWN, "down"},
	};
	const struct device *dev = ports[CONFIG_BUTTON_SCAN_FOCUS_PORT].dev;
	const uint8_t pin = CONFIG_BUTTON_SCAN_FOCUS_PIN;
	int level[ARRAY_SIZE(modes)];

	for (size_t i = 0; i < ARRAY_SIZE(modes); i++) {
		int ret = gpio_pin_configure(dev, pin, GPIO_INPUT | modes[i].flags);

		if (ret) {
			LOG_ERR("P%u.%02u configure(%s) failed: %d",
				CONFIG_BUTTON_SCAN_FOCUS_PORT, pin, modes[i].name, ret);
			return;
		}

		k_sleep(K_MSEC(SETTLE_MS));
		level[i] = gpio_pin_get_raw(dev, pin);
	}

	LOG_INF("P%u.%02u  no-pull=%d  pull-up=%d  pull-down=%d",
		CONFIG_BUTTON_SCAN_FOCUS_PORT, pin, level[0], level[1], level[2]);
}

int button_scan_run(uint32_t duration_ms, uint32_t period_ms)
{
	uint32_t now = k_uptime_get_32();
	uint32_t deadline = now + duration_ms;
	/* Force the first cycle to print, so the idle baseline is on record
	 * even if nothing ever changes.
	 */
	uint32_t last_forced = now - CONFIG_BUTTON_SCAN_HEARTBEAT_MS;
	uint32_t cycles = 0;
	bool forever = duration_ms == 0;
	int ret = ports_bind();

	if (ret) {
		return ret;
	}

	LOG_INF("Pin scan running. Hold the button; the pin it is on appears below.");
	LOG_INF("A pin listed both held and released is noise, not the button.");

	while (forever || (int32_t)(deadline - k_uptime_get_32()) > 0) {
		bool force;

		now = k_uptime_get_32();
		force = (uint32_t)(now - last_forced) >= CONFIG_BUTTON_SCAN_HEARTBEAT_MS;

		if (force) {
			last_forced = now;
			/* Proves the thread is still alive, so a quiet log can
			 * be read as "nothing moved" rather than "scan died".
			 */
			LOG_INF("-- alive, cycle %u --", cycles);

			if (IS_ENABLED(CONFIG_BUTTON_SCAN_FOCUS)) {
				focus_probe();
			}
		}

		scan_cycle(force);
		cycles++;
		k_sleep(K_MSEC(period_ms));
	}

	scan_release();

	return 0;
}

#if CONFIG_BUTTON_SCAN_AT_BOOT
static void button_scan_thread(void)
{
	/* Let the rest of init settle so peripheral pins are already claimed
	 * by their drivers and the reserved masks above are accurate.
	 */
	k_sleep(K_MSEC(CONFIG_BUTTON_SCAN_BOOT_DELAY_MS));

	LOG_INF("=== boot pin scan ===");

	/* Duration 0: loop until reset. */
	(void)button_scan_run(0, CONFIG_BUTTON_SCAN_PERIOD_MS);
}

K_THREAD_DEFINE(button_scan_tid, 1024, button_scan_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(10), 0, 0);
#endif /* CONFIG_BUTTON_SCAN_AT_BOOT */

static int cmd_button_scan(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t duration_ms = CONFIG_BUTTON_SCAN_DURATION_MS;

	if (argc == 2) {
		duration_ms = strtoul(argv[1], NULL, 10) * 1000;
	}

	shell_print(shell, "Scanning for %u ms, hold the button", duration_ms);

	return button_scan_run(duration_ms, CONFIG_BUTTON_SCAN_PERIOD_MS);
}

SHELL_STATIC_SUBCMD_SET_CREATE(button_scan_cmd,
			       SHELL_CMD(run, NULL, "Scan pins for a held button. Arg: seconds",
					 cmd_button_scan),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(btnscan, &button_scan_cmd, "Locate an unknown button pin", NULL);
