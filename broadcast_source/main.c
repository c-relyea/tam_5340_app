/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * TAM broadcast source — nRF5340 application core.
 *
 * Two independent radio activities run concurrently:
 *   LE audio BIS broadcast — upstream nRF5340 Audio broadcast_source, unmodified
 *   PPG beacon             — legacy non-connectable advertiser carrying raw
 *                            optical samples (see the PPG section)
 *
 * Sensor threads, all sharing i2c1:
 *   battery     prio 13, 10 s poll   BQ27427 SOC, firmware charge termination
 *   ppg_sample  prio 12, 5 ms poll   MAX30101 FIFO drain, 100 Hz
 *   ppg_stream  prio 10, queue-fed   packs 2 samples per beacon update
 *   imu         prio 12, 16 ms poll  LSM6DSV16X orientation + motion flag
 *
 * File layout:
 *   1. LE audio broadcast source   (upstream)
 *   2. Sensor bus                  (shared i2c1 bring-up)
 *   3. Battery — BQ27427
 *   4. PPG — MAX30101 sampling and beacon streaming
 *   5. IMU — LSM6DSV16X orientation and motion detection
 *   6. main()
 */

#include "streamctrl.h"

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>

#include <math.h>
#include <string.h>

#include "audio_system.h"
#include "bt_mgmt.h"
#include "broadcast_source.h"
#include "button_assignments.h"
#include "fw_info_app.h"
#include "led.h"
#include "macros_common.h"
#include "nrf5340_audio_dk.h"
#include "zbus_common.h"

LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

/* =========================================================================
 * 1. LE audio broadcast source — upstream nRF5340 Audio sample
 * =========================================================================
 */

ZBUS_SUBSCRIBER_DEFINE(button_evt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_MSG_SUBSCRIBER_DEFINE(le_audio_evt_sub);

ZBUS_CHAN_DECLARE(button_chan);
ZBUS_CHAN_DECLARE(le_audio_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);
ZBUS_CHAN_DECLARE(sdu_ref_chan);

ZBUS_OBS_DECLARE(sdu_ref_msg_listen);

static struct k_thread button_msg_sub_thread_data;
static struct k_thread le_audio_msg_sub_thread_data;

static k_tid_t button_msg_sub_thread_id;
static k_tid_t le_audio_msg_sub_thread_id;

struct bt_le_ext_adv *ext_adv;

K_THREAD_STACK_DEFINE(button_msg_sub_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(le_audio_msg_sub_thread_stack, CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE);

static enum stream_state strm_state = STATE_PAUSED;

/* Buffer for the UUIDs. */
#define EXT_ADV_UUID_BUF_SIZE (128)
NET_BUF_SIMPLE_DEFINE_STATIC(uuid_data, EXT_ADV_UUID_BUF_SIZE);
NET_BUF_SIMPLE_DEFINE_STATIC(uuid_data2, EXT_ADV_UUID_BUF_SIZE);

/* Buffer for periodic advertising BASE data. */
NET_BUF_SIMPLE_DEFINE_STATIC(base_data, 128);
NET_BUF_SIMPLE_DEFINE_STATIC(base_data2, 128);

/* Extended advertising buffer. */
static struct bt_data ext_adv_buf[CONFIG_BT_ISO_MAX_BIG][CONFIG_EXT_ADV_BUF_MAX];

/* Periodic advertising buffer. */
static struct bt_data per_adv_buf[CONFIG_BT_ISO_MAX_BIG];

#if (CONFIG_AURACAST)
/* Total size of the PBA buffer includes the 16-bit UUID, 8-bit features and the
 * meta data size.
 */
#define BROADCAST_SRC_PBA_BUF_SIZE                                                                 \
	(BROADCAST_SOURCE_PBA_HEADER_SIZE + CONFIG_BT_AUDIO_BROADCAST_PBA_METADATA_SIZE)

/* Number of metadata items that can be assigned. */
#define BROADCAST_SOURCE_PBA_METADATA_VACANT                                                       \
	(CONFIG_BT_AUDIO_BROADCAST_PBA_METADATA_SIZE / (sizeof(struct bt_data)))

/* Make sure pba_buf is large enough for a 16bit UUID and meta data
 * (any addition to pba_buf requires an increase of this value)
 */
uint8_t pba_data[CONFIG_BT_ISO_MAX_BIG][BROADCAST_SRC_PBA_BUF_SIZE];

/**
 * @brief	Broadcast source static extended advertising data.
 */
static struct broadcast_source_ext_adv_data ext_adv_data[] = {
	{.uuid_buf = &uuid_data,
	 .pba_metadata_vacant_cnt = BROADCAST_SOURCE_PBA_METADATA_VACANT,
	 .pba_buf = pba_data[0]},
	{.uuid_buf = &uuid_data2,
	 .pba_metadata_vacant_cnt = BROADCAST_SOURCE_PBA_METADATA_VACANT,
	 .pba_buf = pba_data[1]}};
#else
/**
 * @brief	Broadcast source static extended advertising data.
 */
static struct broadcast_source_ext_adv_data ext_adv_data[] = {{.uuid_buf = &uuid_data},
							      {.uuid_buf = &uuid_data2}};
#endif /* (CONFIG_AURACAST) */

/**
 * @brief	Broadcast source static periodic advertising data.
 */
static struct broadcast_source_per_adv_data per_adv_data[] = {{.base_buf = &base_data},
							      {.base_buf = &base_data2}};

/* Function for handling all stream state changes */
static void stream_state_set(enum stream_state stream_state_new)
{
	strm_state = stream_state_new;
}

/**
 * @brief	Handle button activity.
 */
static void button_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&button_evt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct button_msg msg;

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		LOG_DBG("Got btn evt from queue - id = %d, action = %d", msg.button_pin,
			msg.button_action);

		if (msg.button_action != BUTTON_PRESS) {
			LOG_WRN("Unhandled button action");
			return;
		}

		switch (msg.button_pin) {
		case BUTTON_PLAY_PAUSE:
			if (strm_state == STATE_STREAMING) {
				ret = broadcast_source_stop(0);
				if (ret) {
					LOG_WRN("Failed to stop broadcaster: %d", ret);
				}
			} else if (strm_state == STATE_PAUSED) {
				ret = broadcast_source_start(0, ext_adv);
				if (ret) {
					LOG_WRN("Failed to start broadcaster: %d", ret);
				}
			} else {
				LOG_WRN("In invalid state: %d", strm_state);
			}

			break;

		case BUTTON_4:
			if (IS_ENABLED(CONFIG_AUDIO_TEST_TONE)) {
				static bool test_tone_active;

				if (strm_state != STATE_STREAMING) {
					LOG_INF("BUTTON_4: not streaming (state=%d)", strm_state);
					break;
				}

				test_tone_active = !test_tone_active;

				if (test_tone_active) {
					ret = audio_system_encode_test_tone_step();
				} else {
					ret = audio_system_encode_test_tone_set(0);
					LOG_INF("Test tone OFF");
				}

				if (ret) {
					LOG_WRN("Failed to set test tone: %d", ret);
					test_tone_active = false;
				}

				break;
			}

			break;

		default:
			LOG_WRN("Unexpected/unhandled button id: %d", msg.button_pin);
		}

		STACK_USAGE_PRINT("button_msg_thread", &button_msg_sub_thread_data);
	}
}

/**
 * @brief	Handle Bluetooth LE audio events.
 */
static void le_audio_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		struct le_audio_msg msg;

		ret = zbus_sub_wait_msg(&le_audio_evt_sub, &chan, &msg, K_FOREVER);
		ERR_CHK(ret);

		LOG_DBG("Received event = %d, current state = %d", msg.event, strm_state);

		switch (msg.event) {
		case LE_AUDIO_EVT_STREAMING:
			LOG_DBG("LE audio evt streaming");

			audio_system_encoder_start();

			if (strm_state == STATE_STREAMING) {
				LOG_DBG("Got streaming event in streaming state");
				break;
			}

			audio_system_start();
			stream_state_set(STATE_STREAMING);
			ret = led_blink(LED_APP_1_BLUE);
			ERR_CHK(ret);


			break;

		case LE_AUDIO_EVT_NOT_STREAMING:
			LOG_DBG("LE audio evt not_streaming");

			audio_system_encoder_stop();

			if (strm_state == STATE_PAUSED) {
				LOG_DBG("Got not_streaming event in paused state");
				break;
			}

			stream_state_set(STATE_PAUSED);
			audio_system_stop();
			ret = led_on(LED_APP_1_BLUE);
			ERR_CHK(ret);

			break;

		case LE_AUDIO_EVT_STREAM_SENT:
			/* Nothing to do. */
			break;

		default:
			LOG_WRN("Unexpected/unhandled le_audio event: %d", msg.event);

			break;
		}

		STACK_USAGE_PRINT("le_audio_msg_thread", &le_audio_msg_sub_thread_data);
	}
}

/**
 * @brief	Create zbus subscriber threads.
 *
 * @return	0 for success, error otherwise.
 */
static int zbus_subscribers_create(void)
{
	int ret;

	button_msg_sub_thread_id = k_thread_create(
		&button_msg_sub_thread_data, button_msg_sub_thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)button_msg_sub_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(button_msg_sub_thread_id, "BUTTON_MSG_SUB");
	if (ret) {
		LOG_ERR("Failed to create button_msg thread");
		return ret;
	}

	le_audio_msg_sub_thread_id = k_thread_create(
		&le_audio_msg_sub_thread_data, le_audio_msg_sub_thread_stack,
		CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE, (k_thread_entry_t)le_audio_msg_sub_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_LE_AUDIO_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(le_audio_msg_sub_thread_id, "LE_AUDIO_MSG_SUB");
	if (ret) {
		LOG_ERR("Failed to create le_audio_msg thread");
		return ret;
	}

	ret = zbus_chan_add_obs(&sdu_ref_chan, &sdu_ref_msg_listen, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add timestamp listener");
		return ret;
	}

	return 0;
}

/**
 * @brief	Zbus listener to receive events from bt_mgmt.
 *
 * @param[in]	chan	Zbus channel.
 *
 * @note	Will in most cases be called from BT_RX context,
 *		so there should not be too much processing done here.
 */
static void bt_mgmt_evt_handler(const struct zbus_channel *chan)
{
	int ret;
	const struct bt_mgmt_msg *msg;

	msg = zbus_chan_const_msg(chan);

	switch (msg->event) {
	case BT_MGMT_EXT_ADV_WITH_PA_READY:
		LOG_INF("Ext adv ready");

		ext_adv = msg->ext_adv;

		ret = broadcast_source_start(msg->index, ext_adv);
		if (ret) {
			LOG_ERR("Failed to start broadcaster: %d", ret);
		}

		break;

	default:
		LOG_WRN("Unexpected/unhandled bt_mgmt event: %d", msg->event);
		break;
	}
}

ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen, bt_mgmt_evt_handler);

/**
 * @brief	Link zbus producers and observers.
 *
 * @return	0 for success, error otherwise.
 */
static int zbus_link_producers_observers(void)
{
	int ret;

	if (!IS_ENABLED(CONFIG_ZBUS)) {
		return -ENOTSUP;
	}

	ret = zbus_chan_add_obs(&button_chan, &button_evt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add button sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&le_audio_chan, &le_audio_evt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add le_audio sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&bt_mgmt_chan, &bt_mgmt_evt_listen, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add bt_mgmt listener");
		return ret;
	}

	return 0;
}

/*
 * @brief  The following configures the data for the extended advertising.
 *         This includes the Broadcast Audio Announcements [BAP 3.7.2.1] and Broadcast_ID
 *         [BAP 3.7.2.1.1] in the AUX_ADV_IND Extended Announcements.
 *
 * @param  big_index         Index of the Broadcast Isochronous Group (BIG) to get
 *                           advertising data for.
 * @param  ext_adv_data      Pointer to the extended advertising buffers.
 * @param  ext_adv_buf       Pointer to the bt_data used for extended advertising.
 * @param  ext_adv_buf_size  Size of @p ext_adv_buf.
 * @param  ext_adv_count     Pointer to the number of elements added to @p adv_buf.
 *
 * @return  0 for success, error otherwise.
 */
static int ext_adv_populate(uint8_t big_index, struct broadcast_source_ext_adv_data *ext_adv_data,
			    struct bt_data *ext_adv_buf, size_t ext_adv_buf_size,
			    size_t *ext_adv_count)
{
	int ret;
	size_t ext_adv_buf_cnt = 0;

	if (IS_ENABLED(CONFIG_BT_AUDIO_USE_BROADCAST_NAME_ALT)) {
		if (sizeof(CONFIG_BT_AUDIO_BROADCAST_NAME_ALT) >
		    ARRAY_SIZE(ext_adv_data->brdcst_name_buf)) {
			LOG_ERR("CONFIG_BT_AUDIO_BROADCAST_NAME_ALT is too long");
			return -EINVAL;
		}

		size_t brdcst_name_size = sizeof(CONFIG_BT_AUDIO_BROADCAST_NAME_ALT) - 1;

		memcpy(ext_adv_data->brdcst_name_buf, CONFIG_BT_AUDIO_BROADCAST_NAME_ALT,
		       brdcst_name_size);
	} else {
		if (sizeof(CONFIG_BT_AUDIO_BROADCAST_NAME) >
		    ARRAY_SIZE(ext_adv_data->brdcst_name_buf)) {
			LOG_ERR("CONFIG_BT_AUDIO_BROADCAST_NAME is too long");
			return -EINVAL;
		}

		size_t brdcst_name_size = sizeof(CONFIG_BT_AUDIO_BROADCAST_NAME) - 1;

		memcpy(ext_adv_data->brdcst_name_buf, CONFIG_BT_AUDIO_BROADCAST_NAME,
		       brdcst_name_size);
	}

	ext_adv_buf[ext_adv_buf_cnt].type = BT_DATA_UUID16_ALL;
	ext_adv_buf[ext_adv_buf_cnt].data = ext_adv_data->uuid_buf->data;
	ext_adv_buf_cnt++;

	ret = bt_mgmt_manufacturer_uuid_populate(ext_adv_data->uuid_buf,
						 CONFIG_BT_DEVICE_MANUFACTURER_ID);
	if (ret) {
		LOG_ERR("Failed to add adv data with manufacturer ID: %d", ret);
		return ret;
	}

	bool fixed_id = !IS_ENABLED(CONFIG_BT_AUDIO_USE_BROADCAST_ID_RANDOM);

	uint32_t broadcast_id = CONFIG_BT_AUDIO_BROADCAST_ID_FIXED;

	ret = broadcast_source_ext_adv_populate(big_index, fixed_id, broadcast_id, ext_adv_data,
						&ext_adv_buf[ext_adv_buf_cnt],
						ext_adv_buf_size - ext_adv_buf_cnt);
	if (ret < 0) {
		LOG_ERR("Failed to add ext adv data for broadcast source: %d", ret);
		return ret;
	}

	ext_adv_buf_cnt += ret;

	/* Add the number of UUIDs */
	ext_adv_buf[0].data_len = ext_adv_data->uuid_buf->len;

	LOG_DBG("Size of adv data: %d, num_elements: %d", sizeof(struct bt_data) * ext_adv_buf_cnt,
		ext_adv_buf_cnt);

	*ext_adv_count = ext_adv_buf_cnt;

	return 0;
}

/*
 * @brief  The following configures the data for the periodic advertising.
 *         This includes the Basic Audio Announcement, including the
 *         BASE [BAP 3.7.2.2] and BIGInfo.
 *
 * @param  big_index         Index of the Broadcast Isochronous Group (BIG) to get
 *                           advertising data for.
 * @param  pre_adv_data      Pointer to the periodic advertising buffers.
 * @param  per_adv_buf       Pointer to the bt_data used for periodic advertising.
 * @param  per_adv_buf_size  Size of @p ext_adv_buf.
 * @param  per_adv_count     Pointer to the number of elements added to @p adv_buf.
 *
 * @return  0 for success, error otherwise.
 */
static int per_adv_populate(uint8_t big_index, struct broadcast_source_per_adv_data *pre_adv_data,
			    struct bt_data *per_adv_buf, size_t per_adv_buf_size,
			    size_t *per_adv_count)
{
	int ret;
	size_t per_adv_buf_cnt = 0;

	ret = broadcast_source_per_adv_populate(big_index, pre_adv_data, per_adv_buf,
						per_adv_buf_size - per_adv_buf_cnt);
	if (ret < 0) {
		LOG_ERR("Failed to add per adv data for broadcast source: %d", ret);
		return ret;
	}

	per_adv_buf_cnt += ret;

	LOG_DBG("Size of per adv data: %d, num_elements: %d",
		sizeof(struct bt_data) * per_adv_buf_cnt, per_adv_buf_cnt);

	*per_adv_count = per_adv_buf_cnt;

	return 0;
}

uint8_t stream_state_get(void)
{
	return strm_state;
}

void streamctrl_send(void const *const data, size_t size, uint8_t num_ch)
{
	int ret;
	static int prev_ret;

	struct le_audio_encoded_audio enc_audio = {.data = data, .size = size, .num_ch = num_ch};

	if (strm_state == STATE_STREAMING) {
		ret = broadcast_source_send(0, 0, enc_audio);

		if (ret != 0 && ret != prev_ret) {
			if (ret == -ECANCELED) {
				LOG_WRN("Sending operation cancelled");
			} else {
				LOG_WRN("Problem with sending LE audio data, ret: %d", ret);
			}
		}

		prev_ret = ret;
	}
}

#if CONFIG_CUSTOM_BROADCASTER
/* Example of how to create a custom broadcaster */
/**
 * Remember to increase:
 * CONFIG_BT_BAP_BROADCAST_SRC_SUBGROUP_COUNT
 * CONFIG_BT_CTLR_ADV_ISO_STREAM_COUNT (set in hci_ipc.conf)
 * CONFIG_BT_ISO_TX_BUF_COUNT
 * CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT
 * CONFIG_BT_ISO_MAX_CHAN
 */
#error Feature is incomplete and should only be used as a guideline for now
static struct bt_bap_lc3_preset lc3_preset_48 = BT_BAP_LC3_BROADCAST_PRESET_48_4_1(
	BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT, BT_AUDIO_CONTEXT_TYPE_MEDIA);

static void broadcast_create(struct broadcast_source_big *broadcast_param)
{
	static enum bt_audio_location location[2] = {BT_AUDIO_LOCATION_FRONT_LEFT,
						     BT_AUDIO_LOCATION_FRONT_RIGHT};
	static struct subgroup_config subgroups[2];

	subgroups[0].group_lc3_preset = lc3_preset_48;
	subgroups[0].num_bises = 2;
	subgroups[0].context = BT_AUDIO_CONTEXT_TYPE_MEDIA;
	subgroups[0].location = location;

	subgroups[1].group_lc3_preset = lc3_preset_48;
	subgroups[1].num_bises = 2;
	subgroups[1].context = BT_AUDIO_CONTEXT_TYPE_MEDIA;
	subgroups[1].location = location;

	broadcast_param->packing = BT_ISO_PACKING_INTERLEAVED;

	broadcast_param->encryption = false;

	bt_audio_codec_cfg_meta_set_bcast_audio_immediate_rend_flag(
		&subgroups[0].group_lc3_preset.codec_cfg);
	bt_audio_codec_cfg_meta_set_bcast_audio_immediate_rend_flag(
		&subgroups[1].group_lc3_preset.codec_cfg);

	uint8_t spanish_src[3] = "spa";
	uint8_t english_src[3] = "eng";

	bt_audio_codec_cfg_meta_set_stream_lang(&subgroups[0].group_lc3_preset.codec_cfg,
						(uint32_t)sys_get_le24(english_src));
	bt_audio_codec_cfg_meta_set_stream_lang(&subgroups[1].group_lc3_preset.codec_cfg,
						(uint32_t)sys_get_le24(spanish_src));

	broadcast_param->subgroups = subgroups;
	broadcast_param->num_subgroups = 2;
}
#endif /* CONFIG_CUSTOM_BROADCASTER */

/* =========================================================================
 * 2. Sensor bus — shared i2c1
 * =========================================================================
 */

/* The MAX30101, LSM6DSV16X and BQ27427 all hang off i2c1. */
#define SENSOR_I2C_NODE DT_NODELABEL(i2c1)

/* Sensor rails come up with PWR_EN in nrf5340_audio_dk_init(). Writes issued
 * before that fail silently, so every sensor thread waits this long before
 * touching the bus.
 */
#define SENSOR_RAIL_SETTLE_MS 2000

/* Returns the sensor bus once the rails have settled, or NULL if it never
 * came ready.
 */
static const struct device *sensor_bus_get(const char *who)
{
	const struct device *i2c_dev = DEVICE_DT_GET(SENSOR_I2C_NODE);

	k_msleep(SENSOR_RAIL_SETTLE_MS);

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("%s: i2c1 not ready", who);
		return NULL;
	}

	return i2c_dev;
}

/* =========================================================================
 * 3. Battery — BQ27427 fuel gauge
 * =========================================================================
 */

/* BQ27427 at 0x55, standard command set. */
#define BQ27427_ADDR     0x55
#define BQ27427_REG_VOLT 0x04 /* Voltage,         u16 LE, mV */
#define BQ27427_REG_SOC  0x1C /* State of charge, u16 LE, %  */

#define BATTERY_POLL_MS 10000 /* SOC changes slowly */

/* The charger never hits its own current-based termination: LED and radio
 * current peaks keep the average draw above its ~10 % threshold. Terminate in
 * firmware once SOC reaches the 80-90 % target band instead.
 */
#define CHG_TERM_SOC_PCT 80

/* Last SOC read, published into the beacon payload. 0xFF until first read. */
static volatile uint8_t battery_soc_pct = 0xFF;

static void battery_thread(void)
{
	const struct device *i2c_dev = sensor_bus_get("battery");

	if (i2c_dev == NULL) {
		return;
	}

	LOG_INF("Battery gauge ready");

	while (1) {
		uint8_t buf[2];
		int ret = i2c_burst_read(i2c_dev, BQ27427_ADDR, BQ27427_REG_SOC, buf, sizeof(buf));

		if (ret) {
			LOG_WRN("Battery SOC read error: %d", ret);
		} else {
			uint8_t soc = (uint8_t)sys_get_le16(buf);

			LOG_INF("Battery: %u%%", soc);
			battery_soc_pct = soc;
			tam_board_chg_term_set(soc >= CHG_TERM_SOC_PCT);
		}

		k_msleep(BATTERY_POLL_MS);
	}
}

K_THREAD_DEFINE(battery_thread_id, 1024, battery_thread, NULL, NULL, NULL, 13, 0, 0);

/* =========================================================================
 * 4. PPG — MAX30101 sampling and beacon streaming
 *
 * The board only samples and streams raw IR/red/green; all HR and SpO2
 * analysis happens downstream at the sink.
 * =========================================================================
 */

/* MAX30101 at 0x57. */
#define MAX30101_ADDR          0x57
#define MAX30101_REG_FIFO_WR   0x04 /* WR_PTR, OVF_CNT, RD_PTR at 0x04-0x06 */
#define MAX30101_REG_OVF_CNT   0x05
#define MAX30101_REG_FIFO_RD   0x06
#define MAX30101_REG_FIFO_DATA 0x07
#define MAX30101_REG_FIFO_CFG  0x08
#define MAX30101_REG_MODE_CFG  0x09
#define MAX30101_REG_SPO2_CFG  0x0A
#define MAX30101_REG_LED1_PA   0x0C /* RED           */
#define MAX30101_REG_LED2_PA   0x0D /* IR            */
#define MAX30101_REG_LED3_PA   0x0E /* GREEN, DAC 1  */
#define MAX30101_REG_LED4_PA   0x0F /* GREEN, DAC 2  */
#define MAX30101_REG_SLOT12    0x11
#define MAX30101_REG_SLOT34    0x12
#define MAX30101_REG_PART_ID   0xFF
#define MAX30101_PART_ID       0x15
#define MAX30101_MODE_MULTILED 0x07
#define MAX30101_ADC_MASK      0x3FFFF /* 18-bit samples */

/* The FIFO fills at 10 ms/sample; a 5 ms poll keeps each drain burst to 1-2
 * samples so timestamps stay close to sample time.
 */
#define PPG_POLL_MS      5
#define PPG_FIFO_DEPTH   32 /* FIFO pointers are 5-bit */
#define PPG_SAMPLE_BYTES 9  /* one sample: red, ir, green — 3 bytes each */

/* Set to 1 to stream raw samples over RTT as CSV:
 * P,<uptime_ms>,<ir>,<red>,<green>
 * Capture with log_ppg.sh and filter lines starting with "P," (~100 lines/s).
 */
#define PPG_RAW_LOG 0

/* -------------------------------------------------------------------------
 * PPG beacon payload
 *
 * A GATT connection never held up at usable range, so raw samples ride in the
 * AD payload of a dedicated non-connectable advertiser instead — no
 * connection, subscription or supervision timeout involved.
 *
 * The PDU is deliberately legacy (no BT_LE_ADV_OPT_EXT_ADV): the Raspberry Pi
 * 5 receiver's BlueZ/controller stack scans legacy-only (verified with btmon —
 * zero extended reports even with Experimental mode on), so an extended
 * beacon is invisible to it. Legacy caps the payload at 31 bytes, hence 2
 * samples per update at the 20 ms minimum interval → 100 Hz target (~65-70 Hz
 * received; the rest lost to contention with the BIS broadcast).
 *
 * Payload layout (manufacturer-specific AD, little-endian):
 *   [0..1]   company_id  required first field of BT_DATA_MANUFACTURER_DATA
 *   [2]      magic       'T' — sanity check vs other Nordic boards
 *   [3]      seq         per-update counter, for drop detection
 *   [4]      bat_pct     battery % (0xFF = not yet read)
 *   [5..6]   sample_idx  u16 raw-sample counter, wraps every ~655 s
 *   [7..15]  sample A    { ir u24, red u24, green u24 }
 *   [16..24] sample B    same
 * -------------------------------------------------------------------------
 */
#define TAM_BEACON_MAGIC        'T'
#define TAM_BEACON_PAYLOAD_LEN  23
#define TAM_BEACON_MFG_LEN      (2 + TAM_BEACON_PAYLOAD_LEN) /* + company_id */
#define TAM_BEACON_ADV_INTERVAL 0x0020 /* 20 ms in 0.625 ms units — spec min */

/* Payload field offsets, company_id included. */
#define TAM_BEACON_OFF_COMPANY 0
#define TAM_BEACON_OFF_MAGIC   2
#define TAM_BEACON_OFF_SEQ     3
#define TAM_BEACON_OFF_BAT     4
#define TAM_BEACON_OFF_IDX     5
#define TAM_BEACON_OFF_SAMPLE  7
#define TAM_BEACON_SAMPLE_LEN  9

static uint8_t beacon_payload[TAM_BEACON_MFG_LEN];

static const struct bt_data beacon_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, beacon_payload, sizeof(beacon_payload)),
};

static const struct bt_le_adv_param *beacon_adv_param =
	BT_LE_ADV_PARAM(0, TAM_BEACON_ADV_INTERVAL, TAM_BEACON_ADV_INTERVAL, NULL);

/* -------------------------------------------------------------------------
 * Sample queue
 *
 * bt_le_adv_update_data() runs a full HCI exchange and can outlast a sample
 * period, so it must not run on the sampling thread — that thread stalling
 * means the sensor FIFO stops draining. The sampler enqueues raw samples
 * (drop-on-full) and the stream thread batches 2 per beacon update.
 * -------------------------------------------------------------------------
 */
struct ppg_sample {
	uint32_t ir;
	uint32_t red;
	uint32_t green;
};

/* 64 samples ≈ 0.64 s at 100 Hz — absorbs BIS-induced gaps. */
K_MSGQ_DEFINE(ppg_msgq, sizeof(struct ppg_sample), 64, 4);

static uint32_t ppg_stream_drops;

/* Non-blocking: drop rather than stall the sampler when the radio is busy.
 * The beacon broadcasts unconditionally, so there is no subscriber state to
 * gate on as there would be with a GATT notify.
 */
static void ppg_stream_put(uint32_t ir, uint32_t red, uint32_t green)
{
	struct ppg_sample s = {.ir = ir, .red = red, .green = green};

	if (k_msgq_put(&ppg_msgq, &s, K_NO_WAIT) != 0) {
		ppg_stream_drops++;
	}
}

static void ppg_pack_sample(uint8_t *p, const struct ppg_sample *s)
{
	sys_put_le24(s->ir, &p[0]);
	sys_put_le24(s->red, &p[3]);
	sys_put_le24(s->green, &p[6]);
}

static void ppg_stream_thread(void)
{
	uint8_t seq = 0;
	uint16_t sample_idx = 0;
	uint32_t last_warn_ms = 0;

	sys_put_le16(CONFIG_BT_DEVICE_MANUFACTURER_ID, &beacon_payload[TAM_BEACON_OFF_COMPANY]);

	while (1) {
		struct ppg_sample a, b;
		int ret;

		/* K_FOREVER, so a failure here only means the queue was purged. */
		if (k_msgq_get(&ppg_msgq, &a, K_FOREVER) != 0 ||
		    k_msgq_get(&ppg_msgq, &b, K_FOREVER) != 0) {
			continue;
		}

		beacon_payload[TAM_BEACON_OFF_MAGIC] = TAM_BEACON_MAGIC;
		beacon_payload[TAM_BEACON_OFF_SEQ] = seq++;
		beacon_payload[TAM_BEACON_OFF_BAT] = battery_soc_pct;
		sys_put_le16(sample_idx, &beacon_payload[TAM_BEACON_OFF_IDX]);
		ppg_pack_sample(&beacon_payload[TAM_BEACON_OFF_SAMPLE], &a);
		ppg_pack_sample(&beacon_payload[TAM_BEACON_OFF_SAMPLE + TAM_BEACON_SAMPLE_LEN], &b);
		sample_idx += 2;

		ret = bt_le_adv_update_data(beacon_ad, ARRAY_SIZE(beacon_ad), NULL, 0);

		/* Rate-limited: a busy radio fails these back to back. */
		if (ret && k_uptime_get_32() - last_warn_ms > 1000) {
			last_warn_ms = k_uptime_get_32();
			LOG_WRN("Beacon adv update failed: %d (drops=%u)", ret, ppg_stream_drops);
		}
	}
}

K_THREAD_DEFINE(ppg_stream_id, 2048, ppg_stream_thread, NULL, NULL, NULL, 10, 0, 0);

/* -------------------------------------------------------------------------
 * MAX30101 sampler
 * -------------------------------------------------------------------------
 */

/* The chip boots into shutdown, so this must run after the rails are up or
 * the FIFO never fills.
 */
static int max30101_configure(const struct device *i2c_dev)
{
	int ret;
	uint8_t id = 0;
	uint8_t mode = 0;

	/* Wait for the sensor to come up on the rails. */
	for (int tries = 0; tries < 50; tries++) {
		ret = i2c_reg_read_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_PART_ID, &id);
		if (ret == 0 && id == MAX30101_PART_ID) {
			break;
		}
		k_msleep(100);
	}

	if (id != MAX30101_PART_ID) {
		LOG_ERR("MAX30101 not responding (PART_ID=0x%02X)", id);
		return -ENODEV;
	}

	ret = i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_FIFO_CFG, 0x2F);

	/* Gain, 200 Hz, 411 us pulse width. 200 Hz is the chip's max with 3 LED
	 * slots at 411 us — programming 400 Hz gets clipped internally. With
	 * avg=2 that gives a true 100 Hz FIFO rate.
	 */
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_SPO2_CFG, 0x4B);

	/* LED currents, 0.2 mA/LSB, tuned for reflective NECK placement.
	 * RED is the SpO2 SNR-limiting channel (PI ~0.017 %, AC ~25 counts), so
	 * it runs hottest — watch for DC clipping near 262143 if contact
	 * improves. GREEN carries HR on the neck (5-10x the pulsation of
	 * IR/red) and in multi-LED mode with SLOTx=011 it sinks from both
	 * LED3_PA and LED4_PA (datasheet Table 9), hence both DACs.
	 */
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_LED1_PA, 0x80); /* RED   25.6 mA */
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_LED2_PA, 0x60); /* IR    19.2 mA */
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_LED3_PA, 0x50); /* GREEN 16.0 mA */
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_LED4_PA, 0x50); /* GREEN 16.0 mA */

	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_SLOT12, 0x21); /* RED, IR */
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_SLOT34, 0x03); /* GREEN   */

	/* Clear FIFO pointers, then enable multi-LED mode. */
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_FIFO_WR, 0x00);
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_OVF_CNT, 0x00);
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_FIFO_RD, 0x00);
	ret |= i2c_reg_write_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_MODE_CFG,
				  MAX30101_MODE_MULTILED);

	if (ret) {
		LOG_ERR("MAX30101 config write failed");
		return -EIO;
	}

	ret = i2c_reg_read_byte(i2c_dev, MAX30101_ADDR, MAX30101_REG_MODE_CFG, &mode);
	if (ret || mode != MAX30101_MODE_MULTILED) {
		LOG_ERR("MAX30101 mode verify failed (mode=0x%02X, err=%d)", mode, ret);
		return -EIO;
	}

	LOG_INF("Pulse oximeter configured");

	return 0;
}

static void ppg_sample_thread(void)
{
	const struct device *i2c_dev = sensor_bus_get("ppg");

	if (i2c_dev == NULL || max30101_configure(i2c_dev)) {
		return;
	}

	while (1) {
		uint8_t ptrs[3]; /* FIFO_WR_PTR, OVF_COUNTER, FIFO_RD_PTR */
		int avail;
		int ret;

		ret = i2c_burst_read(i2c_dev, MAX30101_ADDR, MAX30101_REG_FIFO_WR, ptrs,
				     sizeof(ptrs));
		if (ret) {
			LOG_WRN("FIFO ptr read error %d", ret);
			k_msleep(8 * PPG_POLL_MS);
			continue;
		}

		avail = (ptrs[0] - ptrs[2]) & (PPG_FIFO_DEPTH - 1);

		while (avail-- > 0) {
			uint8_t fifo[PPG_SAMPLE_BYTES];
			uint32_t red, ir, green;

			ret = i2c_burst_read(i2c_dev, MAX30101_ADDR, MAX30101_REG_FIFO_DATA, fifo,
					     sizeof(fifo));
			if (ret) {
				LOG_WRN("FIFO read error %d", ret);
				break;
			}

			red = sys_get_be24(&fifo[0]) & MAX30101_ADC_MASK;
			ir = sys_get_be24(&fifo[3]) & MAX30101_ADC_MASK;
			green = sys_get_be24(&fifo[6]) & MAX30101_ADC_MASK;

#if PPG_RAW_LOG
			printk("P,%u,%u,%u,%u\n", k_uptime_get_32(), ir, red, green);
#endif
			ppg_stream_put(ir, red, green);
		}

		k_msleep(PPG_POLL_MS);
	}
}

K_THREAD_DEFINE(ppg_sample_id, 2048, ppg_sample_thread, NULL, NULL, NULL, 12, 0, 0);

/* =========================================================================
 * 5. IMU — LSM6DSV16X orientation and motion detection
 *
 * Two derived quantities are computed on-chip from the accel/gyro poll:
 *
 *   Orientation — a single-pole low pass on the accelerometer estimates the
 *   gravity vector, giving absolute pitch and roll. Gravity is an external
 *   reference so neither drifts. Yaw is unobservable with 6 axes and no
 *   magnetometer, so it is not computed at all.
 *
 *   Motion flag — |‖a‖ − 1 g| plus gyro rate magnitude, with hysteresis and a
 *   hold-off. This is the gate for discarding PPG windows corrupted by
 *   movement: the accel term catches translation, the gyro term catches
 *   rotation about the gravity axis that the accel cannot see.
 * =========================================================================
 */

/* LSM6DSV16X at 0x6A. */
#define LSM6DSV16X_ADDR         0x6A
#define LSM6DSV16X_REG_WHO_AM_I 0x0F /* Reads 0x71                           */
#define LSM6DSV16X_REG_CTRL1    0x10 /* Accel: op_mode_xl[6:4] odr_xl[3:0]   */
#define LSM6DSV16X_REG_CTRL2    0x11 /* Gyro:  op_mode_g[6:4]  odr_g[3:0]    */
#define LSM6DSV16X_REG_CTRL3    0x12 /* boot[7] bdu[6] if_inc[2] sw_reset[0] */
#define LSM6DSV16X_REG_CTRL6    0x15 /* Gyro  full scale: fs_g[3:0]          */
#define LSM6DSV16X_REG_CTRL8    0x17 /* Accel full scale: fs_xl[1:0]         */
#define LSM6DSV16X_REG_OUTX_L_G 0x22 /* Gyro XYZ then accel XYZ — 12 bytes   */
#define LSM6DSV16X_WHOAMI       0x71

/* Register values, kept next to the sensitivities they imply. */
#define LSM6DSV16X_CTRL3_SW_RESET  0x01
#define LSM6DSV16X_CTRL3_BDU_IFINC 0x44 /* required for coherent burst reads */
#define LSM6DSV16X_CTRL6_FS_125DPS 0x00
#define LSM6DSV16X_CTRL8_FS_2G     0x00
#define LSM6DSV16X_CTRL1_HP_60HZ   0x05 /* op_mode 000, odr 0101 */
#define LSM6DSV16X_CTRL2_HP_60HZ   0x05

/* Requested period. Measured on hardware the loop lands at ~20 ms (50 Hz):
 * ppg_stream_thread runs at a higher priority every 20 ms to match the beacon
 * interval and paces this thread, and the 100 kHz bus costs ~1.4 ms per burst.
 * Harmless here — BDU keeps each burst coherent, and polling below the 60 Hz
 * ODR just re-reads the newest sample. Raising the rate would mean outranking
 * the beacon thread, which the PPG stream needs more than this does.
 */
#define IMU_POLL_MS   16
#define IMU_RAW_BYTES 12

/* Must match the full scales written to CTRL6/CTRL8 above. */
#define IMU_ACCEL_MG_PER_LSB  0.061f /* ±2 g     */
#define IMU_GYRO_MDPS_PER_LSB 4.375f /* ±125 dps */
#define IMU_RAD_TO_DEG        57.29577951f
#define IMU_ONE_G_MG          1000.0f

/* Gravity low pass. Time constant is IMU_POLL_MS/alpha, so 0.05 at 16 ms is
 * ~320 ms: slow enough to reject walking-rate linear acceleration, fast
 * enough that a new tilt settles well inside a second.
 */
#define IMU_GRAV_ALPHA 0.05f

/* Motion thresholds, separate enter/exit levels so a signal hovering at the
 * limit does not chatter the flag.
 */
#define IMU_MOTION_ACC_ENTER_MG  50.0f
#define IMU_MOTION_ACC_EXIT_MG   25.0f
#define IMU_MOTION_GYR_ENTER_DPS 12.0f
#define IMU_MOTION_GYR_EXIT_DPS  6.0f

/* Keep the flag asserted this long after the last trigger. A PPG artifact
 * outlasts the movement that caused it — perfusion needs time to settle — so
 * the gate has to stay closed past the end of the motion itself.
 */
#define IMU_MOTION_HOLD_MS 500

/* Set to 1 to stream raw samples over RTT as CSV:
 * I,<uptime_ms>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>
 */
#define IMU_RAW_LOG 0

/* Set to 1 to stream derived state over RTT as CSV, decimated to ~10 Hz:
 * O,<uptime_ms>,<pitch_deg>,<roll_deg>,<accel_dev_mg>,<gyro_dps>,<moving>
 */
#define IMU_ORIENT_LOG     1
#define IMU_ORIENT_LOG_DIV 6 /* 60 Hz / 6 ≈ 10 Hz */

/* Latest sample and derived state. Single writer (the IMU thread); readers
 * take individual fields, each a naturally aligned 32-bit-or-smaller load, so
 * no lock is needed. Pitch and roll may come from adjacent samples when read
 * separately, which at 60 Hz is far below the resolution anything here cares
 * about.
 */
static struct {
	int16_t gyro[3];    /* raw LSB, 4.375 mdps/LSB at ±125 dps      */
	int16_t accel[3];   /* raw LSB, 0.061 mg/LSB   at ±2 g          */
	float pitch_deg;    /* about Y, −90..+90, absolute vs gravity   */
	float roll_deg;     /* about X, −180..+180, absolute vs gravity */
	float accel_dev_mg; /* |‖a‖ − 1 g|, static tilt removed        */
	float gyro_mag_dps; /* vector magnitude of the rate             */
	bool moving;
	bool valid; /* false until the IMU answered and the LPF primed */
} imu;

/* Motion gate for the PPG path: true when the sample stream is untrustworthy.
 * Fails closed — if the IMU never came up, nothing is vouched for.
 */
bool imu_motion_active(void)
{
	return !imu.valid || imu.moving;
}

/* Absolute tilt vs gravity. False if the IMU is not up yet, in which case the
 * outputs are left untouched.
 */
bool imu_get_orientation(float *pitch_deg, float *roll_deg)
{
	if (!imu.valid) {
		return false;
	}

	*pitch_deg = imu.pitch_deg;
	*roll_deg = imu.roll_deg;

	return true;
}

static int lsm6dsv16x_configure(const struct device *i2c_dev)
{
	uint8_t who_am_i = 0;
	int ret;

	ret = i2c_reg_read_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_WHO_AM_I, &who_am_i);
	if (ret || who_am_i != LSM6DSV16X_WHOAMI) {
		LOG_ERR("IMU not found (WHO_AM_I=0x%02X, err=%d)", who_am_i, ret);
		return -ENODEV;
	}

	/* Reset first — a warm reboot leaves the IMU configured from last run. */
	ret = i2c_reg_write_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_CTRL3,
				 LSM6DSV16X_CTRL3_SW_RESET);
	k_msleep(10);

	ret |= i2c_reg_write_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_CTRL3,
				  LSM6DSV16X_CTRL3_BDU_IFINC);
	ret |= i2c_reg_write_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_CTRL6,
				  LSM6DSV16X_CTRL6_FS_125DPS);
	ret |= i2c_reg_write_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_CTRL8,
				  LSM6DSV16X_CTRL8_FS_2G);
	ret |= i2c_reg_write_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_CTRL1,
				  LSM6DSV16X_CTRL1_HP_60HZ);
	ret |= i2c_reg_write_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_CTRL2,
				  LSM6DSV16X_CTRL2_HP_60HZ);
	if (ret) {
		LOG_ERR("IMU config write failed");
		return -EIO;
	}

	k_msleep(20); /* one ODR period, so the first sample is ready */

	LOG_INF("IMU ready (WHO_AM_I=0x%02X)", who_am_i);

	return 0;
}

/* Updates imu.pitch_deg / imu.roll_deg from the smoothed gravity vector. Roll
 * rotates about X so it resolves the full circle; pitch is bounded to ±90
 * because beyond that it aliases with roll.
 */
static void imu_update_orientation(const float grav_mg[3])
{
	float horiz = sqrtf(grav_mg[1] * grav_mg[1] + grav_mg[2] * grav_mg[2]);

	imu.roll_deg = atan2f(grav_mg[1], grav_mg[2]) * IMU_RAD_TO_DEG;
	imu.pitch_deg = atan2f(-grav_mg[0], horiz) * IMU_RAD_TO_DEG;
}

/* Updates imu.accel_dev_mg / imu.gyro_mag_dps / imu.moving. Deviation is taken
 * from the instantaneous magnitude rather than per-axis deltas, which makes it
 * blind to static orientation: hold the board at any fixed angle and it reads
 * ~0.
 */
static void imu_update_motion(const float acc_mg[3], const float gyr_dps[3])
{
	static uint32_t last_trigger_ms;
	uint32_t now = k_uptime_get_32();
	float acc_norm_mg = sqrtf(acc_mg[0] * acc_mg[0] + acc_mg[1] * acc_mg[1] +
				  acc_mg[2] * acc_mg[2]);

	imu.accel_dev_mg = fabsf(acc_norm_mg - IMU_ONE_G_MG);
	imu.gyro_mag_dps = sqrtf(gyr_dps[0] * gyr_dps[0] + gyr_dps[1] * gyr_dps[1] +
				 gyr_dps[2] * gyr_dps[2]);

	if (imu.accel_dev_mg > IMU_MOTION_ACC_ENTER_MG ||
	    imu.gyro_mag_dps > IMU_MOTION_GYR_ENTER_DPS) {
		last_trigger_ms = now;
		imu.moving = true;
	} else if (imu.moving && imu.accel_dev_mg < IMU_MOTION_ACC_EXIT_MG &&
		   imu.gyro_mag_dps < IMU_MOTION_GYR_EXIT_DPS &&
		   (now - last_trigger_ms) > IMU_MOTION_HOLD_MS) {
		imu.moving = false;
	}
}

static void imu_thread(void)
{
	const struct device *i2c_dev = sensor_bus_get("imu");
	float grav_mg[3] = {0.0f, 0.0f, 0.0f};
	bool grav_primed = false;
	uint32_t log_div = 0;

	if (i2c_dev == NULL || lsm6dsv16x_configure(i2c_dev)) {
		return;
	}

	while (1) {
		uint8_t raw[IMU_RAW_BYTES];
		float acc_mg[3];
		float gyr_dps[3];
		int ret;

		/* 0x22..0x2D: gyro XYZ then accel XYZ, each int16 little-endian. */
		ret = i2c_burst_read(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_OUTX_L_G, raw,
				     sizeof(raw));
		if (ret) {
			LOG_WRN("IMU read error: %d", ret);
			k_msleep(IMU_POLL_MS);
			continue;
		}

		for (int i = 0; i < 3; i++) {
			imu.gyro[i] = (int16_t)sys_get_le16(&raw[i * 2]);
			imu.accel[i] = (int16_t)sys_get_le16(&raw[6 + i * 2]);

			acc_mg[i] = imu.accel[i] * IMU_ACCEL_MG_PER_LSB;
			gyr_dps[i] = imu.gyro[i] * IMU_GYRO_MDPS_PER_LSB * 0.001f;
		}

		/* Seed the filter with the first sample instead of ramping up
		 * from zero, which would report a bogus tilt for the first few
		 * hundred ms.
		 */
		if (!grav_primed) {
			memcpy(grav_mg, acc_mg, sizeof(grav_mg));
			grav_primed = true;
		} else {
			for (int i = 0; i < 3; i++) {
				grav_mg[i] += IMU_GRAV_ALPHA * (acc_mg[i] - grav_mg[i]);
			}
		}

		imu_update_orientation(grav_mg);
		imu_update_motion(acc_mg, gyr_dps);
		imu.valid = true;

#if IMU_RAW_LOG
		printk("I,%u,%d,%d,%d,%d,%d,%d\n", k_uptime_get_32(), imu.gyro[0], imu.gyro[1],
		       imu.gyro[2], imu.accel[0], imu.accel[1], imu.accel[2]);
#endif

#if IMU_ORIENT_LOG
		if (++log_div >= IMU_ORIENT_LOG_DIV) {
			log_div = 0;
			printk("O,%u,%.1f,%.1f,%.0f,%.1f,%d\n", k_uptime_get_32(),
			       (double)imu.pitch_deg, (double)imu.roll_deg,
			       (double)imu.accel_dev_mg, (double)imu.gyro_mag_dps,
			       imu.moving ? 1 : 0);
		}
#endif

		k_msleep(IMU_POLL_MS);
	}
}

K_THREAD_DEFINE(imu_thread_id, 2048, imu_thread, NULL, NULL, NULL, 12, 0, 0);

/* =========================================================================
 * 6. Entry point
 * =========================================================================
 */
int main(void)
{
	static struct broadcast_source_big broadcast_param;
	size_t ext_adv_buf_cnt = 0;
	size_t per_adv_buf_cnt = 0;
	int ret;

	LOG_DBG("Main started");

	/* Asserts PWR_EN, which brings up the sensor rails. The sensor threads
	 * are already running and waiting SENSOR_RAIL_SETTLE_MS for this.
	 */
	ret = nrf5340_audio_dk_init();
	ERR_CHK(ret);

	ret = fw_info_app_print();
	ERR_CHK(ret);

	ret = bt_mgmt_init();
	ERR_CHK(ret);

	ret = audio_system_init();
	ERR_CHK(ret);

	ret = zbus_subscribers_create();
	ERR_CHK_MSG(ret, "Failed to create zbus subscriber threads");

	ret = zbus_link_producers_observers();
	ERR_CHK_MSG(ret, "Failed to link zbus producers and observers");

	broadcast_source_default_create(&broadcast_param);

	/* Only one BIG supported at the moment */
	ret = broadcast_source_enable(&broadcast_param, 0);
	ERR_CHK_MSG(ret, "Failed to enable broadcaster(s)");

	ret = audio_system_config_set(
		bt_audio_codec_cfg_freq_to_freq_hz(CONFIG_BT_AUDIO_PREF_SAMPLE_RATE_VALUE),
		CONFIG_BT_AUDIO_BITRATE_BROADCAST_SRC, VALUE_NOT_SET);
	ERR_CHK_MSG(ret, "Failed to set sample- and bitrate");

	/* Get advertising set for BIG0 */
	ret = ext_adv_populate(0, &ext_adv_data[0], ext_adv_buf[0], ARRAY_SIZE(ext_adv_buf[0]),
			       &ext_adv_buf_cnt);
	ERR_CHK(ret);

	ret = per_adv_populate(0, &per_adv_data[0], &per_adv_buf[0], 1, &per_adv_buf_cnt);
	ERR_CHK(ret);

	/* Start broadcaster */
	ret = bt_mgmt_adv_start(0, ext_adv_buf[0], ext_adv_buf_cnt, &per_adv_buf[0],
				per_adv_buf_cnt, false);
	ERR_CHK_MSG(ret, "Failed to start first advertiser");

	LOG_INF("Broadcast source: %s started", CONFIG_BT_AUDIO_BROADCAST_NAME);

	ret = bt_le_adv_start(beacon_adv_param, beacon_ad, ARRAY_SIZE(beacon_ad), NULL, 0);
	if (ret) {
		LOG_ERR("Failed to start PPG beacon: %d", ret);
	} else {
		LOG_INF("PPG beacon advertising started");
	}

	return 0;
}
