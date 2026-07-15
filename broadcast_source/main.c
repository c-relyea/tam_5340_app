/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "streamctrl.h"

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/i2c.h>

#include "broadcast_source.h"
#include "zbus_common.h"
#include "nrf5340_audio_dk.h"
#include "led.h"
#include "button_assignments.h"
#include "macros_common.h"
#include "audio_system.h"
#include "bt_mgmt.h"
#include "fw_info_app.h"

#include <zephyr/logging/log.h>
#include <string.h>

#define SENSOR_ADDR 0x57

/* BQ27427 fuel gauge — I2C addr 0x55, standard command set */
#define BQ27427_ADDR     0x55
#define BQ27427_REG_SOC  0x1C  /* State of Charge, 2-byte LE, units: %  */
#define BQ27427_REG_VOLT 0x04  /* Voltage,          2-byte LE, units: mV */

/* LSM6DSV16X IMU — I2C addr 0x6A */
#define LSM6DSV16X_ADDR         0x6A
#define LSM6DSV16X_REG_WHO_AM_I 0x0F  /* Should read 0x71                       */
#define LSM6DSV16X_REG_CTRL1    0x10  /* Accel: op_mode_xl[6:4] odr_xl[3:0]     */
#define LSM6DSV16X_REG_CTRL2    0x11  /* Gyro:  op_mode_g[6:4]  odr_g[3:0]      */
#define LSM6DSV16X_REG_CTRL3    0x12  /* boot[7] bdu[6] if_inc[2] sw_reset[0]   */
#define LSM6DSV16X_REG_OUTX_L_G 0x22  /* Gyro XYZ then Accel XYZ — 12 bytes    */
#define LSM6DSV16X_WHOAMI       0x71



LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

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

/* -------------------------------------------------------------------------
 * PPG beacon — connectionless broadcast of raw PPG samples
 *
 * A GATT connection never held up at usable range (the receiver's BLE
 * radio couldn't keep it alive at distance). Raw samples are broadcast
 * instead, riding in the AD payload of a dedicated non-connectable
 * advertiser that any passive scanner in range can read — no connection,
 * subscription, or supervision timeout involved.
 *
 * A legacy (non-extended) PDU is used deliberately: BT_LE_ADV_PARAM below
 * omits BT_LE_ADV_OPT_EXT_ADV, so any BLE 4.x-era scanner can receive it.
 * This is not just for broad compatibility — the Raspberry Pi 5 receiver's
 * BlueZ/controller stack does legacy-only scanning (verified with btmon:
 * 100% "LE Advertising Report", zero "LE Extended Advertising Report", even
 * with bluetoothd Experimental mode on), so an extended-advertising beacon
 * is simply invisible to it. Legacy caps the payload at 31 bytes, so each
 * beacon update carries 2 samples; broadcasting at the fastest legal
 * non-directed interval (20 ms) targets 100 Hz (≈65-70 Hz received in
 * practice, the rest lost to radio contention with the BIS audio broadcast).
 *
 * Packet layout (manufacturer-specific AD, little-endian):
 *   [0..1]   company_id  CONFIG_BT_DEVICE_MANUFACTURER_ID — the AD type's
 *                         "Manufacturer Specific Data" format requires this
 *                         as the first 2 bytes; BT_DATA_MANUFACTURER_DATA
 *                         does not add it automatically
 *   [2]      magic       'T' (0x54) — cheap sanity check against other
 *                         nearby Nordic boards sharing the same placeholder
 *                         company ID
 *   [3]      seq         increments per beacon update (~50/s) — drop detection
 *   [4]      bat_pct     battery %  (0xFF = not yet read)
 *   [5..6]   sample_idx  running raw-sample counter (u16, wraps ~every 655 s)
 *   [7..15]  sample A: { ir u24, red u24, green u24 }
 *   [16..24] sample B: { ir u24, red u24, green u24 }
 * -------------------------------------------------------------------------
 */
static volatile uint8_t sens_bat = 0xFF;

#define TAM_BEACON_MAGIC        'T'
#define TAM_BEACON_PAYLOAD_LEN  23
#define TAM_BEACON_MFG_LEN      (2 + TAM_BEACON_PAYLOAD_LEN) /* company_id + payload */
#define TAM_BEACON_ADV_INTERVAL 0x0020 /* 20 ms in 0.625 ms units — BLE's spec minimum */

static uint8_t beacon_payload[TAM_BEACON_MFG_LEN];

static const struct bt_data beacon_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, beacon_payload, sizeof(beacon_payload)),
};

static const struct bt_le_adv_param *beacon_adv_param =
	BT_LE_ADV_PARAM(0, TAM_BEACON_ADV_INTERVAL, TAM_BEACON_ADV_INTERVAL, NULL);

static void sensor_notify_bat(uint8_t pct)
{
	sens_bat = pct;
}

/* -------------------------------------------------------------------------
 * PPG streaming pipeline
 *
 * The sampling thread (polling_thread_spo2) must never block or fault, or
 * it stops draining the sensor FIFO and stops the RTT "P," ground-truth
 * log.  bt_le_adv_update_data() runs the full HCI advertising-data-update
 * exchange and can take longer than a sample period, especially with the
 * concurrent BIS audio broadcast on the same radio, so it is NOT called
 * from the sampling thread.  Instead the sampler drops each raw sample into
 * a queue (non-blocking, drop-on-full) and a dedicated thread batches 2
 * samples per beacon update and pushes them out.
 * -------------------------------------------------------------------------
 */
struct ppg_sample {
	uint32_t ir;
	uint32_t red;
	uint32_t green;
};

/* 64 samples ≈ 0.64 s of buffering at 100 Hz — absorbs BIS-induced gaps */
K_MSGQ_DEFINE(ppg_msgq, sizeof(struct ppg_sample), 64, 4);
static uint32_t ppg_stream_drops; /* samples dropped because queue was full */

/* Enqueue one raw sample for the beacon.  Non-blocking: if the queue is
 * full (radio busy with the BIS broadcast) the sample is dropped rather
 * than stalling the sampler.  The beacon broadcasts unconditionally, so
 * unlike a GATT notify there is no subscriber state to gate on.
 */
static void ppg_stream_put(uint32_t ir, uint32_t red, uint32_t green)
{
	struct ppg_sample s = { ir, red, green };

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
	uint8_t  seq = 0;
	uint16_t sample_idx = 0;
	uint32_t last_warn_ms = 0;

	sys_put_le16(CONFIG_BT_DEVICE_MANUFACTURER_ID, &beacon_payload[0]);

	while (1) {
		struct ppg_sample a, b;

		if (k_msgq_get(&ppg_msgq, &a, K_FOREVER) != 0) {
			continue;
		}
		if (k_msgq_get(&ppg_msgq, &b, K_FOREVER) != 0) {
			continue;
		}

		beacon_payload[2] = TAM_BEACON_MAGIC;
		beacon_payload[3] = seq++;
		beacon_payload[4] = sens_bat;
		sys_put_le16(sample_idx, &beacon_payload[5]);
		ppg_pack_sample(&beacon_payload[7], &a);
		ppg_pack_sample(&beacon_payload[16], &b);
		sample_idx += 2;

		int ret = bt_le_adv_update_data(beacon_ad, ARRAY_SIZE(beacon_ad), NULL, 0);

		if (ret && k_uptime_get_32() - last_warn_ms > 1000) {
			last_warn_ms = k_uptime_get_32();
			LOG_WRN("Beacon adv update failed: %d (drops=%u)", ret, ppg_stream_drops);
		}
	}
}

K_THREAD_DEFINE(ppg_stream_id, 2048, ppg_stream_thread, NULL, NULL, NULL, 10, 0, 0);

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


/* -------------------------------------------------------------------------
 * IMU thread — LSM6DSV16X raw I2C, accel + gyro at 50 Hz
 *
 * Delays 2 s at startup so PWR_EN rails have time to stabilize and
 * main() has time to run before we touch the I2C bus.
 *
 * Raw values:
 *   Accel sensitivity  0.061 mg/LSB  (±2 g range)
 *   Gyro  sensitivity  4.375 mdps/LSB (±125 dps range)
 * -------------------------------------------------------------------------
 */
static void polling_thread_imu(void)
{
	/* Let power rails stabilize and main() run first */
	k_msleep(2000);

	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	if (!device_is_ready(i2c_dev)) {
		printk("IMU: I2C not ready\n");
		return;
	}

	/* Verify WHO_AM_I */
	uint8_t who_am_i = 0;
	int ret = i2c_reg_read_byte(i2c_dev, LSM6DSV16X_ADDR,
				    LSM6DSV16X_REG_WHO_AM_I, &who_am_i);

	if (ret || who_am_i != LSM6DSV16X_WHOAMI) {
		printk("IMU not found (WHO_AM_I=0x%02X, err=%d)\n", who_am_i, ret);
		return;
	}

	/* CTRL3: BDU=1 (bit6), IF_INC=1 (bit2) — required for coherent burst reads */
	i2c_reg_write_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_CTRL3, 0x44);
	/* Accel: HP mode (op_mode=000), 60 Hz (odr=0101) → bits[6:4]=000 bits[3:0]=0101 → 0x05 */
	i2c_reg_write_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_CTRL1, 0x05);
	/* Gyro:  HP mode (op_mode=000), 60 Hz (odr=0101) → 0x05 */
	i2c_reg_write_byte(i2c_dev, LSM6DSV16X_ADDR, LSM6DSV16X_REG_CTRL2, 0x05);
	/* Wait one ODR period for first sample to be ready (~17 ms at 60 Hz) */
	k_msleep(20);

	printk("IMU ready (WHO_AM_I=0x%02X)\n", who_am_i);

	uint8_t raw[12];

	while (1) {
		/* Burst-read gyro XYZ + accel XYZ (12 bytes from 0x22) */
		ret = i2c_burst_read(i2c_dev, LSM6DSV16X_ADDR,
				     LSM6DSV16X_REG_OUTX_L_G, raw, sizeof(raw));
		if (ret) {
			printk("IMU read error: %d\n", ret);
			k_msleep(40);
			continue;
		}

		/* IMU data available in raw[0..11] — reserved for future
		 * motion-artifact correction in the on-chip PPG pipeline.
		 * Raw values no longer transmitted over NUS. */
		k_msleep(40); /* 25 Hz */
	}
}

K_THREAD_DEFINE(imu_thread_id, 1536, polling_thread_imu, NULL, NULL, NULL, 12, 0, 0);

/* -------------------------------------------------------------------------
 * Battery thread — BQ27427 state-of-charge every 10 s
 * -------------------------------------------------------------------------
 */
static void polling_thread_battery(void)
{
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	if (!device_is_ready(i2c_dev)) {
		printk("Battery gauge I2C not ready!\n");
		return;
	}
	printk("Battery gauge ready!\n");

	while (1) {
		uint8_t buf[2];
		int ret;

		ret = i2c_burst_read(i2c_dev, BQ27427_ADDR, BQ27427_REG_SOC, buf, 2);
		if (ret) {
			printk("Battery SOC read error: %d\n", ret);
		} else {
			uint16_t soc = buf[0] | ((uint16_t)buf[1] << 8); /* little-endian */

			printk("Battery: %d%%\n", soc);
			sensor_notify_bat((uint8_t)soc);
		}

		k_msleep(10000); /* every 10 s — SOC changes slowly */
	}
}

K_THREAD_DEFINE(battery_thread_id, 1024, polling_thread_battery, NULL, NULL, NULL, 13, 0, 0);

/* -------------------------------------------------------------------------
 * On-chip HR / SpO2 computation
 *
 * HR:   Peak detection on DC-removed IR signal.
 *       Rolling mean of last 8 RR intervals → BPM.
 *       Emits  "H:nn\n"     every PPG_FS samples (≈ 1 s).
 *
 * SpO2: Ratio-of-ratios over a 5 s sliding window.
 *       R = (AC_red / DC_red) / (AC_ir / DC_ir)
 *       SpO2 ≈ 110 − 25 × R   (empirical linear fit, ±3 %)
 *       Emits  "O:nn.n\n"   every PPG_O2_PERIOD samples (≈ 5 s).
 *
 * Raw samples arrive at PPG_RAW_FS (100 Hz FIFO output); the HR/SpO2
 * window math below is tuned for 25 Hz, so the polling thread decimates
 * by PPG_RAW_FS/PPG_FS before calling ppg_add().
 * All state is static (BSS), not on stack.
 * -------------------------------------------------------------------------
 */
#define PPG_RAW_FS    100              /* FIFO output rate (200 Hz SR, avg 2) */
#define PPG_FS         25              /* algorithm Hz            */
#define PPG_WIN       200              /* 8 s sliding window      */
#define PPG_HR_PERIOD  PPG_FS          /* emit H: every 1 s       */
#define PPG_O2_PERIOD  PPG_FS          /* emit O: every 1 s       */

/* Autocorrelation lag range: 8..38 ticks @ 25 Hz = 187..39 bpm */
#define PPG_LAG_MIN     8
#define PPG_LAG_MAX    38
/* Minimum normalized autocorrelation (×1024) to accept an HR lock.
 * Neck recordings with good contact score 700-900; noise scores < 200.
 */
#define PPG_MIN_QUAL  300
/* Minimum DC level (counts) on IR and red to consider skin contact.
 * Air reads < 1500; skin contact reads > 50000.
 */
#define PPG_MIN_DC  20000

static int32_t  ppg_ir_win[PPG_WIN];
static int32_t  ppg_red_win[PPG_WIN];
static int32_t  ppg_grn_win[PPG_WIN];
static uint16_t ppg_win_head;
static uint16_t ppg_win_n;

/* Scratch (static: keep off the 2 KB thread stack) */
static int32_t ppg_lin[PPG_WIN];
static int32_t ppg_det[PPG_WIN];
static int64_t ppg_ps[PPG_WIN + 1];

/* Quality (×1024) of the most recent HR lock; SpO2 requires a locked HR */
static int32_t ppg_last_qual;

static void ppg_add(uint32_t ir_raw, uint32_t red_raw, uint32_t grn_raw)
{
	ppg_ir_win[ppg_win_head]  = (int32_t)ir_raw;
	ppg_red_win[ppg_win_head] = (int32_t)red_raw;
	ppg_grn_win[ppg_win_head] = (int32_t)grn_raw;
	ppg_win_head = (ppg_win_head + 1) % PPG_WIN;
	if (ppg_win_n < PPG_WIN) {
		ppg_win_n++;
	}
}

/* Copy a circular window into ppg_lin[] in time order, then remove the
 * baseline with a centered 1 s moving average: ppg_det[] = signal - trend.
 * Also returns the window mean (DC level).
 */
static int32_t ppg_detrend(const int32_t *win)
{
	const int half = PPG_FS / 2;

	for (int i = 0; i < PPG_WIN; i++) {
		ppg_lin[i] = win[(ppg_win_head + i) % PPG_WIN];
	}

	ppg_ps[0] = 0;
	for (int i = 0; i < PPG_WIN; i++) {
		ppg_ps[i + 1] = ppg_ps[i] + ppg_lin[i];
	}

	for (int i = 0; i < PPG_WIN; i++) {
		int lo = MAX(0, i - half);
		int hi = MIN(PPG_WIN - 1, i + half);
		int32_t ma = (int32_t)((ppg_ps[hi + 1] - ppg_ps[lo]) / (hi - lo + 1));

		ppg_det[i] = ppg_lin[i] - ma;
	}

	return (int32_t)(ppg_ps[PPG_WIN] / PPG_WIN);
}

static uint32_t ppg_isqrt64(uint64_t v)
{
	uint64_t r = 0;
	uint64_t bit = 1ULL << 62;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit) {
		if (v >= r + bit) {
			v -= r + bit;
			r = (r >> 1) + bit;
		} else {
			r >>= 1;
		}
		bit >>= 2;
	}
	return (uint32_t)r;
}

/* Heart rate from autocorrelation of the detrended GREEN channel.
 * Green has 5-10x the relative pulsation of IR/red in reflective
 * (neck/wrist) placement, so it is the HR channel of choice.
 * Returns bpm ×10, or 0 if no confident lock.  Updates ppg_last_qual.
 */
static int ppg_hr_bpm_x10(void)
{
	ppg_last_qual = 0;

	if (ppg_win_n < PPG_WIN) {
		return 0;
	}

	(void)ppg_detrend(ppg_grn_win);

	int64_t energy = 0;

	for (int i = 0; i < PPG_WIN; i++) {
		energy += (int64_t)ppg_det[i] * ppg_det[i];
	}
	if (energy == 0) {
		return 0;
	}

	int32_t r_at[PPG_LAG_MAX + 2] = { 0 };
	int best_lag = 0;
	int32_t best_r = 0;

	for (int lag = PPG_LAG_MIN; lag <= PPG_LAG_MAX; lag++) {
		int64_t s = 0;

		for (int i = 0; i < PPG_WIN - lag; i++) {
			s += (int64_t)ppg_det[i] * ppg_det[i + lag];
		}
		r_at[lag] = (int32_t)((1024 * s) / energy);
		if (r_at[lag] > best_r) {
			best_r = r_at[lag];
			best_lag = lag;
		}
	}

	if (best_r < PPG_MIN_QUAL) {
		return 0;
	}

	/* Harmonic guard: if the half-lag (double rate) correlates almost as
	 * well, the true beat is the faster one.
	 */
	int L = best_lag;

	if (L / 2 >= PPG_LAG_MIN && 10 * r_at[L / 2] >= 7 * best_r) {
		L = L / 2;
		best_r = r_at[L];
	}

	/* Parabolic interpolation around the peak, Q8 sub-lag resolution */
	int32_t r1 = (L - 1 >= PPG_LAG_MIN) ? r_at[L - 1] : 0;
	int32_t r2 = r_at[L];
	int32_t r3 = (L + 1 <= PPG_LAG_MAX) ? r_at[L + 1] : 0;
	int32_t den = r1 - 2 * r2 + r3;
	int32_t delta_q8 = den ? (128 * (r1 - r3)) / (2 * den) : 0;

	delta_q8 = CLAMP(delta_q8, -128, 128);

	ppg_last_qual = best_r;

	/* bpm ×10 = 60*FS*10 / (L + delta/256) */
	return (int)((15000 * 2560) / (L * 256 + delta_q8) / 10);
}

/* SpO2 from RMS ratio-of-ratios over the same 8 s window.
 * R = (ACrms_red/DC_red) / (ACrms_ir/DC_ir);  SpO2 = 110 - 25R.
 * Only emitted with skin contact (DC floor), measurable IR perfusion,
 * and a currently locked HR (cardiac signal present).
 * Returns SpO2 ×10 (700..1000) or 0.
 */
static int ppg_spo2_x10(void)
{
	if (ppg_win_n < PPG_WIN || ppg_last_qual < PPG_MIN_QUAL) {
		return 0;
	}

	int32_t dc_ir = ppg_detrend(ppg_ir_win);
	int64_t e_ir = 0;

	for (int i = 0; i < PPG_WIN; i++) {
		e_ir += (int64_t)ppg_det[i] * ppg_det[i];
	}

	int32_t dc_red = ppg_detrend(ppg_red_win);
	int64_t e_red = 0;

	for (int i = 0; i < PPG_WIN; i++) {
		e_red += (int64_t)ppg_det[i] * ppg_det[i];
	}

	int32_t ac_ir  = (int32_t)ppg_isqrt64((uint64_t)(e_ir / PPG_WIN));
	int32_t ac_red = (int32_t)ppg_isqrt64((uint64_t)(e_red / PPG_WIN));

	if (dc_ir < PPG_MIN_DC || dc_red < PPG_MIN_DC || ac_ir == 0) {
		return 0;
	}

	/* IR perfusion index ×10000; require >= 0.03% */
	int32_t pi_ir = (int32_t)((10000LL * ac_ir) / dc_ir);

	if (pi_ir < 3) {
		return 0;
	}

	int32_t R1000 = (int32_t)(((int64_t)ac_red * dc_ir * 1000) /
				  ((int64_t)dc_red * ac_ir));
	int32_t sp10 = 1100 - (25 * R1000) / 100;

	return (sp10 >= 700 && sp10 <= 1000) ? sp10 : 0;
}

/* -------------------------------------------------------------------------
 * SpO2 / PPG thread — MAX30101 FIFO drained at 100 Hz
 * -------------------------------------------------------------------------
 */

/* Set to 1 to stream raw PPG samples over RTT as CSV lines:
 *   P,<uptime_ms>,<ir>,<red>,<green>
 * Capture with log_ppg.sh (or any RTT logger) and filter lines starting
 * with "P," into a .csv.  ~100 lines/s, modest RTT bandwidth.
 */
#define PPG_RAW_LOG 1

/* Configure the MAX30101 once it responds on the bus.  Must run after
 * nrf5340_audio_dk_init() has asserted PWR_EN — register writes to the
 * unpowered chip fail silently and it boots into shutdown mode (no
 * samples, FIFO never fills).  Waits for PART_ID, configures, then
 * verifies the mode register took.
 */
static int max30101_configure(const struct device *i2c_dev)
{
	int ret;
	uint8_t id = 0;

	/* Wait for the sensor to come up on the rails (PART_ID = 0x15) */
	for (int tries = 0; tries < 50; tries++) {
		ret = i2c_reg_read_byte(i2c_dev, SENSOR_ADDR, 0xFF, &id);
		if (ret == 0 && id == 0x15) {
			break;
		}
		k_msleep(100);
	}

	if (id != 0x15) {
		printk("MAX30101 not responding (PART_ID=0x%02X)\n", id);
		return -ENODEV;
	}

	ret = 0;

	// FIFO config
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x08, 0x2F); // avg=2

	// SpO2 config: gain, sample rate 200 Hz, pulse width 411 µs.
	// 200 Hz is the chip's max with 3 active LED slots at 411 µs —
	// programming 400 Hz gets internally clipped to 200 (measured as
	// ~50 Hz FIFO output with avg=4).  200 Hz / avg=2 → true 100 Hz.
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x0A, 0x4B);

	/* LED currents: each LSB = 0.2 mA.  Tuned for NECK (reflective)
	 * placement from recorded data:
	 * RED   0x60 = 19.2 mA  (neck DC was ~101k/262k at 16 mA — headroom)
	 * IR    0x60 = 19.2 mA  (neck DC was ~81k/262k at 9.6 mA; SpO2 needs
	 *                        every bit of IR/red AC SNR at 0.1-0.2 % PI)
	 * GREEN 0x50+0x50 = 32 mA total: green is the HR channel on the neck
	 *   (5-10x the relative pulsation of IR/red) but its DC was only
	 *   ~4.7k/262k at 9.6 mA.  In multi-LED mode with SLOTx=011 the green
	 *   LED sinks current from BOTH LED3_PA and LED4_PA (datasheet
	 *   Table 9 note), so both are set.
	 * Watch for clipping near 262143 counts if contact/coupling improves. */
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x0C, 0x60); // RED   19.2 mA
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x0D, 0x60); // IR    19.2 mA
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x0E, 0x50); // GREEN 16.0 mA (DAC 1)
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x0F, 0x50); // GREEN 16.0 mA (DAC 2)

	// Multi-LED slots
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x11, 0x21); // slot1=RED, slot2=IR
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x12, 0x03); // slot3=GREEN

	// Clear FIFO
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x04, 0x00);
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x05, 0x00);
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x06, 0x00);

	// Enable multi-LED mode (RED+IR+GREEN)
	ret |= i2c_reg_write_byte(i2c_dev, SENSOR_ADDR, 0x09, 0x07);

	if (ret) {
		printk("MAX30101 config write failed\n");
		return -EIO;
	}

	uint8_t mode = 0;

	ret = i2c_reg_read_byte(i2c_dev, SENSOR_ADDR, 0x09, &mode);
	if (ret || mode != 0x07) {
		printk("MAX30101 mode verify failed (mode=0x%02X, err=%d)\n", mode, ret);
		return -EIO;
	}

	printk("Pulse oximeter configured!\n");

	return 0;
}

void polling_thread_spo2(void)
{
	k_msleep(2000);

	const struct device *pulse_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	if (!device_is_ready(pulse_dev)) {
		printk("SpO2 device not ready!\n");
		return;
	}

	if (max30101_configure(pulse_dev)) {
		return;
	}

	uint8_t  fifo[9];        /* 3 bytes each: red, ir, green */
	uint32_t sample_n = 0;   /* raw samples at PPG_RAW_FS */
	uint32_t dec_n = 0;      /* decimated samples fed to HR/SpO2 at PPG_FS */

	while (1) {
		/* FIFO_WR_PTR (0x04), OVF_COUNTER (0x05), FIFO_RD_PTR (0x06) */
		uint8_t ptrs[3];
		int ret = i2c_burst_read(pulse_dev, SENSOR_ADDR, 0x04, ptrs, 3);

		if (ret) {
			printk("FIFO ptr read error %d\n", ret);
			k_msleep(40);
			continue;
		}

		int avail = (ptrs[0] - ptrs[2]) & 0x1F;

		while (avail-- > 0) {
			ret = i2c_burst_read(pulse_dev, SENSOR_ADDR, 0x07, fifo, 9);

			if (ret) {
				printk("FIFO read error %d\n", ret);
				break;
			}

			uint32_t red   = ((fifo[0] << 16) | (fifo[1] << 8) | fifo[2]) & 0x3FFFF;
			uint32_t ir    = ((fifo[3] << 16) | (fifo[4] << 8) | fifo[5]) & 0x3FFFF;
			uint32_t green = ((fifo[6] << 16) | (fifo[7] << 8) | fifo[8]) & 0x3FFFF;

#if PPG_RAW_LOG
			printk("P,%u,%u,%u,%u\n", k_uptime_get_32(), ir, red, green);
#endif

			sample_n++;

			/* Hand off to the streaming thread — never blocks here */
			ppg_stream_put(ir, red, green);

			/* HR/SpO2 window math is tuned for PPG_FS (25 Hz) */
			if (sample_n % (PPG_RAW_FS / PPG_FS) != 0) {
				continue;
			}

			ppg_add(ir, red, green);
			dec_n++;

			if (dec_n % PPG_HR_PERIOD == 0) {
				int hr10 = ppg_hr_bpm_x10();

#if PPG_RAW_LOG
				printk("H,%u,%d,%d\n", k_uptime_get_32(), hr10,
				       ppg_last_qual);
#endif
				(void)hr10;
			}

			if (dec_n % PPG_O2_PERIOD == 0) {
				int sp10 = ppg_spo2_x10();

#if PPG_RAW_LOG
				printk("O,%u,%d\n", k_uptime_get_32(), sp10);
#endif
				(void)sp10;
			}
		}

		/* FIFO fills at 10 ms/sample; a 5 ms poll keeps the drain burst
		 * to 1-2 samples so CSV timestamps stay close to sample time.
		 */
		k_msleep(5);
	}
}

K_THREAD_DEFINE(spo2_thread_id, 2048, polling_thread_spo2,
		NULL, NULL, NULL, 12, 0, 0);


int main(void)
{
	/* The MAX30101 is configured in polling_thread_spo2 once PWR_EN
	 * rails are up — see max30101_configure().
	 */
	int ret;
	static struct broadcast_source_big broadcast_param;

	LOG_DBG("Main started");

	size_t ext_adv_buf_cnt = 0;
	size_t per_adv_buf_cnt = 0;

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
