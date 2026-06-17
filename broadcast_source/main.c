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
 * Non-connectable sensor beacon — HR / SpO2 / battery in manufacturer data
 *
 * ADV_NONCONN_IND at ~1 s interval.  No ACL connection = zero scheduling
 * conflict with BIS ISO events on the shared nRF5340 radio.
 *
 * Payload (company ID 0xFFFF, 4 data bytes):
 *   [0]  hr_bpm    (0 = not yet computed)
 *   [1]  spo2_int  integer part of SpO2 %  (0 = not yet computed)
 *   [2]  spo2_frac fractional part × 10    (e.g. 5 → .5 %)
 *   [3]  bat_pct   battery %               (0xFF = not yet read)
 * -------------------------------------------------------------------------
 */
#define SENSOR_COMPANY_ID  0xFFFF

/* Payload: 2-byte company ID + 5 data bytes.
 * [2] hr_bpm  [3] spo2_int  [4] spo2_frac  [5] bat_pct  [6] seq
 * seq increments on every advertising update so macOS Core Bluetooth's
 * duplicate-advertisement filter never suppresses the callback.
 */
static uint8_t sensor_mfr_data[7] = {
	(SENSOR_COMPANY_ID & 0xFF),
	(SENSOR_COMPANY_ID >> 8),
	0, 0, 0, 0xFF, 0,
};

static struct bt_data sensor_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, sensor_mfr_data, sizeof(sensor_mfr_data)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static volatile uint8_t sens_hr;
static volatile uint8_t sens_spo2_i;
static volatile uint8_t sens_spo2_f;
static volatile uint8_t sens_bat = 0xFF;

static void sensor_adv_work_fn(struct k_work *work);
K_WORK_DEFINE(sensor_adv_work, sensor_adv_work_fn);

static void sensor_adv_work_fn(struct k_work *work)
{
	sensor_mfr_data[2] = sens_hr;
	sensor_mfr_data[3] = sens_spo2_i;
	sensor_mfr_data[4] = sens_spo2_f;
	sensor_mfr_data[5] = sens_bat;
	sensor_mfr_data[6]++;  /* always-changing seq → bypasses macOS dup filter */
	int ret = bt_le_adv_update_data(sensor_ad, ARRAY_SIZE(sensor_ad), NULL, 0);

	if (ret && ret != -EAGAIN) {
		LOG_DBG("Sensor adv update: %d", ret);
	}
}

static void sensor_notify_hr(uint8_t hr)
{ sens_hr = hr; k_work_submit(&sensor_adv_work); }

static void sensor_notify_spo2(uint8_t spo2_i, uint8_t spo2_f)
{ sens_spo2_i = spo2_i; sens_spo2_f = spo2_f; k_work_submit(&sensor_adv_work); }

static void sensor_notify_bat(uint8_t pct)
{ sens_bat = pct; k_work_submit(&sensor_adv_work); }

/* Periodic timer: force an adv update every 2 s even when sensor values are
 * unchanged.  Without this, a stable signal produces identical packets and
 * macOS stops delivering BleakScanner callbacks after the first one.
 */
static void sensor_beacon_timer_fn(struct k_timer *timer)
{
	k_work_submit(&sensor_adv_work);
}

K_TIMER_DEFINE(sensor_beacon_timer, sensor_beacon_timer_fn, NULL);

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
 * Effective sample rate: 25 Hz (one FIFO read every 40 ms sleep).
 * All state is static (BSS), not on stack.
 * -------------------------------------------------------------------------
 */
#define PPG_FS         25              /* effective Hz            */
#define PPG_WIN       125              /* 5 s sliding window      */
#define PPG_RR_HIST     8              /* beat intervals to keep  */
#define PPG_HR_PERIOD  PPG_FS         /* emit H: every 1 s       */
#define PPG_O2_PERIOD  (5 * PPG_FS)   /* emit O: every 5 s       */

static int32_t  ppg_ir_win[PPG_WIN];
static int32_t  ppg_red_win[PPG_WIN];
static uint16_t ppg_win_head;
static uint16_t ppg_win_n;

static int32_t  ppg_dc_ir;
static int32_t  ppg_ac_prev;
static int32_t  ppg_ac_pprev;
static uint32_t ppg_tick;
static uint32_t ppg_beat_tick;
static uint32_t ppg_rr[PPG_RR_HIST];
static uint8_t  ppg_rr_wr;
static uint8_t  ppg_rr_n;

static int ppg_hr_bpm(void)
{
	if (ppg_rr_n < 2) {
		return 0;
	}
	uint32_t sum = 0;

	for (int i = 0; i < ppg_rr_n; i++) {
		sum += ppg_rr[i];
	}
	uint32_t avg = sum / ppg_rr_n;

	return avg ? (int)((60u * PPG_FS + avg / 2u) / avg) : 0;
}

/* Returns SpO2 × 10 (e.g. 985 → 98.5 %), or 0 if no valid signal. */
static int ppg_spo2_x10(void)
{
	if (ppg_win_n < PPG_WIN) {
		return 0;
	}

	int64_t sum_ir  = ppg_ir_win[0];
	int64_t sum_red = ppg_red_win[0];
	int32_t mn_ir   = ppg_ir_win[0],  mx_ir  = ppg_ir_win[0];
	int32_t mn_red  = ppg_red_win[0], mx_red = ppg_red_win[0];

	for (int i = 1; i < PPG_WIN; i++) {
		sum_ir  += ppg_ir_win[i];
		sum_red += ppg_red_win[i];
		if (ppg_ir_win[i]  < mn_ir)  mn_ir  = ppg_ir_win[i];
		if (ppg_ir_win[i]  > mx_ir)  mx_ir  = ppg_ir_win[i];
		if (ppg_red_win[i] < mn_red) mn_red = ppg_red_win[i];
		if (ppg_red_win[i] > mx_red) mx_red = ppg_red_win[i];
	}

	int32_t dc_ir  = (int32_t)(sum_ir  / PPG_WIN);
	int32_t dc_red = (int32_t)(sum_red / PPG_WIN);
	int32_t ac_ir  = mx_ir  - mn_ir;
	int32_t ac_red = mx_red - mn_red;

	/* Require ≥ 0.5% AC swing as proxy for finger contact */
	if (dc_ir == 0 || dc_red == 0 || ac_ir < dc_ir / 200) {
		return 0;
	}

	/* R = (AC_red / DC_red) / (AC_ir / DC_ir) — integer × 1000 */
	int32_t R = (int32_t)(((int64_t)ac_red * dc_ir * 1000) /
			       ((int64_t)dc_red * ac_ir));

	/* SpO2% × 10:  SpO2 ≈ 110 − 25 × R */
	int32_t sp10 = 1100 - (25 * R) / 100;

	return (sp10 >= 700 && sp10 <= 1000) ? sp10 : 0;
}

static void ppg_add(uint32_t ir_raw, uint32_t red_raw)
{
	int32_t ir  = (int32_t)ir_raw;
	int32_t red = (int32_t)red_raw;

	ppg_ir_win[ppg_win_head]  = ir;
	ppg_red_win[ppg_win_head] = red;
	ppg_win_head = (ppg_win_head + 1) % PPG_WIN;
	if (ppg_win_n < PPG_WIN) {
		ppg_win_n++;
	}

	ppg_tick++;

	/* DC removal: EMA alpha ≈ 1/64; seed from first sample */
	if (ppg_tick == 1) {
		ppg_dc_ir = ir;
	} else {
		ppg_dc_ir += (ir - ppg_dc_ir) >> 6;
	}
	int32_t ac = ir - ppg_dc_ir;

	/* Peak: local maximum above ~0.8% of DC, after 3 s warm-up.
	 * Detects when ac[t-2] < ac[t-1] > ac[t] and ac[t-1] > threshold.
	 */
	if (ppg_ac_pprev < ppg_ac_prev &&
	    ppg_ac_prev  >  ac         &&
	    ppg_ac_prev  > (ppg_dc_ir >> 7) &&
	    ppg_tick     > (3u * PPG_FS)) {

		if (ppg_beat_tick > 0) {
			uint32_t rr = (ppg_tick - 1u) - ppg_beat_tick;
			/* Accept 18–180 BPM at 25 Hz: 8–83 ticks */
			if (rr >= 8 && rr <= 83) {
				ppg_rr[ppg_rr_wr % PPG_RR_HIST] = rr;
				ppg_rr_wr++;
				if (ppg_rr_n < PPG_RR_HIST) {
					ppg_rr_n++;
				}
			}
		}
		ppg_beat_tick = ppg_tick - 1u;
	}

	ppg_ac_pprev = ppg_ac_prev;
	ppg_ac_prev  = ac;
}

/* -------------------------------------------------------------------------
 * SpO2 / PPG thread — MAX30101 FIFO at 25 Hz effective rate
 * -------------------------------------------------------------------------
 */
void polling_thread_spo2(void)
{
	k_msleep(2000);

	const struct device *pulse_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	if (!device_is_ready(pulse_dev)) {
		printk("SpO2 device not ready!\n");
		return;
	}

	uint8_t  fifo[9]; /* 3 bytes each: red, ir, green */
	uint32_t sample_n = 0;

	while (1) {
		int ret = i2c_burst_read(pulse_dev, SENSOR_ADDR, 0x07, fifo, 9);

		if (ret) {
			printk("FIFO read error %d\n", ret);
			k_msleep(40);
			continue;
		}

		uint32_t red = ((fifo[0] << 16) | (fifo[1] << 8) | fifo[2]) & 0x3FFFF;
		uint32_t ir  = ((fifo[3] << 16) | (fifo[4] << 8) | fifo[5]) & 0x3FFFF;

		ppg_add(ir, red);
		sample_n++;

		if (sample_n % PPG_HR_PERIOD == 0) {
			int hr = ppg_hr_bpm();

			if (hr > 0) {
				sensor_notify_hr((uint8_t)hr);
			}
		}

		if (sample_n % PPG_O2_PERIOD == 0) {
			int sp10 = ppg_spo2_x10();

			if (sp10 > 0) {
				sensor_notify_spo2((uint8_t)(sp10 / 10),
						   (uint8_t)(sp10 % 10));
			}
		}

		k_msleep(40);
	}
}

K_THREAD_DEFINE(spo2_thread_id, 2048, polling_thread_spo2,
		NULL, NULL, NULL, 12, 0, 0);


int main(void)



{
	const struct device *pulse_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(pulse_dev)) {
            printk("Pulse oximeter not ready!\n");
            return;         
    } else {
            printk("Pulse oximeter ready!\n");
    }
        printk("Configuring pulse oximeter...\n");

        // FIFO config
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x08, 0x4F); // avg=4, rollover

        // SpO2 config: gain, sample rate 100Hz, pulse width 411 µs
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x0A, 0x4F);

        /* LED currents: each LSB = 0.2 mA.
         * RED   0x50 = 80  × 0.2 mA = 16.0 mA  (unchanged — good mid-range level)
         * IR    0x30 = 48  × 0.2 mA =  9.6 mA  (was 0x80 = 25.6 mA — was saturating ADC)
         * GREEN 0x30 = 48  × 0.2 mA =  9.6 mA  (was 0x80 = 25.6 mA)
         * Target: all channels at 40–70 % of 18-bit ADC full scale with finger contact.
         * Tune 0x0C/0x0D/0x0E up or down if any channel still clips or is too dim. */
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x0C, 0x50); // RED   16.0 mA
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x0D, 0x30); // IR     9.6 mA
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x0E, 0x30); // GREEN  9.6 mA

        // *** MULTI-LED SLOTS (THIS WAS MISSING!) ***
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x11, 0x21); // slot1=RED, slot2=IR
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x12, 0x03); // slot3=GREEN

        // Clear FIFO
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x04, 0x00);
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x05, 0x00);
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x06, 0x00);

        // Enable multi-LED mode (RED+IR+GREEN)
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x09, 0x07);


    printk("Pulse oximeter configured!\n");

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

	k_timer_start(&sensor_beacon_timer, K_MSEC(2000), K_MSEC(2000));

	static const struct bt_le_adv_param sensor_adv_param = {
		.id                 = BT_ID_DEFAULT,
		.sid                = 0U,
		.secondary_max_skip = 0U,
		.options            = BT_LE_ADV_OPT_NONE,
		.interval_min       = 0x0640U, /* 1000 ms */
		.interval_max       = 0x0800U, /* 1280 ms */
		.peer               = NULL,
	};
	int adv_ret = bt_le_adv_start(&sensor_adv_param,
				       sensor_ad, ARRAY_SIZE(sensor_ad), NULL, 0);

	if (adv_ret && adv_ret != -EALREADY) {
		LOG_WRN("Sensor adv start failed: %d", adv_ret);
	} else {
		LOG_INF("Sensor beacon started (ADV_NONCONN_IND, 1 s) as %s",
			CONFIG_BT_DEVICE_NAME);
	}

	return 0;
}
