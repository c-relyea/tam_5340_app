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
#include <zephyr/drivers/sensor.h>


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

#define SENSOR_ADDR 0x57

/* BQ27427 fuel gauge — I2C addr 0x55, standard command set */
#define BQ27427_ADDR     0x55
#define BQ27427_REG_SOC  0x1C  /* State of Charge, 2-byte LE, units: %  */
#define BQ27427_REG_VOLT 0x04  /* Voltage,          2-byte LE, units: mV */



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
				if (strm_state != STATE_STREAMING) {
					LOG_WRN("Not in streaming state");
					break;
				}

				ret = audio_system_encode_test_tone_step();
				if (ret) {
					LOG_WRN("Failed to play test tone, ret: %d", ret);
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


/* -------------------------------------------------------------------------
 * IMU thread — LSM6DSV16X accelerometer + gyroscope at 50 Hz
 * -------------------------------------------------------------------------
 */
static void polling_thread_imu(void)
{
	const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(imu));

	if (!device_is_ready(imu_dev)) {
		printk("IMU not ready!\n");
		return;
	}
	printk("IMU ready!\n");

	struct sensor_value accel[3], gyro[3];

	while (1) {
		int ret = sensor_sample_fetch(imu_dev);

		if (ret) {
			printk("IMU fetch error: %d\n", ret);
			k_msleep(20);
			continue;
		}

		sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);

		/* val1 = integer part, val2 = fractional part in millionths */
		printk("Accel [m/s2]: X=%d.%06d Y=%d.%06d Z=%d.%06d\n",
		       accel[0].val1, abs(accel[0].val2),
		       accel[1].val1, abs(accel[1].val2),
		       accel[2].val1, abs(accel[2].val2));
		printk("Gyro [rad/s]: X=%d.%06d Y=%d.%06d Z=%d.%06d\n",
		       gyro[0].val1, abs(gyro[0].val2),
		       gyro[1].val1, abs(gyro[1].val2),
		       gyro[2].val1, abs(gyro[2].val2));

		k_msleep(20); /* 50 Hz */
	}
}

K_THREAD_DEFINE(imu_thread_id, 2048, polling_thread_imu, NULL, NULL, NULL, 5, 0, 0);

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
		}

		k_msleep(10000); /* every 10 s — SOC changes slowly */
	}
}

K_THREAD_DEFINE(battery_thread_id, 1024, polling_thread_battery, NULL, NULL, NULL, 6, 0, 0);

/* -------------------------------------------------------------------------
 * SpO2 / PPG thread — MAX30101 raw FIFO at ~100 Hz
 * -------------------------------------------------------------------------
 */
void polling_thread_spo2(void)
{
    const struct device *pulse_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(pulse_dev)) {
        printk("SpO2 device not ready!\n");
        return;
    }

    uint8_t fifo[9];   // 9 bytes per sample

    while (1) {
        /* READ A FULL RED+IR+GREEN FIFO ENTRY */
        int ret = i2c_burst_read(pulse_dev, SENSOR_ADDR, 0x07, fifo, 9);
        if (ret) {
            printk("FIFO read error %d\n", ret);
            k_msleep(10);
            continue;
        }

        uint32_t red   = ((fifo[0] << 16) | (fifo[1] << 8) | fifo[2]) & 0x3FFFF;
        uint32_t ir    = ((fifo[3] << 16) | (fifo[4] << 8) | fifo[5]) & 0x3FFFF;
        uint32_t green = ((fifo[6] << 16) | (fifo[7] << 8) | fifo[8]) & 0x3FFFF;

		printk("RED: %d, IR: %d, GREEN: %d\n", red, ir, green);

        // /* Package into struct */
        // struct ppg_sample sample = {
        //     .red   = red,
        //     .ir    = ir,
        //     .green = green
        // };

        /* ============================
        //      SEND OVER BLUETOOTH
        //    ============================ */
        // if (spo2_notify_enabled && current_conn) {

        //     /* Packet layout: 12 bytes
        //        [0-3] = RED
        //        [4-7] = IR
        //        [8-11] = GREEN
        //     */

        //     uint32_t pkt[3] = { red, ir, green };

        //     bt_gatt_notify(current_conn, 
        //                    &audio_svc.attrs[5],   // your SpO2 characteristic
        //                    pkt, sizeof(pkt));
        // }

        k_msleep(100);  // ~100Hz sensor rate
    }
}

K_THREAD_DEFINE(spo2_thread_id, 2048, polling_thread_spo2,
                NULL, NULL, NULL, 5, 0, 0);  // Priority 5 (medium)


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

        // LED currents
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x0C, 0x50); // RED
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x0D, 0x80); // IR
        i2c_reg_write_byte(pulse_dev, SENSOR_ADDR, 0x0E, 0x80); // GREEN

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

	return 0;
}
