/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _AUDIO_SYSTEM_H_
#define _AUDIO_SYSTEM_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define VALUE_NOT_SET 0

/**
 * @brief	Start the execution of the encoder thread.
 */
void audio_system_encoder_start(void);

/**
 * @brief	Stop the encoder thread from executing.
 *
 * @note	Using this allows the encode thread to always be enabled,
 *		but disables the execution when not needed, saving power.
 */
void audio_system_encoder_stop(void);

/**
 * @brief	Toggle a test tone on and off.
 *
 * @note	A stream must already be running to use this feature.
 *
 * @param[in]	freq	Desired frequency of tone. Off if set to 0.
 *
 * @retval	-ENOMEM	The frequency is too low (buffer overflow).
 * @retval	0	Success.
 */
int audio_system_encode_test_tone_set(uint32_t freq);

/**
 * @brief	Step through different test tones.
 *
 * @note	A stream must already be running to use this feature.
 *		Will step through test tones: 1 kHz, 2 kHz, 4 kHz and off.
 *
 * @return	0 on success, error otherwise.
 */
int audio_system_encode_test_tone_step(void);

/** @brief	What the encoder feeds into the stream.
 *
 * The cycle order is the order the button steps through.
 */
enum audio_system_src {
	AUDIO_SYSTEM_SRC_TONE,  /**< Fixed-frequency sine, CONFIG_AUDIO_TEST_TONE_HZ. */
	AUDIO_SYSTEM_SRC_NOISE, /**< White noise. */
	AUDIO_SYSTEM_SRC_MIC,   /**< The real microphone input. */
	AUDIO_SYSTEM_SRC_CNT,
};

/**
 * @brief	Select what the encoder sends.
 *
 * Both synthetic sources replace the microphone input rather than mixing
 * with it. Takes effect on the next encoded frame; a stream does not have
 * to be running first.
 *
 * @param[in]	src	Source to switch to.
 *
 * @retval	-EINVAL	Unknown source.
 * @retval	-ENOMEM	Tone period does not fit the tone buffer.
 * @retval	0	Success.
 */
int audio_system_encode_src_set(enum audio_system_src src);

/**
 * @brief	Advance to the next source in the cycle.
 *
 * Falls back to the microphone if the next source fails to start, so a
 * failure never leaves the stream silent.
 *
 * @return	The source now in effect.
 */
enum audio_system_src audio_system_encode_src_step(void);

/**
 * @brief	Name of a source, for logging.
 */
const char *audio_system_src_name(enum audio_system_src src);

/**
 * @brief	Set the sample rates for the encoder and the decoder, and the bit rate for encoder.
 *
 * @note	If any of the values are 0, the corresponding configuration will not be set.
 *
 * @param[in]	encoder_sample_rate_hz	Sample rate to be used by the encoder; can be 0.
 * @param[in]	encoder_bitrate		Bit rate to be used by the encoder (bps); can be 0.
 * @param[in]	decoder_sample_rate_hz	Sample rate to be used by the decoder; can be 0.
 *
 * @retval	-EINVAL	Invalid sample rate given.
 * @retval	0	On success.
 */
int audio_system_config_set(uint32_t encoder_sample_rate_hz, uint32_t encoder_bitrate,
			    uint32_t decoder_sample_rate_hz);

/**
 * @brief	Decode data and then add it to TX FIFO buffer.
 *
 * @param[in]	encoded_data		Pointer to encoded data.
 * @param[in]	encoded_data_size	Size of encoded data.
 * @param[in]	bad_frame		Indication on missed or incomplete frame.
 *
 * @return	0 on success, error otherwise.
 */
int audio_system_decode(void const *const encoded_data, size_t encoded_data_size, bool bad_frame);

/**
 * @brief	Initialize and start both HW and SW audio codec.
 */
void audio_system_start(void);

/**
 * @brief	Stop all activities related to audio.
 */
void audio_system_stop(void);

/**
 * @brief	Drop oldest block from the fifo_rx buffer.
 *
 * @note	This can be used to reduce latency by adjusting the timing of the completed frame
 *		that was sampled in relation to the connection interval in Bluetooth LE.
 *
 * @return	0 on success, -ECANCELED otherwise.
 */
int audio_system_fifo_rx_block_drop(void);

/**
 * @brief	Get number of decoder channels.
 *
 * @return	Number of decoder channels.
 */
int audio_system_decoder_num_ch_get(void);

/**
 * @brief	Initialize the audio_system.
 *
 * @return	0 on success, error otherwise.
 */
int audio_system_init(void);

#endif /* _AUDIO_SYSTEM_H_ */
