/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _NRF5340_AUDIO_DK_H_
#define _NRF5340_AUDIO_DK_H_

#include <stdbool.h>

#include "led.h"

/**
 * @brief	Initialize the hardware related modules on the nRF5340 Audio DK/PCA10121.
 *
 * @return	0 if successful, error otherwise.
 */
int nrf5340_audio_dk_init(void);

/**
 * @brief	Drive the charger's charge-termination pin (CHG_TERM, P1.05).
 *
 * @param	terminate	true: disable charging (pin HIGH).
 *				false: enable charging (pin LOW).
 *
 * @return	0 if successful, error otherwise.
 */
int tam_board_chg_term_set(bool terminate);

#endif /* _NRF5340_AUDIO_DK_H_ */
