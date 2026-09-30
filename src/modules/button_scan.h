/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BUTTON_SCAN_H_
#define _BUTTON_SCAN_H_

#include <stdint.h>

/** @brief Pins reading against their pull, as of the last scan cycle.
 *
 * Readable over SWD so a scan can be recovered without a working console.
 * First index is the pass (0 = pull-up, 1 = pull-down), second is the port
 * (0 = P0, 1 = P1).
 */
struct button_scan_state {
	uint32_t active[2][2];
	uint32_t t_ms;
	uint32_t change_cnt;
};

extern struct button_scan_state button_scan_last;

/** @brief Scan all unreserved GPIOs for a pin held against its pull.
 *
 * Each cycle applies a pull-up and then a pull-down to every candidate pin
 * and logs the pins that disagree with the pull. Detection is on absolute
 * level, so a button already held when the scan starts is still found.
 *
 * @param duration_ms	Total scan time, or 0 to loop until reset.
 * @param period_ms	Delay between cycles.
 *
 * @return 0 if successful, error otherwise. Does not return if
 *	   @p duration_ms is 0.
 */
int button_scan_run(uint32_t duration_ms, uint32_t period_ms);

#endif /* _BUTTON_SCAN_H_ */
