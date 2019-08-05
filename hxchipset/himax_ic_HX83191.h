/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for HX83191 chipset
 *
 *  Copyright (C) 2019 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include "himax_platform.h"
#include "himax_common.h"
#include "himax_ic_core.h"
#include <linux/slab.h>

#define hx83191_fw_addr_fw_mode_status  0x1000b088
#define hx83191_fw_addr_sorting_mode_en 0x1000bf04
#define hx83191_fw_addr_set_frame_addr  0x1000b294
#define hx83191_fw_addr_raw_out_sel     0x100072ec

#define hx83191_driver_addr_fw_define_flash_reload      0x10007f00
#define hx83191_driver_addr_fw_define_2nd_flash_reload  0x100072c0
#define hx83191_driver_addr_fw_define_rxnum_txnum_maxpt 0x1000b0f4
#define hx83191_data_df_rx      48
#define hx83191_data_df_tx      24
#define hx83191_data_df_x_res   2160
#define hx83191_data_df_y_res   3840

static int hx_ic_Amount;

enum IC_CATEGORY {
	MASTER,
	SLAVE_ALL,
	SLAVE1,
	SLAVE2,
};

