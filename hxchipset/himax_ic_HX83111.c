/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for HX83111 chipset
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

#include "himax_ic_HX83111.h"
#include "himax_modular.h"

static void hx83111_chip_init(void)
{
	(*kp_private_ts)->chip_cell_type = CHIP_IS_IN_CELL;
	I("%s:IC cell type = %d\n", __func__, (*kp_private_ts)->chip_cell_type);
	*kp_IC_CHECKSUM	= HX_TP_BIN_CHECKSUM_CRC;
	/*Himax: Set FW and CFG Flash Address*/
	*kp_FW_VER_MAJ_FLASH_ADDR = 49157;  /*0x00C005*/
	*kp_FW_VER_MAJ_FLASH_LENG = 1;
	*kp_FW_VER_MIN_FLASH_ADDR = 49158;  /*0x00C006*/
	*kp_FW_VER_MIN_FLASH_LENG = 1;
	*kp_CFG_VER_MAJ_FLASH_ADDR = 49408;  /*0x00C100*/
	*kp_CFG_VER_MAJ_FLASH_LENG = 1;
	*kp_CFG_VER_MIN_FLASH_ADDR = 49409;  /*0x00C101*/
	*kp_CFG_VER_MIN_FLASH_LENG = 1;
	*kp_CID_VER_MAJ_FLASH_ADDR = 49154;  /*0x00C002*/
	*kp_CID_VER_MAJ_FLASH_LENG = 1;
	*kp_CID_VER_MIN_FLASH_ADDR = 49155;  /*0x00C003*/
	*kp_CID_VER_MIN_FLASH_LENG = 1;
	/*PANEL_VERSION_ADDR = 49156;  //0x00C004
	 *PANEL_VERSION_LENG = 1;
	 */
}

static void hx83111_func_re_init(void)
{
	kp_g_core_fp->fp_chip_init = hx83111_chip_init;
}
static bool hx83111_chip_detect(void)
{
	uint8_t tmp_data[DATA_LEN_4];
	bool ret_data = false;
	int ret = 0;
	int i = 0;

	if (himax_ic_setup_external_symbols())
		return false;

	ret = kp_himax_mcu_in_cmd_struct_init();
	if (ret < 0) {
		ret_data = false;
		E("%s:cmd_struct_init Fail:\n", __func__);
		return ret_data;
	}

	kp_himax_mcu_in_cmd_init();

	hx83111_func_re_init();

	msleep(50);
	kp_g_core_fp->fp_sense_off(false);

	for (i = 0; i < 5; i++) {
		kp_g_core_fp->fp_register_read((*kp_pfw_op)->addr_icid_addr,
				DATA_LEN_4, tmp_data, false);
		I("%s:Read driver IC ID = %X,%X,%X\n", __func__,
				tmp_data[3], tmp_data[2], tmp_data[1]);

		if ((tmp_data[3] == 0x83)
		&& (tmp_data[2] == 0x11)
		&& ((tmp_data[1] == 0x1a)
		|| (tmp_data[1] == 0x1b))) {
			strlcpy((*kp_private_ts)->chip_name,
				HX_83111B_SERIES_PWON, 30);
			I("%s:IC name = %s\n", __func__,
				(*kp_private_ts)->chip_name);
			ret_data = true;
			break;
		}
		ret_data = false;
		E("%s:Read driver ID register Fail:\n", __func__);
		E("Could NOT find Himax Chipset\n");
		E("Please check 1.VCCD,VCCA,VSP,VSN\n");
		E("2. LCM_RST,TP_RST\n");
		E("3. Power On Sequence\n");
	}

	return ret_data;
}

DECLARE(HX_MOD_KSYM_HX83111);

static int himax_hx83111_probe(void)
{
	himax_add_chip_dt(hx83111_chip_detect);

	return 0;
}

static int himax_hx83111_remove(void)
{
	free_chip_dt_table();
	return 0;
}

static int __init himax_hx83111_init(void)
{
	int ret = 0;

	I("%s\n", __func__);
	ret = himax_hx83111_probe();
	return 0;
}

static void __exit himax_hx83111_exit(void)
{
	himax_hx83111_remove();
}

module_init(himax_hx83111_init);
module_exit(himax_hx83111_exit);

MODULE_DESCRIPTION("HIMAX HX83111 touch driver");
MODULE_LICENSE("GPL");
