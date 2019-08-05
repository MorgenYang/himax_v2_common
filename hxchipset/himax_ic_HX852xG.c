/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for HX852xG chipset
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

#include "himax_ic_HX852xG.h"
#include "himax_modular.h"

static void hx852xG_chip_init(void)
{
	(*kp_private_ts)->chip_cell_type = CHIP_IS_ON_CELL;
	I("%s:IC cell type = %d\n", __func__, (*kp_private_ts)->chip_cell_type);
	*kp_IC_CHECKSUM = HX_TP_BIN_CHECKSUM_CRC;
	/*Himax: Set FW and CFG Flash Address*/
	*kp_FW_VER_MAJ_FLASH_ADDR = 64517;  /*0xFC05*/
	*kp_FW_VER_MAJ_FLASH_LENG = 1;
	*kp_FW_VER_MIN_FLASH_ADDR = 64518;  /*0xFC06*/
	*kp_FW_VER_MIN_FLASH_LENG = 1;
	*kp_CFG_VER_MAJ_FLASH_ADDR = 64672;  /*0xFCA0*/
	*kp_CFG_VER_MAJ_FLASH_LENG = 1;
	/*CFG_VER_MIN_FLASH_ADDR = 49409;*/  /*0xC101*/
	/*CFG_VER_MIN_FLASH_LENG = 1;*/
	*kp_CID_VER_MAJ_FLASH_ADDR = 64514;  /*0x0xFC02*/
	*kp_CID_VER_MAJ_FLASH_LENG = 1;
	*kp_CID_VER_MIN_FLASH_ADDR = 64515;  /*0x0xFC03*/
	*kp_CID_VER_MIN_FLASH_LENG = 1;
	/*PANEL_VERSION_ADDR = 49156;*/  /*0x00C004*/
	/*PANEL_VERSION_LENG = 1;*/
}

static void himax_hx852xG_func_re_init(void)
{
	/*g_core_fp.fp_chip_init = hx852xG_chip_init;*/
	kp_g_core_fp->fp_chip_init = hx852xG_chip_init;
}

static void himax_hx852xG_reg_re_init(void)
{
	/*himax_on_parse_assign_cmd(hx852xg_data_df_rx,
			on_pdriver_op->data_df_rx,
			sizeof(on_pdriver_op->data_df_rx));*/
	/*himax_on_parse_assign_cmd(hx852xg_data_df_tx,
			on_pdriver_op->data_df_tx,
			sizeof(on_pdriver_op->data_df_tx));*/
	kp_himax_on_parse_assign_cmd(hx852xg_data_df_rx,
			(*kp_on_pdriver_op)->data_df_rx,
			sizeof((*kp_on_pdriver_op)->data_df_rx));
	kp_himax_on_parse_assign_cmd(hx852xg_data_df_tx,
			(*kp_on_pdriver_op)->data_df_tx,
			sizeof((*kp_on_pdriver_op)->data_df_tx));
	kp_himax_on_parse_assign_cmd(hx852xg_addr_fw_xy_rev_int_edge,
			(*kp_on_pdriver_op)->addr_fw_xy_rev_int_edge,
			sizeof((*kp_on_pdriver_op)->addr_fw_xy_rev_int_edge));
}

static bool hx852xG_chip_detect(void)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t tmp_addr[DATA_LEN_4];
	bool ret_data = false;
	int ret = 0;
	int i = 0;

	if (himax_ic_setup_external_symbols())
		return false;

	ret = kp_himax_mcu_on_cmd_struct_init();
	if (ret < 0) {
		E("%s:cmd_struct_init Fail:\n", __func__);
		return false;
	}

	kp_himax_mcu_on_cmd_init();

	himax_hx852xG_reg_re_init();

	himax_hx852xG_func_re_init();

	msleep(50);

	kp_himax_bus_read(0x84, tmp_data, sizeof(tmp_data),
		HIMAX_I2C_RETRY_TIMES);
	I("R84H : [0]=0x%02X,[1]=0x%02X,[2]=0x%02X,[3]=0x%02X\n",
		tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

	kp_g_core_fp->fp_idle_mode(1);
	for (i = 0; i < 5; i++) {
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x08;
		tmp_addr[1] = 0x80;
		tmp_addr[0] = 0x2C;
		kp_g_core_fp->fp_register_read(tmp_addr, DATA_LEN_4,
			tmp_data, 0);
		I("%s:Read driver IC ID = %X,%X,%X\n", __func__,
			tmp_data[3], tmp_data[2], tmp_data[1]); /*85,32,00*/

		if ((tmp_data[3] == 0x85) && (tmp_data[2] == 0x32)) {
			I("%s: detect IC successfully\n", __func__);
			ret_data = true;
			break;
		}
	}
	kp_g_core_fp->fp_idle_mode(0);

	if (ret_data == false) {
		E("%s: Read driver ID register Fail:\n", __func__);
		E("Could NOT find Himax Chipset, Please check\n");
		E("1. VCCD,VCCA,VSP,VSN\n");
		E("2. LCM_RST,TP_RST\n");
		E("3. Power On Sequence\n");
	}

	return ret_data;
}

DECLARE(HX_MOD_KSYM_HX852xG);

static int himax_hx852xG_probe(void)
{
	I("%s: Enter\n", __func__);
	himax_add_chip_dt(hx852xG_chip_detect);
	return 0;
}

static int himax_hx852xG_remove(void)
{
	free_chip_dt_table();
	return 0;
}

static int __init himax_hx852xG_init(void)
{
	int ret = 0;

	I("%s\n", __func__);
	ret = himax_hx852xG_probe();
	return 0;
}

static void __exit himax_hx852xG_exit(void)
{
	himax_hx852xG_remove();
}

module_init(himax_hx852xG_init);
module_exit(himax_hx852xG_exit);

MODULE_DESCRIPTION("HIMAX HX852xG touch driver");
MODULE_LICENSE("GPL");
