/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for HX83103 chipset
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

#include "himax_ic_HX83103.h"
#include "himax_modular.h"

static void hx83103_chip_init(void)
{
	(*kp_private_ts)->chip_cell_type = CHIP_IS_IN_CELL;
	I("%s: IC cell type = %d\n", __func__,
		(*kp_private_ts)->chip_cell_type);
	*kp_IC_CHECKSUM = HX_TP_BIN_CHECKSUM_CRC;
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
}

static void hx83103_sense_on(uint8_t FlashMode)
{
	kp_g_core_fp->fp_interface_on();
	kp_g_core_fp->fp_register_write((*kp_pfw_op)->addr_ctrl_fw_isr,
		sizeof((*kp_pfw_op)->data_clear),
		(*kp_pfw_op)->data_clear,
		false);
	msleep(20);
	kp_g_core_fp->fp_system_reset();
}

static bool hx83103_sense_off(bool check_en)
{
	uint8_t cnt = 0;
	uint8_t tmp_data[DATA_LEN_4];
	int ret = 0;

	do {
		if (cnt == 0
		|| (tmp_data[0] != 0xA5
		&& tmp_data[0] != 0x00
		&& tmp_data[0] != 0x87))
			kp_g_core_fp->fp_register_write(
				(*kp_pfw_op)->addr_ctrl_fw_isr,
				sizeof((*kp_pfw_op)->data_fw_stop),
				(*kp_pfw_op)->data_fw_stop,
				0);

		msleep(20);

		/* check fw status */
		kp_g_core_fp->fp_register_read(
			(*kp_pic_op)->addr_cs_central_state,
			sizeof(tmp_data),
			tmp_data,
			0);

		if (tmp_data[0] != 0x05) {
			I("%s: Do not need wait FW, Status = 0x%02X!\n",
				__func__, tmp_data[0]);
			break;
		}

		kp_g_core_fp->fp_register_read((*kp_pfw_op)->addr_ctrl_fw_isr,
				sizeof(tmp_data), tmp_data, false);
		I("%s: cnt = %d, data[0] = 0x%02X!\n", __func__,
				cnt, tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (++cnt < 10) && check_en == true);

	cnt = 0;

	do {
		tmp_data[0] = (*kp_pic_op)->data_i2c_psw_lb[0];

		ret = kp_himax_bus_write((*kp_pic_op)->adr_i2c_psw_lb[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		tmp_data[0] = (*kp_pic_op)->data_i2c_psw_ub[0];

		ret = kp_himax_bus_write((*kp_pic_op)->adr_i2c_psw_ub[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		kp_g_core_fp->fp_register_read(
			(*kp_pic_op)->addr_cs_central_state,
			sizeof(tmp_data),
			tmp_data,
			0);
		I("%s: Check enter_save_mode data[0]=%X\n",
			__func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			kp_g_core_fp->fp_register_write(
				(*kp_pic_op)->addr_tcon_on_rst,
				sizeof((*kp_pic_op)->data_rst),
				(*kp_pic_op)->data_rst,
				0);

			usleep_range(1000, 1100);
			tmp_data[3] = (*kp_pic_op)->data_rst[3];
			tmp_data[2] = (*kp_pic_op)->data_rst[2];
			tmp_data[1] = (*kp_pic_op)->data_rst[1];
			tmp_data[0] = (*kp_pic_op)->data_rst[0] | 0x01;
			kp_g_core_fp->fp_register_write(
				(*kp_pic_op)->addr_tcon_on_rst,
				sizeof(tmp_data),
				tmp_data,
				0);

			kp_g_core_fp->fp_register_write(
				(*kp_pic_op)->addr_adc_on_rst,
				sizeof((*kp_pic_op)->data_rst),
				(*kp_pic_op)->data_rst,
				0);

			usleep_range(1000, 1100);
			tmp_data[3] = (*kp_pic_op)->data_rst[3];
			tmp_data[2] = (*kp_pic_op)->data_rst[2];
			tmp_data[1] = (*kp_pic_op)->data_rst[1];
			tmp_data[0] = (*kp_pic_op)->data_rst[0] | 0x01;
			kp_g_core_fp->fp_register_write(
				(*kp_pic_op)->addr_adc_on_rst,
				sizeof(tmp_data),
				tmp_data,
				0);
			return true;
		}

		usleep_range(10000, 10100);
#ifdef HX_RST_PIN_FUNC
		kp_g_core_fp->fp_ic_reset(false, false);
#endif
	} while (cnt++ < 15);

	return false;
}

#ifdef HX_ESD_RECOVERY
static void hx83103_esd_ic_reset(void)
{
	*kp_HX_ESD_RESET_ACTIVATE = 1;
#ifdef HX_RST_PIN_FUNC
	kp_g_core_fp->fp_pin_reset();
#endif
	I("%s\n", __func__);
}
#endif

static void hx83103_func_re_init(void)
{
	kp_g_core_fp->fp_chip_init = hx83103_chip_init;
	kp_g_core_fp->fp_sense_on = hx83103_sense_on;
	kp_g_core_fp->fp_sense_off = hx83103_sense_off;
#ifdef HX_ESD_RECOVERY
	kp_g_core_fp->fp_esd_ic_reset = hx83103_esd_ic_reset;
#endif
}

static void hx83103_reg_re_init(void)
{
	kp_himax_in_parse_assign_cmd(hx83103_fw_addr_raw_out_sel,
			(*kp_pfw_op)->addr_raw_out_sel,
			sizeof((*kp_pfw_op)->addr_raw_out_sel));
	kp_himax_in_parse_assign_cmd(hx83103_fw_addr_ctrl_fw,
			(*kp_pfw_op)->addr_ctrl_fw_isr,
			sizeof((*kp_pfw_op)->addr_ctrl_fw_isr));
	kp_himax_in_parse_assign_cmd(hx83103_data_df_x_res,
			(*kp_pdriver_op)->data_df_x_res,
			sizeof((*kp_pdriver_op)->data_df_x_res));
	kp_himax_in_parse_assign_cmd(hx83103_data_df_y_res,
			(*kp_pdriver_op)->data_df_y_res,
			sizeof((*kp_pdriver_op)->data_df_y_res));
}

static bool hx83103_chip_detect(void)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t tmp_addr[DATA_LEN_4];
	bool ret_data = false;
	int ret = 0;
	int i = 0;

	if (himax_ic_setup_external_symbols())
		return false;

	ret = kp_himax_mcu_in_cmd_struct_init();
	if (ret < 0) {
		E("%s: cmd_struct_init Fail:\n", __func__);
		return false;
	}

	kp_himax_mcu_in_cmd_init();

	hx83103_reg_re_init();
	hx83103_func_re_init();

	kp_g_core_fp->fp_sense_off(false);

	for (i = 0; i < 5; i++) {
		/*1. Set DDREG_Req = 1 (0x9000_0020 = 0x0000_0001)
		(Lock register R/W from driver)*/
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x20;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x01;
		kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
				tmp_data, false);
		/*2. Set bank as 0 (0x8001_BD01 = 0x0000_0000)*/
		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x01;
		tmp_addr[1] = 0xBD;
		tmp_addr[0] = 0x01;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = 0x00;
		tmp_data[0] = 0x00;
		kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
				tmp_data, false);
		/*3. Read driver ID register RF4H 1 byte (0x8001_F401)*/
		/*Driver register RF4H 1 byte value = 0x84H, read back
			value will become 0x84848484*/
		tmp_addr[3] = 0x80;
		tmp_addr[2] = 0x01;
		tmp_addr[1] = 0xF4;
		tmp_addr[0] = 0x01;
		kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data),
				tmp_data, 0);
		I("%s:Read driver IC ID = %X, %X, %X\n", __func__,
				tmp_data[3], tmp_data[2], tmp_data[1]);

		if (tmp_data[0] == 0x42) {
			strlcpy((*kp_private_ts)->chip_name,
				HX_83103A_SERIES_PWON, 30);
			I("%s:IC name = %s\n", __func__,
				(*kp_private_ts)->chip_name);

			I("Himax IC package %x%x%x in\n", tmp_data[3],
				tmp_data[2], tmp_data[1]);
			ret_data = true;
			break;
		}
	}

	if (ret_data == false) {
		E("%s: Read driver ID register Fail:\n", __func__);
		E("Could NOT find Himax Chipset, please check\n");
		E("1. VCCD,VCCA,VSP,VSN\n");
		E("2. LCM_RST,TP_RST\n");
		E("3. Power On Sequence\n");
	}

	return ret_data;
}

/*struct himax_chip_entry HX_MOD_KSYM_HX83103;*/
/*EXPORT_SYMBOL(HX_MOD_KSYM_HX83103);*/
DECLARE(HX_MOD_KSYM_HX83103);

static int himax_hx83103_probe(void)
{
	I("%s\n", __func__);
	himax_add_chip_dt(hx83103_chip_detect);
	return 0;
}

static int himax_hx83103_remove(void)
{
	free_chip_dt_table();
	return 0;
}

static int __init himax_hx83103_init(void)
{
	int ret = 0;

	I("%s\n", __func__);
	ret = himax_hx83103_probe();
	return 0;
}

static void __exit himax_hx83103_exit(void)
{
	himax_hx83103_remove();
}

module_init(himax_hx83103_init);
module_exit(himax_hx83103_exit);

MODULE_DESCRIPTION("HIMAX HX83103 touch driver");
MODULE_LICENSE("GPL");
