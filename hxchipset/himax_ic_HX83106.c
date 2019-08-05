/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for HX83106 chipset
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

#include "himax_ic_HX83106.h"
#include "himax_modular.h"

static void hx83106_chip_init(void)
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

#if defined(HX_ZERO_FLASH)
static void himax_hx83106_reload_to_active(void)
{
	uint8_t addr[DATA_LEN_4] = {0};
	uint8_t data[DATA_LEN_4] = {0};
	uint8_t retry_cnt = 0;

	addr[3] = 0x90;
	addr[2] = 0x00;
	addr[1] = 0x00;
	addr[0] = 0x48;

	do {
		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0x00;
		data[0] = 0xEC;
		kp_g_core_fp->fp_register_write(addr, sizeof(data), data, 0);

		kp_g_core_fp->fp_register_read(addr, sizeof(data), data, 0);
		I("%s: data[1]=%d, data[0]=%d, retry_cnt=%d\n", __func__,
				data[1], data[0], retry_cnt);
		retry_cnt++;
	} while ((data[1] != 0x01
		|| data[0] != 0xEC)
		&& retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static void himax_hx83106_resume_ic_action(void)
{
#ifndef HX_RESUME_HW_RESET
	himax_hx83106_reload_to_active();
#endif
}

static void himax_hx83106_suspend_ic_action(void)
{
#ifndef HX_RESUME_HW_RESET
	himax_hx83106_reload_to_active();
#endif
}

static void himax_hx83106_sense_on(uint8_t FlashMode)
{
	uint8_t tmp_data[DATA_LEN_4];
	int retry = 0;
	int ret = 0;

	I("%s\n", __func__);

	kp_g_core_fp->fp_interface_on();
	kp_g_core_fp->fp_register_write((*kp_pfw_op)->addr_ctrl_fw_isr,
		sizeof((*kp_pfw_op)->data_clear),
		(*kp_pfw_op)->data_clear, 0);

	msleep(20);

	if (!FlashMode) {
#ifdef HX_RST_PIN_FUNC
		kp_g_core_fp->fp_ic_reset(false, false);
#else
		kp_g_core_fp->fp_system_reset();
#endif
		himax_hx83106_reload_to_active();
	} else {
		himax_hx83106_reload_to_active();
		do {
			kp_g_core_fp->fp_register_write(
				(*kp_pfw_op)->addr_safe_mode_release_pw,
				sizeof((*kp_pfw_op)->data_safe_mode_release_pw_active),
				(*kp_pfw_op)->data_safe_mode_release_pw_active,
				0);

			kp_g_core_fp->fp_register_read(
				(*kp_pfw_op)->addr_flag_reset_event,
				sizeof(tmp_data),
				tmp_data,
				0);

			I("%s: Read status from IC = %X,%X\n", __func__,
					tmp_data[0], tmp_data[1]);
		} while ((tmp_data[1] != 0x01
			|| tmp_data[0] != 0x00)
			&& retry++ < 5);

		if (retry >= 5) {
			E("%s: Fail:\n", __func__);
#ifdef HX_RST_PIN_FUNC
			kp_g_core_fp->fp_ic_reset(false, false);
#else
			kp_g_core_fp->fp_system_reset();
#endif
			himax_hx83106_reload_to_active();
		} else {
			I("%s: OK and Read status from IC = %X,%X\n",
				__func__,
				tmp_data[0],
				tmp_data[1]);
			/* reset code*/
			tmp_data[0] = 0x00;

			ret = kp_himax_bus_write(
				(*kp_pic_op)->adr_i2c_psw_lb[0],
				tmp_data,
				1,
				HIMAX_I2C_RETRY_TIMES);
			if (ret < 0)
				E("%s: i2c access fail!\n", __func__);

			ret = kp_himax_bus_write(
				(*kp_pic_op)->adr_i2c_psw_ub[0],
				tmp_data,
				1,
				HIMAX_I2C_RETRY_TIMES);

			if (ret < 0)
				E("%s: i2c access fail!\n", __func__);

			kp_g_core_fp->fp_register_write(
				(*kp_pfw_op)->addr_safe_mode_release_pw,
				sizeof((*kp_pfw_op)->data_safe_mode_release_pw_reset),
				(*kp_pfw_op)->data_safe_mode_release_pw_reset,
				0);
		}
	}
}
#endif

static bool hx83106_sense_off(bool check_en)
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
		/**
		 *I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 */
		tmp_data[0] = (*kp_pic_op)->data_i2c_psw_lb[0];

		ret = kp_himax_bus_write((*kp_pic_op)->adr_i2c_psw_lb[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 *I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95
		 */
		tmp_data[0] = (*kp_pic_op)->data_i2c_psw_ub[0];

		ret = kp_himax_bus_write((*kp_pic_op)->adr_i2c_psw_ub[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 *I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x00
		 */
		tmp_data[0] = 0x00;

		ret = kp_himax_bus_write((*kp_pic_op)->adr_i2c_psw_lb[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 *I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 */
		tmp_data[0] = (*kp_pic_op)->data_i2c_psw_lb[0];

		ret = kp_himax_bus_write((*kp_pic_op)->adr_i2c_psw_lb[0],
				tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 *I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95
		 */
		tmp_data[0] = (*kp_pic_op)->data_i2c_psw_ub[0];

		ret = kp_himax_bus_write((*kp_pic_op)->adr_i2c_psw_ub[0],
			tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 *Check enter_save_mode
		 */
		kp_g_core_fp->fp_register_read(
			(*kp_pic_op)->addr_cs_central_state,
			sizeof(tmp_data),
			tmp_data,
			0);

		I("%s: Check enter_save_mode data[0]=%X\n",
			__func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/**
			 *Reset TCON
			 */
			kp_g_core_fp->fp_register_write(
				(*kp_pic_op)->addr_tcon_on_rst,
				sizeof((*kp_pic_op)->data_rst),
				(*kp_pic_op)->data_rst, 0);
			/*msleep(1);*/
			usleep_range(1000, 1100);

			/**
			 *Reset ADC
			 */
			kp_g_core_fp->fp_register_write(
				(*kp_pic_op)->addr_adc_on_rst,
				sizeof((*kp_pic_op)->data_rst),
				(*kp_pic_op)->data_rst, 0);
			/*msleep(1);*/
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

		/*msleep(10);*/
		usleep_range(10000, 10100);
#ifdef HX_RST_PIN_FUNC
		kp_g_core_fp->fp_ic_reset(false, false);
#endif
	} while (cnt++ < 15);

	return false;
}

#if defined(HX_AUTO_UPDATE_FW) || defined(HX_ZERO_FLASH)
#if defined(HX_EN_DYNAMIC_NAME)
static void hx83106a_read_ic_ver(void)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t tmp_addr[DATA_LEN_4];

	/* Enable read DD */
	kp_himax_in_parse_assign_cmd(hx83106_ic_osc_en, tmp_addr,
			sizeof(tmp_addr));
	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data),
			tmp_data, 0);
	tmp_data[0] = 0x01;
	kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
			tmp_data, false);

	kp_himax_in_parse_assign_cmd(hx83106_ic_osc_pw, tmp_addr,
			sizeof(tmp_addr));
	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data),
			tmp_data, 0);
	tmp_data[0] = 0xA5;
	kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
			tmp_data, false);

	tmp_data[0] = 0x00;
	tmp_data[1] = 0x83;
	tmp_data[2] = 0x11;
	tmp_data[3] = 0x2A;
	kp_himax_in_parse_assign_cmd(hx83106_ic_b9_en, tmp_addr,
			sizeof(tmp_addr));
	kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
			tmp_data, false);

	tmp_data[0] = 0x00;
	tmp_data[1] = 0x55;
	tmp_data[2] = 0xAA;
	tmp_data[3] = 0x00;
	kp_himax_in_parse_assign_cmd(hx83106_ic_eb_en, tmp_addr,
		sizeof(tmp_addr));
	kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
			tmp_data, false);

	/* Read DD data */
	kp_himax_in_parse_assign_cmd(hx83106_cb_ic_fw,
		tmp_addr,
		sizeof(tmp_addr));

	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data), tmp_data, 0);
	I("CB=%X\n", tmp_data[0]);
	(*kp_ic_data)->vendor_old_ic_ver = tmp_data[0];

	kp_himax_in_parse_assign_cmd(hx83106_e8_ic_fw,
		tmp_addr,
		sizeof(tmp_addr));

	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data), tmp_data, 0);
	I("E8=%X\n", tmp_data[0]);
	(*kp_ic_data)->vendor_ic_ver = tmp_data[0];

	/* Disable read DD */
	kp_himax_in_parse_assign_cmd(hx83106_ic_osc_en, tmp_addr,
			sizeof(tmp_addr));
	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data), tmp_data, 0);
	tmp_data[0] = 0x00;
	kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
			tmp_data, false);
}

static void hx83106b_read_ic_ver(void)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t tmp_addr[DATA_LEN_4];

	/* Enable read DD */
	kp_himax_in_parse_assign_cmd(hx83106_ic_osc_pw, tmp_addr,
			sizeof(tmp_addr));
	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data),
			tmp_data, 0);
	tmp_data[0] = 0xA5;
	kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
			tmp_data, false);

	tmp_data[0] = 0x00;
	tmp_data[1] = 0x83;
	tmp_data[2] = 0x11;
	tmp_data[3] = 0x2B;
	kp_himax_in_parse_assign_cmd(hx83106_ic_b9_en, tmp_addr,
			sizeof(tmp_addr));
	kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
			tmp_data, false);

	/* Read DD data */
	kp_himax_in_parse_assign_cmd(hx83106_cb_ic_fw, tmp_addr,
			sizeof(tmp_addr));
	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data),
			tmp_data, 0);
	I("CB=%X\n", tmp_data[0]);
	(*kp_ic_data)->vendor_old_ic_ver = tmp_data[0];

	kp_himax_in_parse_assign_cmd(hx83106_e8_ic_fw, tmp_addr,
			sizeof(tmp_addr));
	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data),
			tmp_data, 0);
	I("E8=%X\n", tmp_data[0]);
	(*kp_ic_data)->vendor_ic_ver = tmp_data[0];
}

static void hx83106e_read_ic_ver(void)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t tmp_addr[DATA_LEN_4];

	/* Enable read DD */
	kp_himax_in_parse_assign_cmd(hx83106_ic_osc_pw, tmp_addr,
			sizeof(tmp_addr));
	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data), tmp_data, 0);
	tmp_data[0] = 0xA5;
	kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
			tmp_data, false);

	tmp_data[0] = 0x00;
	tmp_data[1] = 0x83;
	tmp_data[2] = 0x11;
	tmp_data[3] = 0x2E;
	kp_himax_in_parse_assign_cmd(hx83106_ic_b9_en,
		tmp_addr,
		sizeof(tmp_addr));

	kp_g_core_fp->fp_register_write(tmp_addr, sizeof(tmp_data),
		tmp_data, false);

	/* Read DD data */
	kp_himax_in_parse_assign_cmd(hx83106_cb_ic_fw, tmp_addr,
		sizeof(tmp_addr));
	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data), tmp_data, 0);
	I("CB=%X\n", tmp_data[0]);
	(*kp_ic_data)->vendor_old_ic_ver = tmp_data[0];

	kp_himax_in_parse_assign_cmd(hx83106_e8_ic_fw, tmp_addr,
		sizeof(tmp_addr));
	kp_g_core_fp->fp_register_read(tmp_addr, sizeof(tmp_data), tmp_data, 0);
	I("E8=%X\n", tmp_data[0]);
	(*kp_ic_data)->vendor_ic_ver = tmp_data[0];
}

static void hx83106_dynamic_fw_name(uint8_t ic_name)
{
	char firmware_name[64] = "Himax_firmware.bin";

	if (ic_name == 0x2a)
		hx83106a_read_ic_ver();
	else if (ic_name == 0x2b)
		hx83106b_read_ic_ver();
	else
		hx83106e_read_ic_ver();

	if (*kp_i_CTPM_firmware_name != NULL) {
		kfree(*kp_i_CTPM_firmware_name);
		*kp_i_CTPM_firmware_name = NULL;
	}
	memset(firmware_name, 0x00, sizeof(firmware_name));

	if (((*kp_ic_data)->vendor_ic_ver == 0x13)
	|| ((*kp_ic_data)->vendor_ic_ver == 0x14)
	|| ((*kp_ic_data)->vendor_ic_ver == 0x15)
	|| ((*kp_ic_data)->vendor_ic_ver == 0x16)
	|| ((*kp_ic_data)->vendor_ic_ver == 0x23)) {
		(*kp_ic_data)->vendor_semifac = 2;
	} else if (((*kp_ic_data)->vendor_old_ic_ver == 0x44)
	|| ((*kp_ic_data)->vendor_old_ic_ver == 0x77)
	|| ((*kp_ic_data)->vendor_ic_ver == 0x03)
	|| ((*kp_ic_data)->vendor_ic_ver == 0x04)) {
		(*kp_ic_data)->vendor_semifac = 1;
	} else {
		(*kp_ic_data)->vendor_semifac = 0;
	}

	/*memcpy(firmware_name, "Himax_firmware.bin",
		sizeof(char)*strlen("Himax_firmware.bin"));*/
	/**kp_i_CTPM_firmware_name =
		kzalloc((sizeof(char)*(strlen(firmware_name)+1)),
		GFP_KERNEL);*/

	*kp_i_CTPM_firmware_name = kzalloc((strlen(firmware_name)+1),
		GFP_KERNEL);
	if (*kp_i_CTPM_firmware_name != NULL) {
		/*memcpy(*kp_i_CTPM_firmware_name, firmware_name,
				(sizeof(char)*(strlen(firmware_name)+1)));*/
		strlcpy(*kp_i_CTPM_firmware_name, firmware_name,
				strlen(firmware_name)+1);
		I("%s: *kp_i_CTPM_firmware_name : %s\n", __func__,
				*kp_i_CTPM_firmware_name);
	} else {
		E("%s: allocate memory fail!!!\n", __func__);
	}
}
#endif
#endif

static void hx83106_func_re_init(void)
{
	kp_g_core_fp->fp_sense_off = hx83106_sense_off;
	kp_g_core_fp->fp_chip_init = hx83106_chip_init;
#if defined(HX_ZERO_FLASH)
	kp_g_core_fp->fp_resume_ic_action = himax_hx83106_resume_ic_action;
	kp_g_core_fp->fp_suspend_ic_action = himax_hx83106_suspend_ic_action;
	kp_g_core_fp->fp_sense_on = himax_hx83106_sense_on;
#endif
}

static void hx83106_reg_re_init(void)
{
	I("%s\n", __func__);
	kp_himax_in_parse_assign_cmd(hx83106a_fw_addr_raw_out_sel,
			(*kp_pfw_op)->addr_raw_out_sel,
			sizeof((*kp_pfw_op)->addr_raw_out_sel));
}

static bool hx83106_chip_detect(void)
{
	uint8_t tmp_data[DATA_LEN_4];
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

	hx83106_reg_re_init();
	hx83106_func_re_init();

	kp_g_core_fp->fp_sense_off(false);

	for (i = 0; i < 5; i++) {
		kp_g_core_fp->fp_register_read((*kp_pfw_op)->addr_icid_addr,
				sizeof(tmp_data), tmp_data, false);
		I("%s: Read driver IC ID = %X, %X, %X\n", __func__,
				tmp_data[3], tmp_data[2], tmp_data[1]);

		if ((tmp_data[3] == 0x83)
		&& (tmp_data[2] == 0x10)
		&& (tmp_data[1] == 0x6a)) {

			/*if (tmp_data[1] == 0x6a)*/
			strlcpy((*kp_private_ts)->chip_name,
				HX_83106A_SERIES_PWON,
				sizeof((*kp_private_ts)->chip_name));

			I("%s: IC name = %s\n", __func__,
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

#if defined(HX_AUTO_UPDATE_FW) || defined(HX_ZERO_FLASH)
#if defined(HX_EN_DYNAMIC_NAME)
	hx83106_dynamic_fw_name(tmp_data[1]);
#endif
#endif

	return ret_data;
}

/*struct himax_chip_entry HX_MOD_KSYM_HX83106;*/
/*EXPORT_SYMBOL(HX_MOD_KSYM_HX83106);*/
DECLARE(HX_MOD_KSYM_HX83106);

static int himax_hx83106_probe(void)
{
	I("%s\n", __func__);
	himax_add_chip_dt(hx83106_chip_detect);
	return 0;
}

static int himax_hx83106_remove(void)
{
	free_chip_dt_table();
	return 0;
}

static int __init himax_hx83106_init(void)
{
	int ret = 0;

	I("%s\n", __func__);
	ret = himax_hx83106_probe();
	return 0;
}

static void __exit himax_hx83106_exit(void)
{
	himax_hx83106_remove();
}

module_init(himax_hx83106_init);
module_exit(himax_hx83106_exit);

MODULE_DESCRIPTION("HIMAX HX83106 touch driver");
MODULE_LICENSE("GPL");
