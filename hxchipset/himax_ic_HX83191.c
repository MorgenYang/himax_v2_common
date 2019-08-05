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

#include "himax_ic_HX83191.h"
#include "himax_modular.h"

static void hx83191_chip_init(void)
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
}

static void himaxRegReadCascade(uint8_t *read_data, int data_length,
		uint8_t *addr, int addr_Length, uint8_t icCategory)
{
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[DATA_LEN_4] = {0};

	if (icCategory == SLAVE2) {
		if (addr_Length == 1) {
			tmp_addr[3] = 0x38;
			tmp_addr[2] = 0xFF;
			tmp_addr[1] = addr[0];
			tmp_addr[0] = 0x00;
		} else {
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x09;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x00;

			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = addr[3];
			kp_g_core_fp->fp_register_write(tmp_addr, DATA_LEN_4,
				tmp_data, 0);

			tmp_addr[3] = 0x38;
			tmp_addr[2] = addr[2];
			tmp_addr[1] = addr[1];
			tmp_addr[0] = addr[0];
		}
		kp_g_core_fp->fp_register_read(tmp_addr, data_length,
				read_data, false);
	} else if (icCategory == SLAVE1) {
		if (addr_Length == 1) {
			tmp_addr[3] = 0x28;
			tmp_addr[2] = 0xFF;
			tmp_addr[1] = addr[0];
			tmp_addr[0] = 0x00;
		} else {
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x07;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x00;

			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = addr[3];
			kp_g_core_fp->fp_register_write(tmp_addr, DATA_LEN_4,
				tmp_data, 0);

			tmp_addr[3] = 0x28;
			tmp_addr[2] = addr[2];
			tmp_addr[1] = addr[1];
			tmp_addr[0] = addr[0];
		}
		kp_g_core_fp->fp_register_read(tmp_addr, data_length,
				read_data, false);
	} else { /* else if (icCategory == IC_CATEGORY.MASTART) */
		kp_g_core_fp->fp_register_read(tmp_addr, data_length,
				read_data, false);
	}
}

static void himaxRegWriteCascade(int addr_length, uint8_t *addr,
		int data_length, uint8_t *write_data, uint8_t icCategory)
{
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[DATA_LEN_4] = {0};

	if (icCategory == MASTER) {
		kp_g_core_fp->fp_register_write(addr, data_length,
			write_data, 0);
	} else {
		if ((icCategory == SLAVE_ALL && hx_ic_Amount == 3)
		|| icCategory == SLAVE2) {
			if (addr_length == 1) {
				tmp_addr[3] = 0x38;
				tmp_addr[2] = 0xFF;
				tmp_addr[1] = addr[0];
				tmp_addr[0] = 0x00;
			} else {
				tmp_addr[3] = 0x80;
				tmp_addr[2] = 0x09;
				tmp_addr[1] = 0x00;
				tmp_addr[0] = 0x00;

				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = 0x00;
				tmp_data[0] = addr[3];
				kp_g_core_fp->fp_register_write(tmp_addr,
						DATA_LEN_4,
						tmp_data,
						0);

				tmp_addr[3] = 0x38;
				tmp_addr[2] = addr[2];
				tmp_addr[1] = addr[1];
				tmp_addr[0] = addr[0];
			}
			kp_g_core_fp->fp_register_write(tmp_addr, data_length,
					write_data, 0);
		}

		if ((icCategory == SLAVE_ALL && hx_ic_Amount >= 2)
		|| icCategory == SLAVE1) {
			if (addr_length == 1) {
				tmp_addr[3] = 0x28;
				tmp_addr[2] = 0xFF;
				tmp_addr[1] = addr[0];
				tmp_addr[0] = 0x00;
			} else {
				tmp_addr[3] = 0x80;
				tmp_addr[2] = 0x07;
				tmp_addr[1] = 0x00;
				tmp_addr[0] = 0x00;

				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = 0x00;
				tmp_data[0] = addr[3];
				kp_g_core_fp->fp_register_write(tmp_addr,
						DATA_LEN_4,
						tmp_data,
						0);

				tmp_addr[3] = 0x28;
				tmp_addr[2] = addr[2];
				tmp_addr[1] = addr[1];
				tmp_addr[0] = addr[0];
			}
			kp_g_core_fp->fp_register_write(tmp_addr, data_length,
					write_data, 0);
		}
	}
}

static bool AHBI2C_TCON_Reset_Slave(void)
{
	uint8_t addr[DATA_LEN_4] = {0};
	uint8_t data[DATA_LEN_4] = {0};

	addr[3] = 0x80;
	addr[2] = 0x02;
	addr[1] = 0x00;
	addr[0] = 0x20;

	data[3] = 0x00;
	data[2] = 0x00;
	data[1] = 0x00;
	data[0] = 0x00;
	himaxRegWriteCascade(DATA_LEN_4, addr, DATA_LEN_4, data, SLAVE_ALL);

	return true;
}

static bool AHBI2C_ADC_Reset_Slave(void)
{
	uint8_t addr[DATA_LEN_4] = {0};
	uint8_t data[DATA_LEN_4] = {0};

	addr[3] = 0x80;
	addr[2] = 0x02;
	addr[1] = 0x00;
	addr[0] = 0x94;

	data[3] = 0x00;
	data[2] = 0x00;
	data[1] = 0x00;
	data[0] = 0x00;
	himaxRegWriteCascade(DATA_LEN_4, addr, DATA_LEN_4, data, SLAVE_ALL);
	data[3] = 0x00;
	data[2] = 0x00;
	data[1] = 0x00;
	data[0] = 0x01;
	himaxRegWriteCascade(DATA_LEN_4, addr, DATA_LEN_4, data, SLAVE_ALL);

	return true;
}

static bool AHBI2C_TurnOffWatchdog_Slave(void)
{
	bool retStatus = 0;
	uint8_t addr[DATA_LEN_4] = {0};
	uint8_t data[DATA_LEN_4] = {0};
	int cnt = 0;

	do {
		addr[3] = 0x90;
		addr[2] = 0x00;
		addr[1] = 0x80;
		addr[0] = 0x0C;

		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0xAC;
		data[0] = 0x53;
		himaxRegWriteCascade(DATA_LEN_4, addr, DATA_LEN_4,
			data, SLAVE_ALL);
		himaxRegReadCascade(data, DATA_LEN_4, addr, DATA_LEN_4, SLAVE1);

		/*Check WDT*/
		if (data[3] == 0x00
		&& data[2] == 0x00
		&& data[1] == 0xAC
		&& data[0] == 0x53)
			break;

	} while (cnt++ < 100);

	if (cnt >= 100) {
		E("%s: FAIL\n", __func__);
		retStatus = false;
	} else {
		addr[3] = 0x90;
		addr[2] = 0x00;
		addr[1] = 0x80;
		addr[0] = 0x10;

		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0x35;
		data[0] = 0xCA;
		himaxRegWriteCascade(DATA_LEN_4, addr, DATA_LEN_4,
			data, SLAVE_ALL);
	}
	return retStatus;
}

static bool himax_get_ic_Amount(void)
{
	bool result = false;
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[DATA_LEN_4] = {0};
	int cascadeenb;

	tmp_addr[3] = 0x90;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0xEC;
	kp_g_core_fp->fp_register_read(tmp_addr, DATA_LEN_4, tmp_data, false);
	if (result) {
		cascadeenb = (tmp_data[1] >> 2);
		switch (cascadeenb) {
		case 0:
			hx_ic_Amount = 3;
			break;
		case 2:
			hx_ic_Amount = 2;
			break;
		case 3:
			hx_ic_Amount = 1;
			break;
		default:
			result = false;
			break;
		}
	}
	I("hx_ic_Amount : %d\n", hx_ic_Amount);

	return result;
}

static bool hx83191_sense_off(bool check_en)
{
	bool result = true;
	uint8_t cnt = 0;
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t cMax = 7;
	uint8_t check = 0x87;
	int ret = 0;

	kp_g_core_fp->fp_register_read(
		(*kp_pic_op)->addr_cs_central_state,
		DATA_LEN_4,
		tmp_data,
		0);

	if (tmp_data[0] == 0x05) {

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x5C;
		cnt = 0;
		do {
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0xA5;
			kp_g_core_fp->fp_register_write(tmp_addr, DATA_LEN_4,
				tmp_data, 0);
			msleep(20);
			kp_g_core_fp->fp_register_read(tmp_addr, DATA_LEN_4,
				tmp_data, 0);
			I("%s: Check 9000005C data[0]=%X\n", __func__,
				tmp_data[0]);
			if (cnt++ >= cMax)
				break;
		} while (tmp_data[0] != check);
	}

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
			DATA_LEN_4,
			tmp_data,
			0);

		I("%s: Check enter_save_mode data[0]=%X\n", __func__,
			tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			return true;
		} else if (cnt == 6) {
			usleep_range(10000, 11000);
#ifdef HX_RST_PIN_FUNC
			kp_g_core_fp->fp_ic_reset(false, false);
#else
			kp_g_core_fp->fp_system_reset();
#endif
		}

	} while (cnt++ < 15);

	return result;
}

static void hx83191_func_re_init(void)
{
	kp_g_core_fp->fp_sense_off = hx83191_sense_off;
	kp_g_core_fp->fp_chip_init = hx83191_chip_init;
	kp_g_core_fp->fp_slave_tcon_reset = AHBI2C_TCON_Reset_Slave;
	kp_g_core_fp->fp_slave_adc_reset_slave = AHBI2C_ADC_Reset_Slave;
	kp_g_core_fp->fp_slave_wdt_off_slave = AHBI2C_TurnOffWatchdog_Slave;
}

static void hx83191_reg_re_init(void)
{
	kp_himax_in_parse_assign_cmd(hx83191_fw_addr_fw_mode_status,
			(*kp_pfw_op)->addr_fw_mode_status,
			sizeof((*kp_pfw_op)->addr_fw_mode_status));
	kp_himax_in_parse_assign_cmd(hx83191_fw_addr_sorting_mode_en,
			(*kp_pfw_op)->addr_sorting_mode_en,
			sizeof((*kp_pfw_op)->addr_sorting_mode_en));
	kp_himax_in_parse_assign_cmd(hx83191_fw_addr_set_frame_addr,
			(*kp_pfw_op)->addr_set_frame_addr,
			sizeof((*kp_pfw_op)->addr_set_frame_addr));
	kp_himax_in_parse_assign_cmd(hx83191_fw_addr_raw_out_sel,
			(*kp_pfw_op)->addr_raw_out_sel,
			sizeof((*kp_pfw_op)->addr_raw_out_sel));
	kp_himax_in_parse_assign_cmd(hx83191_driver_addr_fw_define_flash_reload,
			(*kp_pdriver_op)->addr_fw_define_flash_reload,
			sizeof((*kp_pdriver_op)->addr_fw_define_flash_reload));
	kp_himax_in_parse_assign_cmd(
			hx83191_driver_addr_fw_define_2nd_flash_reload,
			(*kp_pdriver_op)->addr_fw_define_2nd_flash_reload,
			sizeof((*kp_pdriver_op)->addr_fw_define_2nd_flash_reload));
	kp_himax_in_parse_assign_cmd(
			hx83191_driver_addr_fw_define_rxnum_txnum_maxpt,
			(*kp_pdriver_op)->addr_fw_define_rxnum_txnum_maxpt,
			sizeof((*kp_pdriver_op)->addr_fw_define_rxnum_txnum_maxpt));
	kp_himax_in_parse_assign_cmd(hx83191_data_df_rx,
			(*kp_pdriver_op)->data_df_rx,
			sizeof((*kp_pdriver_op)->data_df_rx));
	kp_himax_in_parse_assign_cmd(hx83191_data_df_tx,
			(*kp_pdriver_op)->data_df_tx,
			sizeof((*kp_pdriver_op)->data_df_tx));
	kp_himax_in_parse_assign_cmd(hx83191_data_df_x_res,
			(*kp_pdriver_op)->data_df_x_res,
			sizeof((*kp_pdriver_op)->data_df_x_res));
	kp_himax_in_parse_assign_cmd(hx83191_data_df_y_res,
			(*kp_pdriver_op)->data_df_y_res,
			sizeof((*kp_pdriver_op)->data_df_y_res));
}

static bool hx83191_chip_detect(void)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t tmp_addr[DATA_LEN_4] = {0};
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

	hx83191_reg_re_init();
	hx83191_func_re_init();

	kp_g_core_fp->fp_sense_off(false);

	for (i = 0; i < 5; i++) {
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xD0;
		kp_g_core_fp->fp_register_read(tmp_addr, DATA_LEN_4,
			tmp_data, 0);
		I("%s:Read driver IC ID = %X,%X,%X\n", __func__,
				tmp_data[3], tmp_data[2], tmp_data[1]);

		if ((tmp_data[3] == 0x83)
		&& (tmp_data[2] == 0x19)
		&& (tmp_data[1] == 0x1a)) {
			strlcpy((*kp_private_ts)->chip_name,
				HX_83191A_SERIES_PWON, 30);
			I("%s:IC name = %s\n", __func__,
				(*kp_private_ts)->chip_name);
			I("Himax IC package %x%x%x in\n", tmp_data[3],
				tmp_data[2], tmp_data[1]);

			ret_data = true;
			himax_get_ic_Amount();
			return ret_data;
		}
	}
	ret_data = false;
	E("%s:Read driver ID register Fail:\n", __func__);
	E("Could NOT find Himax Chipset\n");
	E("Please check 1.VCCD,VCCA,VSP,VSN\n");
	E("2.LCM_RST,TP_RST\n");
	E("3.Power On Sequence\n");

	return ret_data;
}

DECLARE(HX_MOD_KSYM_HX83191);

static int himax_hx83191_probe(void)
{
	I("%s:Enter\n", __func__);
	himax_add_chip_dt(hx83191_chip_detect);

	return 0;
}

static int himax_hx83191_remove(void)
{
	free_chip_dt_table();
	return 0;
}

static int __init himax_hx83191_init(void)
{
	int ret = 0;

	I("%s\n", __func__);
	ret = himax_hx83191_probe();
	return 0;
}

static void __exit himax_hx83191_exit(void)
{
	himax_hx83191_remove();
}

module_init(himax_hx83191_init);
module_exit(himax_hx83191_exit);

MODULE_DESCRIPTION("HIMAX HX83191 touch driver");
MODULE_LICENSE("GPL");

