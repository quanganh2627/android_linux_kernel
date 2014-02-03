/*
 * intel_em_config.c : Intel EM configuration setup code
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: Kotakonda, Venkataramana <venkataramana.kotakonda@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <asm/intel_em_config.h>

#define EM_CONFIG_OEM0_NAME "OEM0"
#define EM_CONFIG_OEM1_NAME "OEM1"


static int em_config_get_acpi_table(char *name, void *data, int data_size)
{
	struct acpi_table_header *acpi_tbl = NULL;
	acpi_size tbl_size;
	acpi_status status;
	int ret = 0;
	int hdr_size = sizeof(struct acpi_table_header);

	status = acpi_get_table_with_size(name , 0,
					&acpi_tbl, &tbl_size);
	if (ACPI_SUCCESS(status)) {
		pr_info("EM:%s  table found, size=%d\n", name, tbl_size);
		if (tbl_size < (data_size + hdr_size)) {
			pr_err("EM:%s table incomplete!!\n", name);
		} else {
			memcpy(data, ((char *)acpi_tbl) + hdr_size, data_size);
			ret = data_size;
		}
	} else {
		pr_err("EM:%s table not found!!\n", name);
	}

	return ret;
}

int em_config_get_oem0_data(struct em_config_oem0_data *data)
{
	return em_config_get_acpi_table(EM_CONFIG_OEM0_NAME,
					data, sizeof(struct em_config_oem0_data));
}
EXPORT_SYMBOL(em_config_get_oem0_data);

int em_config_get_oem1_data(struct em_config_oem1_data *data)
{
	return em_config_get_acpi_table(EM_CONFIG_OEM1_NAME,
					data, sizeof(struct em_config_oem1_data));
}
EXPORT_SYMBOL(em_config_get_oem1_data);

int em_config_get_charge_profile(struct ps_pse_mod_prof *chrg_prof)
{
	struct em_config_oem0_data oem0_data;
	int i, ret = 0;
	int ranges = EM_CONFIG_BATT_TEMP_NR_RNG;

	if (chrg_prof == NULL)
		return 0;
	ret = em_config_get_oem0_data(&oem0_data);

	if (ret <= 0)
		goto chrg_prof_error;

	memcpy(chrg_prof->batt_id, oem0_data.batt_id, BATTID_STR_LEN);
	chrg_prof->battery_type = oem0_data.batt_type;
	chrg_prof->capacity = oem0_data.capacity;
	chrg_prof->voltage_max = oem0_data.volt_max;
	chrg_prof->temp_mon_ranges =
			oem0_data.temp_mon_ranges;

	if (oem0_data.temp_mon_ranges < EM_CONFIG_BATT_TEMP_NR_RNG)
		ranges = oem0_data.temp_mon_ranges;

	/* Copy the temperature ranges */
	for (i = 0; i < ranges; i++) {
		chrg_prof->temp_mon_range[i].temp_up_lim =
		oem0_data.temp_mon_range[i].temp_up_lim;

		chrg_prof->temp_mon_range[i].full_chrg_vol =
		oem0_data.temp_mon_range[i].full_chrg_vol;

		chrg_prof->temp_mon_range[i].full_chrg_cur =
		oem0_data.temp_mon_range[i].full_chrg_cur;

		chrg_prof->temp_mon_range[i].maint_chrg_vol_ll =
		oem0_data.temp_mon_range[i].maint_chrg_vol_ll;

		chrg_prof->temp_mon_range[i].maint_chrg_vol_ul =
		oem0_data.temp_mon_range[i].maint_chrg_vol_ul;

		chrg_prof->temp_mon_range[i].maint_chrg_cur =
		oem0_data.temp_mon_range[i].maint_chrg_cur;
	}

	chrg_prof->temp_low_lim = oem0_data.temp_low_lim;

chrg_prof_error:
	return ret;
}
EXPORT_SYMBOL(em_config_get_charge_profile);
