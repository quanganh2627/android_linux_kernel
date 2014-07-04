/*
 * intel_dcove_regulator.h - Support for dollar cove pmic
 * Copyright (c) 2013, Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef __INTEL_DCOVE_REGULATOR_H_
#define __INTEL_DCOVE_REGULATOR_H_

#include <linux/regulator/driver.h>

enum {
	DCOVE_ID_BUCK1 = 1,
	DCOVE_ID_BUCK2,
	DCOVE_ID_BUCK3,
	DCOVE_ID_BUCK4,
	DCOVE_ID_BUCK5,
	DCOVE_ID_BUCK6,

	DCOVE_ID_LDO1,
	DCOVE_ID_LDO2,
	DCOVE_ID_LDO3,
	DCOVE_ID_LDO4,
	DCOVE_ID_LDO5,	/* ELDO1 */
	DCOVE_ID_LDO6,
	DCOVE_ID_LDO7,
	DCOVE_ID_LDO8,	/* FLDO1 */
	DCOVE_ID_LDO9,
	DCOVE_ID_LDO10,
	DCOVE_ID_LDO11,	/* ALDO1 */
	DCOVE_ID_LDO12,
	DCOVE_ID_LDO13,

	DCOVE_ID_GPIO1,

	DCOVE_ID_MAX,
};

struct dcove_regulator_info {
	struct regulator_desc	desc;
	struct regulator_dev	*regulator;
	struct regulator_init_data *init_data;
	int vol_reg;
	int vol_nbits;
	int vol_shift;
	int enable_reg;		/* enable register base  */
	int enable_bit;
};

#endif
