/*
 *
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __M10MO_ATOMISP_HEADER__
#define __M10MO_ATOMISP_HEADER__

struct m10mo_atomisp_spi_platform_data {
	void (*setup)(struct m10mo_atomisp_spi_platform_data *data);
	int spi_enabled;
	int spi_bus_num;
	int spi_cs_gpio;
	int spi_speed_hz;
	int spi_clock_flis;
	int spi_dataout_flis;
	int spi_datain_flis;
	int spi_cs_flis;
};

#endif

