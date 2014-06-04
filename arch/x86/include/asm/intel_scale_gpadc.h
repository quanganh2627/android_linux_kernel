#ifndef __INTEL_SCALE_GPADC_H__
#define __INTEL_SCALE_GPADC_H__

#define SCALE_CH_NUM	6

struct iio_dev;

enum scale_thermistor_type {
	NTC_10K = 0,
	NTC_47K,
};

struct channel_thrms_map {
	int chan_num;
	char chan_name[24];
	enum scale_thermistor_type thrms;
};

struct temp_lookup {
	int adc_val;
	int temp;
	int temp_err;
};

struct intel_scale_gpadc_platform_data {
	int channel_num;
	struct iio_map *gpadc_iio_maps;
	struct iio_chan_spec *gpadc_channels;
	struct channel_thrms_map *scale_chan_map;
	int (*pmic_adc_temp_conv)(int, int *, int);
};

#define TEMP_LOOKUP_TBL_SIZE 166

struct channel_lookup_map {
	enum scale_thermistor_type thrms;
	struct temp_lookup adc_tbl[TEMP_LOOKUP_TBL_SIZE];
};

enum scale_adc_channels { PMIC_DIE, BAT0, BAT1, SYS0, SYS1, SYS2, };

static struct channel_lookup_map thrms_lookup[] = {
	{ NTC_10K,
		{
			{ 0x35, 125, 0},
			{ 0x36, 124, 0},
			{ 0x37, 123, 0},
			{ 0x38, 122, 0},
			{ 0x3A, 121, 0},
			{ 0x3B, 120, 0},
			{ 0x3D, 119, 0},
			{ 0x3E, 118, 0},
			{ 0x40, 117, 0},
			{ 0x41, 116, 0},
			{ 0x43, 115, 0},
			{ 0x44, 114, 0},
			{ 0x46, 113, 0},
			{ 0x48, 112, 0},
			{ 0x49, 111, 0},
			{ 0x4C, 110, 0},
			{ 0x4E, 109, 0},
			{ 0x50, 108, 0},
			{ 0x52, 107, 0},
			{ 0x54, 106, 0},
			{ 0x56, 105, 0},
			{ 0x58, 104, 0},
			{ 0x5A, 103, 0},
			{ 0x5D, 102, 0},
			{ 0x5F, 101, 0},
			{ 0x61, 100, 0},
			{ 0x64, 99, 0},
			{ 0x67, 98, 0},
			{ 0x69, 97, 0},
			{ 0x6C, 96, 0},
			{ 0x6F, 95, 0},
			{ 0x72, 94, 0},
			{ 0x75, 93, 0},
			{ 0x78, 92, 0},
			{ 0x7B, 91, 0},
			{ 0x7F, 90, 0},
			{ 0x82, 89, 0},
			{ 0x86, 88, 0},
			{ 0x89, 87, 0},
			{ 0x8D, 86, 0},
			{ 0x91, 85, 0},
			{ 0x95, 84, 0},
			{ 0x99, 83, 0},
			{ 0x9E, 82, 0},
			{ 0xA2, 81, 0},
			{ 0xA7, 80, 0},
			{ 0xAC, 79, 0},
			{ 0xB1, 78, 0},
			{ 0xB6, 77, 0},
			{ 0xBB, 76, 0},
			{ 0xC0, 75, 0},
			{ 0xC6, 74, 0},
			{ 0xCC, 73, 0},
			{ 0xD2, 72, 0},
			{ 0xD8, 71, 0},
			{ 0xDF, 70, 0},
			{ 0xE5, 69, 0},
			{ 0xEC, 68, 0},
			{ 0xF4, 67, 0},
			{ 0xFB, 66, 0},
			{ 0x103, 65, 0},
			{ 0x10B, 64, 0},
			{ 0x113, 63, 0},
			{ 0x11B, 62, 0},
			{ 0x124, 61, 0},
			{ 0x12D, 60, 0},
			{ 0x137, 59, 0},
			{ 0x141, 58, 0},
			{ 0x14C, 57, 0},
			{ 0x156, 56, 0},
			{ 0x162, 55, 0},
			{ 0x16D, 54, 0},
			{ 0x179, 53, 0},
			{ 0x186, 52, 0},
			{ 0x193, 51, 0},
			{ 0x1A0, 50, 0},
			{ 0x1AE, 49, 0},
			{ 0x1BD, 48, 0},
			{ 0x1CC, 47, 0},
			{ 0x1DB, 46, 0},
			{ 0x1EC, 45, 0},
			{ 0x1FD, 44, 0},
			{ 0x20E, 43, 0},
			{ 0x221, 42, 0},
			{ 0x234, 41, 0},
			{ 0x247, 40, 0},
			{ 0x25C, 39, 0},
			{ 0x271, 38, 0},
			{ 0x288, 37, 0},
			{ 0x29F, 36, 0},
			{ 0x2B7, 35, 0},
			{ 0x2D0, 34, 0},
			{ 0x2EA, 33, 0},
			{ 0x305, 32, 0},
			{ 0x322, 31, 0},
			{ 0x33F, 30, 0},
			{ 0x35E, 29, 0},
			{ 0x37F, 28, 0},
			{ 0x3A0, 27, 0},
			{ 0x3C3, 26, 0},
			{ 0x3E8, 25, 0},
			{ 0x40E, 24, 0},
			{ 0x436, 23, 0},
			{ 0x45F, 22, 0},
			{ 0x48B, 21, 0},
			{ 0x4B8, 20, 0},
			{ 0x4E7, 19, 0},
			{ 0x519, 18, 0},
			{ 0x54D, 17, 0},
			{ 0x583, 16, 0},
			{ 0x5BB, 15, 0},
			{ 0x5F7, 14, 0},
			{ 0x635, 13, 0},
			{ 0x675, 12, 0},
			{ 0x6B9, 11, 0},
			{ 0x701, 10, 0},
			{ 0x74B, 9, 0},
			{ 0x799, 8, 0},
			{ 0x7EB, 7, 0},
			{ 0x840, 6, 0},
			{ 0x89A, 5, 0},
			{ 0x8F8, 4, 0},
			{ 0x95B, 3, 0},
			{ 0x9C3, 2, 0},
			{ 0xA30, 1, 0},
			{ 0xAA2, 0, 0},
			{ 0xB1A, -1, 0},
			{ 0xB99, -2, 0},
			{ 0xC1E, -3, 0},
			{ 0xCAA, -4, 0},
			{ 0xD3D, -5, 0},
			{ 0xDD8, -6, 0},
			{ 0xE7B, -7, 0},
			{ 0xF27, -8, 0},
			{ 0xFDC, -9, 0},
			{ 0x109B, -10, 0},
			{ 0x1163, -11, 0},
			{ 0x1235, -12, 0},
			{ 0x1313, -13, 0},
			{ 0x13FE, -14, 0},
			{ 0x14F5, -15, 0},
			{ 0x15FA, -16, 0},
			{ 0x170D, -17, 0},
			{ 0x1830, -18, 0},
			{ 0x1963, -19, 0},
			{ 0x1AA8, -20, 0},
			{ 0x1BFF, -21, 0},
			{ 0x1D6B, -22, 0},
			{ 0x1EEC, -23, 0},
			{ 0x2084, -24, 0},
			{ 0x2233, -25, 0},
			{ 0x23FD, -26, 0},
			{ 0x25E1, -27, 0},
			{ 0x27E3, -28, 0},
			{ 0x2A04, -29, 0},
			{ 0x2C46, -30, 0},
			{ 0x2EAD, -31, 0},
			{ 0x313A, -32, 0},
			{ 0x33EF, -33, 0},
			{ 0x36D1, -34, 0},
			{ 0x39E1, -35, 0},
			{ 0x3D23, -36, 0},
			{ 0x409B, -37, 0},
			{ 0x444C, -38, 0},
			{ 0x483B, -39, 0},
			{ 0x4C6D, -40, 0},
		},
	},
};

#endif
