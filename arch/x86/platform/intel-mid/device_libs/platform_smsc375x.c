
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/extcon/extcon-smsc375x.h>
#include <asm/intel-mid.h>
#include <asm/intel_crystalcove_pwrsrc.h>
#include <asm/intel_bytcr_bcntl.h>
#include <asm/intel_em_config.h>

#define EM_CONFIG_USB_COMP_MASK (1 << 0)
static struct smsc375x_pdata smsc_pdata;

/* dummy functions */
static int smsc375x_enable_vbus(void *ptr)
{
	return 0;
}
static int smsc375x_disable_vbus(void *ptr)
{
	return 0;
}
static int smsc375x_is_vbus_online(void *ptr)
{
	return 0;
}

void *smsc375x_platform_data(void)
{
	int ret = 0;
	struct em_config_oem1_data oem1_data;


	smsc_pdata.enable_vbus = intel_bytcr_boost_enable;
	smsc_pdata.disable_vbus = intel_bytcr_boost_disable;
	smsc_pdata.is_vbus_online = crystal_cove_vbus_on_status;

	memset(&oem1_data, 0, sizeof(struct em_config_oem1_data));
	ret = em_config_get_oem1_data(&oem1_data);
	/* If usb override  is set, then platform can voilate USB spec */
	if ((ret > 0) && !(oem1_data.fpo_0 & EM_CONFIG_USB_COMP_MASK))
		smsc_pdata.charging_compliance_override = false;
	else
		smsc_pdata.charging_compliance_override = true;

	return &smsc_pdata;
}
