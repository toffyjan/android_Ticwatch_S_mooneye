#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include "tpd_custom_mxt1144u.h"

#define TPD_I2C_BUS             0

static struct i2c_board_info __initdata i2c_mxt_tpd[]={
    { I2C_BOARD_INFO("atmel_mxt_ts", (0x4a)),
        .platform_data = &mxt_platform_data,
        .irq = 1,
    },
};

static int __init mxt_touch_init(void)
{
	return i2c_register_board_info(TPD_I2C_BUS, i2c_mxt_tpd, 1);
}

device_initcall(mxt_touch_init);

MODULE_DESCRIPTION("MTK Power Management Init Driver");
MODULE_LICENSE("GPL");
