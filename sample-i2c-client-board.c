#include <linux/kernel.h>
#include <linux/i2c.h>

/* you might add a irq or platform_data if need
 *
 * { I2C_BOARD_INFO("sample_i2c_client", 0x50),
 *   .irq = 100,
 *   .platform_data = &sample_i2c_2_pdata,
 * }
 */
static struct i2c_board_info i2c_2_sample_devs[] __initdata = {
	{ I2C_BOARD_INFO("sample_i2c_client", 0x50), },

};

int machine_init(void)
{
	i2c_register_board_info(2, i2c_2_sample_devs, 
			ARRAY_SIZE(i2c_2_sample_devs));

}
