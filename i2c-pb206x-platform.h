#ifndef I2C_PB206X_PLATFORM_H
#define I2C_PB206X_PLATFORM_H

struct i2c_pb206x_platform_data {
	int (*platform_init)(void); /* Machine dependent init codes.
				     * You should define the pin config
				     * and the init ebi at here if needed
				     */
	u32 iomem;
	u32 speed;
	int gpio_pdn;               /* power down pin */
	int gpio_reset;             /* hw reset (optional) */
	int external_main_clock;    /* 19200 or 24000 (kHz) if needed */
	int level_shift0_en:1;
	int level_shift0_dir:1;
	int level_shift1_en:1;
	int level_shift1_dir:1;
	int select_register:1; /* select external or 'internal+external'
			        * resister for RC oscillator
			        */
	int scl_pulldown:1;

	int sda_pulldown:1;
	u8  scl_pullup;
	u8  sda_pullup;
	u8  ebi_drive_strength;
	u8  sda_drive_strength;
	u8  scl_drive_strength;

};

#ifdef CONFIG_PB206X_I2C_SMBUS_2
export s32 i2c_smbus_read_byte_data_2(struct i2c_client *client, u16 command);
export s32 i2c_smbus_write_byte_data_2(struct i2c_client *client, u16 command,
		u8 value);
export s32 i2c_smbus_read_word_data_2(struct i2c_client *client, u16 command);
export s32 i2c_smbus_write_word_data_2(struct i2c_client *cleint, u16 command,
		u16 value);
export s32 i2c_smbus_read_i2c_block_data_2(struct i2c_client *client,
		u16 command, u8 length, u8 *values);
export s32 i2c_smbus_write_i2c_block_data_2(struct i2c_client *client,
		u16 command, u8 length, u8 *values);
#else
inline s32 i2c_smbus_read_byte_data_2(struct i2c_client *client, u16 command)
{
	return 0;
}
inline s32 i2c_smbus_write_byte_data_2(struct i2c_client *client, u16 command, 
		u8 value)
{
	return 0;
}
inline s32 i2c_smbus_read_word_data_2(struct i2c_client *client, u16 command) 
{
	return 0;
}
inline s32 i2c_smbus_write_word_data_2(struct i2c_client *cleint, u16 command,
		u16 value)
{
	return 0;
}
inline s32 i2c_smbus_read_i2c_block_data_2(struct i2c_client *client,
		u16 command, u8 length, u8 *values)
{
	return 0;
}
inline s32 i2c_smbus_write_i2c_block_data_2(struct i2c_client *client,
		u16 command, u8 length, u8 *values)
{
	return 0;
}
#endif /* I2C_PB206X_PLATFORM_H */
