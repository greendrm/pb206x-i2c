#ifndef I2C_PB206X_PLATFORM_H
#define I2C_PB206X_PLATFORM_H

struct i2c_pb206x_platform_data {
	int (*platform_init)(void); /* Machine dependent init codes.
				     * You should define the pin config
				     * and the init ebi at here if needed
				     */
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

#endif /* I2C_PB206X_PLATFORM_H */
