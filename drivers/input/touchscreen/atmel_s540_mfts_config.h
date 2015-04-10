/*
 * Atmel maXTouch Touchscreen driver
 *

 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/i2c/atmel_s540.h>

static u8 config_t6[] = { 6, 0, 0, 0, 0, 0, 0, 0,};

static u8 config_t38[] = { 38, 0, 0, 0, 0, 0, 0, 0, 0,};

static u8 config_t71[] = { 71, 
	77, 20, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 
};

static u8 config_t7[] = { 7, 48, 7, 5, 67,};

static u8 config_t8[] = { 8, 1, 0, 5, 2, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0,};

static u8 config_t15[] = { 15, 0, 0, 0, 4, 2, 0, 0, 30, 0, 0, 0,};

static u8 config_t18[] = { 18, 0, 0,};

static u8 config_t19[] = { 19, 0, 0, 0, 0, 0, 0,};

static u8 config_t23[] = { 23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

static u8 config_t25[] = { 25, 3, 0, 48, 117, 240, 85, 48, 117, 240, 85,
	48, 117, 240, 85, 200, 160, 15, 160, 15, 160, 15,
};

static u8 config_t40[] = { 40, 0, 0, 0, 0, 0,};

static u8 config_t42[] = { 42, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0,};

static u8 config_t46[] = { 46, 12, 0, 8, 8, 0, 1, 4, 0, 0, 1,};

static u8 config_t47[] = { 47, 9, 20, 60, 254, 5, 30, 10, 150, 2, 32,
	224, 224, 3, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 158, 35, 170, 60,
};

static u8 config_t55[] = { 55, 0, 0, 0, 0, 0, 0, 0,};

static u8 config_t56[] = { 56, 1, 0, 1, 70, 6, 6, 6, 6, 6, 6,
	6, 6, 6, 6, 6, 6, 5, 6, 6, 6,
	6, 6, 6, 6, 6, 6, 6, 6, 6, 5,
	8, 8, 8, 0, 0, 0,
};

static u8 config_t61[] = { 61, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

static u8 config_t65[] = { 65, 1, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,};

static u8 config_t66[] = { 66, 0, 0, 0,};

static u8 config_t69[] = { 69, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

static u8 config_t70[] = { 70, 3, 23, 0, 100, 0, 0, 32, 0, 0, 0,
	3, 21, 0, 100, 0, 0, 32, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

static u8 config_t72[] = { 72, 1, 0, 0, 17, 0, 1, 1, 5, 2, 64,
	10, 10, 8, 5, 16, 100, 13, 64, 120, 0,
	0, 2, 3, 2, 3, 2, 32, 32, 32, 32,
	32, 32, 32, 32, 32, 32, 0, 0, 7, 2,
	1, 2, 3, 2, 14, 16, 48, 48, 48, 48,
	48, 48, 48, 48, 48, 48, 0, 6, 17, 21,
	1, 65, 70, 74, 76, 90, 52, 52, 52, 52,
	52, 48, 48, 48, 48, 48, 0, 15, 0, 5,
};

static u8 config_t78[] = { 78, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

static u8 config_t80[] = { 80, 0, 0, 0, 0, 0, 0, 0,};

static u8 config_t84[] = { 84, 0, 0, 0, 0,};

static u8 config_t100[] = { 100, 
	131, 60, 15, 10, 0, 0, 10, 136, 0, 30,
	45, 1, 1, 255, 14, 20, 45, 0, 0, 0,
	17, 45, 5, 5, 111, 8, 35, 65, 6, 3,
	60, 10, 20, 0, 0, 150, 200, 15, 0, 1,
	1, 0, 0, 45, 2, 220, 48, 10, 0, 4,
	0, 0, 0, 10,
};

static u8 config_t101[] = { 101, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
};

static u8 config_t102[] = { 102, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

static u8 config_t103[] = { 103, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
};

static u8 config_t104[] = { 104, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

static u8 config_t105[] = { 105, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, };

static const u8 *config_mfts[] = {
	config_t6,
	config_t38,
	config_t71,
	config_t7,
	config_t8,
	config_t15,
	config_t18,
	config_t19,
	config_t23,
	config_t25,
	config_t40,
	config_t42,
	config_t46,
	config_t47,
	config_t55,
	config_t56,
	config_t61,
	config_t65,
	config_t66,
	config_t69,
	config_t70,
	config_t72,
	config_t78,
	config_t80,
	config_t84,
	config_t100,
	config_t101,
	config_t102,
	config_t103,
	config_t104,
	config_t105,
};

