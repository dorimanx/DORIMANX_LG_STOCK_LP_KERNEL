/* sound/soc/codecs/tpa2015d1.h
 *
 * Copyright (C) 2012 LG Electronics Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __TPA2015D1_H__
#define __TPA2015D1_H__

struct tpa2015d1_platform_data {

	unsigned int gpio_spkamp_en;
	unsigned char *pdev_name;
};

void tpa2015d1_ext_spk_power_amp_enable(u32 on);
extern bool tpa_enabled;
#endif /* __TPA2015D1_H__ */
