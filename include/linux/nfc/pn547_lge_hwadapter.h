#ifndef _PN547_LGE_HWADAPTER_H_
#define _PN547_LGE_HWADAPTER_H_

#include <linux/nfc/pn547_lge.h>

#include <linux/of_gpio.h>

#ifdef CONFIG_LGE_NFC_USE_PMIC
#include <linux/clk.h>
#include "../../arch/arm/mach-msm/clock-rpm.h"

#define D1_ID		 2
DEFINE_CLK_RPM_SMD_XO_BUFFER(cxo_d1, cxo_d1_a, D1_ID);
DEFINE_CLK_RPM_SMD_XO_BUFFER_PINCTRL(cxo_d1_pin, cxo_d1_a_pin, D1_ID);
#endif

int pn547_get_hw_revision(void);
unsigned int pn547_get_irq_pin(struct pn547_dev *dev);
int pn547_gpio_to_irq(struct pn547_dev *dev);
void pn547_gpio_enable(struct pn547_dev *pn547_dev);
void pn547_shutdown_cb(struct pn547_dev *pn547_dev);

#ifdef CONFIG_LGE_NFC_USE_PMIC
void pn547_get_clk_source(struct pn547_dev *pn547_dev);
#endif

void pn547_parse_dt(struct device *dev, struct pn547_dev *pn547_dev);

#endif /* _PN547_LGE_HWADAPTER_H_ */
