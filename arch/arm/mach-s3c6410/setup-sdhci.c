/* linux/arch/arm/mach-s3c6410/setup-sdhci.c
 *
 * Copyright 2008 Simtec Electronics
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C6410 - Helper functions for settign up SDHCI device(s) (HSMMC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>

//#include <mach/gpio.h>
//#include <plat/gpio-cfg.h>
//#include <plat/regs-sdhci.h>
//#include <plat/sdhci.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <plat/regs-sdhci.h>
#include <plat/sdhci.h>

/* clock sources for the mmc bus clock, order as for the ctrl2[5..4] */

char *s3c6410_hsmmc_clksrcs[4] = {
	[0] = "hsmmc",
	[1] = "hsmmc",
	[2] = "mmc_bus",
	/* [3] = "48m", - note not succesfully used yet */
};

void s3c6410_setup_sdhci0_cfg_gpio(struct platform_device *dev, int width)
{
	unsigned int gpio;
	unsigned int end;

	end = S3C64XX_GPG(2 + width);

	/* Set all the necessary GPG pins to special-function 0 */
	for (gpio = S3C64XX_GPG(0); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
#if 0
	s3c_gpio_setpull(S3C64XX_GPG(6), S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(S3C64XX_GPG(6), S3C_GPIO_SFN(2));
#endif
}

void s3c6410_setup_sdhci0_cfg_card(struct platform_device *dev,
				    void __iomem *r,
				    struct mmc_ios *ios,
				    struct mmc_card *card)
{
	u32 ctrl2, ctrl3 = 0;

	/* don't need to alter anything acording to card-type */

	writel(S3C64XX_SDHCI_CONTROL4_DRIVE_9mA, r + S3C64XX_SDHCI_CONTROL4);

	/* finally, we don't need to add delay values in HS-MMC interface.
	 * all delay values are removed. by scsuh.
	 */
	ctrl2 = readl(r + S3C_SDHCI_CONTROL2);
	ctrl2 &= S3C_SDHCI_CTRL2_SELBASECLK_MASK;
	ctrl2 |= (S3C64XX_SDHCI_CTRL2_ENSTAASYNCCLR |
		  S3C64XX_SDHCI_CTRL2_ENCMDCNFMSK |
		  S3C_SDHCI_CTRL2_DFCNT_NONE |
		  S3C_SDHCI_CTRL2_ENCLKOUTHOLD);

	writel(ctrl2, r + S3C_SDHCI_CONTROL2);
	writel(ctrl3, r + S3C_SDHCI_CONTROL3);
}


void s3c6410_setup_sdhci1_cfg_gpio(struct platform_device *dev, int width)
{
	unsigned int gpio;
	unsigned int end;

	end = S3C64XX_GPH(2 + width);

	/* Set all the necessary GPG pins to special-function 0 */
	for (gpio = S3C64XX_GPH(0); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

#if 1 /* only ch0 will have card detection pin. by scsuh. */
	s3c_gpio_setpull(S3C64XX_GPG(6), S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(S3C64XX_GPG(6), S3C_GPIO_SFN(3));
#endif
}

void s3c6410_setup_sdhci1_cfg_card(struct platform_device *dev,
				    void __iomem *r,
				    struct mmc_ios *ios,
				    struct mmc_card *card)
{
	u32 ctrl2, ctrl3 = 0;

	/* don't need to alter anything acording to card-type */

	writel(S3C64XX_SDHCI_CONTROL4_DRIVE_9mA, r + S3C64XX_SDHCI_CONTROL4);

	/* finally, we don't need to add delay values in HS-MMC interface.
	 * all delay values are removed. by scsuh.
	 */
	ctrl2 = readl(r + S3C_SDHCI_CONTROL2);
	ctrl2 &= S3C_SDHCI_CTRL2_SELBASECLK_MASK;
	ctrl2 |= (S3C64XX_SDHCI_CTRL2_ENSTAASYNCCLR |
		  S3C64XX_SDHCI_CTRL2_ENCMDCNFMSK |
		  S3C_SDHCI_CTRL2_DFCNT_NONE |
		  S3C_SDHCI_CTRL2_ENCLKOUTHOLD);

	writel(ctrl2, r + S3C_SDHCI_CONTROL2);
	writel(ctrl3, r + S3C_SDHCI_CONTROL3);
}


void s3c6410_setup_sdhci2_cfg_gpio(struct platform_device *dev, int width)
{
	unsigned int gpio;
	unsigned int end;

	/* CMD */
	s3c_gpio_cfgpin(S3C64XX_GPC(4), S3C_GPIO_SFN(3));
	s3c_gpio_setpull(S3C64XX_GPC(4), S3C_GPIO_PULL_NONE);

	/* CLK */
	s3c_gpio_cfgpin(S3C64XX_GPC(5), S3C_GPIO_SFN(3));
	s3c_gpio_setpull(S3C64XX_GPC(5), S3C_GPIO_PULL_NONE);

	end = S3C64XX_GPH(6+width);

	/* Set all the necessary GPG pins to special-function 0 */
	for (gpio = S3C64XX_GPH(6); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
}

void s3c6410_setup_sdhci2_cfg_card(struct platform_device *dev,
				    void __iomem *r,
				    struct mmc_ios *ios,
				    struct mmc_card *card)
{
	u32 ctrl2, ctrl3 = 0;

	/* don't need to alter anything acording to card-type */

	writel(S3C64XX_SDHCI_CONTROL4_DRIVE_9mA, r + S3C64XX_SDHCI_CONTROL4);

	/* finally, we don't need to add delay values in HS-MMC interface.
	 * all delay values are removed. by scsuh.
	 */
	ctrl2 = readl(r + S3C_SDHCI_CONTROL2);
	ctrl2 &= S3C_SDHCI_CTRL2_SELBASECLK_MASK;
	ctrl2 |= (S3C64XX_SDHCI_CTRL2_ENSTAASYNCCLR |
		  S3C64XX_SDHCI_CTRL2_ENCMDCNFMSK |
		  S3C_SDHCI_CTRL2_DFCNT_NONE |
		  S3C_SDHCI_CTRL2_ENCLKOUTHOLD);

	writel(ctrl2, r + S3C_SDHCI_CONTROL2);
	writel(ctrl3, r + S3C_SDHCI_CONTROL3);
}

static void setup_sdhci0_irq_cd (void)
{
	/* init GPIO as a ext irq */
	s3c_gpio_cfgpin(S3C64XX_GPN(12), S3C_GPIO_SFN(2));
	s3c_gpio_setpull(S3C64XX_GPN(12), S3C_GPIO_PULL_NONE);

	set_irq_type(S3C_EINT(12), IRQ_TYPE_EDGE_BOTH);
}

static uint detect_sdhci0_irq_cd (void)
{
	uint detect;

	detect = readl(S3C64XX_GPNDAT);
	detect &= 0x1000;	/* GPN12 */

	return (!detect);
}

static struct s3c_sdhci_platdata s3c_hsmmc0_platdata = {
	.max_width	= 4,
	.host_caps	= (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED |
				MMC_CAP_SD_HIGHSPEED | MMC_CAP_BOOT_ONTHEFLY),
	.cfg_ext_cd	= setup_sdhci0_irq_cd,
	.detect_ext_cd	= detect_sdhci0_irq_cd,
	.ext_cd		= S3C_EINT(12),
};

void smdk6410_setup_sdhci0 (void)
{
	s3c_sdhci0_set_platdata(&s3c_hsmmc0_platdata);
}

