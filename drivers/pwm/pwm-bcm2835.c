// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2014 Bart Tanghe <bart.tanghe@thomasmore.be>
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

/* PWM Control */
#define BCM2835PWM_CTL		0x00
#define BCM2835PWM_CTL_MSEN2		BIT(15)
#define BCM2835PWM_CTL_USEF2		BIT(13)
#define BCM2835PWM_CTL_POLA2		BIT(12)
#define BCM2835PWM_CTL_SBIT2		BIT(11)
#define BCM2835PWM_CTL_RPTL2		BIT(10)
#define BCM2835PWM_CTL_MODE2		BIT(9)
#define BCM2835PWM_CTL_PWEN2		BIT(8)
#define BCM2835PWM_CTL_MSEN1		BIT(7)
#define BCM2835PWM_CTL_CLRF1		BIT(6)
#define BCM2835PWM_CTL_USEF1		BIT(5)
#define BCM2835PWM_CTL_POLA1		BIT(4)
#define BCM2835PWM_CTL_SBIT1		BIT(3)
#define BCM2835PWM_CTL_RPTL1		BIT(2)
#define BCM2835PWM_CTL_MODE1		BIT(1)
#define BCM2835PWM_CTL_PWEN1		BIT(0)

/* PWM Channel 1 Range */
#define BCM2835PWM_RNG1		0x10

/* PWM Channel 1 Data */
#define BCM2835PWM_DAT1		0x14

/* PWM Channel 2 Range */
#define BCM2835PWM_RNG2		0x20

/* PWM Channel 2 Data */
#define BCM2835PWM_DAT2		0x24

struct bcm2835_pwm {
	void __iomem *base;
	struct clk *clk;
	unsigned long rate;
	u64 max_period_ns;
};

static inline struct bcm2835_pwm *to_bcm2835_pwm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

struct bcm2835_pwm_wfhw {
	u32 ctl;
	u32 rng;
	u32 dat;
};

static int bcm2835_pwm_round_waveform_tohw(struct pwm_chip *chip,
					   struct pwm_device *pwm,
					   const struct pwm_waveform *wf,
					   void *_wfhw)
{
	struct bcm2835_pwm_wfhw *wfhw = _wfhw;
	struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
	u64 period_ticks, duty_ticks;
	int ret = 0;
	bool inversed = false;

	if (wf->period_length_ns == 0) {
		*wfhw = (struct bcm2835_pwm_wfhw){
			.ctl = 0,
		};

		return 0;
	}

	if (wf->period_length_ns >= pc->max_period_ns)
		period_ticks = U32_MAX;
	else
		period_ticks = DIV_ROUND_DOWN_ULL(wf->period_length_ns * pc->rate,
						  NSEC_PER_SEC);
	if (!period_ticks) {
		period_ticks = 1;
		ret = 1;
	}

	if (wf->duty_length_ns >= pc->max_period_ns)
		duty_ticks = U32_MAX;
	else
		duty_ticks = DIV_ROUND_DOWN_ULL(wf->duty_length_ns * pc->rate,
						NSEC_PER_SEC);

	if (wf->duty_length_ns && wf->duty_offset_ns &&
	    wf->duty_length_ns + wf->duty_offset_ns >= wf->period_length_ns) {
		inversed = true;
		duty_ticks = period_ticks - duty_ticks;
	}

	*wfhw = (struct bcm2835_pwm_wfhw){
		.ctl = BCM2835PWM_CTL_PWEN1 | BCM2835PWM_CTL_MSEN1 |
			(inversed ? BCM2835PWM_CTL_POLA1 : 0),
		.rng = period_ticks,
		.dat = duty_ticks,
	};

	dev_dbg(&chip->dev,
		"pwm#%u: %lld/%lld [+%lld] @%lu -> CTL: %08x, RNG: %08x, DAT: %08x\n",
		pwm->hwpwm, wf->duty_length_ns, wf->period_length_ns, wf->duty_offset_ns,
		pc->rate, wfhw->ctl, wfhw->rng, wfhw->dat);

	return ret;
}

static int bcm2835_pwm_round_waveform_fromhw(struct pwm_chip *chip, struct pwm_device *pwm,
					     const void *_wfhw, struct pwm_waveform *wf)
{
	const struct bcm2835_pwm_wfhw *wfhw = _wfhw;
	struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);

	if (wfhw->ctl & BCM2835PWM_CTL_PWEN1) {
		*wf = (struct pwm_waveform){
			.period_length_ns = DIV64_U64_ROUND_UP((u64)wfhw->rng * NSEC_PER_SEC,
							       pc->rate),
		};

		if (wfhw->ctl & BCM2835PWM_CTL_POLA1) {
			wf->duty_offset_ns = DIV64_U64_ROUND_UP((u64)wfhw->dat * NSEC_PER_SEC,
								pc->rate);
			wf->duty_length_ns =
				DIV64_U64_ROUND_UP((u64)(wfhw->rng - wfhw->dat) * NSEC_PER_SEC,
						   pc->rate);
		} else {
			wf->duty_length_ns = DIV64_U64_ROUND_UP((u64)wfhw->dat * NSEC_PER_SEC,
								pc->rate);
		}
	} else {
		*wf = (struct pwm_waveform){
			.period_length_ns = 0,
		};
	}

	dev_dbg(&chip->dev,
		"pwm#%u: CTL: %08x, RNG: %08x, DAT: %08x @%lu -> %lld/%lld [+%lld]\n",
		pwm->hwpwm, wfhw->ctl, wfhw->rng, wfhw->dat, pc->rate,
		wf->duty_length_ns, wf->period_length_ns, wf->duty_offset_ns);

	return 0;
}

static int bcm2835_pwm_read_waveform(struct pwm_chip *chip, struct pwm_device *pwm,
				     void *_wfhw)
{
	struct bcm2835_pwm_wfhw *wfhw = _wfhw;
	struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
	u32 ctl;

	ctl = readl(pc->base + BCM2835PWM_CTL);
	if (pwm->hwpwm)
		ctl >>= 8;

	wfhw->ctl = ctl;

	wfhw->rng = readl(pc->base + (pwm->hwpwm ? BCM2835PWM_RNG2 : BCM2835PWM_RNG1));
	wfhw->dat = readl(pc->base + (pwm->hwpwm ? BCM2835PWM_DAT2 : BCM2835PWM_DAT1));

	return 0;
}

static int bcm2835_pwm_write_waveform(struct pwm_chip *chip, struct pwm_device *pwm,
				      const void *_wfhw)
{
	const struct bcm2835_pwm_wfhw *wfhw = _wfhw;
	struct bcm2835_pwm *pc = to_bcm2835_pwm(chip);
	u32 ctl;
	u32 ctl_shift = pwm->hwpwm ? 8 : 0;
	u32 ctl_mask = BCM2835PWM_CTL_PWEN1 | BCM2835PWM_CTL_MODE1 | BCM2835PWM_CTL_POLA1 |
		BCM2835PWM_CTL_USEF1 | BCM2835PWM_CTL_MSEN1;

	ctl = readl(pc->base + BCM2835PWM_CTL);
	ctl &= ~(ctl_mask << ctl_shift);
	ctl |= (wfhw->ctl & ctl_mask) << ctl_shift;

	dev_dbg(&chip->dev, "pwm#%u: write CTL: %08x, RNG: %08x, DAT: %08x, actual CTL: %08x\n",
		pwm->hwpwm, wfhw->ctl, wfhw->rng, wfhw->dat, ctl);

	writel(wfhw->rng, pc->base + (pwm->hwpwm ? BCM2835PWM_RNG2 : BCM2835PWM_RNG1));
	writel(wfhw->dat, pc->base + (pwm->hwpwm ? BCM2835PWM_DAT2 : BCM2835PWM_DAT1));
	writel(ctl, pc->base + BCM2835PWM_CTL);

	return 0;
}

static const struct pwm_ops bcm2835_pwm_ops = {
	.sizeof_wfhw = sizeof(struct bcm2835_pwm_wfhw),
	.round_waveform_tohw = bcm2835_pwm_round_waveform_tohw,
	.round_waveform_fromhw = bcm2835_pwm_round_waveform_fromhw,
	.read_waveform = bcm2835_pwm_read_waveform,
	.write_waveform = bcm2835_pwm_write_waveform,
};

static int bcm2835_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_chip *chip;
	struct bcm2835_pwm *pc;
	int ret;

	chip = devm_pwmchip_alloc(dev, 2, sizeof(*pc));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	pc = to_bcm2835_pwm(chip);

	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);

	pc->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(pc->clk))
		return dev_err_probe(dev, PTR_ERR(pc->clk),
				     "clock not found\n");

	ret = devm_clk_rate_exclusive_get(dev, pc->clk);
	if (ret)
		return dev_err_probe(dev, ret,
				     "fail to get exclusive rate\n");

	pc->rate = clk_get_rate(pc->clk);
	if (!pc->rate)
		return dev_err_probe(dev, -EINVAL,
				     "failed to get clock rate\n");

	/*
	 * period_cycles must be a 32 bit value. As all values bigger than that
	 * are mapped to U32_MAX, all period lengths that are shorter than
	 * U32_MAX clock cycles must be converted, the bigger ones are mapped
	 * directly to U32_MAX.
	 * For the shortest period length that maps to U32_MAX we have:
	 *
	 *   floor(period * rate / NSEC_PER_SEC) ≥ U32_MAX
	 * ⇔ period * rate / NSEC_PER_SEC ≥ U32_MAX
	 * ⇔ period * rate ≥ U32_MAX * NSEC_PER_SEC
	 * ⇔ period ≥ (U32_MAX * NSEC_PER_SEC) / rate
	 * ⇔ period ≥ ceil((U32_MAX * NSEC_PER_SEC) / rate)
	 *
	 * As U32_MAX * NSEC_PER_SEC < U64_MAX the intermediate result
	 * period * rate doesn't overflow an u64 with the above inequality.
	 *
	 */
	pc->max_period_ns = DIV_ROUND_UP_ULL((u64)U32_MAX * NSEC_PER_SEC, pc->rate);

	chip->ops = &bcm2835_pwm_ops;
	chip->atomic = true;

	platform_set_drvdata(pdev, pc);

	ret = devm_pwmchip_add(dev, chip);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to add pwmchip\n");

	return 0;
}

static int bcm2835_pwm_suspend(struct device *dev)
{
	struct bcm2835_pwm *pc = dev_get_drvdata(dev);

	clk_disable_unprepare(pc->clk);

	return 0;
}

static int bcm2835_pwm_resume(struct device *dev)
{
	struct bcm2835_pwm *pc = dev_get_drvdata(dev);

	return clk_prepare_enable(pc->clk);
}

static DEFINE_SIMPLE_DEV_PM_OPS(bcm2835_pwm_pm_ops, bcm2835_pwm_suspend,
				bcm2835_pwm_resume);

static const struct of_device_id bcm2835_pwm_of_match[] = {
	{ .compatible = "brcm,bcm2835-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bcm2835_pwm_of_match);

static struct platform_driver bcm2835_pwm_driver = {
	.driver = {
		.name = "bcm2835-pwm",
		.of_match_table = bcm2835_pwm_of_match,
		.pm = pm_ptr(&bcm2835_pwm_pm_ops),
	},
	.probe = bcm2835_pwm_probe,
};
module_platform_driver(bcm2835_pwm_driver);

MODULE_AUTHOR("Bart Tanghe <bart.tanghe@thomasmore.be>");
MODULE_DESCRIPTION("Broadcom BCM2835 PWM driver");
MODULE_LICENSE("GPL v2");
