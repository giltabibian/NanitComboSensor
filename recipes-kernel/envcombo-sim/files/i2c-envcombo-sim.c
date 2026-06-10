// SPDX-License-Identifier: GPL-2.0
// I2C adapter + register simulator for the ENV-COMBO sensor.

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/debugfs.h>

#define ENV_ADDR		0x39
#define WHO_AM_I_VAL		0xEB

#define CFG_ALS_EN		BIT(7)
#define CFG_TEMP_EN		BIT(6)
#define CFG_HUM_EN		BIT(5)
#define CFG_ALS_GAIN_SHIFT	3
#define CFG_ALS_GAIN_MASK	(0x03 << CFG_ALS_GAIN_SHIFT)
#define CFG_ALS_TIME_SHIFT	1
#define CFG_ALS_TIME_MASK	(0x03 << CFG_ALS_TIME_SHIFT)

#define INT_CFG_EN		BIT(7)
#define INT_CFG_LATCH		BIT(6)
#define INT_CFG_POL		BIT(5)

#define PWR_MODE_MASK		0x03
#define PWR_OFF			0x00
#define PWR_SLEEP		0x01
#define PWR_ONE_SHOT		0x02
#define PWR_CONTINUOUS		0x03

enum {
	R_WHO_AM_I		= 0x00,
	R_TEMP_MSB		= 0x01,
	R_TEMP_LSB		= 0x02,
	R_HUMIDITY		= 0x03,
	R_ALS_MSB		= 0x04,
	R_ALS_LSB		= 0x05,
	R_CFG			= 0x06,
	R_INT_CFG		= 0x07,
	R_ALS_TH_LOW_MSB	= 0x08,
	R_ALS_TH_LOW_LSB	= 0x09,
	R_ALS_TH_HIGH_MSB	= 0x0A,
	R_ALS_TH_HIGH_LSB	= 0x0B,
	R_STATUS		= 0x0C,
	R_CAL_TOFF_MSB		= 0x0D,
	R_CAL_TOFF_LSB		= 0x0E,
	R_CAL_HOFF		= 0x0F,
	R_CAL_AGAIN		= 0x10,
	R_CAL_ATIME		= 0x11,
	R_PWR_MODE		= 0x12,
	REG_COUNT,
};

struct envcombo {
	struct i2c_adapter adap;
	struct i2c_client *client;
	struct irq_domain *domain;
	unsigned int irq;

	u8 regs[REG_COUNT];
	u16 als_th_low;
	u16 als_th_high;

	s16 latched_temp;
	u8  latched_hum;
	u16 latched_als;

	bool temp_rdy;
	bool hum_rdy;
	bool als_rdy;
	bool als_int_pending;

	bool als_threshold_active;

	struct delayed_work work;
	struct work_struct irq_work;
	spinlock_t lock;

	struct dentry *debugfs_dir;
};

static struct envcombo *edev;

static s16 sim_temp(s16 calib_off)
{
	u32 sec = (jiffies / HZ) % 60;

	return 1500 + (s16)(sec * 2000 / 60) + calib_off;
}

static u8 sim_hum(s8 calib_off)
{
	u32 sec = (jiffies / HZ) % 40;
	int raw = (int)(((sec * 100) / 40) * 2) + calib_off;

	return (u8)clamp(raw, 0, 255);
}

static const u8 als_gain_table[] = { 1, 4, 16, 64 };
static const u16 als_time_table[] = { 50, 100, 200, 400 };

static u16 sim_als(struct envcombo *st)
{
	u8 cfg = st->regs[R_CFG];
	u8 gain_idx = (cfg & CFG_ALS_GAIN_MASK) >> CFG_ALS_GAIN_SHIFT;
	u8 time_idx = (cfg & CFG_ALS_TIME_MASK) >> CFG_ALS_TIME_SHIFT;
	u16 gain = (u16)als_gain_table[gain_idx] * st->regs[R_CAL_AGAIN];
	u16 time_ms = st->regs[R_CAL_ATIME] ? st->regs[R_CAL_ATIME]
					     : als_time_table[time_idx];
	u32 ticks = (jiffies / (HZ / 20)) % 1000;

	if (time_ms < 50)
		time_ms = 50;
	return (u16)DIV_ROUND_CLOSEST(ticks * gain, time_ms / 50);
}

static bool ec_do_conversion(struct envcombo *st)
{
	u8 cfg = st->regs[R_CFG];
	bool new_bits = false;

	if (cfg & CFG_TEMP_EN) {
		s16 calib = (s16)((st->regs[R_CAL_TOFF_MSB] << 8) |
				   st->regs[R_CAL_TOFF_LSB]);
		st->latched_temp = sim_temp(calib);
		new_bits |= !st->temp_rdy;
		st->temp_rdy = true;
	}

	if (cfg & CFG_HUM_EN) {
		st->latched_hum = sim_hum((s8)st->regs[R_CAL_HOFF]);
		new_bits |= !st->hum_rdy;
		st->hum_rdy = true;
	}

	if (cfg & CFG_ALS_EN) {
		st->latched_als = sim_als(st);
		new_bits |= !st->als_rdy;
		st->als_rdy = true;
	}

	return new_bits;
}

static bool ec_check_als_threshold(struct envcombo *st)
{
	bool new_bit = false;
	bool crossed;

	if (!(st->regs[R_CFG] & CFG_ALS_EN))
		return false;

	crossed = (st->latched_als < st->als_th_low ||
		   st->latched_als > st->als_th_high);

	if (crossed && !st->als_threshold_active) {
		st->als_threshold_active = true;
		new_bit = !st->als_int_pending;
		st->als_int_pending = true;
	} else if (!crossed && st->als_threshold_active) {
		st->als_threshold_active = false;
		if (!(st->regs[R_INT_CFG] & INT_CFG_LATCH))
			st->als_int_pending = false;
	}

	return new_bit;
}

static bool ec_polarity_matches(struct envcombo *st)
{
	unsigned int type = irq_get_trigger_type(st->irq);
	bool active_high = !!(st->regs[R_INT_CFG] & INT_CFG_POL);

	if (active_high)
		return !!(type & (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_EDGE_RISING));
	else
		return !!(type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_EDGE_FALLING));
}

static void ec_work_fn(struct work_struct *work)
{
	struct envcombo *st = container_of(to_delayed_work(work),
					   struct envcombo, work);
	bool fire_irq = false;
	bool continuous;

	scoped_guard(spinlock, &st->lock) {
		continuous = (st->regs[R_PWR_MODE] & PWR_MODE_MASK) == PWR_CONTINUOUS;
		if (continuous) {
			bool new_status_bits = ec_do_conversion(st);

			new_status_bits |= ec_check_als_threshold(st);

			if ((st->regs[R_INT_CFG] & INT_CFG_EN) &&
			    new_status_bits && ec_polarity_matches(st))
				fire_irq = true;
		}
	}

	if (fire_irq)
		handle_nested_irq(st->irq);

	if (continuous)
		schedule_delayed_work(&st->work, msecs_to_jiffies(200));
}

static u8 ec_read_reg(struct envcombo *st, u8 reg)
{
	guard(spinlock)(&st->lock);

	switch (reg) {
	case R_WHO_AM_I:
		return WHO_AM_I_VAL;
	case R_TEMP_MSB:
	case R_TEMP_LSB:
		return (reg == R_TEMP_MSB) ? ((st->latched_temp >> 8) & 0xFF)
					   : (st->latched_temp & 0xFF);
	case R_HUMIDITY:
		return st->latched_hum;
	case R_ALS_MSB:
	case R_ALS_LSB:
		return (reg == R_ALS_MSB) ? ((st->latched_als >> 8) & 0xFF)
					  : (st->latched_als & 0xFF);
	case R_ALS_TH_LOW_MSB:
		return (st->als_th_low >> 8) & 0xFF;
	case R_ALS_TH_LOW_LSB:
		return st->als_th_low & 0xFF;
	case R_ALS_TH_HIGH_MSB:
		return (st->als_th_high >> 8) & 0xFF;
	case R_ALS_TH_HIGH_LSB:
		return st->als_th_high & 0xFF;
	case R_STATUS: {
		u8 v = (st->als_int_pending ? BIT(0) : 0) |
		       (st->temp_rdy        ? BIT(1) : 0) |
		       (st->hum_rdy         ? BIT(2) : 0) |
		       (st->als_rdy         ? BIT(3) : 0);
		st->als_int_pending = false;
		st->temp_rdy = false;
		st->hum_rdy = false;
		st->als_rdy = false;
		return v;
	}
	default:
		return reg < REG_COUNT ? st->regs[reg] : 0;
	}
}

static void ec_write_reg(struct envcombo *st, u8 reg, u8 val,
			  bool *one_shot_irq, bool *start_continuous)
{
	guard(spinlock)(&st->lock);

	switch (reg) {
	case R_WHO_AM_I:
	case R_STATUS:
		break;
	case R_ALS_TH_LOW_MSB:
		st->als_th_low = ((u16)val << 8) | (st->als_th_low & 0xFF);
		break;
	case R_ALS_TH_LOW_LSB:
		st->als_th_low = (st->als_th_low & 0xFF00) | val;
		break;
	case R_ALS_TH_HIGH_MSB:
		st->als_th_high = ((u16)val << 8) | (st->als_th_high & 0xFF);
		break;
	case R_ALS_TH_HIGH_LSB:
		st->als_th_high = (st->als_th_high & 0xFF00) | val;
		break;
	case R_PWR_MODE: {
		u8 mode;
		bool new_status_bits = false;

		st->regs[reg] = val;

		mode = val & PWR_MODE_MASK;
		if (mode == PWR_ONE_SHOT) {
			new_status_bits |= ec_do_conversion(st);
			new_status_bits |= ec_check_als_threshold(st);
			st->regs[R_PWR_MODE] = PWR_SLEEP;
			*one_shot_irq |= new_status_bits;
		} else if (mode == PWR_CONTINUOUS) {
			*start_continuous = true;
		}
		break;
	}
	default:
		if (reg < REG_COUNT)
			st->regs[reg] = val;
	}
}

static void ec_irq_work_fn(struct work_struct *work)
{
	struct envcombo *st = container_of(work, struct envcombo, irq_work);

	handle_nested_irq(st->irq);
}

static int ec_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			  int nmsg)
{
	struct envcombo *st = container_of(adap, struct envcombo, adap);
	bool one_shot_irq = false;
	bool start_continuous = false;
	u8 reg;
	int i;

	for (i = 0; i < nmsg; i++) {
		if (msgs[i].addr != ENV_ADDR)
			return -EIO;
	}

	if (nmsg == 2 && !(msgs[0].flags & I2C_M_RD) &&
	    (msgs[1].flags & I2C_M_RD) && msgs[0].len == 1) {
		reg = msgs[0].buf[0];
		for (i = 0; i < msgs[1].len && reg < REG_COUNT; i++, reg++)
			msgs[1].buf[i] = ec_read_reg(st, reg);
		return nmsg;
	}

	if (nmsg == 1 && !(msgs[0].flags & I2C_M_RD) && msgs[0].len >= 2) {
		reg = msgs[0].buf[0];
		for (i = 1; i < msgs[0].len && reg < REG_COUNT; i++, reg++)
			ec_write_reg(st, reg, msgs[0].buf[i],
				     &one_shot_irq, &start_continuous);

		if (one_shot_irq && (st->regs[R_INT_CFG] & INT_CFG_EN) &&
		    ec_polarity_matches(st))
			schedule_work(&st->irq_work);

		if (start_continuous)
			schedule_delayed_work(&st->work,
					      msecs_to_jiffies(200));

		return nmsg;
	}

	return -EIO;
}

static u32 ec_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA |
	       I2C_FUNC_SMBUS_WORD_DATA;
}

static const struct i2c_algorithm ec_algo = {
	.master_xfer	= ec_master_xfer,
	.functionality	= ec_func,
};

static void sim_irq_mask(struct irq_data *d) { }
static void sim_irq_unmask(struct irq_data *d) { }

static int sim_irq_set_type(struct irq_data *d, unsigned int type)
{
	return 0;
}

static struct irq_chip sim_irq_chip = {
	.name		= "envcombo-sim",
	.irq_mask	= sim_irq_mask,
	.irq_unmask	= sim_irq_unmask,
	.irq_set_type	= sim_irq_set_type,
};

static int sim_irq_map(struct irq_domain *d, unsigned int virq,
		       irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(virq, &sim_irq_chip, handle_simple_irq);
	irq_set_nested_thread(virq, true);
	irq_set_noprobe(virq);
	return 0;
}

static const struct irq_domain_ops sim_irq_domain_ops = {
	.map = sim_irq_map,
};

static ssize_t debugfs_regs_read(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct envcombo *st = file->private_data;
	u8 buf[REG_COUNT];

	if (*ppos >= REG_COUNT)
		return 0;
	if (*ppos + count > REG_COUNT)
		count = REG_COUNT - *ppos;

	scoped_guard(spinlock, &st->lock) {
		memcpy(buf, st->regs, REG_COUNT);

		buf[R_TEMP_MSB] = (st->latched_temp >> 8) & 0xFF;
		buf[R_TEMP_LSB] = st->latched_temp & 0xFF;
		buf[R_HUMIDITY] = st->latched_hum;
		buf[R_ALS_MSB] = (st->latched_als >> 8) & 0xFF;
		buf[R_ALS_LSB] = st->latched_als & 0xFF;
		buf[R_ALS_TH_LOW_MSB] = (st->als_th_low >> 8) & 0xFF;
		buf[R_ALS_TH_LOW_LSB] = st->als_th_low & 0xFF;
		buf[R_ALS_TH_HIGH_MSB] = (st->als_th_high >> 8) & 0xFF;
		buf[R_ALS_TH_HIGH_LSB] = st->als_th_high & 0xFF;
		buf[R_STATUS] = (st->als_int_pending ? BIT(0) : 0) |
				(st->temp_rdy        ? BIT(1) : 0) |
				(st->hum_rdy         ? BIT(2) : 0) |
				(st->als_rdy         ? BIT(3) : 0);
	}

	if (copy_to_user(ubuf, buf + *ppos, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static ssize_t debugfs_regs_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct envcombo *st = file->private_data;
	u8 buf[REG_COUNT];

	if (*ppos >= REG_COUNT)
		return -EINVAL;
	if (*ppos + count > REG_COUNT)
		count = REG_COUNT - *ppos;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	scoped_guard(spinlock, &st->lock) {
		size_t i;

		for (i = 0; i < count; i++)
			st->regs[*ppos + i] = buf[i];
	}

	*ppos += count;
	return count;
}

static int debugfs_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debugfs_regs_fops = {
	.owner = THIS_MODULE,
	.open = debugfs_regs_open,
	.read = debugfs_regs_read,
	.write = debugfs_regs_write,
};

static int __init envcombo_sim_init(void)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("envcombo", ENV_ADDR),
	};
	int ret;

	edev = kzalloc(sizeof(*edev), GFP_KERNEL);
	if (!edev)
		return -ENOMEM;

	spin_lock_init(&edev->lock);

	edev->regs[R_CFG] = 0x00;
	edev->regs[R_INT_CFG] = 0x00;
	edev->als_th_low = 0x0000;
	edev->als_th_high = 0xFFFF;
	edev->regs[R_CAL_AGAIN] = 0x01;
	edev->regs[R_CAL_ATIME] = 0x00;
	edev->regs[R_PWR_MODE] = PWR_SLEEP;

	edev->domain = irq_domain_add_linear(NULL, 1,
					     &sim_irq_domain_ops, NULL);
	if (!edev->domain) {
		ret = -ENOMEM;
		goto err_free;
	}

	edev->irq = irq_create_mapping(edev->domain, 0);
	if (!edev->irq) {
		ret = -ENOMEM;
		goto err_domain;
	}

	edev->adap.owner = THIS_MODULE;
	edev->adap.class = I2C_CLASS_HWMON;
	edev->adap.algo  = &ec_algo;
	strscpy(edev->adap.name, "envcombo-sim", sizeof(edev->adap.name));

	ret = i2c_add_adapter(&edev->adap);
	if (ret)
		goto err_irq;

	INIT_DELAYED_WORK(&edev->work, ec_work_fn);
	INIT_WORK(&edev->irq_work, ec_irq_work_fn);

	info.irq = edev->irq;
	edev->client = i2c_new_client_device(&edev->adap, &info);
	if (IS_ERR(edev->client)) {
		ret = PTR_ERR(edev->client);
		goto err_adapter;
	}

	edev->debugfs_dir = debugfs_create_dir("envcombo-sim", NULL);
	debugfs_create_file("regs", 0600, edev->debugfs_dir, edev,
			    &debugfs_regs_fops);

	pr_info("envcombo-sim: adapter #%d, device at 0x%02x, irq %u\n",
		edev->adap.nr, ENV_ADDR, edev->irq);
	return 0;

err_adapter:
	i2c_del_adapter(&edev->adap);
err_irq:
	irq_dispose_mapping(edev->irq);
err_domain:
	irq_domain_remove(edev->domain);
err_free:
	kfree(edev);
	return ret;
}

static void __exit envcombo_sim_exit(void)
{
	debugfs_remove_recursive(edev->debugfs_dir);
	cancel_delayed_work_sync(&edev->work);
	cancel_work_sync(&edev->irq_work);
	i2c_unregister_device(edev->client);
	i2c_del_adapter(&edev->adap);
	irq_dispose_mapping(edev->irq);
	irq_domain_remove(edev->domain);
	kfree(edev);
	pr_info("envcombo-sim: unloaded\n");
}

module_init(envcombo_sim_init);
module_exit(envcombo_sim_exit);

MODULE_AUTHOR("Nanit");
MODULE_DESCRIPTION("I2C bus and register simulator for ENV-COMBO sensor");
MODULE_LICENSE("GPL");
