/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Functions private to power supply class
 *
 *  Copyright © 2007  Anton Vorontsov <cbou@mail.ru>
 *  Copyright © 2004  Szabolcs Gyurko
 *  Copyright © 2003  Ian Molton <spyro@f2s.com>
 *
 *  Modified: 2004, Oct     Szabolcs Gyurko
 */

struct device;
struct device_type;
struct lenovo_supply;

#ifdef CONFIG_SYSFS

extern void lenovo_supply_init_attrs(struct device_type *dev_type);
extern int lenovo_supply_uevent(struct device *dev, struct kobj_uevent_env *env);

#else

static inline void lenovo_supply_init_attrs(struct device_type *dev_type) {}
#define lenovo_supply_uevent NULL

#endif /* CONFIG_SYSFS */
