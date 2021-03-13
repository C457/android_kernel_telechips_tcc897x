#ifndef _LINUX_PCA950X_H
#define _LINUX_PCA950X_H

#include <linux/types.h>
#include <linux/i2c.h>

/* platform data for the PCA9506 40-bit I/O expander driver */

struct pca950x_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* initial polarity inversion setting */
	uint16_t	invert;

	/* interrupt base */
	int		irq_base;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	const char	*const *names;
};

#endif /* _LINUX_PCA950X_H */
