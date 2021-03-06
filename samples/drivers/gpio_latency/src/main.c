/*
 * Copyright (c) 2019 EVL
 * Author: Jorge Ramirez-Ortiz <jro@xenomai.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Usage:
 *  The DUT running Linux (ie on rpi3b) using GPIOS 23 (to receive) and GPIO24 (to
 *  send) would be running the gpio-echo code  capable of responding to the
 *  requests sent by this latency measurement test code.
 *
 *  gpio-echo is realtime capable (via EVL) or non-realtime capable (standard
 *  linux behviour) - it depends on how you build it.
 *
 *   $ gpio-echo -n gpiochip0 -o 23 -t 24 -O -T -f
 *
 * Once that process is started and the necessary cabling is done start this
 * Zephyr program, get the console * and follow the instructions
 *
 * Connections:
 *
 *  Zephyr - FRDMk64F:                Linux - rpi3b
 *    PIN 20 (PTE-24) ----------------  PIN 16 (GPIO 23)
 *    PIN 18 (PTE-25) ----------------  PIN 18 (GPIO 24)
 *
 */

#include <atomic.h>
#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <gpio.h>
#include <misc/util.h>
#include <console.h>
#include <logging/log.h>

#define ARRAY_LEN(x)		(sizeof(x)/sizeof(x[0]))
#define MIN(a, b)		(((a) < (b)) ? (a) : (b))

#define LOG_MODULE_NAME		latency_tester
#define LOG_LEVEL		LOG_LEVEL_INF
LOG_MODULE_REGISTER();

#define GPIO_OUT0_DRV_NAME	LED0_GPIO_CONTROLLER
#define GPIO_OUT0_PIN		LED0_GPIO_PIN

#define GPIO_OUT1_DRV_NAME	TX_GPIO_CONTROLLER
#define GPIO_OUT1_PIN		TX_GPIO_PIN

#define GPIO_IN_DRV_NAME	RX_GPIO_CONTROLLER
#define GPIO_INT_PIN		RX_GPIO_PIN

#define LOOPBACK 		0	/* enable loopback for calibration */
#define CALIBRATED_LATENCY_NS	1000	/* FRDMk64F calibration	*/
#define STAMP_SEED		0xFFFFFFFF
#define MAX_SAMPLES		20000
#define HISTOGRAM_BINS		100
#define FIXED_HISTOGRAM_BINS 	20

static atomic_t stamp = ATOMIC_INIT(STAMP_SEED);
static struct device *gpio_out1_dev;
static u64_t calibrated_latency;
static u64_t samples[MAX_SAMPLES];
static u32_t cnt;

/* avoid stack overflow */
static struct {
	u64_t val;
	int nbr;
} bins[HISTOGRAM_BINS];

static struct {
	u64_t val;
	int nbr;
} fixed_bins[FIXED_HISTOGRAM_BINS];

void gpio_callback(struct device *port, struct gpio_callback *cb, u32_t pins)
{
	u64_t delta = SYS_CLOCK_HW_CYCLES_TO_NS64(k_cycle_get_32() - stamp);
	int ret;

	if (atomic_get(&stamp) == STAMP_SEED)
		return;

#if LOOPBACK == 1
	/* deassert signal to DUT */
	ret = gpio_pin_write(gpio_out1_dev, GPIO_OUT1_PIN, 1);
	if (ret) {
		LOG_ERR("gpio out1 error: clear");
		return;
	}
#endif
	atomic_set(&stamp, STAMP_SEED);

	if (cnt < ARRAY_LEN(samples)) {
		samples[cnt] = delta - calibrated_latency;
		cnt = cnt + 1;
	}
}

void main(void)
{
	static struct gpio_callback gpio_cb;
	struct device *gpio_out0_dev;
	struct device *gpio_in_dev;
	struct test_config {
		int loop_for_ever;
		s64_t loop_for_ever_start;
		u64_t max_latency;
		int max_latency_on;
		int duration;
		int period;
	} config = {
		.max_latency = 0,
		.max_latency_on = 0,
		.duration = 0,
		.period = 0,
		.loop_for_ever = 0,
	};

	s64_t reftime;
	int nsamples, min_bin_size, bin_size, max_bin_len, fixed_max_bin_len;
	int ret, i, j, min_i, max_i;
	u64_t min, max = 0;
	int led_on = 1, led_period;
	char *val;
	u64_t max_latency, min_latency;
	u64_t runs = 0;
	int fixed_bin_overflows;

	console_getline_init();

	gpio_out0_dev = device_get_binding(GPIO_OUT0_DRV_NAME);
	if (!gpio_out0_dev) {
		printk("cannot find %s\n", GPIO_OUT0_DRV_NAME);
		return;
	}

	gpio_out1_dev = device_get_binding(GPIO_OUT1_DRV_NAME);
	if (!gpio_out1_dev) {
		printk("cannot find %s\n", GPIO_OUT1_DRV_NAME);
		return;
	}

	gpio_in_dev = device_get_binding(GPIO_IN_DRV_NAME);
	if (!gpio_in_dev) {
		printk("cannot find %s\n", GPIO_IN_DRV_NAME);
		return;
	}

	/* configured for FRDM_k64F LED */
	ret = gpio_pin_configure(gpio_out0_dev, GPIO_OUT0_PIN, (GPIO_DIR_OUT));
	if (ret) {
		printk("error configuring pin %d\n", GPIO_OUT0_PIN);
		return;
	}

	/* configured for FRDM_k64F signal out */
	ret = gpio_pin_configure(gpio_out1_dev, GPIO_OUT1_PIN, (GPIO_DIR_OUT));
	if (ret) {
		printk("error configuring pin %d\n", GPIO_OUT1_PIN);
		return;
	}

	ret = gpio_pin_write(gpio_out1_dev, GPIO_OUT1_PIN, 1);
	if (ret) {
		LOG_ERR("gpio out error: set");
		return;
	}

	/* Setup GPIO input, and triggers on level low */
	ret = gpio_pin_configure(gpio_in_dev, GPIO_INT_PIN,
				 (GPIO_DIR_IN | GPIO_INT |
				  GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW));
	if (ret) {
		printk("error configuring pin %d\n", GPIO_INT_PIN);
		return;
	}

	gpio_init_callback(&gpio_cb, gpio_callback, BIT(GPIO_INT_PIN));

	ret = gpio_add_callback(gpio_in_dev, &gpio_cb);
	if (ret) {
		printk("cannot setup callback\n");
		return;
	}

	ret = gpio_pin_enable_callback(gpio_in_dev, GPIO_INT_PIN);
	if (ret) {
		printk("error enabling callback\n");
		return;
	}
again:
	printk("\n=======================\n");
	printk("Interrupt Latency tester\n");
	printk("Max samples: %d\n", ARRAY_LEN(samples));
	printk("=========================\n");

	printk("Select test (default t)\n");
	printk(" *  : run until latency hit\n");
	printk(" ~  : run for ever\n");
	printk(" t  : time bounded\n");
	printk("    : ");
	max_latency = 0;
	min_latency = 2000000;

	config.max_latency_on = 0;
	config.loop_for_ever = 0;
	config.loop_for_ever_start = 0;
	config.max_latency = 0;
	runs = 0;
	val = console_getline();
	if (*val == '*') {
		printk("Max latency that will stop the test [usec, default 80] : ");
		val = console_getline();
		config.max_latency = 1000 * atoi(val);
		if (!config.max_latency)
			config.max_latency = 80 * 1000;

		/* make it small so user wont have to wait when threshold hit */
		config.duration = 1;
	} else if (*val == '~') {
		config.loop_for_ever = 1;
		config.loop_for_ever_start = k_uptime_get();
		config.duration = 1;

		for (i =0; i < ARRAY_LEN(fixed_bins); i++) {
			fixed_bins[i].val = i * 10 * 1000; /* nsecs */
			fixed_bins[i].nbr = 0;
	}

	} else {
		printk("Enter test duration in seconds (default 10) :");
		val = console_getline();
		config.duration = atoi(val);
		if (!config.duration)
			config.duration = 10;
	}

	printk("\n");

	if (config.max_latency || config.loop_for_ever) {
		/* the lowest reasonable value for a 1 second test loop */
		printk("Enter period [default 1 msec]: ");
	} else
		printk("Enter period [default 5 msec]: ");

	val = console_getline();
	config.period = atoi(val) * 1000;
	if (!config.period)
		config.period = (config.max_latency || config.loop_for_ever) ? 1000 : 5000;

	printk("Enter calibrated latency [default %d nsec]: ",
		CALIBRATED_LATENCY_NS);
	val = console_getline();
	calibrated_latency = atoi(val);
	if (!calibrated_latency)
		calibrated_latency = CALIBRATED_LATENCY_NS;

	if (config.max_latency)
		printk(" - test duration (1 sec) until latency hits %lld\n",
			config.max_latency);
	else if (config.loop_for_ever)
		printk(" - test duration unbounded\n");
	else
		printk(" - test duration %d sec\n" ,config.duration);

	printk(" - period %d usec\n"
	       " - calibrated latency %lld ns\n",
		config.period, calibrated_latency);

	nsamples = config.duration * 1000000 / config.period;
	if (nsamples > ARRAY_LEN(samples)) {
		nsamples = ARRAY_LEN(samples);
		printk("\nWARNING: not enough space, test will finish in %d secs!!\n",
			ARRAY_LEN(samples) * config.period / 1000000);
	}

	if (!config.max_latency_on && !config.loop_for_ever)
		printk("\nTest will capture %d samples\n", nsamples);

	printk("Press any key to begin ");
	val = console_getline();
	if (config.max_latency)
		config.max_latency_on = 1;

	/* work on milliseconds */
	config.duration = config.duration * 1000;
	fixed_max_bin_len = 0;
	fixed_bin_overflows = 0;
run:
	runs++;
	/* always clear the array and the histogram data */
	memset(samples, 0, sizeof(samples));
	for (i =0; i < ARRAY_LEN(bins); i++) {
		bins[i].val = 0;
		bins[i].nbr = 0;
	}
	led_period = 200000;
	cnt = 0;
	reftime = k_uptime_get();
	while ((k_uptime_get() - reftime < config.duration) &&
	       (cnt < ARRAY_LEN(samples))) {

		if (atomic_get(&stamp) != STAMP_SEED) {
			if (config.loop_for_ever) {
				s64_t now = k_uptime_get() / 1000;
				s64_t secs = now  - config.loop_for_ever_start / 1000;
				s64_t hours, minutes;
				hours = secs / 3600;
				secs = secs - hours * 3600;
				minutes = secs / 60;
				secs = secs - minutes * 60;

				printk("\nLatency test:\n");
				printk(" nsamples    : %lld\n", (runs - 1) * nsamples + cnt);
				printk(" max latency : %lld nsec\n", max_latency);
				printk(" min latency : %lld nsec\n", min_latency);
				printk(" overflows   : %04d\n", fixed_bin_overflows);
				printk(" duration    : %04lld:%02lld:%02lld\n",
					hours, minutes, secs);
				printk("------- latency --------- -- nbr --  ---------------- chart ----------------------\n");
				for (i = 0; i < ARRAY_LEN(fixed_bins); i++) {
					char bar[50] = { '\0' };
					int k;

					for (j = 0; j < FIXED_HISTOGRAM_BINS; j++) {
						if (fixed_bins[i].nbr < (j * fixed_max_bin_len / 50))
							break;
					}

					for (k = 0; k < j - 1; k++ ) {
						bar[k] = '-';
					}

					bar[j - 1] = '|';

					printk(" [%07lld - %07lld ns]   %04d     %s\n",
						i ? fixed_bins[i - 1].val : 0,
						fixed_bins[i].val, fixed_bins[i].nbr, fixed_bins[i].nbr ? bar : "");
				}

				do {
					char bar[50] = { '\0' };
					int k;

					for (j = 0; j < FIXED_HISTOGRAM_BINS; j++) {
						if (fixed_bin_overflows < (j * fixed_max_bin_len / 50))
							break;
					}

					for (k = 0; k < j - 1; k++ ) {
						bar[k] = '-';
					}

					bar[k] = '|';

					printk(" [%07lld -        max]   %04d     %s\n",
						fixed_bins[i - 1].val, fixed_bin_overflows,
						fixed_bin_overflows > 0 ? bar : "");
				} while (0);
			} else
				printk("Deadline missed, increase period\n\n");

			atomic_set(&stamp, STAMP_SEED);
			goto again;
		}

		/* start waiting for notifications */
		atomic_set(&stamp, k_cycle_get_32());

		/* send the signal */
		ret = gpio_pin_write(gpio_out1_dev, GPIO_OUT1_PIN, 0);
		if (ret) {
			printk("gpio out error: set");
			break;
		}

		/* toggle the LED */
		ret = gpio_pin_write(gpio_out0_dev, GPIO_OUT0_PIN, led_on);
		if (ret) {
			printk("gpio out error: set");
			break;
		}

#if LOOPBACK == 0
		/* deassert signal to DUT */
		k_busy_wait(1);
		ret = gpio_pin_write(gpio_out1_dev, GPIO_OUT1_PIN, 1);
		if (ret) {
			printk("gpio out1 error: clear");
			return;
		}
#endif
		k_busy_wait(config.period);
		led_period -= config.period;
		if (led_period <= 0) {
			led_period = 200000;
			led_on ^= 1;
		}
	}

	/* make sure the caches are hot */
	if (runs < 2)
		goto run;

	min = max = min_i = max_i = 0;
	for (i = 0; i < cnt; i++) {
		if (min == 0 || min > samples[i]) {
			min = samples[i];
			min_i = i;
		}

		if (max < samples[i]) {
			max = samples[i];
			max_i = i;
		}
	}

	/* check if max latency was hit */
	if (config.max_latency_on) {
		if (max > max_latency) {
			max_latency = max;
			/* some feedback in the loop for ever case */
			printk(" max latency update : %lld nsec\n", max_latency);
		}

		 if (max < config.max_latency)
			 goto run;
		 else
			 config.max_latency_on = 0;
	}

	if (config.loop_for_ever) {
		if (max > max_latency) {
			s64_t now = k_uptime_get() / 1000;
			s64_t secs = now  - config.loop_for_ever_start / 1000;
			s64_t hours, minutes;

			max_latency = max;
			hours = secs / 3600;
			secs = secs - hours * 3600;
			minutes = secs / 60;
			secs = secs - minutes * 60;
			/* some feedback in the loop for ever case */
			printk(" max latency hit : %08lld nsec, samples: %08lld time: %04lld:%02lld:%02lld\n",
				max_latency, (runs - 1) * nsamples,
				hours, minutes, secs);
		}
		if (min < min_latency)
			min_latency = min;

		for (i = 0; i < cnt; i++) {
			for (j =0; j < ARRAY_LEN(fixed_bins); j++) {
				if (samples[i] <= fixed_bins[j].val)  {
					fixed_bins[j].nbr++;
					if (fixed_bins[j].nbr > fixed_max_bin_len)
						fixed_max_bin_len = fixed_bins[j].nbr;
					break;
				}
			}

			if (j >= ARRAY_LEN(fixed_bins) )
				fixed_bin_overflows += 1;
		}

		goto run;
	}

	/* make sure we have enough bins for the histogram (non RT DUT) */
	min_bin_size = ((max - min) / ARRAY_LEN(bins)) / 1000;
	if (min_bin_size < 1)
		min_bin_size = 1;

	printk("Histogram bin size in usec [min %d, default %d]: ",
		min_bin_size, 5 * min_bin_size);
	val = console_getline();
	bin_size = 1000 * atoi(val);
	if (!bin_size || bin_size < min_bin_size * 1000)
		bin_size = 5 * min_bin_size * 1000;

	for (i = 0; i < ARRAY_LEN(bins); i++) {
		bins[i].val = min + bin_size * i;
		if (bins[i].val > max) {
			bins[i].val = max;
			break;
		}
	}

	max_bin_len = 0;
	for (i = 0; i < cnt; i++) {
		for (j =0; j < ARRAY_LEN(bins); j++) {
			if (samples[i] <= bins[j].val) {
				bins[j].nbr++;
				if (bins[j].nbr > max_bin_len)
					max_bin_len = bins[j].nbr;
				break;
			}
		}
	}

	printk("\nTotal number of samples: %d\n", cnt);
        printk("------- latency -------- -- nbr --  ---------------- chart ----------------------\n");
	for (i = 0; i < ARRAY_LEN(bins); i++) {
		char bar[50] = { '\0' };
		int k;

		if (bins[i].val == 0)
		  	break;

		for (j = 0; j < 50; j++) {
			if (bins[i].nbr < (j * max_bin_len / 50))
				break;
		}

		for (k = 0; k < j - 1; k++ ) {
			bar[k] = '-';
		}

		bar[j - 1] = '|';

		printk(" [%07lld - %07lld ns]   %04d     %s\n",
		       i ? bins[i - 1].val : 0,
			bins[i].val, bins[i].nbr, bins[i].nbr ? bar : "");
	}

	printk("\nLatency limits:\n");
	printk("-------------------------\n");
	printk(" min[%05d] = %07lld nsec at %05d msec\n",
		min_i, min, min_i * config.period / 1000);
	printk(" max[%05d] = %07lld nsec at %05d msec\n",
		max_i, max, max_i * config.period / 1000);
	printk(" test duration: %d msec\n", config.duration);
	printk("\n");

	printk("View the the timeline [y/N]: ");
	val = console_getline();
	if (*val != 'y' && *val != 'Y')
		goto again;

	printk("--- time --- -- value--  ------------------ chart -------------------------\n");
	for (i = 0; i < cnt; i++) {
		char bar[50] = { '\0' };
		int k;

		for (j = 0; j < 50; j++) {
			if (samples[i] < (j * max / 50))
				break;
		}

		for (k = 0; k < j; k++ ) {
			bar[k] = '.';
		}

		printk("[%06d msec]  %06lld    %s\n",
			i * config.period / 1000, samples[i], bar);
	}

	printk("\n\n");

	goto again;
}
