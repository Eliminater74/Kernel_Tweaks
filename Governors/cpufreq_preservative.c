/*
 *  drivers/cpufreq/cpufreq_preservative.c
 *
 *  Based on conservative, which was based on ondemand.
 *  All the logic has been ripped out and replaced with jam.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>

#include "../gpu/msm/kgsl.h"

#define TRANSITION_LATENCY_LIMIT	(10 * 1000 * 1000)
#define SAMPLE_RATE			(40009)
#define OPTIMAL_POSITION		(3)
#define TABLE_SIZE			(14)
#define HYSTERESIS			(7)
#define UP_THRESH			(100)

static const int valid_fqs[TABLE_SIZE] = {384000, 486000, 594000, 702000, 810000,
			918000, 1026000, 1134000, 1242000, 1350000,
			1458000, 1566000, 1674000, 1728000};
static void do_dbs_timer(struct work_struct *work);

static int thresh_adj = 0;
static int opt_pos = OPTIMAL_POSITION;
static unsigned int dbs_enable, down_requests, prev_table_position, freq_table_position, min_sampling_rate;
bool early_suspended = false;
bool plug_boost = false;
bool hyst_flag = false;

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	unsigned int requested_freq;
	int cpu;
	unsigned int enable:1;
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cs_cpu_dbs_info);

static DEFINE_MUTEX(dbs_mutex);

static struct workqueue_struct *dbs_wq;

static struct dbs_tuners {
	unsigned int up_threshold;
} dbs_tuners_ins = {
	.up_threshold = UP_THRESH,
};

static inline u64 get_cpu_idle_time_jiffy(unsigned int cpu, u64 *wall)
{
	u64 idle_time;
	u64 cur_wall_time;
	u64 busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

	busy_time  = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = jiffies_to_usecs(cur_wall_time);

	return jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);
	else
		idle_time += get_cpu_iowait_time_us(cpu, wall);

	return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/* keep track of frequency transitions */
static int
dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_dbs_info_s *this_dbs_info = &per_cpu(cs_cpu_dbs_info,
							freq->cpu);

	struct cpufreq_policy *policy;

	if (!this_dbs_info->enable)
		return 0;

	policy = this_dbs_info->cur_policy;

	/*
	 * we only care if our internally tracked freq moves outside
	 * the 'valid' ranges of freqency available to us otherwise
	 * we do not change it
	*/
	if (this_dbs_info->requested_freq > policy->max
			|| this_dbs_info->requested_freq < policy->min)
		this_dbs_info->requested_freq = freq->new;

	return 0;
}

static struct notifier_block dbs_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier
};

/************************** sysfs interface ************************/
/* cpufreq_conservative Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(up_threshold, up_threshold);

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 127 )
		return -EINVAL;

	dbs_tuners_ins.up_threshold = input;
	return count;
}

define_one_global_rw(up_threshold);

static struct attribute *dbs_attributes[] = {
	&up_threshold.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "preservative",
};

/************************** sysfs end ************************/

static int get_load(struct cpufreq_policy *policy)
{
	unsigned int load = 0;
	unsigned int max_load = 0;
	unsigned int j;

	/* Get Absolute Load */
	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;

		j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
		cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

		wall_time = (unsigned int)
			(cur_wall_time - j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		iowait_time = (unsigned int)
			(cur_iowait_time - j_dbs_info->prev_cpu_iowait);
		j_dbs_info->prev_cpu_iowait = cur_iowait_time;

		if (idle_time >= iowait_time)
			idle_time -= iowait_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 128 * (wall_time - idle_time) / wall_time;

		if (load > max_load)
			max_load = load;
	}
	return max_load;
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int target_table_position = 0;
	unsigned int max_load, freq_target, j;
	struct cpufreq_policy *policy = this_dbs_info->cur_policy;

	if (early_suspended) {
		opt_pos = 1;
	} else {
		opt_pos = OPTIMAL_POSITION;
	}

	max_load = get_load(policy);

	// for power saving, if we hit top fq, make it harder to scale up
	// conversely for performance, if we hit opt_pos, make it easier to scale up
	if (freq_table_position == (TABLE_SIZE - 1)) {
		if (++thresh_adj < 0) thresh_adj = 0;
		if ((dbs_tuners_ins.up_threshold + thresh_adj) > 128) thresh_adj = 128 - dbs_tuners_ins.up_threshold;
	}
	if (freq_table_position <= opt_pos) {
		if (--thresh_adj > 0) thresh_adj = 0;
		if ((dbs_tuners_ins.up_threshold + thresh_adj) < 40) thresh_adj = 40 - dbs_tuners_ins.up_threshold;
	}

	if (max_load > (dbs_tuners_ins.up_threshold + thresh_adj)) {
		if (freq_table_position < opt_pos) {
			freq_table_position = opt_pos; // must hit opt_pos first if going up from 0 pos
		} else {
			freq_table_position = (freq_table_position + TABLE_SIZE) / 2;
			freq_table_position = (freq_table_position + prev_table_position + 1) / 2;
		}
	} else {
		freq_target = max_load * valid_fqs[freq_table_position] / 128;
		for (j = 0; j < TABLE_SIZE; j++) {
			if (valid_fqs[target_table_position] < freq_target) target_table_position++;
		}
		freq_table_position = (freq_table_position + target_table_position + !early_suspended) / 2;
	}

	if (!early_suspended) {
		// apply hysteresis before dropping to lower bus speeds
		if (freq_table_position < opt_pos) {
			if (++down_requests >= HYSTERESIS) {
				hyst_flag = true;
			} else {
				freq_table_position = opt_pos;
			}
		} else {
			down_requests = 0;
		}


		if (plug_boost) {
			freq_table_position = TABLE_SIZE - 1;	//boost for hotplugging
			plug_boost = false;
			down_requests = 0;
		}

	} else {
		if (freq_table_position > opt_pos)
				freq_table_position = OPTIMAL_POSITION;  // if early suspended - limit max fq. 
	}

	this_dbs_info->requested_freq = valid_fqs[freq_table_position];

	prev_table_position = freq_table_position;

	// if already on target, break out early
	if (policy->cur == valid_fqs[freq_table_position])
		return;

	__cpufreq_driver_target(policy, this_dbs_info->requested_freq,
			CPUFREQ_RELATION_H);

	if (hyst_flag) {
		prev_table_position = 0;
		freq_table_position--;
		hyst_flag = false;
	} else {
		prev_table_position = freq_table_position;
	}

	return;
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;

	int delay = usecs_to_jiffies(SAMPLE_RATE);

	delay -= jiffies % delay;

	mutex_lock(&dbs_info->timer_mutex);

	dbs_check_cpu(dbs_info);

	queue_delayed_work_on(cpu, dbs_wq, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	int delay = usecs_to_jiffies(SAMPLE_RATE);
	delay -= jiffies % delay;

	dbs_info->enable = 1;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	queue_delayed_work_on(dbs_info->cpu, dbs_wq, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work_sync(&dbs_info->work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(cs_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
		}
		this_dbs_info->requested_freq = policy->cur;

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_enable++;

		if (dbs_enable == 1) {
			unsigned int latency;
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			min_sampling_rate = SAMPLE_RATE;

			cpufreq_register_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_mutex);

		dbs_timer_init(this_dbs_info);

		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		dbs_enable--;
		mutex_destroy(&this_dbs_info->timer_mutex);

		if (dbs_enable == 0)
			cpufreq_unregister_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);

		mutex_unlock(&dbs_mutex);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_PRESERVATIVE
static
#endif
struct cpufreq_governor cpufreq_gov_preservative = {
	.name			= "preservative",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	dbs_wq = alloc_workqueue("preservative_dbs_wq", WQ_HIGHPRI, 0);
	if (!dbs_wq) {
		printk(KERN_ERR "Failed to create preservative_dbs_wq workqueue\n");
		return -EFAULT;
	}

	return cpufreq_register_governor(&cpufreq_gov_preservative);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_preservative);
	destroy_workqueue(dbs_wq);
}

MODULE_AUTHOR("bedalus");
MODULE_DESCRIPTION("Jelly, jam and preserves are all made from fruit mixed with sugar and"
		   "pectin. The difference between them comes in the form that the fruit"
		   "takes. In jelly, the fruit comes in the form of fruit juice. In jam,"
		   "the fruit comes in the form of fruit pulp or crushed fruit (and is"
		   "less stiff than jelly as a result). In preserves, the fruit comes"
		   "in the form of chunks in a syrup or a jam.");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_PRESERVATIVE
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
