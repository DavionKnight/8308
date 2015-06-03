/*
 * linux/kernel/irq/proc.c
 *
 * Copyright (C) 1992, 1998-2004 Linus Torvalds, Ingo Molnar
 *
 * This file contains the /proc/irq/ handling code.
 */

#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/profile.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>

#include "internals.h"

static struct proc_dir_entry *root_irq_dir;

#ifdef CONFIG_SMP

static int irq_affinity_proc_show(struct seq_file *m, void *v)
{
	struct irq_desc *desc = irq_to_desc((long)m->private);
	const struct cpumask *mask = desc->affinity;

#ifdef CONFIG_GENERIC_PENDING_IRQ
	if (desc->status & IRQ_MOVE_PENDING)
		mask = desc->pending_mask;
#endif
	seq_cpumask(m, mask);
	seq_putc(m, '\n');
	return 0;
}

#ifndef is_affinity_mask_valid
#define is_affinity_mask_valid(val) 1
#endif

int no_irq_affinity;
static ssize_t irq_affinity_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int irq = (int)(long)PDE(file->f_path.dentry->d_inode)->data;
	cpumask_var_t new_value;
	int err;

	if (!irq_to_desc(irq)->chip->set_affinity || no_irq_affinity ||
	    irq_balancing_disabled(irq))
		return -EIO;

	if (!alloc_cpumask_var(&new_value, GFP_KERNEL))
		return -ENOMEM;

	err = cpumask_parse_user(buffer, count, new_value);
	if (err)
		goto free_cpumask;

	if (!is_affinity_mask_valid(new_value)) {
		err = -EINVAL;
		goto free_cpumask;
	}

	/*
	 * Do not allow disabling IRQs completely - it's a too easy
	 * way to make the system unusable accidentally :-) At least
	 * one online CPU still has to be targeted.
	 */
	if (!cpumask_intersects(new_value, cpu_online_mask)) {
		/* Special case for empty set - allow the architecture
		   code to set default SMP affinity. */
		err = irq_select_affinity_usr(irq) ? -EINVAL : count;
	} else {
		irq_set_affinity(irq, new_value);
		err = count;
	}

free_cpumask:
	free_cpumask_var(new_value);
	return err;
}

static int irq_affinity_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, irq_affinity_proc_show, PDE(inode)->data);
}

static const struct file_operations irq_affinity_proc_fops = {
	.open		= irq_affinity_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= irq_affinity_proc_write,
};

static int default_affinity_show(struct seq_file *m, void *v)
{
	seq_cpumask(m, irq_default_affinity);
	seq_putc(m, '\n');
	return 0;
}

static ssize_t default_affinity_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	cpumask_var_t new_value;
	int err;

	if (!alloc_cpumask_var(&new_value, GFP_KERNEL))
		return -ENOMEM;

	err = cpumask_parse_user(buffer, count, new_value);
	if (err)
		goto out;

	if (!is_affinity_mask_valid(new_value)) {
		err = -EINVAL;
		goto out;
	}

	/* create /proc/irq/prof_cpu_mask */
	create_prof_cpu_mask(root_irq_dir);

	/*
	 * Do not allow disabling IRQs completely - it's a too easy
	 * way to make the system unusable accidentally :-) At least
	 * one online CPU still has to be targeted.
	 */
	if (!cpumask_intersects(new_value, cpu_online_mask)) {
		err = -EINVAL;
		goto out;
	}

	cpumask_copy(irq_default_affinity, new_value);
	err = count;

out:
	free_cpumask_var(new_value);
	return err;
}

static int default_affinity_open(struct inode *inode, struct file *file)
{
	return single_open(file, default_affinity_show, NULL);
}

static const struct file_operations default_affinity_proc_fops = {
	.open		= default_affinity_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= default_affinity_write,
};
#endif

static int irq_spurious_read(char *page, char **start, off_t off,
				  int count, int *eof, void *data)
{
	struct irq_desc *desc = irq_to_desc((long) data);
	return sprintf(page, "count %u\n"
			     "unhandled %u\n"
			     "last_unhandled %u ms\n",
			desc->irq_count,
			desc->irqs_unhandled,
			jiffies_to_msecs(desc->last_unhandled));
}

#define MAX_NAMELEN 10

void register_irq_proc(unsigned int irq, struct irq_desc *desc)
{
	char name [MAX_NAMELEN];
	struct proc_dir_entry *entry;

	if (!root_irq_dir || (desc->chip == &no_irq_chip) || desc->dir)
		return;

	memset(name, 0, MAX_NAMELEN);
	sprintf(name, "%d", irq);

	/* create /proc/irq/1234 */
	desc->dir = proc_mkdir(name, root_irq_dir);

#ifdef CONFIG_SMP
	/* create /proc/irq/<irq>/smp_affinity */
	proc_create_data("smp_affinity", 0600, desc->dir,
			 &irq_affinity_proc_fops, (void *)(long)irq);
#endif

	entry = create_proc_entry("spurious", 0444, desc->dir);
	if (entry) {
		entry->data = (void *)(long)irq;
		entry->read_proc = irq_spurious_read;
	}
}

#undef MAX_NAMELEN

void unregister_handler_proc(unsigned int irq, struct irqaction *action)
{
	if (action->threaded)
		remove_proc_entry(action->threaded->name, action->dir);
	if (action->dir) {
		struct irq_desc *desc = irq_to_desc(irq);

		remove_proc_entry(action->dir->name, desc->dir);
	}
}

static void register_default_affinity_proc(void)
{
#ifdef CONFIG_SMP
	proc_create("irq/default_smp_affinity", 0600, NULL,
		    &default_affinity_proc_fops);
#endif
}

#ifndef CONFIG_PREEMPT_RT

static int threaded_read_proc(char *page, char **start, off_t off,
			      int count, int *eof, void *data)
{
	return sprintf(page, "%c\n",
		((struct irqaction *)data)->flags & IRQF_NODELAY ? '0' : '1');
}

static int threaded_write_proc(struct file *file, const char __user *buffer,
			       unsigned long count, void *data)
{
	int c;
	struct irqaction *action = data;
	irq_desc_t *desc = irq_to_desc(action->irq);

	if (get_user(c, buffer))
		return -EFAULT;
	if (c != '0' && c != '1')
		return -EINVAL;

	spin_lock_irq(&desc->lock);

	if (c == '0')
		action->flags |= IRQF_NODELAY;
	if (c == '1')
		action->flags &= ~IRQF_NODELAY;
	recalculate_desc_flags(desc);

	spin_unlock_irq(&desc->lock);

	return 1;
}

#endif

#define MAX_NAMELEN 128

static int name_unique(unsigned int irq, struct irqaction *new_action)
{
	struct irq_desc *desc = irq_to_desc(irq);
	struct irqaction *action;

	for (action = desc->action ; action; action = action->next)
		if ((action != new_action) && action->name &&
				!strcmp(new_action->name, action->name))
			return 0;
	return 1;
}

void register_handler_proc(unsigned int irq, struct irqaction *action)
{
	char name [MAX_NAMELEN];
	struct irq_desc *desc = irq_to_desc(irq);

	if (!desc->dir || action->dir || !action->name ||
					!name_unique(irq, action))
		return;

	memset(name, 0, MAX_NAMELEN);
	snprintf(name, MAX_NAMELEN, "%s", action->name);

	/* create /proc/irq/1234/handler/ */
	action->dir = proc_mkdir(name, desc->dir);

	if (!action->dir)
		return;
#ifndef CONFIG_PREEMPT_RT
	{
		struct proc_dir_entry *entry;
		/* create /proc/irq/1234/handler/threaded */
		entry = create_proc_entry("threaded", 0600, action->dir);
		if (!entry)
			return;
		entry->nlink = 1;
		entry->data = (void *)action;
		entry->read_proc = threaded_read_proc;
		entry->write_proc = threaded_write_proc;
		action->threaded = entry;
	}
#endif
}

#undef MAX_NAMELEN

void init_irq_proc(void)
{
	unsigned int irq;
	struct irq_desc *desc;

	/* create /proc/irq */
	root_irq_dir = proc_mkdir("irq", NULL);
	if (!root_irq_dir)
		return;

	register_default_affinity_proc();

	/*
	 * Create entries for all existing IRQs.
	 */
	for_each_irq_desc(irq, desc) {
		if (!desc)
			continue;

		register_irq_proc(irq, desc);
	}
}

