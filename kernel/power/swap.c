//+[TCCQB] Modify QuickBoot Image Write to disk
/*
 * linux/kernel/power/swap.c
 *
 * This file provides functions for reading the suspend image from
 * and writing it to a swap partition.
 *
 * Copyright (C) 1998,2001-2005 Pavel Machek <pavel@ucw.cz>
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 * Copyright (C) 2010-2012 Bojan Smojver <bojan@rexursive.com>
 *
 * This file is released under the GPLv2.
 *
 */

#include <linux/module.h>
#include <linux/file.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/genhd.h>
#include <linux/device.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/lzo.h>
#include <linux/vmalloc.h>
#include <linux/cpumask.h>
#include <linux/atomic.h>
#include <linux/kthread.h>
#include <linux/crc32.h>

#include <linux/lz4.h>
#include <linux/lz4hc.h>

#include "power.h"


#if defined(CONFIG_DAUDIO)
#include <mach/daudio.h>

#if defined(QUICKBOOT_SIG)
#define HIBERNATE_SIG	QUICKBOOT_SIG
#else
#define HIBERNATE_SIG	"S1SUSPEND"
#endif

void get_hibernate_sig(char *sig) {
	if (sig != NULL) {
		memcpy(sig, HIBERNATE_SIG, QB_SIG_SIZE);
	}
}
#else
#define HIBERNATE_SIG	"S1SUSPEND"
#endif

#if defined(CONFIG_TSHQB_ENABLE)
#define TSQB_SWSUSP_SIG	"TSQB_SUSP" // 10 byte
#endif

extern void tccwdt_qb_kickdog(void);


/*
 *	The swap map is a data structure used for keeping track of each page
 *	written to a swap partition.  It consists of many swap_map_page
 *	structures that contain each an array of MAP_PAGE_ENTRIES swap entries.
 *	These structures are stored on the swap and linked together with the
 *	help of the .next_swap member.
 *
 *	The swap map is created during suspend.  The swap map pages are
 *	allocated and populated one at a time, so we only need one memory
 *	page to set up the entire structure.
 *
 *	During resume we pick up all swap_map_page structures into a list.
 */

#define MAP_PAGE_ENTRIES	(PAGE_SIZE / sizeof(sector_t) - 1)


/*	To Show Progress Bar when Make QuickBoot Image.	*/
#if defined(CONFIG_QUICK_BOOT_PROGRESS_BAR)
extern int fb_quickboot_progress_bar(int percent);
#endif

/*
 * Number of free pages that are not high.
 */
static inline unsigned long low_free_pages(void)
{
	return nr_free_pages() - nr_free_highpages();
}

/*
 * Number of pages required to be kept free while writing the image. Always
 * half of all available low pages before the writing starts.
 */
static inline unsigned long reqd_free_pages(void)
{
	return low_free_pages() / 2;
}

struct swap_map_page {
	sector_t entries[MAP_PAGE_ENTRIES];
	sector_t next_swap;
};

struct swap_map_page_list {
	struct swap_map_page *map;
	struct swap_map_page_list *next;
};

/**
 *	The swap_map_handle structure is used for handling swap in
 *	a file-alike way
 */

struct swap_map_handle {
	struct swap_map_page *cur;
	struct swap_map_page_list *maps;
	sector_t cur_swap;
	sector_t first_sector;
	unsigned int k;
	unsigned long reqd_free_pages;
	u32 master_checksum;
	u32 crc32;
};

struct swsusp_header {
	char    secure_sig[16];
	u32 	crc32;	// == master_checksum	
	sector_t image;
	unsigned int flags;	/* Flags to pass to the "boot" kernel */	
	unsigned int reg[64];

#if !defined(CONFIG_DAUDIO_KK)
	char reserved[PAGE_SIZE - sizeof(char)*16 - sizeof(u32) - sizeof(sector_t) - sizeof(unsigned int)
		 - sizeof(unsigned int)*64 - 20];
	
	char	orig_sig[10];
	char	sig[10];
#else
	char reserved[PAGE_SIZE - sizeof(char)*16 - sizeof(u32) - sizeof(sector_t) - sizeof(unsigned int)
		 - sizeof(unsigned int)*64 - QB_SIG_SIZE*2];


	char	orig_sig[QB_SIG_SIZE];
	char	sig[QB_SIG_SIZE];
#endif
#if 0
	char reserved[PAGE_SIZE - 20 - sizeof(sector_t) - sizeof(int) -
	              sizeof(u32)];
	u32	crc32;
	sector_t image;
	unsigned int flags;	/* Flags to pass to the "boot" kernel */
	char	orig_sig[10];
	char	sig[10];
#endif
} __packed;

static struct swsusp_header *swsusp_header;

/**
 *	The following functions are used for tracing the allocated
 *	swap pages, so that they can be freed in case of an error.
 */

struct swsusp_extent {
	struct rb_node node;
	unsigned long start;
	unsigned long end;
};

static struct rb_root swsusp_extents = RB_ROOT;

static int swsusp_extents_insert(unsigned long swap_offset)
{
	struct rb_node **new = &(swsusp_extents.rb_node);
	struct rb_node *parent = NULL;
	struct swsusp_extent *ext;

	/* Figure out where to put the new node */
	while (*new) {
		ext = rb_entry(*new, struct swsusp_extent, node);
		parent = *new;
		if (swap_offset < ext->start) {
			/* Try to merge */
			if (swap_offset == ext->start - 1) {
				ext->start--;
				return 0;
			}
			new = &((*new)->rb_left);
		} else if (swap_offset > ext->end) {
			/* Try to merge */
			if (swap_offset == ext->end + 1) {
				ext->end++;
				return 0;
			}
			new = &((*new)->rb_right);
		} else {
			/* It already is in the tree */
			return -EINVAL;
		}
	}
	/* Add the new node and rebalance the tree. */
	ext = kzalloc(sizeof(struct swsusp_extent), GFP_KERNEL);
	if (!ext)
		return -ENOMEM;

	ext->start = swap_offset;
	ext->end = swap_offset;
	rb_link_node(&ext->node, parent, new);
	rb_insert_color(&ext->node, &swsusp_extents);
	return 0;
}

/**
 *	alloc_swapdev_block - allocate a swap page and register that it has
 *	been allocated, so that it can be freed in case of an error.
 */

sector_t alloc_swapdev_block(int swap)
{
	unsigned long offset;

	offset = swp_offset(get_swap_page_of_type(swap));
	if (offset) {
		if (swsusp_extents_insert(offset))
			swap_free(swp_entry(swap, offset));
		else
			return swapdev_block(swap, offset);
	}
	return 0;
}

/**
 *	free_all_swap_pages - free swap pages allocated for saving image data.
 *	It also frees the extents used to register which swap entries had been
 *	allocated.
 */

void free_all_swap_pages(int swap)
{
	struct rb_node *node;

	while ((node = swsusp_extents.rb_node)) {
		struct swsusp_extent *ext;
		unsigned long offset;

		ext = container_of(node, struct swsusp_extent, node);
		rb_erase(node, &swsusp_extents);
		for (offset = ext->start; offset <= ext->end; offset++)
			swap_free(swp_entry(swap, offset));

		kfree(ext);
	}
}

int swsusp_swap_in_use(void)
{
	return (swsusp_extents.rb_node != NULL);
}

/*
 * General things
 */

static unsigned short root_swap = 0xffff;
struct block_device *hib_resume_bdev;

/*
 * Saving part
 */
#ifdef CONFIG_SNAPSHOT_BOOT
//static unsigned int *pSave;
extern int save_cpu_reg_for_snapshot(unsigned int ptable, unsigned int pReg, void *);
extern void restore_snapshot_cpu_reg(void);
extern unsigned int reg[64];
#endif
static int mark_swapfiles(struct swap_map_handle *handle, unsigned int flags)
{
	int error;
#if !defined(CONFIG_DAUDIO_KK)
	char swap_sig[11];
#else
	char swap_sig[QB_SIG_SIZE];
#endif

    unsigned long flag;
	hib_bio_read_page(swsusp_resume_block, swsusp_header, NULL);
#if !defined(CONFIG_DAUDIO_KK)
	if (!memcmp("SWAP-SPACE",swsusp_header->sig, 10) ||
	    !memcmp("SWAPSPACE2",swsusp_header->sig, 10)) {
		memcpy(swsusp_header->orig_sig,swsusp_header->sig, 10);
		memset(swsusp_header->sig, 0, 10);
#if defined(CONFIG_TSHQB_ENABLE)
		memcpy(swsusp_header->sig, TSQB_SWSUSP_SIG, sizeof(TSQB_SWSUSP_SIG));
#else
		memcpy(swsusp_header->sig, HIBERNATE_SIG, sizeof(HIBERNATE_SIG));
#endif
#else
	if (!memcmp("SWAP-SPACE",swsusp_header->sig + QB_SIG_SIZE-10, 10) ||
	    !memcmp("SWAPSPACE2",swsusp_header->sig + QB_SIG_SIZE-10, 10)) {
		memcpy(swsusp_header->orig_sig,swsusp_header->sig, QB_SIG_SIZE);
		memset(swsusp_header->sig, 0, QB_SIG_SIZE);
#if defined(CONFIG_TSHQB_ENABLE)
		memcpy(swsusp_header->sig, TSQB_SWSUSP_SIG, sizeof(TSQB_SWSUSP_SIG));
#else
		memcpy(swsusp_header->sig, HIBERNATE_SIG, QB_SIG_SIZE-1);
#endif
#endif
		
        local_irq_save(flag);
        local_irq_disable();
#ifdef CONFIG_SNAPSHOT_BOOT
        memcpy(swsusp_header->reg, reg, sizeof(reg));
#endif
        local_irq_enable();
        local_irq_restore(flag);
		
		swsusp_header->image = handle->first_sector;
		swsusp_header->flags = flags;

#if (defined(CONFIG_LZ4_COMPRESS) || defined(CONFIG_LZ4_HC_COMPRESS)) && defined (CONFIG_LZ4_DECOMPRESS)
		swsusp_header->crc32 = handle->master_checksum;
#else
		if (flags & SF_CRC32_MODE)
			swsusp_header->crc32 = handle->crc32;
#endif
		error = hib_bio_write_page(swsusp_resume_block,
					swsusp_header, NULL);

#if !defined(CONFIG_DAUDIO_KK)
		memset(swap_sig, 0, 11);
		memcpy(swap_sig, swsusp_header->sig, 10);
#else
		memset(swap_sig, 0, QB_SIG_SIZE);
		memcpy(swap_sig, swsusp_header->sig, QB_SIG_SIZE - 1);
#endif
		printk(KERN_CRIT "\n%s swsusp_header[%lx] %s\n", __FUNCTION__, (unsigned long)&swsusp_header, swap_sig);
	} else {
		printk(KERN_ERR "PM: Swap header not found!\n");
		error = -ENODEV;
	}
	return error;
}

/**
 *	swsusp_swap_check - check if the resume device is a swap device
 *	and get its index (if so)
 *
 *	This is called before saving image
 */
#if defined(CONFIG_PLATFORM_AVN)
int swsusp_swap_check(void)
#else
static int swsusp_swap_check(void)
#endif
{
	int res;

	res = swap_type_of(swsusp_resume_device, swsusp_resume_block,
			&hib_resume_bdev);
	if (res < 0)
		return res;

	root_swap = res;
	res = blkdev_get(hib_resume_bdev, FMODE_WRITE, NULL);
	if (res)
		return res;

	res = set_blocksize(hib_resume_bdev, PAGE_SIZE);
	if (res < 0)
		blkdev_put(hib_resume_bdev, FMODE_WRITE);

	return res;
}

/**
 *	write_page - Write one page to given swap location.
 *	@buf:		Address we're writing.
 *	@offset:	Offset of the swap page we're writing to.
 *	@bio_chain:	Link the next write BIO here
 */

static int write_page(void *buf, sector_t offset, struct bio **bio_chain)
{
	void *src;
	int ret;

	if (!offset)
		return -ENOSPC;

	if (bio_chain) {
		src = (void *)__get_free_page(__GFP_WAIT | __GFP_NOWARN |
		                              __GFP_NORETRY);
		if (src) {
			copy_page(src, buf);
		} else {
			ret = hib_wait_on_bio_chain(bio_chain); /* Free pages */
			if (ret)
				return ret;
			src = (void *)__get_free_page(__GFP_WAIT |
			                              __GFP_NOWARN |
			                              __GFP_NORETRY);
			if (src) {
				copy_page(src, buf);
			} else {
				WARN_ON_ONCE(1);
				bio_chain = NULL;	/* Go synchronous */
				src = buf;
			}
		}
	} else {
		src = buf;
	}
	return hib_bio_write_page(offset, src, bio_chain);
}

static void release_swap_writer(struct swap_map_handle *handle)
{
	printk(KERN_CRIT  "=====>> %s\n", __func__);

	if (handle->cur)
		free_page((unsigned long)handle->cur);
	handle->cur = NULL;
}

static int get_swap_writer(struct swap_map_handle *handle)
{
	int ret;

	ret = swsusp_swap_check();
	if (ret) {
		if (ret != -ENOSPC)
			printk(KERN_ERR "PM: Cannot find swap device, try "
					"swapon -a.\n");
		return ret;
	}
	handle->cur = (struct swap_map_page *)get_zeroed_page(GFP_KERNEL);
	if (!handle->cur) {
		ret = -ENOMEM;
		goto err_close;
	}
	handle->cur_swap = alloc_swapdev_block(root_swap);
	if (!handle->cur_swap) {
		ret = -ENOSPC;
		goto err_rel;
	}
	handle->k = 0;
	handle->reqd_free_pages = reqd_free_pages();
	handle->first_sector = handle->cur_swap;
	return 0;
err_rel:
	release_swap_writer(handle);
err_close:
	swsusp_close(FMODE_WRITE);
	return ret;
}

static int swap_write_page(struct swap_map_handle *handle, void *buf,
				struct bio **bio_chain)
{
	int error = 0;
	sector_t offset;

	if (!handle->cur)
		return -EINVAL;
	offset = alloc_swapdev_block(root_swap);
	error = write_page(buf, offset, bio_chain);
	if (error)
		return error;
	handle->cur->entries[handle->k++] = offset;
	if (handle->k >= MAP_PAGE_ENTRIES) {
		offset = alloc_swapdev_block(root_swap);
		if (!offset)
			return -ENOSPC;
		handle->cur->next_swap = offset;
		error = write_page(handle->cur, handle->cur_swap, bio_chain);
		if (error)
			goto out;
		clear_page(handle->cur);
		handle->cur_swap = offset;
		handle->k = 0;

		if (bio_chain && low_free_pages() <= handle->reqd_free_pages) {
			error = hib_wait_on_bio_chain(bio_chain);
			if (error)
				goto out;
			/*
			 * Recalculate the number of required free pages, to
			 * make sure we never take more than half.
			 */
			handle->reqd_free_pages = reqd_free_pages();
		}
	}
 out:
	return error;
}

static int flush_swap_writer(struct swap_map_handle *handle)
{
	if (handle->cur && handle->cur_swap)
		return write_page(handle->cur, handle->cur_swap, NULL);
	else
		return -EINVAL;
}

static int swap_writer_finish(struct swap_map_handle *handle,
		unsigned int flags, int error)
{
	printk(KERN_CRIT  "=====>> Here\n");

	if (!error) {
		flush_swap_writer(handle);
		printk(KERN_INFO "PM: S");
		error = mark_swapfiles(handle, flags);
		printk("|\n");
		flush_swap_writer(handle);
	}

	if (error)
		free_all_swap_pages(root_swap);
	release_swap_writer(handle);
	swsusp_close(FMODE_WRITE);

	return error;
}

/* We need to remember how much compressed data we need to read. */
#define LZO_HEADER	sizeof(size_t)

/* Number of pages/bytes we'll compress at one time. */
#define LZO_UNC_PAGES	32
#define LZO_UNC_SIZE	(LZO_UNC_PAGES * PAGE_SIZE)

/* Number of pages/bytes we need for compressed data (worst case). */
#define LZO_CMP_PAGES	DIV_ROUND_UP(lzo1x_worst_compress(LZO_UNC_SIZE) + \
			             LZO_HEADER, PAGE_SIZE)
#define LZO_CMP_SIZE	(LZO_CMP_PAGES * PAGE_SIZE)

/* Maximum number of threads for compression/decompression. */
#define LZO_THREADS	3

/* Minimum/maximum number of pages for read buffering. */
#define LZO_MIN_RD_PAGES	1024
#define LZO_MAX_RD_PAGES	8192


/**
 *	save_image - save the suspend image data
 */

static int save_image(struct swap_map_handle *handle,
                      struct snapshot_handle *snapshot,
                      unsigned int nr_to_write)
{
	unsigned int m;
	int ret;
	int nr_pages;
	int err2;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;

	printk(KERN_INFO "PM: Saving image data pages (%u pages)...\n",
		nr_to_write);
	m = nr_to_write / 10;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	while (1) {
		ret = snapshot_read_next(snapshot);
		if (ret <= 0)
			break;
		ret = swap_write_page(handle, data_of(*snapshot), &bio);
		if (ret)
			break;
		if (!(nr_pages % m))
			printk(KERN_INFO "PM: Image saving progress: %3d%%\n",
			       nr_pages / m * 10);
		nr_pages++;
	}
	err2 = hib_wait_on_bio_chain(&bio);
	do_gettimeofday(&stop);
	if (!ret)
		ret = err2;
	if (!ret)
		printk(KERN_INFO "PM: Image saving done.\n");
	swsusp_show_speed(&start, &stop, nr_to_write, "Wrote");
	return ret;
}

/**
 * Structure used for CRC32.
 */
struct crc_data {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	unsigned run_threads;                     /* nr current threads */
	wait_queue_head_t go;                     /* start crc update */
	wait_queue_head_t done;                   /* crc update done */
	u32 *crc32;                               /* points to handle's crc32 */
	size_t *unc_len[LZO_THREADS];             /* uncompressed lengths */
	unsigned char *unc[LZO_THREADS];          /* uncompressed data */
};

/**
 * CRC32 update function that runs in its own thread.
 */
static int crc32_threadfn(void *data)
{
	struct crc_data *d = data;
	unsigned i;

	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		for (i = 0; i < d->run_threads; i++)
			*d->crc32 = crc32_le(*d->crc32,
			                     d->unc[i], *d->unc_len[i]);
		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}
/**
 * Structure used for LZO data compression.
 */
struct cmp_data {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	int ret;                                  /* return code */
	wait_queue_head_t go;                     /* start compression */
	wait_queue_head_t done;                   /* compression done */
	size_t unc_len;                           /* uncompressed length */
	size_t cmp_len;                           /* compressed length */
	unsigned char unc[LZO_UNC_SIZE];          /* uncompressed buffer */
	unsigned char cmp[LZO_CMP_SIZE];          /* compressed buffer */
	unsigned char wrk[LZO1X_1_MEM_COMPRESS];  /* compression workspace */
};

/**
 * Compression function that runs in its own thread.
 */
static int lzo_compress_threadfn(void *data)
{
	struct cmp_data *d = data;

	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			d->ret = -1;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		d->ret = lzo1x_1_compress(d->unc, d->unc_len,
		                          d->cmp + LZO_HEADER, &d->cmp_len,
		                          d->wrk);
		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}

/**
 * save_image_lzo - Save the suspend image data compressed with LZO.
 * @handle: Swap map handle to use for saving the image.
 * @snapshot: Image to read data from.
 * @nr_to_write: Number of pages to save.
 */
static int save_image_lzo(struct swap_map_handle *handle,
                          struct snapshot_handle *snapshot,
                          unsigned int nr_to_write)
{
	unsigned int m;
	int ret = 0;
	int nr_pages;
	int err2;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	size_t off;
	unsigned thr, run_threads, nr_threads;
	unsigned char *page = NULL;
	struct cmp_data *data = NULL;
	struct crc_data *crc = NULL;

	/*
	 * We'll limit the number of threads for compression to limit memory
	 * footprint.
	 */
	nr_threads = num_online_cpus() - 1;
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);

	page = (void *)__get_free_page(__GFP_WAIT | __GFP_HIGH);
	if (!page) {
		printk(KERN_ERR "PM: Failed to allocate LZO page\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct cmp_data, go));

	crc = kmalloc(sizeof(*crc), GFP_KERNEL);
	if (!crc) {
		printk(KERN_ERR "PM: Failed to allocate crc\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	memset(crc, 0, offsetof(struct crc_data, go));

	/*
	 * Start the compression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].thr = kthread_run(lzo_compress_threadfn,
		                            &data[thr],
		                            "image_compress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start compression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	/*
	 * Start the CRC32 thread.
	 */
	init_waitqueue_head(&crc->go);
	init_waitqueue_head(&crc->done);

	handle->crc32 = 0;
	crc->crc32 = &handle->crc32;
	for (thr = 0; thr < nr_threads; thr++) {
		crc->unc[thr] = data[thr].unc;
		crc->unc_len[thr] = &data[thr].unc_len;
	}

	crc->thr = kthread_run(crc32_threadfn, crc, "image_crc32");
	if (IS_ERR(crc->thr)) {
		crc->thr = NULL;
		printk(KERN_ERR "PM: Cannot start CRC32 thread\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	/*
	 * Adjust the number of required free pages after all allocations have
	 * been done. We don't want to run out of pages when writing.
	 */
	handle->reqd_free_pages = reqd_free_pages();

	printk(KERN_INFO
		"PM: Using %u thread(s) for compression.\n"
		"PM: Compressing and saving image data (%u pages)...\n",
		nr_threads, nr_to_write);
	m = nr_to_write / 10;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	for (;;) {
		for (thr = 0; thr < nr_threads; thr++) {
			for (off = 0; off < LZO_UNC_SIZE; off += PAGE_SIZE) {
				ret = snapshot_read_next(snapshot);
				if (ret < 0)
					goto out_finish;

				if (!ret)
					break;

				memcpy(data[thr].unc + off,
				       data_of(*snapshot), PAGE_SIZE);

				if (!(nr_pages % m))
					printk(KERN_INFO
					       "PM: Image saving progress: "
					       "%3d%%\n",
				               nr_pages / m * 10);
				nr_pages++;
			}
			if (!off)
				break;

			data[thr].unc_len = off;

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		if (!thr)
			break;

		crc->run_threads = thr;
		atomic_set(&crc->ready, 1);
		wake_up(&crc->go);

		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR "PM: LZO compression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len >
			             lzo1x_worst_compress(data[thr].unc_len))) {
				printk(KERN_ERR
				       "PM: Invalid LZO compressed length\n");
				ret = -1;
				goto out_finish;
			}

			*(size_t *)data[thr].cmp = data[thr].cmp_len;

			/*
			 * Given we are writing one page at a time to disk, we
			 * copy that much from the buffer, although the last
			 * bit will likely be smaller than full page. This is
			 * OK - we saved the length of the compressed data, so
			 * any garbage at the end will be discarded when we
			 * read it.
			 */
			for (off = 0;
			     off < LZO_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
				memcpy(page, data[thr].cmp + off, PAGE_SIZE);

				ret = swap_write_page(handle, page, &bio);
				if (ret)
					goto out_finish;
			}
		}

		wait_event(crc->done, atomic_read(&crc->stop));
		atomic_set(&crc->stop, 0);
	}

out_finish:
	err2 = hib_wait_on_bio_chain(&bio);
	do_gettimeofday(&stop);
	if (!ret)
		ret = err2;
	if (!ret)
		printk(KERN_INFO "PM: Image saving done.\n");
	swsusp_show_speed(&start, &stop, nr_to_write, "Wrote");
out_clean:
	if (crc) {
		if (crc->thr)
			kthread_stop(crc->thr);
		kfree(crc);
	}
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	if (page) free_page((unsigned long)page);

	return ret;
}






	/********************************************************************
	 *																	*
	 * 					Save Image with LZ4 Thread Compress				*
	 *																	*
	 *******************************************************************/

#if (defined(CONFIG_LZ4_COMPRESS) || defined(CONFIG_LZ4_HC_COMPRESS)) && defined (CONFIG_LZ4_DECOMPRESS)

static inline u32 checksum(u32 *addr, int len)
{
	u32 csum = 0;

	len /= sizeof(*addr);
	while (len-- > 0)
		csum ^= *addr++;
	csum = ((csum>>1) & 0x55555555)  ^  (csum & 0x55555555);

	return csum;
}

/* We need to remember how much compressed data we need to read. */
#define LZ4_HEADER	sizeof(size_t)*2

/* Number of pages/bytes we'll compress at one time. */
#define LZ4_UNC_PAGES	32
#define LZ4_UNC_SIZE	(LZ4_UNC_PAGES * PAGE_SIZE)

/* Number of pages/bytes we need for compressed data (worst case). */
#define LZ4_CMP_PAGES	DIV_ROUND_UP(LZ4_compressBound(LZ4_UNC_SIZE) + \
			             LZ4_HEADER, PAGE_SIZE)
#define LZ4_CMP_SIZE	(LZ4_CMP_PAGES * PAGE_SIZE)

/* Maximum number of threads for compression/decompression. */
#define LZ4_THREADS	1

/* Minimum/maximum number of pages for read buffering. */
#define LZ4_MIN_RD_PAGES	1024
#define LZ4_MAX_RD_PAGES	8192

/**
 * Structure used for CRC32.
 */
/*		Do not use CRC32 with LZ4 */
#if 0
struct crc_data_lz4 {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	unsigned run_threads;                     /* nr current threads */
	wait_queue_head_t go;                     /* start crc update */
	wait_queue_head_t done;                   /* crc update done */
	u32 *crc32;                               /* points to handle's crc32 */
	size_t *unc_len[LZ4_THREADS];             /* uncompressed lengths */
	unsigned char *unc[LZ4_THREADS];          /* uncompressed data */
	size_t *checksumResult[LZ4_THREADS];
};

/**
 * CRC32 update function that runs in its own thread.
 */
static int crc32_threadfn_lz4(void *data)
{
	struct crc_data_lz4 *d = data;
	unsigned i;

	*d->crc32 = 0;		// Clear Master CRC

	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		for (i = 0; i < d->run_threads; i++) {
			*d->crc32 = *d->crc32 ^ *d->checksumResult[i];
			printk(KERN_INFO "Master CRC : [0x%08x] checksumResult : [0x%08x]\n", *d->crc32, *d->checksumResult[i]);
//			*d->crc32 = crc32_le(*d->crc32, d->unc[i], *d->unc_len[i]);
		}
		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}
#endif

/**
 * Structure used for LZ4 data compression.
 */
struct cmp_data_lz4 {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	int ret;                                  /* return code */
	wait_queue_head_t go;                     /* start compression */
	wait_queue_head_t done;                   /* compression done */
	size_t unc_len;                           /* uncompressed length */
	size_t cmp_len;                           /* compressed length */
	unsigned char unc[LZ4_UNC_SIZE];          /* uncompressed buffer */
	unsigned char cmp[LZ4_CMP_SIZE];          /* compressed buffer */
//	unsigned char wrk[LZ41X_1_MEM_COMPRESS];  /* compression workspace */
	size_t checksumResult;						// For 32 PAGE's Checksum Check 
	size_t master_checksum_thread;				// Each Thread Checksum's XOR
};

/**
 * Compression function that runs in its own thread.
 */
static int lz4_compress_threadfn(void *data)
{
	struct cmp_data_lz4 *d = data;
	unsigned char *hashTable;	// For Each Thread hashTable.

	d->master_checksum_thread = 0;

#if defined(CONFIG_LZ4_HC_COMPRESS)
	hashTable = vmalloc(sizeof(LZ4HC_Data_Structure));
#else
	hashTable = vmalloc(PAGE_SIZE*sizeof(uint32_t));
#endif
	if (!hashTable) {
		printk(KERN_ERR "PM: Failed to allocate LZ4 hashTable\n");
		d->ret = -1;
		return 0;
	}

	/*		Compress UNC to CMP		*/
	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			d->ret = -1;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		/*			Create Checksum		*/
		d->checksumResult = checksum((u32 *)d->unc, (int)d->unc_len);
		d->master_checksum_thread = d->master_checksum_thread ^ d->checksumResult;

//		d->ret = lzo1x_1_compress(d->unc, d->unc_len,
//		                          d->cmp + LZ4_HEADER, &d->cmp_len,
//		                          d->wrk);
#if defined(CONFIG_LZ4_HC_COMPRESS)
		memset(hashTable, 0, sizeof(LZ4HC_Data_Structure));
		d->cmp_len = LZ4_compressHC(d->unc, d->cmp+LZ4_HEADER, d->unc_len, hashTable);
#else
		memset(hashTable, 0, PAGE_SIZE*sizeof(uint32_t));
		d->cmp_len = LZ4_compress(d->unc, d->cmp+LZ4_HEADER, d->unc_len, hashTable);
#endif
		if (unlikely(d->cmp_len < 0)) {
			printk(KERN_ERR "PM: Invalid lz4 compressed length\n");
			d->ret = -1;
			break;
		}

		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}

	vfree((void *)hashTable);
	return 0;
}

/**
 * save_image_lz4 - Save the suspend image data compressed with LZ4.
 * @handle: Swap mam handle to use for saving the image.
 * @snapshot: Image to read data from.
 * @nr_to_write: Number of pages to save.
 */
static int save_image_lz4_thread(struct swap_map_handle *handle,
                          struct snapshot_handle *snapshot,
                          unsigned int nr_to_write)
{
	unsigned int m;
	int ret = 0;
	int nr_pages;
	int lz4_nr_to_write = 0;
	int err2;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	size_t off;
	unsigned thr, run_threads, nr_threads;
	unsigned char *page = NULL;
	struct cmp_data_lz4 *data = NULL;
//	struct crc_data_lz4 *crc = NULL;

	printk(KERN_INFO "PM: Save_image with %s()\n", __func__);

	/*
	 * We'll limit the number of threads for compression to limit memory
	 * footprint.
	 */
	nr_threads = num_online_cpus() - 1;
	nr_threads = clamp_val(nr_threads, 1, LZ4_THREADS);

	page = (void *)__get_free_page(__GFP_WAIT | __GFP_HIGH);
	if (!page) {
		printk(KERN_ERR "PM: Failed to allocate LZO page\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct cmp_data_lz4, go));

	/*	Do not use CRC32 with LZ4	*/
	/*
	crc = kmalloc(sizeof(*crc), GFP_KERNEL);
	if (!crc) {
		printk(KERN_ERR "PM: Failed to allocate crc\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	memset(crc, 0, offsetof(struct crc_data_lz4, go));
	*/


	/*
	 * Start the compression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].thr = kthread_run(lz4_compress_threadfn,
		                            &data[thr],
		                            "image_compress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start compression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	/*
	 * Start the CRC32 thread.       It's not Used with LZ4.
	 *
	init_waitqueue_head(&crc->go);
	init_waitqueue_head(&crc->done);

	handle->crc32 = 0;
	crc->crc32 = &handle->crc32;
	for (thr = 0; thr < nr_threads; thr++) {
		crc->unc[thr] = data[thr].unc;
		crc->unc_len[thr] = &data[thr].unc_len;
		crc->checksumResult[thr] = &data[thr].checksumResult;
	}

	crc->thr = kthread_run(crc32_threadfn_lz4, crc, "image_crc32");
	if (IS_ERR(crc->thr)) {
		crc->thr = NULL;
		printk(KERN_ERR "PM: Cannot start CRC32 thread\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	*/

	/*
	 * Adjust the number of required free pages after all allocations have
	 * been done. We don't want to run out of pages when writing.
	 */
	handle->reqd_free_pages = reqd_free_pages();

	printk(KERN_INFO
		"PM: Using %u thread(s) for compression.\n"
		"PM: LZ4 thread Compressing and saving image data (%u pages) ...     \n",
		nr_threads, nr_to_write);
	m = nr_to_write / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	for (;;) {
		tccwdt_qb_kickdog();    // Kick QuickBoot WatchDog
		for (thr = 0; thr < nr_threads; thr++) {
			for (off = 0; off < LZ4_UNC_SIZE; off += PAGE_SIZE) {
				ret = snapshot_read_next(snapshot);
				if (ret < 0)
					goto out_finish;

				if (!ret)
					break;

				memcpy(data[thr].unc + off,
				       data_of(*snapshot), PAGE_SIZE);

				if (!(nr_pages % m)) {
					printk(KERN_INFO "PM: Image saving progress: %3d %% \n", (int)(nr_pages / m));
				#if defined(CONFIG_QUICK_BOOT_PROGRESS_BAR)
					fb_quickboot_progress_bar(nr_pages / m);
				#endif
				}
				nr_pages++;
			}
			if (!off)
				break;

			data[thr].unc_len = off;

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		if (!thr)
			break;

		/*	Do not use CRC32 with LZ4	*/
		/*
		crc->run_threads = thr;
		atomic_set(&crc->ready, 1);
		wake_up(&crc->go);
		*/

		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR "\x1b[1;31mPM: LZ4 compression failed\x1b[0m\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len >
//			             lzo1x_worst_compress(data[thr].unc_len))) {
			             LZ4_compressBound(data[thr].unc_len))) {
				printk(KERN_ERR "\x1b[1;31mPM: Invalid LZ4 compressed length\x1b[0m\n");
				ret = -1;
				goto out_finish;
			}

			/*	data[thr].cmp is for the Mem space of compressed 32 PAGE DATA. */
			*(size_t *)data[thr].cmp = data[thr].cmp_len;
			*(size_t *)(data[thr].cmp + sizeof(size_t)) = data[thr].checksumResult;

			/*
			 * Given we are writing one page at a time to disk, we
			 * copy that much from the buffer, although the last
			 * bit will likely be smaller than full page. This is
			 * OK - we saved the length of the compressed data, so
			 * any garbage at the end will be discarded when we
			 * read it.
			 */
			for (off = 0;
			     off < LZ4_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
				memcpy(page, data[thr].cmp + off, PAGE_SIZE);

				ret = swap_write_page(handle, page, &bio);

				lz4_nr_to_write++;
				if (ret) {
					printk(KERN_ERR "\x1b[1;31mPM: Not enough free swap space!!!\x1b[0m\n");
					goto out_finish;
				}
			}
		}

		/*	Do not use CRC32 with LZ4	*/
		//wait_event(crc->done, atomic_read(&crc->stop));
		//atomic_set(&crc->stop, 0);
	}

	tccwdt_qb_kickdog();    // Kick QuickBoot WatchDog

	/*	Cobine Each Thread's master_checksum_thread to handle->master_checksum	*/
	handle->master_checksum = 0;
	for ( thr = 0; thr < nr_threads; thr++) {
		handle->master_checksum = handle->master_checksum ^ data[thr].master_checksum_thread;	
	}

out_finish:
	tccwdt_qb_kickdog();    // Kick QuickBoot WatchDog
	err2 = hib_wait_on_bio_chain(&bio);
	do_gettimeofday(&stop);
	if (!ret)
		ret = err2;
	if (!ret) {
		printk(KERN_INFO "PM: Image saving progress: done.  \n");
#if defined(CONFIG_QUICK_BOOT_PROGRESS_BAR)
		fb_quickboot_progress_bar(100);
#endif
	} else {
		printk(KERN_CONT "\n");
	}
	swsusp_show_speed(&start, &stop, lz4_nr_to_write, "Wrote with Lz4");
out_clean:
	tccwdt_qb_kickdog();    // Kick QuickBoot WatchDog

	/*	Do not use CRC32 with LZ4	*/
	/*
	if (crc) {
		if (crc->thr)
			kthread_stop(crc->thr);
		kfree(crc);
	}
	*/

	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	if (page) free_page((unsigned long)page);

	return ret;
}

#endif




/**
 *	enough_swap - Make sure we have enough swap to save the image.
 *
 *	Returns TRUE or FALSE after checking the total amount of swap
 *	space avaiable from the resume partition.
 */
#if defined(CONFIG_PLATFORM_AVN)
unsigned long free_swap_size;
unsigned long required_swap_size;


int enough_swap(unsigned int nr_pages, unsigned int flags)
#else
static int enough_swap(unsigned int nr_pages, unsigned int flags)
#endif
{
	unsigned int free_swap = count_swap_pages(root_swap, 1);
	unsigned int required;

	pr_debug("PM: Free swap pages: %u\n", free_swap);

	if (flags & SF_NOCOMPRESS_MODE) {
		required = PAGES_FOR_IO + nr_pages;
	}
	else {
	#if (defined(CONFIG_LZ4_COMPRESS) || defined(CONFIG_LZ4_HC_COMPRESS)) && defined (CONFIG_LZ4_DECOMPRESS)
		required = PAGES_FOR_IO + ((nr_pages * LZ4_CMP_PAGES) / LZ4_UNC_PAGES + 1)/2;
	#else
		required = PAGES_FOR_IO + ((nr_pages * LZO_CMP_PAGES) / LZO_UNC_PAGES + 1)/2;
	#endif
	}

	//printk("PM: Free swap pages: %u required swap pages %u %u \n", free_swap, required, nr_pages);

#if defined(CONFIG_PLATFORM_AVN)
    free_swap_size = free_swap; 
    required_swap_size = required; 
#endif
	return free_swap > required;
}

/**
 *	swsusp_write - Write entire image and metadata.
 *	@flags: flags to pass to the "boot" kernel in the image header
 *
 *	It is important _NOT_ to umount filesystems at this point. We want
 *	them synced (in case something goes wrong) but we DO not want to mark
 *	filesystem clean: it is not. (And it does not matter, if we resume
 *	correctly, we'll mark system clean, anyway.)
 */

int swsusp_write(unsigned int flags)
{
	struct swap_map_handle handle;
	struct snapshot_handle snapshot;
	struct swsusp_info *header;
	unsigned long pages;
	int error;

	tccwdt_qb_kickdog();    // Kick QuickBoot WatchDog

	pages = snapshot_get_image_size();
	error = get_swap_writer(&handle);
	if (error) {
		printk(KERN_ERR "PM: Cannot get swap writer\n");
		return error;
	}
#if defined(CONFIG_SWAP_PREDICTION)
	if (flags & SF_NOCOMPRESS_MODE) {
		if (!enough_swap(pages, flags)) {
			printk(KERN_ERR "\x1b[1;31mPM: Not enough predicted free swap space!!!\x1b[0m\n");
			error = -ENOSPC;
			goto out_finish;
		}
	}
#endif

	memset(&snapshot, 0, sizeof(struct snapshot_handle));
	error = snapshot_read_next(&snapshot);
	if (error < PAGE_SIZE) {
		if (error >= 0)
			error = -EFAULT;

		goto out_finish;
	}
	header = (struct swsusp_info *)data_of(snapshot);
	error = swap_write_page(&handle, header, NULL);
	if (!error) {
		error = (flags & SF_NOCOMPRESS_MODE) ?
			save_image(&handle, &snapshot, pages - 1) :
#if (defined(CONFIG_LZ4_COMPRESS) || defined(CONFIG_LZ4_HC_COMPRESS)) && defined (CONFIG_LZ4_DECOMPRESS)
			save_image_lz4_thread(&handle, &snapshot, pages - 1);
#else
			save_image_lzo(&handle, &snapshot, pages - 1);
#endif
	}
out_finish:
	tccwdt_qb_kickdog();    // Kick QuickBoot WatchDog
	if(error) 
		printk(KERN_ERR "\x1b[1;31mPM: Failed to Make QuickBoot Image!!!\x1b[0m\n");
	error = swap_writer_finish(&handle, flags, error);
	return error;
}

/**
 *	The following functions allow us to read data using a swap map
 *	in a file-alike way
 */

static void release_swap_reader(struct swap_map_handle *handle)
{
	struct swap_map_page_list *tmp;

	while (handle->maps) {
		if (handle->maps->map)
			free_page((unsigned long)handle->maps->map);
		tmp = handle->maps;
		handle->maps = handle->maps->next;
		kfree(tmp);
	}
	handle->cur = NULL;
}

static int get_swap_reader(struct swap_map_handle *handle,
		unsigned int *flags_p)
{
	int error;
	struct swap_map_page_list *tmp, *last;
	sector_t offset;

	*flags_p = swsusp_header->flags;

	if (!swsusp_header->image) /* how can this happen? */
		return -EINVAL;

	handle->cur = NULL;
	last = handle->maps = NULL;
	offset = swsusp_header->image;
	while (offset) {
		tmp = kmalloc(sizeof(*handle->maps), GFP_KERNEL);
		if (!tmp) {
			release_swap_reader(handle);
			return -ENOMEM;
		}
		memset(tmp, 0, sizeof(*tmp));
		if (!handle->maps)
			handle->maps = tmp;
		if (last)
			last->next = tmp;
		last = tmp;

		tmp->map = (struct swap_map_page *)
		           __get_free_page(__GFP_WAIT | __GFP_HIGH);
		if (!tmp->map) {
			release_swap_reader(handle);
			return -ENOMEM;
		}

		error = hib_bio_read_page(offset, tmp->map, NULL);
		if (error) {
			release_swap_reader(handle);
			return error;
		}
		offset = tmp->map->next_swap;
	}
	handle->k = 0;
	handle->cur = handle->maps->map;
	return 0;
}

static int swap_read_page(struct swap_map_handle *handle, void *buf,
				struct bio **bio_chain)
{
	sector_t offset;
	int error;
	struct swap_map_page_list *tmp;

	if (!handle->cur)
		return -EINVAL;
	offset = handle->cur->entries[handle->k];
	if (!offset)
		return -EFAULT;
	error = hib_bio_read_page(offset, buf, bio_chain);
	if (error)
		return error;
	if (++handle->k >= MAP_PAGE_ENTRIES) {
		handle->k = 0;
		free_page((unsigned long)handle->maps->map);
		tmp = handle->maps;
		handle->maps = handle->maps->next;
		kfree(tmp);
		if (!handle->maps)
			release_swap_reader(handle);
		else
			handle->cur = handle->maps->map;
	}
	return error;
}

static int swap_reader_finish(struct swap_map_handle *handle)
{
	release_swap_reader(handle);

	return 0;
}

/**
 *	load_image - load the image using the swap map handle
 *	@handle and the snapshot handle @snapshot
 *	(assume there are @nr_pages pages to load)
 */

static int load_image(struct swap_map_handle *handle,
                      struct snapshot_handle *snapshot,
                      unsigned int nr_to_read)
{
	unsigned int m;
	int ret = 0;
	struct timeval start;
	struct timeval stop;
	struct bio *bio;
	int err2;
	unsigned nr_pages;

	printk(KERN_INFO "PM: Loading image data pages (%u pages)...\n",
		nr_to_read);
	m = nr_to_read / 10;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);
	for ( ; ; ) {
		ret = snapshot_write_next(snapshot);
		if (ret <= 0)
			break;
		ret = swap_read_page(handle, data_of(*snapshot), &bio);
		if (ret)
			break;
		if (snapshot->sync_read)
			ret = hib_wait_on_bio_chain(&bio);
		if (ret)
			break;
		if (!(nr_pages % m))
			printk(KERN_INFO "PM: Image loading progress: %3d%%\n",
			       nr_pages / m * 10);
		nr_pages++;
	}
	err2 = hib_wait_on_bio_chain(&bio);
	do_gettimeofday(&stop);
	if (!ret)
		ret = err2;
	if (!ret) {
		printk(KERN_INFO "PM: Image loading done.\n");
		snapshot_write_finalize(snapshot);
		if (!snapshot_image_loaded(snapshot))
			ret = -ENODATA;
	}
	swsusp_show_speed(&start, &stop, nr_to_read, "Read");
	return ret;
}

/**
 * Structure used for LZO data decompression.
 */
struct dec_data {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	int ret;                                  /* return code */
	wait_queue_head_t go;                     /* start decompression */
	wait_queue_head_t done;                   /* decompression done */
	size_t unc_len;                           /* uncompressed length */
	size_t cmp_len;                           /* compressed length */
	unsigned char unc[LZO_UNC_SIZE];          /* uncompressed buffer */
	unsigned char cmp[LZO_CMP_SIZE];          /* compressed buffer */
};

/**
 * Deompression function that runs in its own thread.
 */
static int lzo_decompress_threadfn(void *data)
{
	struct dec_data *d = data;

	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			d->ret = -1;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		d->unc_len = LZO_UNC_SIZE;
		d->ret = lzo1x_decompress_safe(d->cmp + LZO_HEADER, d->cmp_len,
		                               d->unc, &d->unc_len);
		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}

/**
 * load_image_lzo - Load compressed image data and decompress them with LZO.
 * @handle: Swap map handle to use for loading data.
 * @snapshot: Image to copy uncompressed data into.
 * @nr_to_read: Number of pages to load.
 */
static int load_image_lzo(struct swap_map_handle *handle,
                          struct snapshot_handle *snapshot,
                          unsigned int nr_to_read)
{
	unsigned int m;
	int ret = 0;
	int eof = 0;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	unsigned nr_pages;
	size_t off;
	unsigned i, thr, run_threads, nr_threads;
	unsigned ring = 0, pg = 0, ring_size = 0,
	         have = 0, want, need, asked = 0;
	unsigned long read_pages = 0;
	unsigned char **page = NULL;
	struct dec_data *data = NULL;
	struct crc_data *crc = NULL;

	/*
	 * We'll limit the number of threads for decompression to limit memory
	 * footprint.
	 */
	nr_threads = num_online_cpus() - 1;
	nr_threads = clamp_val(nr_threads, 1, LZO_THREADS);

	page = vmalloc(sizeof(*page) * LZO_MAX_RD_PAGES);
	if (!page) {
		printk(KERN_ERR "PM: Failed to allocate LZO page\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZO data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct dec_data, go));

	crc = kmalloc(sizeof(*crc), GFP_KERNEL);
	if (!crc) {
		printk(KERN_ERR "PM: Failed to allocate crc\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	memset(crc, 0, offsetof(struct crc_data, go));

	/*
	 * Start the decompression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].thr = kthread_run(lzo_decompress_threadfn,
		                            &data[thr],
		                            "image_decompress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start decompression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	/*
	 * Start the CRC32 thread.
	 */
	init_waitqueue_head(&crc->go);
	init_waitqueue_head(&crc->done);

	handle->crc32 = 0;
	crc->crc32 = &handle->crc32;
	for (thr = 0; thr < nr_threads; thr++) {
		crc->unc[thr] = data[thr].unc;
		crc->unc_len[thr] = &data[thr].unc_len;
	}

	crc->thr = kthread_run(crc32_threadfn, crc, "image_crc32");
	if (IS_ERR(crc->thr)) {
		crc->thr = NULL;
		printk(KERN_ERR "PM: Cannot start CRC32 thread\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	/*
	 * Set the number of pages for read buffering.
	 * This is complete guesswork, because we'll only know the real
	 * picture once prepare_image() is called, which is much later on
	 * during the image load phase. We'll assume the worst case and
	 * say that none of the image pages are from high memory.
	 */
	if (low_free_pages() > snapshot_get_image_size())
		read_pages = (low_free_pages() - snapshot_get_image_size()) / 2;
	read_pages = clamp_val(read_pages, LZO_MIN_RD_PAGES, LZO_MAX_RD_PAGES);

	for (i = 0; i < read_pages; i++) {
		page[i] = (void *)__get_free_page(i < LZO_CMP_PAGES ?
		                                  __GFP_WAIT | __GFP_HIGH :
		                                  __GFP_WAIT | __GFP_NOWARN |
		                                  __GFP_NORETRY);

		if (!page[i]) {
			if (i < LZO_CMP_PAGES) {
				ring_size = i;
				printk(KERN_ERR
				       "PM: Failed to allocate LZO pages\n");
				ret = -ENOMEM;
				goto out_clean;
			} else {
				break;
			}
		}
	}
	want = ring_size = i;

	printk(KERN_INFO
		"PM: Using %u thread(s) for decompression.\n"
		"PM: Loading and decompressing image data (%u pages)...\n",
		nr_threads, nr_to_read);
	m = nr_to_read / 10;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);

	ret = snapshot_write_next(snapshot);
	if (ret <= 0)
		goto out_finish;

	for(;;) {
		for (i = 0; !eof && i < want; i++) {
			ret = swap_read_page(handle, page[ring], &bio);
			if (ret) {
				/*
				 * On real read error, finish. On end of data,
				 * set EOF flag and just exit the read loop.
				 */
				if (handle->cur &&
				    handle->cur->entries[handle->k]) {
					goto out_finish;
				} else {
					eof = 1;
					break;
				}
			}
			if (++ring >= ring_size)
				ring = 0;
		}
		asked += i;
		want -= i;

		/*
		 * We are out of data, wait for some more.
		 */
		if (!have) {
			if (!asked)
				break;

			ret = hib_wait_on_bio_chain(&bio);
			if (ret)
				goto out_finish;
			have += asked;
			asked = 0;
			if (eof)
				eof = 2;
		}

		if (crc->run_threads) {
			wait_event(crc->done, atomic_read(&crc->stop));
			atomic_set(&crc->stop, 0);
			crc->run_threads = 0;
		}

		for (thr = 0; have && thr < nr_threads; thr++) {
			data[thr].cmp_len = *(size_t *)page[pg];
			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len >
			             lzo1x_worst_compress(LZO_UNC_SIZE))) {
				printk(KERN_ERR
				       "PM: Invalid LZO compressed length\n");
				ret = -1;
				goto out_finish;
			}

			need = DIV_ROUND_UP(data[thr].cmp_len + LZO_HEADER,
			                    PAGE_SIZE);
			if (need > have) {
				if (eof > 1) {
					ret = -1;
					goto out_finish;
				}
				break;
			}

			for (off = 0;
			     off < LZO_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
				memcpy(data[thr].cmp + off,
				       page[pg], PAGE_SIZE);
				have--;
				want++;
				if (++pg >= ring_size)
					pg = 0;
			}

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);
		}

		/*
		 * Wait for more data while we are decompressing.
		 */
		if (have < LZO_CMP_PAGES && asked) {
			ret = hib_wait_on_bio_chain(&bio);
			if (ret)
				goto out_finish;
			have += asked;
			asked = 0;
			if (eof)
				eof = 2;
		}

		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR
				       "PM: LZO decompression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].unc_len ||
			             data[thr].unc_len > LZO_UNC_SIZE ||
			             data[thr].unc_len & (PAGE_SIZE - 1))) {
				printk(KERN_ERR
				       "PM: Invalid LZO uncompressed length\n");
				ret = -1;
				goto out_finish;
			}

			for (off = 0;
			     off < data[thr].unc_len; off += PAGE_SIZE) {
				memcpy(data_of(*snapshot),
				       data[thr].unc + off, PAGE_SIZE);

				if (!(nr_pages % m))
					printk(KERN_INFO
					       "PM: Image loading progress: "
					       "%3d%%\n",
					       nr_pages / m * 10);
				nr_pages++;

				ret = snapshot_write_next(snapshot);
				if (ret <= 0) {
					crc->run_threads = thr + 1;
					atomic_set(&crc->ready, 1);
					wake_up(&crc->go);
					goto out_finish;
				}
			}
		}

		crc->run_threads = thr;
		atomic_set(&crc->ready, 1);
		wake_up(&crc->go);
	}

out_finish:
	if (crc->run_threads) {
		wait_event(crc->done, atomic_read(&crc->stop));
		atomic_set(&crc->stop, 0);
	}
	do_gettimeofday(&stop);
	if (!ret) {
		printk(KERN_INFO "PM: Image loading done.\n");
		snapshot_write_finalize(snapshot);
		if (!snapshot_image_loaded(snapshot))
			ret = -ENODATA;
		if (!ret) {
			if (swsusp_header->flags & SF_CRC32_MODE) {
				if(handle->crc32 != swsusp_header->crc32) {
					printk(KERN_ERR
					       "PM: Invalid image CRC32!\n");
					ret = -ENODATA;
				}
			}
		}
	}
	swsusp_show_speed(&start, &stop, nr_to_read, "Read");
out_clean:
	for (i = 0; i < ring_size; i++)
		free_page((unsigned long)page[i]);
	if (crc) {
		if (crc->thr)
			kthread_stop(crc->thr);
		kfree(crc);
	}
	if (data) {
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	if (page) vfree(page);

	return ret;
}






	/********************************************************************
	 *																	*
	 * 					Load Image with LZ4 Thread Compress				*
	 *																	*
	 *******************************************************************/

#if (defined(CONFIG_LZ4_COMPRESS) || defined(CONFIG_LZ4_HC_COMPRESS)) && defined (CONFIG_LZ4_DECOMPRESS)
/**
 * Structure used for LZ4 data decompression.
 */
struct dec_data_lz4 {
	struct task_struct *thr;                  /* thread */
	atomic_t ready;                           /* ready to start flag */
	atomic_t stop;                            /* ready to stop flag */
	int ret;                                  /* return code */
	wait_queue_head_t go;                     /* start decompression */
	wait_queue_head_t done;                   /* decompression done */
	size_t unc_len;                           /* uncompressed length */
	size_t cmp_len;                           /* compressed length */
	unsigned char unc[LZ4_UNC_SIZE];          /* uncompressed buffer */
	unsigned char cmp[LZ4_CMP_SIZE];          /* compressed buffer */
	size_t saved_checksum_data;					// Checksum from QB Image
	size_t cal_checksum_data;					// calculated checksum 
	size_t master_checksum_thread;				// Each Thread checksum's XOR
};

/**
 * Deompression function that runs in its own thread.
 */
static int lz4_decompress_threadfn(void *data)
{
	struct dec_data_lz4 *d = data;

	d->master_checksum_thread = 0;

	while (1) {
		wait_event(d->go, atomic_read(&d->ready) ||
		                  kthread_should_stop());
		if (kthread_should_stop()) {
			d->thr = NULL;
			d->ret = -1;
			atomic_set(&d->stop, 1);
			wake_up(&d->done);
			break;
		}
		atomic_set(&d->ready, 0);

		/*	Decompress Image	*/
		d->unc_len = LZ4_UNC_SIZE;
//		d->ret = lzo1x_decompress_safe(d->cmp + LZO_HEADER, d->cmp_len,
//		                               d->unc, &d->unc_len);
		d->unc_len = LZ4_uncompress_unknownOutputSize(d->cmp + LZ4_HEADER, d->unc, d->cmp_len, d->unc_len);
		if (d->unc_len < 0) {
			printk(KERN_ERR "PM: LZ4 decompression failed\n");
			d->ret = -1;
			break;
		}
	
		/*	Check Checksum	*/
		d->cal_checksum_data = checksum((u32 *)d->unc, (int)d->unc_len);
		if ( d->cal_checksum_data != d->saved_checksum_data ) {
			printk(KERN_ERR "PM: LZ4 Checksum check failed. cal : [0x%x] load : [0x%x]\n", d->cal_checksum_data, d->saved_checksum_data);
			d->ret = -1;
			break;
		}

		/*	Master Checksum		*/
		d->master_checksum_thread = d->master_checksum_thread ^ d->cal_checksum_data;

		atomic_set(&d->stop, 1);
		wake_up(&d->done);
	}
	return 0;
}

/**
 * load_image_lz4 - Load compressed image data and decompress them with LZO.
 * @handle: Swap map handle to use for loading data.
 * @snapshot: Image to copy uncompressed data into.
 * @nr_to_read: Number of pages to load.
 */
static int load_image_lz4_thread(struct swap_map_handle *handle,
                          struct snapshot_handle *snapshot,
                          unsigned int nr_to_read)
{
	unsigned int m;
	int ret = 0;
	int eof = 0;
	struct bio *bio;
	struct timeval start;
	struct timeval stop;
	unsigned nr_pages;
	size_t off;
	unsigned i, thr, run_threads, nr_threads;
	unsigned ring = 0, pg = 0, ring_size = 0,
	         have = 0, want, need, asked = 0;
	unsigned long read_pages = 0;
	unsigned char **page = NULL;
	struct dec_data_lz4 *data = NULL;
//	struct crc_data_lz4 *crc = NULL;

	
	printk(KERN_INFO "PM: Load_image with %s()\n", __func__);

	/*
	 * We'll limit the number of threads for decompression to limit memory
	 * footprint.
	 */
	nr_threads = num_online_cpus() - 1;
	nr_threads = clamp_val(nr_threads, 1, LZ4_THREADS);

	page = vmalloc(sizeof(*page) * LZ4_MAX_RD_PAGES);
	if (!page) {
		printk(KERN_ERR "PM: Failed to allocate LZ4 page\n");
		ret = -ENOMEM;
		goto out_clean;
	}

	data = vmalloc(sizeof(*data) * nr_threads);
	if (!data) {
		printk(KERN_ERR "PM: Failed to allocate LZ4 data\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	for (thr = 0; thr < nr_threads; thr++)
		memset(&data[thr], 0, offsetof(struct dec_data_lz4, go));


/*		Do not use CRC32 with LZ4.
	crc = kmalloc(sizeof(*crc), GFP_KERNEL);
	if (!crc) {
		printk(KERN_ERR "PM: Failed to allocate crc\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	memset(crc, 0, offsetof(struct crc_data_lz4, go));
*/


	/*
	 * Start the decompression threads.
	 */
	for (thr = 0; thr < nr_threads; thr++) {
		init_waitqueue_head(&data[thr].go);
		init_waitqueue_head(&data[thr].done);

		data[thr].thr = kthread_run(lz4_decompress_threadfn,
		                            &data[thr],
		                            "image_decompress/%u", thr);
		if (IS_ERR(data[thr].thr)) {
			data[thr].thr = NULL;
			printk(KERN_ERR
			       "PM: Cannot start decompression threads\n");
			ret = -ENOMEM;
			goto out_clean;
		}
	}

	/*
	 * Start the CRC32 thread.	// Do not use crc32 with LZ4.
	 *
	init_waitqueue_head(&crc->go);
	init_waitqueue_head(&crc->done);

	handle->crc32 = 0;
	crc->crc32 = &handle->crc32;
	for (thr = 0; thr < nr_threads; thr++) {
		crc->unc[thr] = data[thr].unc;
		crc->unc_len[thr] = &data[thr].unc_len;
	//	crc->checksumResult[thr] = &data[thr].saved_checksum_data;
	}

	//		Run CRC Thread			
	crc->thr = kthread_run(crc32_threadfn_lz4, crc, "image_crc32");
	if (IS_ERR(crc->thr)) {
		crc->thr = NULL;
		printk(KERN_ERR "PM: Cannot start CRC32 thread\n");
		ret = -ENOMEM;
		goto out_clean;
	}
	*/

	/*
	 * Set the number of pages for read buffering.
	 * This is complete guesswork, because we'll only know the real
	 * picture once prepare_image() is called, which is much later on
	 * during the image load phase. We'll assume the worst case and
	 * say that none of the image pages are from high memory.
	 */
	if (low_free_pages() > snapshot_get_image_size())
		read_pages = (low_free_pages() - snapshot_get_image_size()) / 2;
	read_pages = clamp_val(read_pages, LZ4_MIN_RD_PAGES, LZ4_MAX_RD_PAGES);

	for (i = 0; i < read_pages; i++) {
		page[i] = (void *)__get_free_page(i < LZ4_CMP_PAGES ?
		                                  __GFP_WAIT | __GFP_HIGH :
		                                  __GFP_WAIT | __GFP_NOWARN |
		                                  __GFP_NORETRY);

		if (!page[i]) {
			if (i < LZ4_CMP_PAGES) {
				ring_size = i;
				printk(KERN_ERR
				       "PM: Failed to allocate LZ4 pages\n");
				ret = -ENOMEM;
				goto out_clean;
			} else {
				break;
			}
		}
	}
	want = ring_size = i;

	printk(KERN_INFO
		"PM: Using %u thread(s) for decompression.\n"
		"PM: LZ4 thread Loading and decompressing image data (%u pages) ...     \n",
		nr_threads, nr_to_read);
	m = nr_to_read / 100;
	if (!m)
		m = 1;
	nr_pages = 0;
	bio = NULL;
	do_gettimeofday(&start);

	ret = snapshot_write_next(snapshot);
	if (ret <= 0)
		goto out_finish;

	/*		Get Each 32 PAGES UNC DATA			*/
	for(;;) {

		/*		READ from SWAP Partition to Data Buffer(page[ ])		*/
		for (i = 0; !eof && i < want; i++) {
			ret = swap_read_page(handle, page[ring], &bio);
			if (ret) {
				/*
				 * On real read error, finish. On end of data,
				 * set EOF flag and just exit the read loop.
				 */
				if (handle->cur &&
				    handle->cur->entries[handle->k]) {	// SWAP PARTITION OFFSET
					goto out_finish;
				} else {
					eof = 1;
					break;
				}
			}
			if (++ring >= ring_size)
				ring = 0;
		}
		asked += i;
		want -= i;

		/*
		 * We are out of data, wait for some more.
		 */
		if (!have) {
			if (!asked)
				break;

			ret = hib_wait_on_bio_chain(&bio);
			if (ret)
				goto out_finish;
			have += asked;
			asked = 0;
			if (eof)
				eof = 2;
		}

		/*		Finish CRC32 Thread.	*/	// Do not use CRC32 with LZ4.
		/*
		if (crc->run_threads) {
			wait_event(crc->done, atomic_read(&crc->stop));
			atomic_set(&crc->stop, 0);
			crc->run_threads = 0;
		}
		*/

		/*		Wakeup Decompression Thread			*/
		for (thr = 0; have && thr < nr_threads; thr++) {
			data[thr].cmp_len = *(size_t *)page[pg];	// load cmp len
			data[thr].saved_checksum_data = *(size_t *)(page[pg]+sizeof(size_t)); // load cmp Saved checksum.
			if (unlikely(!data[thr].cmp_len ||
			             data[thr].cmp_len > LZ4_CMP_SIZE)) {
				printk(KERN_ERR "PM: Invalid LZ4 compressed length : %d\n", data[thr].cmp_len);
				ret = -1;
				goto out_finish;
			}

			need = DIV_ROUND_UP(data[thr].cmp_len + LZ4_HEADER,
			                    PAGE_SIZE);
			if (need > have) {
				if (eof > 1) {
					ret = -1;
					goto out_finish;
				}
				break;
			}

			/*		Copy the Buffer Data from Emmc to cmp area		*/
			for (off = 0;
			     off < LZ4_HEADER + data[thr].cmp_len;
			     off += PAGE_SIZE) {
				memcpy(data[thr].cmp + off,
				       page[pg], PAGE_SIZE); // page[] is the data from Emmc.
				have--;
				want++;
				if (++pg >= ring_size)
					pg = 0;
			}

			atomic_set(&data[thr].ready, 1);
			wake_up(&data[thr].go);	// WakeUp any Decompress thread in the threads.
		}

		/*
		 * Wait for more data while we are decompressing.
		 */
		if (have < LZ4_CMP_PAGES && asked) {
			ret = hib_wait_on_bio_chain(&bio);
			if (ret)
				goto out_finish;
			have += asked;
			asked = 0;
			if (eof)
				eof = 2;
		}

		/*	Stop Threads. & Copy UnCompressed Data to Allocated blank Area. 	*/
		for (run_threads = thr, thr = 0; thr < run_threads; thr++) {
			wait_event(data[thr].done,
			           atomic_read(&data[thr].stop));
			atomic_set(&data[thr].stop, 0);

			ret = data[thr].ret;

			if (ret < 0) {
				printk(KERN_ERR
				       "PM: LZ4 decompression failed\n");
				goto out_finish;
			}

			if (unlikely(!data[thr].unc_len ||
			             data[thr].unc_len > LZ4_UNC_SIZE ||
			             data[thr].unc_len & (PAGE_SIZE - 1))) {
				printk(KERN_ERR
				       "PM: Invalid LZ4 uncompressed length\n");
				ret = -1;
				goto out_finish;
			}
	
			/*	Copy UnCompressed DATA(32 PAGES) to allocated blank MEM Area  */
			for (off = 0;
			     off < data[thr].unc_len; off += PAGE_SIZE) {
				memcpy(data_of(*snapshot),
				       data[thr].unc + off, PAGE_SIZE);
				//	snapshot pointer is the blank area's Address.

				if (!(nr_pages % m))
					printk(KERN_INFO "PM: Image loading progress: %3d %% \n", nr_pages / m);
				nr_pages++;

				ret = snapshot_write_next(snapshot);
				if (ret <= 0) {
					/*	Do not use CRC32 with LZ4	*/
					//crc->run_threads = thr + 1;
					//atomic_set(&crc->ready, 1);
					//wake_up(&crc->go);
					goto out_finish;
				}
			}
		}

		/*	Do not user CRC32 with LZ4	*/
		//crc->run_threads = thr;
		//atomic_set(&crc->ready, 1);
		//wake_up(&crc->go);
	}

out_finish:

	/*	Combine Each Thread's master_checksum_thread to handle->master_checksum.	*/
	handle->master_checksum = 0;
	for (thr = 0; thr < nr_threads; thr++) {
		handle->master_checksum = handle->master_checksum ^ data[thr].master_checksum_thread;
	}

	/* 	Do not use CRC32 with LZ4	*/
	/*
	if (crc->run_threads) {
		wait_event(crc->done, atomic_read(&crc->stop));
		atomic_set(&crc->stop, 0);
	}
	*/
	do_gettimeofday(&stop);
	if (!ret) {
		printk(KERN_INFO "PM: Image loading progress: done. \n");
		snapshot_write_finalize(snapshot);
		if (!snapshot_image_loaded(snapshot))
			ret = -ENODATA;
		if (!ret) {
			/*		Compare Loaded Image's Master Checksum and Saved Image's Master Checksum.	*/
			if(handle->master_checksum != swsusp_header->crc32) {
				printk(KERN_ERR "PM: Invalid Master Checksum!\n");
				ret = -ENODATA;
			}
		}
	} else
		printk("\n");
	swsusp_show_speed(&start, &stop, nr_to_read, "Read");

out_clean:
	/*	STOP Threads. Release allocated Mem Spaces.	*/
	for (i = 0; i < ring_size; i++)
		free_page((unsigned long)page[i]);

	/*	Do not use CRC32 with LZ4	*/
	/*
	if (crc) {
		if (crc->thr)
			kthread_stop(crc->thr);
		kfree(crc);
	}
	*/

	if (data) {
		/*	STOP Decompression Threads	*/
		for (thr = 0; thr < nr_threads; thr++)
			if (data[thr].thr)
				kthread_stop(data[thr].thr);
		vfree(data);
	}
	if (page) vfree(page);

	return ret;
}
#endif



/**
 *	swsusp_read - read the hibernation image.
 *	@flags_p: flags passed by the "frozen" kernel in the image header should
 *		  be written into this memory location
 */

int swsusp_read(unsigned int *flags_p)
{
	int error;
	struct swap_map_handle handle;
	struct snapshot_handle snapshot;
	struct swsusp_info *header;

	memset(&snapshot, 0, sizeof(struct snapshot_handle));
	error = snapshot_write_next(&snapshot);
	if (error < PAGE_SIZE)
		return error < 0 ? error : -EFAULT;
	header = (struct swsusp_info *)data_of(snapshot);
	error = get_swap_reader(&handle, flags_p);
	if (error)
		goto end;
	if (!error)
		error = swap_read_page(&handle, header, NULL);
	if (!error) {
		error = (*flags_p & SF_NOCOMPRESS_MODE) ?
			load_image(&handle, &snapshot, header->pages - 1) :
		#if (defined(CONFIG_LZ4_COMPRESS) || defined(CONFIG_LZ4_HC_COMPRESS)) && defined (CONFIG_LZ4_DECOMPRESS)
			load_image_lz4_thread(&handle, &snapshot, header->pages - 1);
		#else
			load_image_lzo(&handle, &snapshot, header->pages - 1);
		#endif
	}
	swap_reader_finish(&handle);
end:
	if (!error)
		pr_debug("PM: Image successfully loaded\n");
	else
		pr_debug("PM: Error %d resuming\n", error);
	return error;
}

/**
 *      swsusp_check - Check for swsusp signature in the resume device
 */

int swsusp_check(void)
{
	int error;

	hib_resume_bdev = blkdev_get_by_dev(swsusp_resume_device,
					    FMODE_READ, NULL);
	if (!IS_ERR(hib_resume_bdev)) {
		set_blocksize(hib_resume_bdev, PAGE_SIZE);
		printk("swsusp_heder sig[%s] orig_sig[%s] sector[%lld]\n", swsusp_header->sig, swsusp_header->orig_sig, swsusp_header->image);
		clear_page(swsusp_header);
		error = hib_bio_read_page(swsusp_resume_block,
					swsusp_header, NULL);	// Read swap header from swap partition.
		if (error)
			goto put;
		printk("swsusp_heder sig[%s] orig_sig[%s] sector[%lld]\n", swsusp_header->sig, swsusp_header->orig_sig, swsusp_header->image);
		if (!memcmp(HIBERNATE_SIG, swsusp_header->sig, 10)) {
			memcpy(swsusp_header->sig, swsusp_header->orig_sig, 10);
			/* Reset swap signature now 
			 * Block following lines for permanent snapshot image. */
//			error = hib_bio_write_page(swsusp_resume_block,	
//						swsusp_header, NULL);
		} else {
			printk("%s SWSUSP_SIG cmp error\n", __FUNCTION__);
			error = -EINVAL;
		}

put:
		if (error)
			blkdev_put(hib_resume_bdev, FMODE_READ);
		else
			pr_debug("PM: Image signature found, resuming\n");
	} else {
		error = PTR_ERR(hib_resume_bdev);
	}

	if (error)
		pr_debug("PM: Image not found (code %d)\n", error);

	return error;
}

/**
 *	swsusp_close - close swap device.
 */

void swsusp_close(fmode_t mode)
{
	if (IS_ERR(hib_resume_bdev)) {
		pr_debug("PM: Image device not initialised\n");
		return;
	}

	blkdev_put(hib_resume_bdev, mode);
}

/**
 *      swsusp_unmark - Unmark swsusp signature in the resume device
 */

#ifdef CONFIG_SUSPEND
int swsusp_unmark(void)
{
	int error;

	hib_bio_read_page(swsusp_resume_block, swsusp_header, NULL);
	if (!memcmp(HIBERNATE_SIG,swsusp_header->sig, 10)) {
		memcpy(swsusp_header->sig,swsusp_header->orig_sig, 10);
		error = hib_bio_write_page(swsusp_resume_block,
					swsusp_header, NULL);
	} else {
		printk(KERN_ERR "PM: Cannot find swsusp signature!\n");
		error = -ENODEV;
	}

	/*
	 * We just returned from suspend, we don't need the image any more.
	 */
	free_all_swap_pages(root_swap);

	return error;
}
#endif

static int swsusp_header_init(void)
{
	swsusp_header = (struct swsusp_header*) __get_free_page(GFP_KERNEL);
	if (!swsusp_header)
		panic("Could not allocate memory for swsusp_header\n");
	return 0;
}

core_initcall(swsusp_header_init);
//-[TCCQB]
//
