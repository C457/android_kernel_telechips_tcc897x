/*
 * drivers/staging/android/ion/ion_carveout_heap.c
 *
 * Copyright (C) 2016 Telechips Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/spinlock.h>

#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "ion.h"
#include "ion_priv.h"

static int autofree_debug = 1;
#define dprintk(msg...)	if (autofree_debug) { printk( "autofree: " msg); }

struct ion_carveout_heap {
	struct ion_heap heap;
	struct gen_pool *pool;
	ion_phys_addr_t base;
};

size_t gHeapSize;
#if defined(CONFIG_SUPPORT_TCC_HEVC_4K)
#define AUTO_FREE_LENGTH 12
#else
#define AUTO_FREE_LENGTH 12
#endif

typedef struct af_desc {
	struct ion_buffer *buffer;
	struct gen_pool *pool;
	int valid;
} af_desc;

static int af_desc_head = 0;
static int af_desc_tail = 0;
static af_desc af_descbuf[AUTO_FREE_LENGTH];

static int is_af_descbuf_full(void)
{
	if (af_desc_tail == (af_desc_head + 1) % AUTO_FREE_LENGTH)
		return 1;
	return 0;
}


static void af_alloc(struct ion_buffer *buffer, struct ion_heap *heap)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);
	
	if (is_af_descbuf_full()) 
	{
		if (af_descbuf[af_desc_tail].valid) 
		{
			dprintk("autofree before buffer allocation head=%d, tail=%d, buffer:0x%x, addr:0x%x, size:0x%x\n",af_desc_head, af_desc_tail, af_descbuf[af_desc_tail].buffer, af_descbuf[af_desc_tail].buffer->priv_phys, af_descbuf[af_desc_tail].buffer->size);
			gen_pool_free(af_descbuf[af_desc_tail].pool, af_descbuf[af_desc_tail].buffer->priv_phys, af_descbuf[af_desc_tail].buffer->size);

			af_descbuf[af_desc_tail].buffer = NULL;
			af_descbuf[af_desc_tail].pool = NULL;
			af_descbuf[af_desc_tail].valid = 0;
		}
		af_desc_tail = (af_desc_tail + 1) % AUTO_FREE_LENGTH;
	}

	af_descbuf[af_desc_head].buffer = buffer;
	af_descbuf[af_desc_head].pool = carveout_heap->pool;
	af_descbuf[af_desc_head].valid = 1;
	dprintk("af_alloc head=%d, tail=%d, buffer:0x%x, addr:0x%x, size:0x%x\n",af_desc_head, af_desc_tail, af_descbuf[af_desc_head].buffer, af_descbuf[af_desc_head].buffer->priv_phys, af_descbuf[af_desc_head].buffer->size);
	af_desc_head = (af_desc_head + 1) % AUTO_FREE_LENGTH;
}

static int af_free(struct ion_buffer *buffer)
{
	int i;

	for (i = af_desc_tail; i != af_desc_head; i = (i + 1) % AUTO_FREE_LENGTH) 
	{
		if (af_descbuf[i].valid && af_descbuf[i].buffer == buffer) 
		{
			dprintk("%s Now Free i=%d, buffer:0x%x\n", __func__, i, af_descbuf[i].buffer);
			af_descbuf[i].buffer = NULL;
			af_descbuf[i].pool = NULL;
			af_descbuf[i].valid = 0;
			return 0;
		}
	}
	dprintk("%s Already Free buffer:0x%x\n", __func__, buffer);
	return 1;	/* already free  */
}

static int block_auto_free(struct ion_buffer *buffer, struct ion_heap *heap)
{
	int i;
	int existingNumfree;
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);

	for (i = af_desc_tail; i != af_desc_head; i = (i + 1) % AUTO_FREE_LENGTH) 
	{
		if (af_descbuf[i].valid) 
		{
			gen_pool_free(af_descbuf[i].pool, af_descbuf[i].buffer->priv_phys, af_descbuf[i].buffer->size);
			af_descbuf[i].buffer = NULL;
			af_descbuf[i].pool = NULL;
			af_descbuf[i].valid = 0;
		}
		af_desc_tail = (af_desc_tail + 1) % AUTO_FREE_LENGTH;
		if(gen_pool_avail(carveout_heap->pool) >= buffer->size)
		{
			dprintk("%s, Success af buffer head=%d, af buffer tail=%d, free=0x%x\n", __func__, af_desc_head, af_desc_tail, gen_pool_avail(carveout_heap->pool));
			return 1;
		}
	}

	printk("%s, Fail autofree buffer head=%d, af buffer tail=%d\n", __func__, af_desc_head, af_desc_tail);
	return 0;
}

ion_phys_addr_t ion_carveout_allocate(struct ion_heap *heap,
				      unsigned long size,
				      unsigned long align)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);
	unsigned long offset = gen_pool_alloc(carveout_heap->pool, size);

	//printk("Alloc %p - 0x%x\n", offset, size);
	if (!offset)
		return ION_CARVEOUT_ALLOCATE_FAIL;

	return offset;
}

void ion_carveout_free(struct ion_heap *heap, ion_phys_addr_t addr,
		       unsigned long size)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);

	//printk("Free %p - 0x%x \n", addr, size);
	if (addr == ION_CARVEOUT_ALLOCATE_FAIL)
		return;
	gen_pool_free(carveout_heap->pool, addr, size);
}

static int ion_carveout_heap_phys(struct ion_heap *heap,
				  struct ion_buffer *buffer,
				  ion_phys_addr_t *addr, size_t *len)
{
	*addr = buffer->priv_phys;
	*len = buffer->size;
	return 0;
}

static int ion_carveout_heap_allocate(struct ion_heap *heap,
				      struct ion_buffer *buffer,
				      unsigned long size, unsigned long align,
				      unsigned long flags)
{
	if (align > PAGE_SIZE)
		return -EINVAL;
#if 0//!defined(CONFIG_SUPPORT_TCC_HEVC_4K)
	if(size>0x800000)               //HDMI IN ->1920x1080x4 = 0x7e9000	
	{
		printk("%s is failed. The requested size is too big.\nPlease check CONFIG_SUPPORT_TCC_HEVC_4K\n", __func__);
		return -ENOMEM;
	}
#endif

	buffer->priv_phys = ion_carveout_allocate(heap, size, align);
	
	if(buffer->priv_phys == ION_CARVEOUT_ALLOCATE_FAIL)
	{
//		int bufferCnt = gHeapSize/size;
//		if(bufferCnt >= 6 && buffer->flags == ION_FLAG_AUTOFREE_ENABLE)
			block_auto_free(buffer, heap);
		return -ENOMEM;
	}
	else
	{
		buffer->size = size;
		if(buffer->flags == ION_FLAG_AUTOFREE_ENABLE)
			af_alloc(buffer, heap);

		return 0;
	}
}

static void ion_carveout_heap_free(struct ion_buffer *buffer)
{
	int already_free = 0;
	struct ion_heap *heap = buffer->heap;

	if(buffer->flags == ION_FLAG_AUTOFREE_ENABLE)
		already_free = af_free(buffer);

	if(!already_free)
	{
		ion_carveout_free(heap, buffer->priv_phys, buffer->size);
		buffer->priv_phys = ION_CARVEOUT_ALLOCATE_FAIL;
	}
}

static struct sg_table *ion_carveout_heap_map_dma(struct ion_heap *heap,
					      struct ion_buffer *buffer)
{
	struct sg_table *table;
	int ret;

	table = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!table)
		return ERR_PTR(-ENOMEM);
	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret) {
		kfree(table);
		return ERR_PTR(ret);
	}
	sg_set_page(table->sgl, phys_to_page(buffer->priv_phys), buffer->size,
		    0);
	return table;
}

static void ion_carveout_heap_unmap_dma(struct ion_heap *heap,
				 struct ion_buffer *buffer)
{
	sg_free_table(buffer->sg_table);
}

static struct ion_heap_ops carveout_heap_ops = {
	.allocate = ion_carveout_heap_allocate,
	.free = ion_carveout_heap_free,
	.phys = ion_carveout_heap_phys,
	.map_dma = ion_carveout_heap_map_dma,
	.unmap_dma = ion_carveout_heap_unmap_dma,
	.map_user = ion_heap_map_user,
	.map_kernel = ion_heap_map_kernel,
	.unmap_kernel = ion_heap_unmap_kernel,
};

struct ion_heap *ion_carveout_heap_create(struct ion_platform_heap *heap_data)
{
	struct ion_carveout_heap *carveout_heap;

	carveout_heap = kzalloc(sizeof(struct ion_carveout_heap), GFP_KERNEL);
	if (!carveout_heap)
		return ERR_PTR(-ENOMEM);

	carveout_heap->pool = gen_pool_create(12, -1);
	if (!carveout_heap->pool) {
		kfree(carveout_heap);
		return ERR_PTR(-ENOMEM);
	}
	carveout_heap->base = heap_data->base;
	gen_pool_add(carveout_heap->pool, carveout_heap->base, heap_data->size,
		     -1);
	carveout_heap->heap.ops = &carveout_heap_ops;
	carveout_heap->heap.type = ION_HEAP_TYPE_CARVEOUT;
//	carveout_heap->heap.flags = ION_HEAP_FLAG_DEFER_FREE;
	printk("%s heap size : 0x%x\n", __func__, heap_data->size);
	gHeapSize = heap_data->size;

	return &carveout_heap->heap;
}

void ion_carveout_heap_destroy(struct ion_heap *heap)
{
	struct ion_carveout_heap *carveout_heap =
	     container_of(heap, struct  ion_carveout_heap, heap);

	gen_pool_destroy(carveout_heap->pool);
	kfree(carveout_heap);
	carveout_heap = NULL;
}
