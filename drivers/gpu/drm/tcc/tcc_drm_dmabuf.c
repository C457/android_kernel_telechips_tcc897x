/* tcc_drm_dmabuf.c
 *
 * Copyright (c) 2016 Telechips Inc.
 *
 * Author:  Telechips Inc.
 * Created: Jan 07, 2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *---------------------------------------------------------------------------
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 * Author: Inki Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <drm/drmP.h>
#include <drm/tcc_drm.h>
#include "tcc_drm_dmabuf.h"
#include "tcc_drm_drv.h"
#include "tcc_drm_gem.h"

#include <linux/dma-buf.h>

struct tcc_drm_dmabuf_attachment {
	struct sg_table sgt;
	enum dma_data_direction dir;
	bool is_mapped;
};

static struct tcc_drm_gem_obj *dma_buf_to_obj(struct dma_buf *buf)
{
	return to_tcc_gem_obj(buf->priv);
}

static int tcc_gem_attach_dma_buf(struct dma_buf *dmabuf,
					struct device *dev,
					struct dma_buf_attachment *attach)
{
	struct tcc_drm_dmabuf_attachment *tcc_attach;

	tcc_attach = kzalloc(sizeof(*tcc_attach), GFP_KERNEL);
	if (!tcc_attach)
		return -ENOMEM;

	tcc_attach->dir = DMA_NONE;
	attach->priv = tcc_attach;

	return 0;
}

static void tcc_gem_detach_dma_buf(struct dma_buf *dmabuf,
					struct dma_buf_attachment *attach)
{
	struct tcc_drm_dmabuf_attachment *tcc_attach = attach->priv;
	struct sg_table *sgt;

	if (!tcc_attach)
		return;

	sgt = &tcc_attach->sgt;

	if (tcc_attach->dir != DMA_NONE)
		dma_unmap_sg(attach->dev, sgt->sgl, sgt->nents,
				tcc_attach->dir);

	sg_free_table(sgt);
	kfree(tcc_attach);
	attach->priv = NULL;
}

static struct sg_table *
		tcc_gem_map_dma_buf(struct dma_buf_attachment *attach,
					enum dma_data_direction dir)
{
	struct tcc_drm_dmabuf_attachment *tcc_attach = attach->priv;
	struct tcc_drm_gem_obj *gem_obj = dma_buf_to_obj(attach->dmabuf);
	struct drm_device *dev = gem_obj->base.dev;
	struct tcc_drm_gem_buf *buf;
	struct scatterlist *rd, *wr;
	struct sg_table *sgt = NULL;
	unsigned int i;
	int nents, ret;

	/* just return current sgt if already requested. */
	if (tcc_attach->dir == dir && tcc_attach->is_mapped)
		return &tcc_attach->sgt;

	buf = gem_obj->buffer;
	if (!buf) {
		DRM_ERROR("buffer is null.\n");
		return ERR_PTR(-ENOMEM);
	}

	sgt = &tcc_attach->sgt;

	ret = sg_alloc_table(sgt, buf->sgt->orig_nents, GFP_KERNEL);
	if (ret) {
		DRM_ERROR("failed to alloc sgt.\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&dev->struct_mutex);

	rd = buf->sgt->sgl;
	wr = sgt->sgl;
	for (i = 0; i < sgt->orig_nents; ++i) {
		sg_set_page(wr, sg_page(rd), rd->length, rd->offset);
		rd = sg_next(rd);
		wr = sg_next(wr);
	}

	if (dir != DMA_NONE) {
		nents = dma_map_sg(attach->dev, sgt->sgl, sgt->orig_nents, dir);
		if (!nents) {
			DRM_ERROR("failed to map sgl with iommu.\n");
			sg_free_table(sgt);
			sgt = ERR_PTR(-EIO);
			goto err_unlock;
		}
	}

	tcc_attach->is_mapped = true;
	tcc_attach->dir = dir;
	attach->priv = tcc_attach;

	DRM_DEBUG_PRIME("buffer size = 0x%lx\n", buf->size);

err_unlock:
	mutex_unlock(&dev->struct_mutex);
	return sgt;
}

static void tcc_gem_unmap_dma_buf(struct dma_buf_attachment *attach,
						struct sg_table *sgt,
						enum dma_data_direction dir)
{
	/* Nothing to do. */
}

static void *tcc_gem_dmabuf_kmap_atomic(struct dma_buf *dma_buf,
						unsigned long page_num)
{
	/* TODO */

	return NULL;
}

static void tcc_gem_dmabuf_kunmap_atomic(struct dma_buf *dma_buf,
						unsigned long page_num,
						void *addr)
{
	/* TODO */
}

static void *tcc_gem_dmabuf_kmap(struct dma_buf *dma_buf,
					unsigned long page_num)
{
	/* TODO */

	return NULL;
}

static void tcc_gem_dmabuf_kunmap(struct dma_buf *dma_buf,
					unsigned long page_num, void *addr)
{
	/* TODO */
}

static int tcc_gem_dmabuf_mmap(struct dma_buf *dma_buf,
	struct vm_area_struct *vma)
{
	return -ENOTTY;
}

static struct dma_buf_ops tcc_dmabuf_ops = {
	.attach			= tcc_gem_attach_dma_buf,
	.detach			= tcc_gem_detach_dma_buf,
	.map_dma_buf		= tcc_gem_map_dma_buf,
	.unmap_dma_buf		= tcc_gem_unmap_dma_buf,
	.kmap			= tcc_gem_dmabuf_kmap,
	.kmap_atomic		= tcc_gem_dmabuf_kmap_atomic,
	.kunmap			= tcc_gem_dmabuf_kunmap,
	.kunmap_atomic		= tcc_gem_dmabuf_kunmap_atomic,
	.mmap			= tcc_gem_dmabuf_mmap,
	.release		= drm_gem_dmabuf_release,
};

struct dma_buf *tcc_dmabuf_prime_export(struct drm_device *drm_dev,
				struct drm_gem_object *obj, int flags)
{
	struct tcc_drm_gem_obj *tcc_gem_obj = to_tcc_gem_obj(obj);

	return dma_buf_export(obj, &tcc_dmabuf_ops,
				tcc_gem_obj->base.size, flags, NULL);
}

struct drm_gem_object *tcc_dmabuf_prime_import(struct drm_device *drm_dev,
				struct dma_buf *dma_buf)
{
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct scatterlist *sgl;
	struct tcc_drm_gem_obj *tcc_gem_obj;
	struct tcc_drm_gem_buf *buffer;
	int ret;

	/* is this one of own objects? */
	if (dma_buf->ops == &tcc_dmabuf_ops) {
		struct drm_gem_object *obj;

		obj = dma_buf->priv;

		/* is it from our device? */
		if (obj->dev == drm_dev) {
			/*
			 * Importing dmabuf exported from out own gem increases
			 * refcount on gem itself instead of f_count of dmabuf.
			 */
			drm_gem_object_reference(obj);
			return obj;
		}
	}

	attach = dma_buf_attach(dma_buf, drm_dev->dev);
	if (IS_ERR(attach))
		return ERR_PTR(-EINVAL);

	get_dma_buf(dma_buf);

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto err_buf_detach;
	}

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto err_unmap_attach;
	}

	tcc_gem_obj = tcc_drm_gem_init(drm_dev, dma_buf->size);
	if (!tcc_gem_obj) {
		ret = -ENOMEM;
		goto err_free_buffer;
	}

	sgl = sgt->sgl;

	buffer->size = dma_buf->size;
	buffer->dma_addr = sg_dma_address(sgl);

	if (sgt->nents == 1) {
		/* always physically continuous memory if sgt->nents is 1. */
		tcc_gem_obj->flags |= TCC_BO_CONTIG;
	} else {
		/*
		 * this case could be CONTIG or NONCONTIG type but for now
		 * sets NONCONTIG.
		 * TODO. we have to find a way that exporter can notify
		 * the type of its own buffer to importer.
		 */
		tcc_gem_obj->flags |= TCC_BO_NONCONTIG;
	}

	tcc_gem_obj->buffer = buffer;
	buffer->sgt = sgt;
	tcc_gem_obj->base.import_attach = attach;

	DRM_DEBUG_PRIME("dma_addr = %pad, size = 0x%lx\n", &buffer->dma_addr,
								buffer->size);

	return &tcc_gem_obj->base;

err_free_buffer:
	kfree(buffer);
	buffer = NULL;
err_unmap_attach:
	dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
err_buf_detach:
	dma_buf_detach(dma_buf, attach);
	dma_buf_put(dma_buf);

	return ERR_PTR(ret);
}

MODULE_AUTHOR("Inki Dae <inki.dae@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC DRM DMABUF Module");
MODULE_LICENSE("GPL");
