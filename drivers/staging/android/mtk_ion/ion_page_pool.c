// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/staging/android/mtk_ion/ion_page_pool.c
 *
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/sched/clock.h>
#include "ion_priv.h"

static unsigned long long last_alloc_ts;

/*
 * We avoid atomic_long_t to minimize cache flushes at the cost of possible
 * race which would result in a small accounting inaccuracy that we can
 * tolerate.
 */
static long nr_total_pages;

static void *ion_page_pool_alloc_pages(struct ion_page_pool *pool)
{
	unsigned long long start, end;
	struct page *page;
	unsigned int i;

	start = sched_clock();
	page = alloc_pages(pool->gfp_mask, pool->order);
	end = sched_clock();

	if ((end - start > 10000000ULL) &&
	    (end - last_alloc_ts > 1000000000ULL)) { /* unit is ns, 1s */
		IONMSG("warn: alloc pages order: %d time: %lld ns\n",
		       pool->order, end - start);
		show_free_areas(0, NULL);
		last_alloc_ts = end;
	}

	if (!page)
		return NULL;
	ion_pages_sync_for_device(g_ion_device->dev.this_device,
				  page, PAGE_SIZE << pool->order,
				  DMA_BIDIRECTIONAL);
	atomic64_add_return((1 << pool->order), &page_sz_cnt);
	for (i = 0; i < (1 << pool->order); i++)
		SetPageIommu(&page[i]);
	return page;
}

static void ion_page_pool_free_pages(struct ion_page_pool *pool,
				     struct page *page)
{
	__free_pages(page, pool->order);
	if (atomic64_sub_return((1 << pool->order), &page_sz_cnt) < 0) {
		IONMSG("underflow!, total_now[%ld]free[%lu]\n",
		       atomic64_read(&page_sz_cnt),
		       (unsigned long)(1 << pool->order));
		atomic64_set(&page_sz_cnt, 0);
	}
}

static int ion_page_pool_add(struct ion_page_pool *pool, struct page *page)
{
	mutex_lock(&pool->mutex);
	if (PageHighMem(page)) {
		list_add_tail(&page->lru, &pool->high_items);
		pool->high_count++;
	} else {
		list_add_tail(&page->lru, &pool->low_items);
		pool->low_count++;
	}

	nr_total_pages += 1 << pool->order;
	mutex_unlock(&pool->mutex);
	return 0;
}

static struct page *ion_page_pool_remove(struct ion_page_pool *pool, bool high)
{
	struct page *page;

	if (high) {
		BUG_ON(!pool->high_count);
		page = list_first_entry(&pool->high_items, struct page, lru);
		pool->high_count--;
	} else {
		BUG_ON(!pool->low_count);
		page = list_first_entry(&pool->low_items, struct page, lru);
		pool->low_count--;
	}

	list_del(&page->lru);
	nr_total_pages -= 1 << pool->order;
	mod_node_page_state(page_pgdat(page),
			    NR_KERNEL_MISC_RECLAIMABLE,
			    -(1 << pool->order));
	return page;
}

struct page *ion_page_pool_alloc(struct ion_page_pool *pool)
{
	struct page *page = NULL;

	BUG_ON(!pool);

	mutex_lock(&pool->mutex);
	if (pool->high_count)
		page = ion_page_pool_remove(pool, true);
	else if (pool->low_count)
		page = ion_page_pool_remove(pool, false);
	mutex_unlock(&pool->mutex);

	if (!page)
		page = ion_page_pool_alloc_pages(pool);

	return page;
}

void ion_page_pool_free(struct ion_page_pool *pool, struct page *page)
{
	int ret;

	if (pool->order != compound_order(page))
		IONMSG("free page = 0x%p, compound_order(page) = 0x%x",
		       page, compound_order(page));

	BUG_ON(pool->order != compound_order(page));

	ret = ion_page_pool_add(pool, page);
	if (ret)
		ion_page_pool_free_pages(pool, page);
}

static int ion_page_pool_total(struct ion_page_pool *pool, bool high)
{
	int count = pool->low_count;

	if (high)
		count += pool->high_count;

	return count << pool->order;
}

long ion_page_pool_nr_pages(void)
{
	/* Correct possible overflow caused by racing writes */
	if (nr_total_pages < 0)
		nr_total_pages = 0;
	return nr_total_pages;
}

int ion_page_pool_shrink(struct ion_page_pool *pool, gfp_t gfp_mask,
			 int nr_to_scan)
{
	int freed = 0;
	bool high;

	if (current_is_kswapd())
		high = true;
	else
		high = !!(gfp_mask & __GFP_HIGHMEM);

	if (nr_to_scan == 0)
		return ion_page_pool_total(pool, high);

	while (freed < nr_to_scan) {
		struct page *page;

		mutex_lock(&pool->mutex);
		if (pool->low_count) {
			page = ion_page_pool_remove(pool, false);
		} else if (high && pool->high_count) {
			page = ion_page_pool_remove(pool, true);
		} else {
			mutex_unlock(&pool->mutex);
			break;
		}
		mutex_unlock(&pool->mutex);
		ion_page_pool_free_pages(pool, page);
		freed += (1 << pool->order);
	}

	return freed;
}

struct ion_page_pool *ion_page_pool_create(gfp_t gfp_mask, unsigned int order,
					   bool cached)
{
	struct ion_page_pool *pool = kmalloc(sizeof(*pool), GFP_KERNEL);

	if (!pool) {
		IONMSG("%s kmalloc failed pool is null.\n", __func__);
		return NULL;
	}
	pool->high_count = 0;
	pool->low_count = 0;
	INIT_LIST_HEAD(&pool->low_items);
	INIT_LIST_HEAD(&pool->high_items);
	pool->gfp_mask = gfp_mask | __GFP_COMP;
	pool->order = order;
	mutex_init(&pool->mutex);
	plist_node_init(&pool->list, order);
	if (cached)
		pool->cached = true;

	return pool;
}

void ion_page_pool_destroy(struct ion_page_pool *pool)
{
	kfree(pool);
}

static int __init ion_page_pool_init(void)
{
	return 0;
}
device_initcall(ion_page_pool_init);
