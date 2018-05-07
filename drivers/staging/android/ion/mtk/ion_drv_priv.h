/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __ION_DRV_PRIV_H__
#define __ION_DRV_PRIV_H__

#include <linux/miscdevice.h>
#include <linux/idr.h>
#include "mtk/ion_drv.h"
#include "ion_priv.h"

/**
 * struct ion_device - the metadata of the ion device node
 * @dev:		the actual misc device
 * @buffers:		an rb tree of all the existing buffers
 * @buffer_lock:	lock protecting the tree of buffers
 * @lock:		rwsem protecting the tree of heaps and clients
 * @heaps:		list of all the heaps in the system
 * @user_clients:	list of all the clients created from userspace
 */
struct ion_device {
	struct miscdevice dev;
	struct rb_root buffers;
	struct mutex buffer_lock;
	struct rw_semaphore lock;
	struct plist_head heaps;
	long (*custom_ioctl)(struct ion_client *client, unsigned int cmd,
			     unsigned long arg);
	struct rb_root clients;
	struct dentry *debug_root;
	struct dentry *heaps_debug_root;
	struct dentry *clients_debug_root;
};

/**
 * struct ion_client - a process/hw block local address space
 * @node:		node in the tree of all clients
 * @dev:		backpointer to ion device
 * @handles:		an rb tree of all the handles in this client
 * @idr:		an idr space for allocating handle ids
 * @lock:		lock protecting the tree of handles
 * @name:		used for debugging
 * @display_name:	used for debugging (unique version of @name)
 * @display_serial:	used for debugging (to make display_name unique)
 * @task:		used for debugging
 *
 * A client represents a list of buffers this client may access.
 * The mutex stored here is used to protect both handles tree
 * as well as the handles themselves, and should be held while modifying either.
 */
struct ion_client {
	struct rb_node node;
	struct ion_device *dev;
	struct rb_root handles;
	struct idr idr;
	struct mutex lock;
	const char *name;
	char *display_name;
	int display_serial;
	struct task_struct *task;
	pid_t pid;
	struct dentry *debug_root;
};

/**
 * ion_handle - a client local reference to a buffer
 * @ref:		reference count
 * @client:		back pointer to the client the buffer resides in
 * @buffer:		pointer to the buffer
 * @node:		node in the client's handle rbtree
 * @kmap_cnt:		count of times this client has mapped to kernel
 * @id:			client-unique id allocated by client->idr
 *
 * Modifications to node, map_cnt or mapping should be protected by the
 * lock in the client.  Other fields are never changed after initialization.
 */
struct ion_handle {
	struct kref ref;
	unsigned int user_ref_count;
	struct ion_client *client;
	struct ion_buffer *buffer;
	struct rb_node node;
	unsigned int kmap_cnt;
	int id;
};

/* STRUCT ION_HEAP *G_ION_HEAPS[ION_HEAP_IDX_MAX]; */

/* Import from multimedia heap */
long ion_mm_ioctl(struct ion_client *client, unsigned int cmd,
			 unsigned long arg, int from_kernel);

void smp_inner_dcache_flush_all(void);
#ifdef CONFIG_MTK_CACHE_FLUSH_RANGE_PARALLEL
int mt_smp_cache_flush(struct sg_table *table, unsigned int sync_type, int npages);
extern int (*ion_sync_kernel_func)(unsigned long start, size_t size, unsigned int sync_type);
#endif

#ifdef ION_HISTORY_RECORD
int ion_history_init(void);
#else
static inline int ion_history_init(void)
{
	return 0;
}
#endif

int ion_mm_heap_for_each_pool(int (*fn)(int high, int order, int cache, size_t size));
struct ion_heap *ion_drv_get_heap(struct ion_device *dev, int heap_id, int need_lock);
int ion_drv_create_heap(struct ion_platform_heap *heap_data);

#ifdef CONFIG_PM
void shrink_ion_by_scenario(void);
#endif

#endif
