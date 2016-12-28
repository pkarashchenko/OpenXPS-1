/************************************************************************
 * Copyright (C) 2016, Cavium, Inc.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * A copy of the GNU General Public License, version 2 is available in the file 
 * named LICENSE-GPLv2.md either in this directory or its root. 
 * Alernatively to obtain a copy, write to the Free Software Foundation, Inc., 
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * File: xp_netdev_core.c
 * 
 * Abstract: This file contains the netdev implementation requried for xpliant.
 ************************************************************************/
#include "xp_common.h"
#include "xp_netdev.h"

#define READ_REG    1U
#define WRITE_REG   0U

#define XPNET_MAX_DMA_SIZE              (4 << 10)

/* RX descriptor and its required bit fields */
#define XP_RX_DESCRIPTOR_BITOFF_ERRORINDICATION  (0)
#define XP_RX_DESCRIPTOR_BITOFF_OWNERSHIP        (4)
#define XP_TX_DESCRIPTOR_BITOFF_ERRORINDICATION  (0)
#define XP_TX_DESCRIPTOR_BITOFF_OWNERSHIP        (4)
#define XPNET_ENABLE_QUEUE                       (8)

/* ACQLOCK tells the callee function that a lock is to be acquired
 * if the lock is already acquired by the caller, this bit is set to 0 */
#define XPNET_AFLAG_ACQLOCK         (0x1)
#define XPNET_AFLAG_DMA             (0x10)

#define XPNET_HWFLAG_A0  0x1

extern int xp_netdev_mode_init(void);
extern int xp_netdev_mode_deinit(void);
extern void xp_rx_skb_process(xpnet_private_t *priv, struct sk_buff *skb);
extern int xpnet_proc_create(xpnet_private_t *net_priv);
extern int xp_dev_reg_read(u32 *rw_value, u32 reg_addr, 
                           u8 reg_size, xp_private_t *priv, u32 count);
extern int xp_dev_reg_write(u32 *rw_value, u32 reg_addr, 
                            u8 reg_size, xp_private_t *priv, u32 count);

typedef int32_t (*reg_rw_func)(xpnet_private_t *, u32, u8, u32 *, u32);

xpnet_private_t *g_net_priv;

int jiffies_defer = 5;

static int __xp_dev_reg_read(u32 *rw_value, u32 reg_addr, 
                             u8 reg_size, xp_private_t *priv)
{
    if (DWORD_SIZE != reg_size) {
        pr_err("%s: Invalid register reg_size = %x\n", __func__, reg_size);
        return -EINVAL;
    }

    *rw_value = *(u32*)((uint8_t*)(priv->vma) + reg_addr);
    /* pr_debug("rw_value = %x, reg_addr = 0x%x\n", *rw_value, reg_addr); */
    /* pr_debug("%s:%d priv->vma = 0x%p reg_addr = %p\n", __func__, __LINE__,
       priv->vma , ((uint8_t*)(priv->vma) + reg_addr)); */

    return 0;
}

static int __xp_dev_reg_write(u32 rw_value, u32 reg_addr, 
                              u8 reg_size, xp_private_t *priv)
{
    if (DWORD_SIZE != reg_size) {
        pr_err("%s: Invalid register reg_size = %x\n", __func__, reg_size);
        return -EINVAL;
    }

    *(u32*)((uint8_t*)(priv->vma) + reg_addr) = rw_value;
    /* pr_debug("rw_value = 0x%x, reg_addr = 0x%x\n", rw_value, reg_addr); */

    return 0;
}

static void __xpnet_endian_swap32(u32 *base, u32 buf_size, 
                                        xp_address_mode_t mode)
{
    uint16_t chr_num = 0;

    /* If device mode is A0 uncompress then only perform the swap. */
    if (mode == XP_A0_UNCOMPRESSED) {
        for (chr_num = 0; chr_num <= (buf_size - 1) / 4; chr_num++) {
            base[chr_num] = __builtin_bswap32(base[chr_num]) & 0xFFFFFFFF;
        }
    }
}

static int32_t __xp_dev_reg_read_q(xpnet_private_t *net_priv, u32 reg_id, 
                                   u8 data_size, u32 *value, u32 qnum)
{
    u32 reg_offset, i;

    reg_offset = 
        XP_GET_PCI_BASE_OFFSET_FROM_REG_NAME(reg_id, net_priv->pci_priv->mode);
    reg_offset = XP_GET_DMA_Q_REG_OFFSET(reg_offset, data_size, qnum);

    /* No testing performed, caller ensures enough allocation. */
    for (i = 0; i < data_size; i++) {
        __xp_dev_reg_read(&value[i], 
                          reg_offset + (i * sizeof(u32)), 
                          4, net_priv->pci_priv);
    }

    return 0;
}

int32_t xp_dev_reg_read_q(xpnet_private_t *net_priv, u32 reg_id, 
                          u8 data_size, u32 *value, u32 qnum)
{
    u32 reg_offset;

    reg_offset = 
        XP_GET_PCI_BASE_OFFSET_FROM_REG_NAME(reg_id, net_priv->pci_priv->mode);
    reg_offset = XP_GET_DMA_Q_REG_OFFSET(reg_offset, data_size, qnum);

    /* No testing performed, caller ensures enough allocation. */
    xp_dev_reg_read(value, reg_offset, 
                    4, net_priv->pci_priv, data_size);

    return 0;
}

static int32_t __xp_dev_write_q(xpnet_private_t *net_priv, u32 reg_id, 
                                u8 data_size, u32 *value, u32 qnum)
{
    u32 reg_offset, i;

    reg_offset = 
        XP_GET_PCI_BASE_OFFSET_FROM_REG_NAME(reg_id, net_priv->pci_priv->mode);
    reg_offset = XP_GET_DMA_Q_REG_OFFSET(reg_offset, data_size, qnum);

    for (i = 0; i < data_size; i++) {
        __xp_dev_reg_write(value[i], 
                           reg_offset + 
                           (i * sizeof(u32)), 4, net_priv->pci_priv);
    }

    return 0;
}

int32_t xp_dev_reg_write_q(xpnet_private_t *net_priv, u32 reg_id, 
                           u8 data_size, u32 *value, u32 qnum)
{
    u32 reg_offset;

    reg_offset = 
        XP_GET_PCI_BASE_OFFSET_FROM_REG_NAME(reg_id, net_priv->pci_priv->mode);
    reg_offset = XP_GET_DMA_Q_REG_OFFSET(reg_offset, data_size, qnum);

    xp_dev_reg_write(value, 
                     reg_offset, 
                     4, net_priv->pci_priv, data_size);
    return 0;
}

/* xpnet_set_tx_q_cdp
 *
 * @xpnet_priv  : xpnet_private structure
 * @q           : the tx_queue to be updated
 * @cdp         : cdp dma address
 *
 * DESC: the cdp is used only if the TX queue is not chained. If chained,
 * the queue's starting descriptor is the cdp and h/w will update it auto-
 * matically.
 */
static void xpnet_set_tx_q_cdp(xpnet_private_t *net_priv,
                            xpnet_tx_queue_struct_t *q, int desc_idx, u8 aflags)
{
    u64 base_cdp = 0;
    u32 reg = DMA0_RX_CDP_REG_E;

    if (q->xpq_type == XPNET_QUEUE_TYPE_RX) {
        pr_err("Error in %s - Invalid queue type\n", __func__);
        return;
    }

    base_cdp = (u64) q->dma +  (desc_idx * (sizeof(xpnet_descriptor_t)));
    fdebug("Entering %s(), cdp = %016llx\n", __func__, base_cdp);

    (aflags & XPNET_AFLAG_ACQLOCK) ?
        xp_dev_reg_write_q(net_priv, reg, 2, (u32 *) & base_cdp, q->xpq_id) :
        __xp_dev_write_q(net_priv, reg, 2, (u32 *) & base_cdp, q->xpq_id);
}

/* xpnet_set_rx_q_cdp
 *
 * @xpnet_priv  : xpnet_private structure
 * @q           : the rx_queue to be updated
 * @cdp         : cdp dma address
 *
 * DESC: the cdp is used only if the RX queue is not chained. If chained,
 * the queue's starting descriptor is the cdp and h/w will update it auto-
 * matically.
 */
static void xpnet_set_rx_q_cdp(xpnet_private_t *net_priv,
                            xpnet_queue_struct_t *q, int desc_idx, u8 aflags)
{
    int deschain = XPNET_DESC_CHAIN_INDEX(desc_idx);
    int deschain_idx = XPNET_DESC_INDEX_IN_CHAIN(desc_idx);
    u64 base_cdp = 0;
    u32 reg = DMA0_TX_CDP_REG_E;

    if (q->xpq_type == XPNET_QUEUE_TYPE_TX) {
        pr_err("Error in %s - Invalid queue type\n", __func__);
        return;
    }

    base_cdp = (u64) q->dma + (((deschain * q->xpq_num_desc) + deschain_idx) * (sizeof(xpnet_descriptor_t)));
    fdebug("Entering %s(), cdp = %016llx\n", __func__, base_cdp);

    (aflags & XPNET_AFLAG_ACQLOCK) ?
        xp_dev_reg_write_q(net_priv, reg, 2, (u32 *) & base_cdp, q->xpq_id) :
        __xp_dev_write_q(net_priv, reg, 2, (u32 *) & base_cdp, q->xpq_id);
}

u64 xpnet_get_q_cdp(xpnet_private_t *net_priv,
                    xpnet_queue_struct_t *q, u8 aflags)
{
    u64 base_cdp = 0;
    u32 reg = (q->xpq_type == XPNET_QUEUE_TYPE_TX) ?
        DMA0_RX_CDP_REG_E :
        DMA0_TX_CDP_REG_E;

    fdebug("Entering %s()\n", __func__);

    (aflags & XPNET_AFLAG_ACQLOCK) ?
        xp_dev_reg_read_q(net_priv, reg, 2, (u32 *) & base_cdp, q->xpq_id) :
        __xp_dev_reg_read_q(net_priv, reg, 2, (u32 *) & base_cdp, q->xpq_id);

    fdebug("Read cdp = %016llx\n", base_cdp);

    return base_cdp;
}


/*
 * __xpnet_enable_queue
 * @xpnet_priv : xpnet_private structure
 * @queue_id   : queue to be updated in register
 * @queue_type : type of queue
 * @enable     : if non zero, enables the queue, else
 *               disables the queue
 * @prio       : if > 8, keeps the priority,
 *               else overwrite with prio
 */
static void xpnet_queue_en_dis(xpnet_private_t *net_priv,
            	u32 queue_id, xpnet_enum_t queue_type,
                                u8 enable, u8 prio, u8 aflags)
{
    u32 reg = (queue_type == XPNET_QUEUE_TYPE_TX) ?
        DMA0_RX_CMD_REG_E :
        DMA0_TX_CMD_REG_E;
    u64 regval;
    reg_rw_func read_fn, write_fn;

    if (aflags & XPNET_AFLAG_ACQLOCK) {
        read_fn = xp_dev_reg_read_q;
        write_fn = xp_dev_reg_write_q;
    } else {
        read_fn = __xp_dev_reg_read_q;
        write_fn = __xp_dev_write_q;
    }

    /* fdebug("Entering %s(), qid = %d, en %d, flag %d\n",
              __func__, q->xpq_id, enable, aflags); */
    read_fn(net_priv, reg, 1, (u32 *) &regval, queue_id);

    if (prio > 7) {
        /* If prio > 7, use the existing priority. */
        prio = regval & 7;
    }

    /* Enable/disable the queue. */
    regval = (enable) ? XPNET_ENABLE_QUEUE | prio : prio;
    /* fdebug("Writing cmd reg %#x, val %#016llx\n", reg, regval); */
    write_fn(net_priv, reg, 1, (u32 *) & regval, queue_id);
}

/*
 * __xpnet_enable_queue
 * @xpnet_priv : xpnet_private structure
 * @q          : queue for which to check the status
 *
 * @return     : [bool] queue is enable or disable
 */
static int xpnet_get_queue_status(xpnet_private_t *net_priv,
                               xpnet_queue_struct_t *q, 
                               u8 aflags)
{
    u32 reg = (q->xpq_type == XPNET_QUEUE_TYPE_TX) ?
        DMA0_RX_CMD_REG_E :
        DMA0_TX_CMD_REG_E;
    u64 regval;
    reg_rw_func read_fn;

    if (aflags & XPNET_AFLAG_ACQLOCK) {
        read_fn = xp_dev_reg_read_q;
    } else {
        read_fn = __xp_dev_reg_read_q;
    }

    read_fn(net_priv, reg, 1, (u32 *) &regval, q->xpq_id);

    return (regval & XPNET_ENABLE_QUEUE);
}

static void reset_mgmt_dma(void)
{
    fdebug("Entering %s()\n", __func__);
}

static inline void xpnet_rx_desc_enable(xpnet_desc_struct_t *d)
{
    u64 regval = d->va->qword[0] & 0x1f;

    /* fdebug("Entering %s()\n", __func__); */
    regval |= (1 << XP_TX_DESCRIPTOR_BITOFF_OWNERSHIP);
    d->va->qword[0] = regval;
}

static void xpnet_rx_queue_desc_link(xpnet_queue_struct_t *q, int circular)
{
    int next = 0;
    int n = 0;
    int deschain = 0;

    /* fdebug("Entering %s()\n", __func__); */
    /* fdebug("q = %p\n", q); */

    /* Go through descriptors to create a linked list. */
    for (deschain = 0; deschain < XPNET_DESC_CHAIN_COUNT; deschain++) {
    	for (n = 0; n < q->xpq_num_desc; n++) {
    	    next = (n + 1) % q->xpq_num_desc;
    	    q->xpq_desc_meta[deschain][n].va->qword[3] =
    	    (u64) q->xpq_desc_meta[deschain][next].cdp;
    	}

    	if (!circular) {
    	    q->xpq_desc_meta[deschain][q->xpq_num_desc - 1].va->qword[3] = 0;
    	}
    }
}

static void xpnet_tx_queue_desc_link(xpnet_tx_queue_struct_t *q, int circular)
{
    int next = 0;
    int n = 0;

    /* fdebug("Entering %s()\n", __func__); */
    /* fdebug("q = %p\n", q); */

    /* Go through descriptors to create a linked list. */
    for (n = 0; n < q->xpq_num_desc; n++) {
        next = (n + 1) % q->xpq_num_desc;
        q->xpq_desc_meta[n].va->qword[3] =
        (u64) q->xpq_desc_meta[next].cdp;
    }

    if (!circular) {
        q->xpq_desc_meta[q->xpq_num_desc - 1].va->qword[3] = 0;
    }
}

static void xpnet_rx_queue_desc_change_own(xpnet_queue_struct_t *q, int cpu_owns)
{
    int n = 0;
    int deschain = 0;

    /* fdebug("Entering %s()\n", __func__); */

    if (cpu_owns) {
        /* Change the ownership bit of all descriptors of this queue. */
    	for (deschain = 0; deschain < XPNET_DESC_CHAIN_COUNT; deschain++) {
            for (n = 0; n < q->xpq_num_desc; n++) {
                xpnet_rx_desc_enable(&q->xpq_desc_meta[deschain][n]);
	    	}
        }
    }
}

static void xpnet_rx_queue_link_and_enable(xpnet_queue_struct_t *q)
{
    /* fdebug("Entering %s()\n", __func__); */

    /* In case of interrupt enabling, disable pre-empting. */
    xpnet_rx_queue_desc_link(q, 0);
    xpnet_rx_queue_desc_change_own(q, 1);
    q->head = 0;
    q->tail = 0; /* Indicate init complete. */
    q->status = XPNET_QUEUE_ACTIVE;
}

/* __xpnet_rx_desc_reset
 * Called with q->xpq_lock held
 * Does not set qword[3] to next for chaining
 */
static int __xpnet_rx_desc_reset(xpnet_queue_struct_t *q, int desc_idx)
{
    int next = xpnet_add_and_wrap(desc_idx, 1, q->xpq_num_desc);
    int deschain = XPNET_DESC_CHAIN_INDEX(desc_idx);
    int deschain_idx = XPNET_DESC_INDEX_IN_CHAIN(desc_idx);

    /* fdebug("Entering %s()\n", __func__); */

    q->xpq_desc_meta[deschain][deschain_idx].va->qword[3] = q->xpq_desc_meta[deschain][next].cdp;
    q->xpq_desc_meta[deschain][deschain_idx].va->qword[2] = q->xpq_desc_meta[deschain][deschain_idx].buf_sta.dma;
    q->xpq_desc_meta[deschain][deschain_idx].va->qword[1] = XPNET_MAX_PACKET_SIZE;
    q->xpq_desc_meta[deschain][deschain_idx].va->qword[0] =
        (1 << XP_TX_DESCRIPTOR_BITOFF_OWNERSHIP);

    return 0;
}

/*
 * @xpnet_priv  : allocated xpnet_private structure
 * @q           : rx queue that needs to be un-initilized
 * @meta_idx    : index of the meta data where skb allocation failed
 *
 * cleans up metadata of one specific queue till `meta_idx`
 * and then the queue itself
 * DOES NOT deallocate the queue itself
 */
static void xpnet_rx_queue_uninit(xpnet_private_t *net_priv, int queue, int meta_idx)
{
    int deschain = 0, deschain_idx = 0;
    int i = 0;
    u32 memlen = 
        XPNET_CEIL_LEN((XPNET_RX_NUM_DESCS * XPNET_DESC_SIZE), PAGE_SIZE);
    xpnet_desc_struct_t *d = NULL;
    xpnet_queue_struct_t *q = &net_priv->rx_queue[queue];

    /* fdebug("Entering %s()\n", __func__); */
    fdebug("Disabling RX queue %d, deschain_idx till %d\n", queue, meta_idx);

    /* Disable the queue in the command register. */
    xpnet_queue_en_dis(net_priv, q->xpq_id, q->xpq_type, 0, 0, XPNET_AFLAG_ACQLOCK);

    /* Uninit till failed index. */
    for (i = 0; i < meta_idx; i++) {
    	deschain = XPNET_DESC_CHAIN_INDEX(i);
    	deschain_idx = XPNET_DESC_INDEX_IN_CHAIN(i);

        d = &q->xpq_desc_meta[deschain][deschain_idx];
        memset(d->va, 0, sizeof(xpnet_descriptor_t));
        d->cdp = 0;

        if (d->buf_sta.va != NULL) {
            dma_free_coherent(&net_priv->pdev->dev, 
                              d->buf_sta.len, d->buf_sta.va, d->buf_sta.dma);
            d->buf_sta.va = NULL;
        }
    }

    /* Deallocate all coherent memory of the descriptors of this queue. */
    fdebug("dma_free_coherent memlen %d, va %p, dma %#llx\n", 
           memlen, q->va, q->dma);
    dma_free_coherent(&net_priv->pdev->dev, memlen, q->va, q->dma);
}

/*
 * @xpnet_priv  : allocated xpnet_private structure
 * @failed_idx  : index of the queue where allocation failed
 *
 * This function cleans up all queues till index `failed_idx`
 */
static void xpnet_rx_teardown(xpnet_private_t *net_priv, int failed_idx)
{
    /* Teardown from 0 -> failed_idx */
    int i = 0;

    /* fdebug("Entering %s()\n", __func__); */
    for (i = 0; i < failed_idx; i++) {
        xpnet_rx_queue_uninit(net_priv, i, net_priv->rx_queue[i].xpq_num_desc * XPNET_DESC_CHAIN_COUNT);
    }
}

static int xpnet_rx_desc_init(xpnet_queue_struct_t *q,
                              int desc_idx, xpnet_shards_struct_t *prealloc)
{
    int deschain = XPNET_DESC_CHAIN_INDEX(desc_idx);
    int deschain_idx = XPNET_DESC_INDEX_IN_CHAIN(desc_idx);

    xpnet_descriptor_t *d = (xpnet_descriptor_t *)(q->dma + (deschain * XPNET_RX_NUM_DESCS * sizeof(xpnet_descriptor_t)));
    xpnet_descriptor_t *v = (xpnet_descriptor_t *)(q->va + (deschain * XPNET_RX_NUM_DESCS * sizeof(xpnet_descriptor_t)));
    xpnet_desc_struct_t *desc = &q->xpq_desc_meta[deschain][deschain_idx];

    /* fdebug("Entering %s()\n", __func__); */
    desc->num_shards = 0;
    desc->va = & v[deschain_idx];
    desc->cdp = (dma_addr_t) & d[deschain_idx];

    desc->buf_sta.dma = prealloc->dma;
    desc->buf_sta.len = prealloc->len;
    desc->buf_sta.va = prealloc->va;

    /* Doing only one descriptor, so caller has to link. */
    /* Do not touch the NDP field. */
    /* desc->va->qword[3]; */
    desc->va->qword[2] = prealloc->dma;
    desc->va->qword[1] = prealloc->len;

    return 0;
}

static int xpnet_prealloc_buf(xpnet_private_t *net_priv, 
                              xpnet_shards_struct_t *tgt)
{
    dma_addr_t dma;
    /* Allocate a memory area with dma_alloc_coherent. */
    void  *va = dma_alloc_coherent(&net_priv->pdev->dev, 
                                   XPNET_MAX_PACKET_SIZE, &dma, GFP_KERNEL);
    if (!va) {
        fdebug("Memory allocation fail.\n");
        return -ENOMEM;
    }

    tgt->va = va;
    tgt->dma = dma;
    tgt->len = XPNET_MAX_PACKET_SIZE;
    return 0;
}

/*
 * Logically, rxqueue is device -> host, that is txqueue of device
 * This queue does NOT get drained fast, hence depth can be > 1
 * This functin will be used to add the queue logic.
 */
static int xpnet_rx_queue_init(xpnet_private_t *net_priv, int i)
{
    xpnet_queue_struct_t *q = NULL;
    u32 memlen = 0;
    int n = 0, rc = 0, chain = 0;
    xpnet_shards_struct_t prealloc;

    /* fdebug("Entering %s()\n", __func__); */
    if (i >= XPNET_RX_NUM_QUEUES) {
        fdebug("Invalid parameter %d (> XPNET_RX_NUM_QUEUES)\n", i);
        return -EINVAL;
    }

    q = &net_priv->rx_queue[i];
    /* Start initalizing all queue elements here. */

    /* Allocate RX descriptors just for this queue. */
    memlen = (XPNET_RX_NUM_DESCS * XPNET_DESC_SIZE * XPNET_DESC_CHAIN_COUNT);
    /* fdebug("Required memlen for RX : %#010x bytes\n", memlen); */
    memlen = XPNET_CEIL_LEN(memlen, PAGE_SIZE);
    /*fdebug("Allocated memlen for RX : %#010x bytes\n", memlen); */
    /* Allocate a memory area with dma_alloc_coherent. */
    q->va = dma_alloc_coherent(&net_priv->pdev->dev, 
                               memlen, &q->dma, GFP_KERNEL);
    if (!q->va) {
        fdebug("Memory allocation fail.\n");
        return -ENOMEM;
    }
    memset(q->va, 0, memlen);

    spin_lock_init(&q->xpq_lock);
    q->xpq_id = i;
    q->xpq_type = XPNET_QUEUE_TYPE_RX;
    q->xpq_num_desc = XPNET_RX_NUM_DESCS;
    q->status = XPNET_QUEUE_STOPPED;
    q->tail = 0;
    q->head = 0;
    q->xpq_deschain_idx = 0;
    q->xpq_next_deschain_idx = 0;

    /* START from here. */
    for (chain = 0; chain < XPNET_DESC_CHAIN_COUNT; chain++) {
    	for (n = 0; n < q->xpq_num_desc; n++) {
    	    if ((rc = xpnet_prealloc_buf(net_priv, &prealloc)) != 0) {
    	        /* Free all allocated in this queue before returning. */
    	        xpnet_rx_queue_uninit(net_priv, q->xpq_id, ((chain * q->xpq_num_desc) + n));
    	        return rc;
    	    }
    	    xpnet_rx_desc_init(q, ((chain * q->xpq_num_desc) + n), &prealloc);
    	}
    }
    
    /* Link and change the ownership bit. */
    xpnet_rx_queue_link_and_enable(q);

    /* Set the queue up with current descriptor pointer CDP. */
    xpnet_set_rx_q_cdp(net_priv, q, 0, XPNET_AFLAG_ACQLOCK);
    /* fdebug("Returning %s()\n", __func__); */
    return 0;
}

static int xpnet_rx_all_queues_start(xpnet_private_t *net_priv)
{
    int i = 0;

    /* fdebug("Entering %s()\n", __func__); */
    for (i = 0; i < net_priv->num_rxqueues; i++) {
        xpnet_queue_en_dis(net_priv, net_priv->rx_queue[i].xpq_id, net_priv->rx_queue[i].xpq_type, 1, 0,
                           XPNET_AFLAG_ACQLOCK);
    }

    return 0;
}

static int xpnet_rx_queue_setup(xpnet_private_t *net_priv, int num_queues)
{
    int i = 0, rc = 0;

    /* fdebug("Entering %s()\n", __func__); */
    /* RX descriptor allocation. */
    net_priv->num_rxqueues = num_queues;
    for (i = 0; i < num_queues; i++) {
        if ((rc = xpnet_rx_queue_init(net_priv, i)) < 0) {
            xpnet_rx_teardown(net_priv, i);
            return rc;
        }
    }

    return 0;
}

/* Initializes a descriptor (and its meta data in the queue)
 * @xpnet_priv  : xpnet private structure
 * @q           : tx queue
 * @desc_idx         : index that is to be filled
 */
static int xpnet_tx_desc_init(xpnet_private_t *net_priv,
                              xpnet_tx_queue_struct_t *q, int desc_idx)
{
    xpnet_descriptor_t *d = (xpnet_descriptor_t *)q->dma;
    xpnet_descriptor_t *v = (xpnet_descriptor_t *)q->va;
    xpnet_desc_struct_t *desc = &q->xpq_desc_meta[desc_idx];

    /* fdebug("Entering %s()\n", __func__); */
    desc->num_shards = 0;
    desc->va = &v[desc_idx];
    desc->cdp = (dma_addr_t) & d[desc_idx];

    desc->buf_sta.va = dma_alloc_coherent(&net_priv->pdev->dev, PAGE_SIZE,
                                          &(desc->buf_sta.dma), GFP_KERNEL);
    if (desc->va == NULL) {
        return -ENOMEM;
    }

    desc->buf_sta.len = 0;  /* Length of buffer filled, not allocated. */
    return XPNET_OK;
}

static void xpnet_tx_queue_uninit(xpnet_private_t *net_priv,
                                  xpnet_tx_queue_struct_t *q, int meta_idx)
{
    int i = 0;
    u32 memlen = 
        XPNET_CEIL_LEN((XPNET_RX_NUM_DESCS * XPNET_DESC_SIZE), PAGE_SIZE);
    xpnet_desc_struct_t *d = NULL;

    /* fdebug("Entering %s()\n", __func__); */
    /* Disable the queue in the command register. */
    xpnet_queue_en_dis(net_priv, q->xpq_id, q->xpq_type, 0, 0, XPNET_AFLAG_ACQLOCK);

    /* Uninit till failed index. */
    for (i = 0; i < meta_idx; i++) {
        d = &q->xpq_desc_meta[i];
        fdebug("i = %d, d = %p, d->va %p\n", i, d, d ? d->va : NULL);
        memset(d->va, 0, sizeof(xpnet_descriptor_t));
        d->cdp = 0;
        d->va = NULL;

        if (d->buf_sta.va != NULL) {
            dma_free_coherent(&net_priv->pdev->dev, 
                              PAGE_SIZE, d->buf_sta.va, d->buf_sta.dma);
            d->buf_sta.va = NULL;
            d->buf_sta.dma = 0;
        }
    }

    /* Deallocate all coherent memory of the descriptors of this queue. */
    fdebug("Freeing q->va %p, q->dma %#llx, memlen %d\n", 
           q->va, q->dma, memlen);
    dma_free_coherent(&net_priv->pdev->dev, memlen, q->va, q->dma);
}

static int xpnet_tx_queue_init(xpnet_private_t *net_priv, int i)
{
    xpnet_tx_queue_struct_t *q = NULL;
    u32 mem_len;
    int n = 0, rc = 0;

    /* fdebug("Entering %s()\n", __func__); */
    if (i >= net_priv->num_txqueues) {
        fdebug("Invalid parameter %d (> num_txqueues(%d))\n", 
               i, net_priv->num_txqueues);
        return -EINVAL;
    }

    q = &net_priv->tx_queue[i];

    /* Start initalizing all queue elements here. */
    spin_lock_init(&q->xpq_lock);
    q->xpq_id = i;
    q->xpq_type = XPNET_QUEUE_TYPE_TX;
    q->xpq_num_desc = XPNET_TX_NUM_DESCS;
    q->tail = 0;
    q->head = q->xpq_num_desc - 1;

    /* Allocate descriptors just for this queue. */
    mem_len = (XPNET_TX_NUM_DESCS * XPNET_DESC_SIZE);
    fdebug("Required memlen for TX : %#010x bytes.\n", mem_len);
    mem_len = XPNET_CEIL_LEN(mem_len, PAGE_SIZE);
    fdebug("Allocated memlen for TX : %#010x bytes.\n", mem_len);

    /* Allocate a memory area with dma_alloc_coherent. */
    q->va = dma_alloc_coherent(&net_priv->pdev->dev, 
                               mem_len, &q->dma, GFP_KERNEL);
    if (!q->va) {
        fdebug("Memory allocation fail.\n");
        return -ENOMEM;
    }

    fdebug("TX q->va %p, q->dma %llx\n", q->va, q->dma);
    memset(q->va, 0, mem_len);

    for (n = 0; n < q->xpq_num_desc; n++) {
        rc = xpnet_tx_desc_init(net_priv, q, n);
        if (rc != XPNET_OK) {
            fdebug("TX_DESC_INIT FAILED, %d\n", n);
            xpnet_tx_queue_uninit(net_priv, q, n);
            return rc;
        }
    }

    if ((net_priv->hw_flags & XPNET_HWFLAG_A0) == 0) {
        /* HW is not A0, can go ahead and link. */
        xpnet_tx_queue_desc_link(q, 0);
    }

    /* Set the queue up with current descriptor pointer CDP. */
    xpnet_set_tx_q_cdp(net_priv, q, 0, XPNET_AFLAG_ACQLOCK);
    q->status = XPNET_QUEUE_ACTIVE;
    return XPNET_OK;
}

static void xpnet_tx_teardown(xpnet_private_t *net_priv, int failed_idx)
{
    xpnet_tx_queue_struct_t *q = NULL;
    /* Teardown from 0 -> failed_idx */
    int i = 0;

    /* fdebug("Entering %s()\n", __func__); */
    for (i = 0; i < failed_idx; i++) {
        q = &net_priv->tx_queue[i];
        xpnet_tx_queue_uninit(net_priv, q, q->xpq_num_desc);
    }
}

static int xpnet_tx_queue_setup(xpnet_private_t *net_priv, int num_queues)
{
    int i = 0, rc = 0;

    /* fdebug("Entering %s()\n", __func__); */
    net_priv->num_txqueues = num_queues;
    /* TX descriptor allocation. */
    for (i = 0; i < num_queues; i++) {
        if ((rc = xpnet_tx_queue_init(net_priv, i)) < 0) {
            xpnet_tx_teardown(net_priv, i);
            return rc;
        }
    }

    return 0;
}

/*
 * xpnet_program_mux_setdma
 *
 * programs the mux for GMAC or DMA
 * @flag  : 0 -> GMAC
 *        : 1 -> DMA
 */
static void xpnet_program_mux_setdma(xpnet_private_t *net_priv, u32 flag)
{
    u32 regval, reg;

    /* fdebug("Entering %s()\n", __func__); */
    reg = MGMT_CTRL_REG_E;
    xp_dev_reg_read_q(net_priv, reg, 1, &regval, 0);
    fdebug("Mux register %x.\n", regval);

    if (flag != 0) {
        /* Set to DMA */
        regval |= (1 << 3);
    } else {
        /* Set to GMAC */
        regval &= ~(1 << 3);
    }

    xp_dev_reg_write_q(net_priv, reg, 1, &regval, 0);
    udelay(10);
    xp_dev_reg_read_q(net_priv, reg, 1, &regval, 0);

    fdebug("Mux register set to %x.\n", regval);
}

static void xpnet_rx_queue_process(xpnet_private_t *net_priv, 
            	int queue, int maxiter)
{
	unsigned long flags = 0;
	uint16_t pkt_size = XPNET_MAX_PACKET_SIZE;
	dma_addr_t dma;
	xpnet_desc_struct_t *d = NULL;
	xpnet_queue_struct_t *q = &net_priv->rx_queue[queue];
	int deschain = 0;
	struct sk_buff *skb = NULL;
	/* process_flag will be used to identify if particular chain is ready for processing - if reception completed for chain or not */
	static unsigned char process_flag = 0;

	/* fdebug("Entering %s()\n", __func__); */
	/* Continious interations till the packets are coming - will break on error and exist this function if packets are not available */
	while(1) {

            spin_lock_irqsave(&q->xpq_lock, flags);

            /* If queue disable then set the CDP to next chain and turn the queue on  */	
            if(!xpnet_get_queue_status(net_priv, q, XPNET_AFLAG_ACQLOCK))
            {
                    deschain = q->xpq_next_deschain_idx + 1;
                    if(deschain >= XPNET_DESC_CHAIN_COUNT)
                    {
                        	deschain= 0;
                    }

                    //In case of multiple chain - Set the next chain for reception, meanwhile scanning can performed parallely on current chain
                    if(q->xpq_deschain_idx != deschain)
                    {
                            if(++(q->xpq_next_deschain_idx) >= XPNET_DESC_CHAIN_COUNT)
                            {
                                    q->xpq_next_deschain_idx = 0;
                            }

                            process_flag = 0;
                            xpnet_set_rx_q_cdp(net_priv, q, (q->xpq_next_deschain_idx * q->xpq_num_desc), XPNET_AFLAG_ACQLOCK);
                            xpnet_queue_en_dis(net_priv, q->xpq_id, q->xpq_type, 1, 0, XPNET_AFLAG_ACQLOCK);
                            pr_info("Descriptor chain changeover - processing chain : %d, receiving chain : %d\n", q->xpq_deschain_idx, q->xpq_next_deschain_idx);
                    }
            }

            /* Check if any descriptor chain available for processing */
            if(process_flag)
            {
                    spin_unlock_irqrestore(&q->xpq_lock, flags);
                    return;
            }

            d = &q->xpq_desc_meta[q->xpq_deschain_idx][q->tail];
            fdebug("Rx tail = %d\n", q->tail);

            /* Check error first. */
            if ((d->va->qword[0] >> 
                                    XP_RX_DESCRIPTOR_BITOFF_ERRORINDICATION) == 1) {
                    fdebug("Dma error. Resetting the descriptor.\n");
                    __xpnet_rx_desc_reset(q, q->tail);
                    xpnet_rx_desc_enable(d);

                    net_priv->stats.rx_dropped++;
                    net_priv->stats.rx_length_errors++;
                    spin_unlock_irqrestore(&q->xpq_lock, flags);
                    break;
            }

            /* Check completion if no error. */
            if ((d->va->qword[0] >> XP_RX_DESCRIPTOR_BITOFF_OWNERSHIP) & 0x01) {
                    //fdebug("Dma still owns the desc. Rx in progress.\n");
                    spin_unlock_irqrestore(&q->xpq_lock, flags);
                    return;
            }

            /* Received a packet from HW. Set the next packet read delay to 0.
               It means that may be we have one more packet
               that should be read immediately. */
            jiffies_defer = 0;

            /* Process one complete packet : SOP to EOP */
            dma = d->buf_sta.dma;

            pkt_size = (d->va->qword[1] >> 16) & 0xffff;

            if (net_priv->pci_priv->mode == XP_A0_UNCOMPRESSED) {
                    pkt_size -= 16;
            }

            if (pkt_size > XPNET_MAX_PACKET_SIZE) {
                    __xpnet_rx_desc_reset(q, q->tail);
                    xpnet_rx_desc_enable(d);
                    net_priv->stats.rx_dropped++;
                    spin_unlock_irqrestore(&q->xpq_lock, flags);
                    break;
            }

            skb = dev_alloc_skb(pkt_size);
            if (!skb) {
                    spin_unlock_irqrestore(&q->xpq_lock, flags);
                    fdebug("No skb available.\n");
                    break;
            }

            __xpnet_endian_swap32(d->buf_sta.va, pkt_size, 
                        	net_priv->pci_priv->mode);

            memcpy(skb->data, d->buf_sta.va, pkt_size);
            skb->len = pkt_size;

            /* Reset the length of dma bufffer. */
            d->va->qword[1] = d->buf_sta.len;

            /* Clear other bits set in the qword[0]. */
            d->va->qword[0] = (1 << XP_RX_DESCRIPTOR_BITOFF_OWNERSHIP);
            q->tail = xpnet_add_and_wrap(q->tail, 1, q->xpq_num_desc);

            if (q->tail == 0) {
                    if(++(q->xpq_deschain_idx) >= XPNET_DESC_CHAIN_COUNT)
                    {
                        	q->xpq_deschain_idx = 0;
                    }

                    fdebug("q->tail == 0, cdp reassign to next chain\n");
                    if(XPNET_DESC_CHAIN_COUNT == 1)
                    {
                        	xpnet_set_rx_q_cdp(net_priv, q, 0, XPNET_AFLAG_ACQLOCK);
                        	xpnet_queue_en_dis(net_priv, q->xpq_id, q->xpq_type, 1, 0, XPNET_AFLAG_ACQLOCK);
                    }
                    else
                    {
                        	if(q->xpq_deschain_idx == q->xpq_next_deschain_idx)
                        	{
                                    process_flag = 1;
                        	}
                    }
            }

            spin_unlock_irqrestore(&q->xpq_lock, flags);
            xp_rx_skb_process(net_priv, skb);
	}

	pr_info("Error Reported in RX Part\n");
}

static int xpnet_rx_handle(xpnet_private_t *net_priv)
{
    int i = 0;
    unsigned long flags = 0;

    spin_lock_irqsave(&net_priv->priv_lock, flags);
    if (0 == net_priv->dma_trigger) {
        printk("DMA engine not enabled.\n");

        /* Schedule rx handler after some time. */
        jiffies_defer = 3 * HZ;
        spin_unlock_irqrestore(&net_priv->priv_lock, flags);
        return 0;
    }

    spin_unlock_irqrestore(&net_priv->priv_lock, flags);
    /* fdebug("Entering %s()\n", __func__); */

    for (i = 0; i < XPNET_RX_NUM_QUEUES; i++) {
        xpnet_rx_queue_process(net_priv, i, XPNET_RX_NUM_DESCS);
    }

    return 0;
}

/*
 * __xpnet_skb_to_shards
 * Splits the skb to shards
 */
static int __xpnet_skb_to_shards(struct sk_buff *skb, xpnet_tx_queue_struct_t *q, 
                                 xpnet_private_t *net_priv)
{
    /* Only the first packet should set SOP, only the last packet
     * should set EOP, length should be a multiple of 8 bytes
     * for all except last. */
    int copied = 0, n = q->tail, max = skb->len, desc_idx = 0, b2c = 0;
    xpnet_desc_struct_t *d;

    /* fdebug("Entering %s()\n", __func__); */

    while (copied < max) {
        d = &q->xpq_desc_meta[n];
        b2c = ((max - copied) < XPNET_MAX_DMA_SIZE) ? (max - copied) :
            XPNET_MAX_DMA_SIZE;

        d->buf_sta.len = b2c;

        /* Handle short packet. */
        if (max < 128) {
            memset(d->buf_sta.va, 0x00, 128);
            d->buf_sta.len = 128;
        }

        memcpy(d->buf_sta.va, skb->data + copied, b2c);
        __xpnet_endian_swap32(d->buf_sta.va, 
                              d->buf_sta.len, net_priv->pci_priv->mode);
        copied += b2c;
        n = xpnet_add_and_wrap(n, 1, q->xpq_num_desc);
        desc_idx++;
    }

    return desc_idx;
}

static int __xpnet_skb_stat_enqueue(xpnet_private_t *net_priv,
                                    xpnet_tx_queue_struct_t *q, int shards)
{
    int desc_idx, max = shards;
    xpnet_desc_struct_t *d = &q->xpq_desc_meta[q->tail];

    /* fdebug("Entering %s()\n", __func__); */
    /* Check for space for all shards. */
    for (desc_idx = q->tail; max > 0; --max) {
        d = &q->xpq_desc_meta[desc_idx];

        if ((d->va->qword[0] >> XP_TX_DESCRIPTOR_BITOFF_OWNERSHIP) & 0x01) {
            /* At least one element is still busy. */
            fdebug("Queue busy = %d\n", desc_idx);
            return XPNET_QUEUE_BUSY;
        }

        if ((d->va->qword[0] >> 
             XP_TX_DESCRIPTOR_BITOFF_ERRORINDICATION) & 0x01) {
            fdebug("Queue error = %d\n", desc_idx);
            /* At least one element of the queue is hosed. */
            return XPNET_QUEUE_ERROR;
        }

        desc_idx = xpnet_add_and_wrap(desc_idx, 1, q->xpq_num_desc);
    }

    max = shards;

    for (desc_idx = q->tail; max > 0; --max) {
        d = &q->xpq_desc_meta[desc_idx];
        //d->va->qword[3] = 0; /* Will be reset in the next go. */
        d->va->qword[2] = d->buf_sta.dma;
        d->va->qword[1] = d->buf_sta.len;

        if (desc_idx == q->tail) {
            /* First element, set SOP only. */
            d->va->qword[0] = (1 << 3);
        } else {
            d->va->qword[0] = (1 << XP_TX_DESCRIPTOR_BITOFF_OWNERSHIP);
        }

        desc_idx = xpnet_add_and_wrap(desc_idx, 1, q->xpq_num_desc);
    }

    /* Set EOP for last element. */
    d->va->qword[0] |= (1 << 2); /* Keep the ownership bit. */

    /* Now all except the first one has ownership bit set. */
    d = &q->xpq_desc_meta[q->tail];
    d->num_shards = shards;
    d->va->qword[0] |= (1 << XP_TX_DESCRIPTOR_BITOFF_OWNERSHIP);

    if (net_priv->hw_flags & XPNET_HWFLAG_A0) {
        xpnet_set_tx_q_cdp(net_priv, q, q->tail, 0);
        /* Do not lock, enable with priority 0. */
        xpnet_queue_en_dis(net_priv, q->xpq_id, q->xpq_type, 1, 0, 0);
    }

    return XPNET_OK;
}

/*
 * __xpnet_enforce_skb_sanity() checks 3 constraints
 * 1. if the skb->len is a multiple of 4
 * 2. if the skb->data starts at 4kB boundary
 * 3. if the skb->data + len crosses 4kB boundary
 *
 * If any of these constraints are met, the buffer is split into
 * chunks of size <= 4kB
 */
static int __xpnet_enforce_skb_sanity(xpnet_private_t *net_priv,
                                      xpnet_tx_queue_struct_t *q,
                                      struct sk_buff *skb)
{
    int new_len = 0;

    /* fdebug("Entering %s()\n", __func__); */
     
    if ((net_priv->hw_flags & XPNET_HWFLAG_A0) == 0) {
        return XPNET_OK;
    }

    /* Check if the skb address+len crosses 4k boundary
     * Since IOMMU is present, it happens only if skb->len > 4kB */
    new_len = XPNET_CEIL_LEN(skb->len, 4);
    if (new_len > XPNET_MAX_DMA_SEGMENT_SIZE) {
        fdebug("Crosses 4k boundary\n");
        return XPNET_REASON_4KCROSS;
    }

    /* Check if skb->len < (60 + 24) */
    if (skb->len < XPNET_MIN_PACKET_SIZE) {
        fdebug("Short packet %d\n", skb->len);
        return XPNET_REASON_SHORTPACKET;
    }

    return XPNET_OK;
}

static void start_dma_reset(xpnet_private_t *net_priv)
{
    /* fdebug("Entering %s()\n", __func__); */
}

static netdev_tx_t xpnet_hard_start_xmit(struct sk_buff *skb,
                                         xpnet_private_t *net_priv, int txqno)
{
    unsigned long flags = 0;
    int shards = 0;
    int rc = 0;
    int num_free_desc = 0;
    xpnet_enum_t skb_good = 0;
    xpnet_desc_struct_t *d = NULL;
    xpnet_tx_queue_struct_t *q = &net_priv->tx_queue[txqno];

    /* fdebug("Entering %s()\n", __func__); */
    spin_lock_irqsave(&q->xpq_lock, flags);

    if (q->status != XPNET_QUEUE_ACTIVE) {
        net_priv->stats.tx_errors++;
        fdebug("Internal queue(%d) not active.\n", txqno);
        goto ret_txdrop;
    }

    if (skb == NULL) {
        fdebug("Null skb.\n");
        goto ret_txdrop;
    }

    /* Must lock q before calling. */
    num_free_desc = xpnet_get_free_desc(q);
    /* fdebug("num_free_desc = %d, tail %d,
       head %d\n", num_free_desc, q->tail, q->head); */

    if (num_free_desc == 0) {
        /* Queue is full, can't xmit. */
        goto ret_txbusy;
    }

    if ((skb->len == 0) || 
        (skb->len > (num_free_desc * XPNET_MAX_DMA_SEGMENT_SIZE))) {
        net_priv->stats.tx_errors++;
        fdebug("Skb len : %d\n", skb->len);
        goto ret_txdrop;
    }

    if (skb->data_len != 0) {
        /* Contains fragments. Currently not handling in the driver. */
        net_priv->stats.tx_errors++;
        goto ret_txdrop;
    }

    d = &q->xpq_desc_meta[q->tail];

    /* fdebug("q->tail = %d\n", n); */
    if ((d->va->qword[0] >> XP_TX_DESCRIPTOR_BITOFF_OWNERSHIP) & 0x01) {
        /* Queue still busy. */
        goto ret_txbusy;
    }

    if ((d->va->qword[0] >> XP_TX_DESCRIPTOR_BITOFF_ERRORINDICATION) & 0x01) {
        /* Queue is hosed. Need a dma reset. */
        start_dma_reset(net_priv);
        goto ret_txbusy;
    }

    /* Queue is ready and has space to transmit. */
    skb_good = __xpnet_enforce_skb_sanity(net_priv, q, skb);
    /* Need to copy the buffer to shard (buf_sta). */
    shards = __xpnet_skb_to_shards(skb, q, net_priv);

    /* The skb is split to n->n+shards descriptors. */
    if (shards > num_free_desc) {
        fdebug("CORRUPTION : shards(%d) > num_free_desc(%d)",
               shards, num_free_desc);
    }

    rc = __xpnet_skb_stat_enqueue(net_priv, q, shards);
    if (rc == XPNET_QUEUE_BUSY) {
        goto ret_txbusy;
    } else if (rc == XPNET_QUEUE_ERROR) {
        net_priv->stats.tx_aborted_errors++;
        net_priv->stats.tx_dropped++;
        start_dma_reset(net_priv);
        goto ret_txbusy;
    }

    /* Data is copied, don't need skb anymore */
    dev_kfree_skb_any(skb);
    net_priv->stats.tx_packets++;
    net_priv->stats.tx_bytes += skb->len;
    q->tail = xpnet_add_and_wrap(q->tail, shards, q->xpq_num_desc);

    goto ret_txok;

ret_txbusy:
    fdebug("Returning txbusy, q->head %d, q->tail %d\n", q->head, q->tail);
    spin_unlock_irqrestore(&q->xpq_lock, flags);
    return NETDEV_TX_BUSY;

ret_txdrop:
    fdebug("Packet dropped.\n");
    dev_kfree_skb_any(skb);
    net_priv->stats.tx_dropped++;
    spin_unlock_irqrestore(&q->xpq_lock, flags);
    return NETDEV_TX_OK;

ret_txok:
    spin_unlock_irqrestore(&q->xpq_lock, flags);
    return NETDEV_TX_OK;
}

static void xpnet_rxtx_handler(struct work_struct *w)
{
    xpnet_private_t *priv = container_of(w, xpnet_private_t, dwork.work);

    /* Set read packet delay to 1 jiffies.
       The value could be overriden inside xpnet_rx_queue_process(). */
    jiffies_defer = 1; 

    /* fdebug("Entering %s()\n", __func__); */
    xpnet_rx_handle(priv);

    /* Queue self, no interrupt to queue self. */
    queue_delayed_work(priv->wqueue, &priv->dwork, jiffies_defer);
}

static void xpnet_tx_complete(xpnet_private_t *net_priv, int qno, int maxiter)
{
    xpnet_tx_queue_struct_t *q = &net_priv->tx_queue[qno];
    xpnet_desc_struct_t *d = NULL;
    int head, tail;
    int iter, busy;
    unsigned long flags = 0;
    int attempt = 0;

    fdebug("Entering %s(), q = %d, maxiter %d\n", __func__, qno, maxiter);
    spin_lock_irqsave(&q->xpq_lock, flags);

    head = q->head;
    tail = q->tail;
    busy = xpnet_get_busy_desc(q); /* Must lock q before calling. */
    maxiter = (busy > maxiter) ? maxiter : busy;
    fdebug("Head %d, tail %d, processing %d pkts.\n", head, tail, busy);

    for (iter = 0; iter < maxiter; iter++) {
        attempt = 0;
        head = xpnet_add_and_wrap(head, 1, q->xpq_num_desc);

        if (head == tail) {
            /* Empty queue*/
            fdebug("Empty queue.\n");
            break;
        }

        d = &q->xpq_desc_meta[head];
        if (((d->va->qword[0] >> 
              XP_TX_DESCRIPTOR_BITOFF_ERRORINDICATION) & 1) == 1) {
            /* Needs a dma reset. */
            reset_mgmt_dma();
            fdebug("Tx error.\n");
            break;
        }

        while ((attempt++ < XPNET_MAX_ATTEMPTS) && 
               (((d->va->qword[0] >> 
                  XP_TX_DESCRIPTOR_BITOFF_OWNERSHIP) & 0x01))) {
            fdebug("TX in progress.\n");
            /* TX in progress. */
            udelay(5);
        }

        if ((d->va->qword[0] >> XP_TX_DESCRIPTOR_BITOFF_OWNERSHIP) & 0x01) {
            /* TX still incomplete. */
            fdebug("TX still incomplete : %d\n", attempt);
            break;
        }

        q->head = xpnet_add_and_wrap(q->head, 1, q->xpq_num_desc);
    }

    if (iter != maxiter) {
        fdebug("Ownership is not released and still releasing the lock.\n");
    }

    if ((net_priv->hw_flags & XPNET_HWFLAG_A0) != 0) {
        q->head = q->xpq_num_desc - 1;
        q->tail = 0;
    }

    spin_unlock_irqrestore(&q->xpq_lock, flags);
}

netdev_tx_t xpnet_start_xmit(struct sk_buff *skb, xpnet_private_t *net_priv)
{
    netdev_tx_t rc = NETDEV_TX_OK;
    unsigned long flags = 0;

    spin_lock_irqsave(&net_priv->priv_lock, flags);

    if (0 == net_priv->dma_trigger) {
        printk("DMA engine not enabled.\n");
        dev_kfree_skb(skb);
        spin_unlock_irqrestore(&net_priv->priv_lock, flags);
        return NETDEV_TX_OK;
    }

    spin_unlock_irqrestore(&net_priv->priv_lock, flags);

    /* fdebug("Entering %s()\n", __func__); */
    /* Move locking from here. */
    spin_lock_irqsave(&net_priv->pci_priv->tx_dma_read_lock, flags);

    rc = xpnet_hard_start_xmit(skb, net_priv, net_priv->txqno);
    if (rc == NETDEV_TX_OK) {
        /* Handle tx complete. */
        xpnet_tx_complete(net_priv, net_priv->txqno,
        net_priv->tx_queue[net_priv->txqno].xpq_num_desc);
    }
   
    /* Move locking from here. */
    spin_unlock_irqrestore(&net_priv->pci_priv->tx_dma_read_lock, flags);
    return rc;
}

int xp_netdev_init(xp_private_t *priv)
{
    char queue_name[25];
    int rc = 0;
    unsigned long flags = 0;
    xpnet_private_t *net_priv = NULL;
    static int instance;
    if (priv->xpnet){
        pr_err("xp_netdev_init already done.\n");
        return 0;
    }

    net_priv = (xpnet_private_t *)kzalloc(sizeof(xpnet_private_t), GFP_KERNEL);
    if (net_priv == NULL) {
        pr_err("Failed to allocate memory for netdev private structure.\n");
        return -ENOMEM;
    }

    priv->xpnet = net_priv;
    g_net_priv = net_priv;
    
    net_priv->pdev = priv->pdev;
    net_priv->pci_priv = priv;
    net_priv->hw_flags = XPNET_HWFLAG_A0;
    net_priv->dma_trigger = 0;
    spin_lock_init(&net_priv->priv_lock);

    xpnet_program_mux_setdma(net_priv, 0);
    xpnet_rx_queue_setup(net_priv, XPNET_RX_NUM_QUEUES);
    xpnet_tx_queue_setup(net_priv, XPNET_TX_NUM_QUEUES);

    rc = xp_netdev_mode_init();
    if (rc) {
        pr_err("Error in xp_netdev_mode_init().\n");
        goto err_netdev_fp_netdev;
    }

    /* Create workqueue. */
    sprintf(queue_name, "xp_queue_%d", instance);
    net_priv->wqueue = create_workqueue(queue_name);
    if (!net_priv->wqueue) {
        fdebug("Failed to create work queue.\n");
        rc = -ENOMEM;
        goto err_queue_create_failure;
    }

    INIT_DELAYED_WORK(&net_priv->dwork, xpnet_rxtx_handler);

    net_priv->instance = instance;
    rc = xpnet_proc_create(net_priv);
    if (rc) {
       pr_err("Error in xpnet_proc_create.\n");
    }
    queue_delayed_work(net_priv->wqueue, &net_priv->dwork, HZ * 5);
    xpnet_rx_all_queues_start(net_priv);

    /* Enable the DMA engine. */
    xpnet_program_mux_setdma(net_priv, 1);
    spin_lock_irqsave(&net_priv->priv_lock, flags);
    net_priv->dma_trigger = 1;
    spin_unlock_irqrestore(&net_priv->priv_lock, flags);

    return 0;


err_queue_create_failure:
    xp_netdev_mode_deinit();

err_netdev_fp_netdev:    
    xpnet_rx_teardown(net_priv, XPNET_RX_NUM_QUEUES);
    xpnet_tx_teardown(net_priv, XPNET_TX_NUM_QUEUES);
    kfree(net_priv);
    priv->xpnet = NULL;
    g_net_priv = NULL;

    return rc;
}

void xp_netdev_deinit(xp_private_t *priv)
{
    xpnet_private_t *net_priv = priv->xpnet;
    int count;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
    char queue_name[25];
#endif 

    if (!net_priv) {
        pr_info("Nothing to remove.\n");
        return;
    }

    /* Flush and destroy workqueue. */
    xp_netdev_mode_deinit();
    cancel_delayed_work_sync(&net_priv->dwork);
    flush_workqueue(net_priv->wqueue);
    xpnet_tx_teardown(net_priv, XPNET_TX_NUM_QUEUES);
    xpnet_rx_teardown(net_priv, XPNET_RX_NUM_QUEUES);
    destroy_workqueue(net_priv->wqueue);

    for (count = 0; count < XPNET_NUM_QUEUES; count++) {
        if (NULL != net_priv->proc_que[count]) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
            proc_remove(net_priv->proc_que[count]);
#else
            memset(queue_name, 0, sizeof(queue_name));
            snprintf(queue_name, sizeof(queue_name) - 1, "queue%d", count);
            remove_proc_entry(queue_name, net_priv->proc_root);
#endif
       }
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    if (NULL != net_priv->proc_stats) { 
        proc_remove(net_priv->proc_stats);
    }
    if (NULL != net_priv->proc_ttable) {
        proc_remove(net_priv->proc_ttable);
    }
    if (NULL != net_priv->proc_debug) {
        proc_remove(net_priv->proc_debug);
    }
    if (NULL != net_priv->proc_netdev) {
        proc_remove(net_priv->proc_netdev);
    }
    if (NULL != net_priv->proc_txhdr) {
        proc_remove(net_priv->proc_txhdr);
    }
#else
    remove_proc_entry(XPNET_PROC_STATS, net_priv->proc_root);
    remove_proc_entry(XPNET_PROC_TRAP_TABLE, net_priv->proc_root);
    remove_proc_entry(XPNET_PROC_DEBUG, net_priv->proc_root);
    remove_proc_entry(XPNET_PROC_NETDEV, net_priv->proc_root);
    remove_proc_entry(XPNET_PROC_TXHDR, net_priv->proc_root);
#endif
 
    if (NULL != net_priv->proc_root) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
        proc_remove(net_priv->proc_root);
#else
        memset(queue_name, 0, sizeof(queue_name));
        snprintf(queue_name, sizeof(queue_name) - 1, "xpnet%d",
                 net_priv->instance);
        remove_proc_entry(queue_name, NULL);
#endif
    }

    kfree(priv->xpnet);
    priv->xpnet = 0;
    fdebug("xp_netdev_deinit() complete.\n");
}

