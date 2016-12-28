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
* File: xp_pcie_slave.h
* 
* Abstract: This file contains the enums, macros, msg defs required for pcie 
* slave component/module of this driver.
************************************************************************/
#ifndef _XP_PCIE_SLAVE_H
#define _XP_PCIE_SLAVE_H

#include "xp_reg_info.h"
#include <linux/cdev.h>
#include "version.h"

#define PROC_FS_NAME_SIZE 50
#define REG_RW_STATUS_SIZE 200
#define INTR_CLR_RW_STATUS_SIZE 100
#define INTR_BLOCK_NAME_SIZE 25
#define HIGH_INTR_SRC_REG_SIZE 5
 
#define XP_PROC_FILE_NAME "xppcie"

#define XPREG_PROC_NAME "regaccess"
#define XPINTR_PROC_NAME "intr_stats"
#define XPINTR_CLEAR_PROC_NAME "reset_intr_stats"

#define NAME_STR_PCI_TRAN_COUNTER   		"pci-transaction-counter"  /*pci transaction (pci read/write) counter string name macro*/
#define NAME_STR_HELP      		"help"     /*help string name macro*/
#define NAME_STR_RST_COUNTER    "reset"    /*reset string name macro*/
#define NAME_STR_INVALID		"invalid"  /*invalid string name macro*/

/* Register Read/Write */
typedef struct __attribute__((__packed__)) xp_reg_rw {
    u32 reg_address;  /* Address to read/write */

    /* 0->Write, 1->Read, Note: Direction will be
       same for all contiguous access */
    u8  direction;

    /* Size to be read/write in bytes,
       valid values are: 1,2,4 */
    u32  size;

    /* Value(s) to be written/read */
    u64 value;
} xp_reg_rw_t;

typedef struct xp_private {
    /* OS specific structure. */
    struct pci_dev *pdev;
    /* Mem-mapped pointer to base of chip regs. */
    u64 __iomem *vma;
    /* End of mem-mapped chip space excluding sendbuf and user regs. */
    u64 __iomem *mmio_end;      
    /* Physical address of chip for io_remap, etc. */
    resource_size_t mmio_start;
    /* User specific structure. */
    xp_reg_rw_t *pci_rw;   
    s64 irq;
    struct workqueue_struct *w_queue;
    struct device *dev;
    struct cdev cdev;
    s32 msi_count;
    s32 msi_fail;
    /* For storing xpnet_private structure. */
    struct xpnet_private *xpnet;
    /* Lock to make tx DMA and reg read mutual exclusive. */ 
    spinlock_t tx_dma_read_lock;
    pid_t app_pid;
    /* For holding the task struct of registered Pid */
    struct task_struct *wtask;
    struct siginfo sig_info;

    /* Device type with mode(compress or uncompress). */
    xp_address_mode_t mode;
    /* Chip version */
    u32 chip_version;

    /* PDE for root xppcie node */
    struct proc_dir_entry *proc_root;
    /* For storing PDE entry for intr */
    struct proc_dir_entry *intr_proc;
    /* For storing PDE entry for reg */
    struct proc_dir_entry *reg_proc;
    /* For storing PDE entry for clear_intr */
    struct proc_dir_entry *clr_intr_proc;

    /* For storing reg proc read/write status */
    char reg_rw_status[REG_RW_STATUS_SIZE];
    /* Register read counter */
    u64 reg_read_count;
    /* Register write counter */
    u64 reg_write_count;

    /* For storing intr proc read/write status */
    char clr_intr_rw_status[INTR_CLR_RW_STATUS_SIZE];
    /* high priority block interrupt counter */
    u64 intr_high_prio_block_counter;
    /* low priority block interrupt counter */
    u64 intr_low_prio_block_counter;
    /* high priority mgmt (dma0) interrupt counter */
    u64 intr_high_prio_mgmt_counter;
} xp_private_t;

#endif /* _XP_PCIE_SLAVE_H */

