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
* File: xp_export.h
* 
* Abstract: This file contains the enums, macros, definitions to be exported.
************************************************************************/
#ifndef _XP_EXPORT_H
#define _XP_EXPORT_H

#include <linux/if.h>
#include <linux/types.h>
#include "xp_header.h"

typedef enum xp_nl_hostif_trap_channel {
    XP_NL_HOSTIF_TRAP_FD,     /* Receive packets via file desriptor */
    XP_NL_HOSTIF_TRAP_CB,     /* Receive packets via callback       */
    XP_NL_HOSTIF_TRAP_NETDEV, /* Receive packets via OS net device  */
    XP_NL_HOSTIF_TRAP_CUSTOM_RANGE_BASE = 0x10000000
} xp_nl_hostif_trap_channel_t;

typedef enum xp_nl_operation {
    XP_NL_OPERATION_ADD,
    XP_NL_OPERATION_REMOVE,
    XP_NL_OPERATION_ENABLE,
    XP_NL_OPERATION_DISABLE,
} xp_nl_operation_t;

/* XP_NL_HOSTIF_TRAP_NETDEV modes in which driver operates.
 
   XP_NL_NETDEV_MODE_1:
   In this mode a separate kernel netdev is created for each physical or logical
   interfaces - front pannel ports, LAGs, L3 interfaces, etc. The
   sending/receiving to/from kernel netdevs is done using a netdev-to-VIF
   mapping which allows to identify right kernel netdev or hardware(physical or
   logical) interface the packet has to be sent to in both directions.
 
   XP_NL_NETDEV_MODE_2:
   In this mode kernel netdevs are created for physical ports only 128 front
   pannel ports and the CPU port. All the L2 packets(and L3 ones received from
   router ports) are set to front pannel port netdevs.
 
   The LAG interface is assumed to be represented in kernel by a Linux bond
   netdev which all its members representing netdevs(front pannel ports netdevs
   created by this driver) have to be added to. The creation of bond interface
   and joining/leaving of its members is out of the scope of this packet driver
   implementation.
 
   The CPU kernel netdev interface serves as an aggregate for L3 VLAN kernel
   netdevs which are created as VLAN subnetdevs of the CPU netdev. In such case
   all the packets trapped from hardware and destined to L3 VLANs are sent to
   the CPU netdev where they are supposed to be demultiplexed using VLAN tags
   from ethernet headers and Linux kernel facilities. If the driver receives an
   untagged packet destined to L3 VLAN netdev it adds a corresponding VLAN tag
   before sending it to the CPU netdev. The packets sent from L3 VLAN
   netdevs(through the CPU netdev) to hardware are injected to the beginning of
   the pipeline where they have to be forwarded according to their VLAN headers
   and pipeline configuartion. The creation/removal of CPU netdev VLAN
   subnetdevs is out of the scope of this packet driver implementation.*/
typedef enum xp_nl_netdev_mode {
    XP_NL_NETDEV_MODE_1,
    XP_NL_NETDEV_MODE_2,
} xp_nl_netdev_mode_t;

typedef enum xp_nl_msg {
    XP_NL_MSG_INTF_ADD,        /* Create a netdev interface     */
    XP_NL_MSG_INTF_DEL,        /* Remove a netdev interface     */

    XP_NL_MSG_LINK_ADD,        /* Link a netdev with a VIF/RIF  */
    XP_NL_MSG_LINK_DEL,        /* Remove a netdev VIF/RIF link  */

    XP_NL_MSG_TX_HDR_SET,      /* Add / remove a TX meta header */
    XP_NL_MSG_TRAP_SET,        /* Add / remove TRAP table entry */
    XP_NL_MSG_CB_FD_SET,       /* Add / remove FD callback      */

    XP_NL_MSG_MIRROR_SET,      /* Enable / disable mirroring    */
    XP_NL_MSG_LINK_STATUS_SET, /* Up / Down link status set     */
    XP_NL_MSG_NETDEV_MODE_SET, /* Set netdev mode to XP_NL_NETDEV_MODE_1 or
                                  XP_NL_NETDEV_MODE_2 */
} xp_nl_msg_t;

typedef struct xp_nl_msg_intf {
    __u32 xpnet_intf_id;
    __u8  intf_name[IFNAMSIZ];
} xp_nl_msg_intf_t;

typedef struct xp_nl_msg_link {
    __u32 xpnet_intf_id;

    union {
        __u32 vif;
        __u32 rif;
    };
} xp_nl_msg_link_t;

typedef struct xp_nl_msg_tx_hdr {
    __u32 xpnet_intf_id;
    __u8  operation;
    __u8  tx_header[sizeof(xphTxHdr)];
} xp_nl_msg_tx_hdr_t;

typedef struct xp_nl_msg_trap {
    __u32 rc;
    __u32 fd;
    __u32 ch;
    __u32 trap_id;
    __u8  operation;
} xp_nl_msg_trap_t;

typedef struct xp_nl_msg_cb_fd {
    __u32 fd;
    __u8  operation;
} xp_nl_msg_cb_fd_t;

typedef struct xp_nl_msg_mirror {
    __u8 operation;
} xp_nl_msg_mirror_t;

typedef struct xp_nl_msg_link_status {
    __u32 xpnet_intf_id;
    __u32 status;
} xp_nl_msg_link_status_t;

typedef struct xp_nl_tlv_msg {
    __u32 msg_type;    /* xp_nl_msg_t      */
    __u32 payload_len; /* Len of payload[] */
    __u8  payload[];   /* Payload          */
} xp_nl_tlv_msg_t;

typedef struct xp_nl_msg_netdev_mode {
    __u8 mode;
} xp_nl_msg_netdev_mode_t;

#endif /* _XP_EXPORT_H */

