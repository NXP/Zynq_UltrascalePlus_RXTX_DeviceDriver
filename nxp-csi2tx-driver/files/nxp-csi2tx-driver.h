/*/NXP Radar Driver
 * SPDX-License-Identifier:GPL-2.0
 * Copyright (C) 2019 NXP
 *
 * AUTHOR: Jorge Hermoso Fernandez
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef CSI2TX_IOCTL_H
#define CSI2TX_IOCTL_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <linux/atomic.h>
#include <linux/skbuff.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/sched/signal.h>
#include <asm/siginfo.h>    //siginfo
#include <linux/rcupdate.h> //rcu_read_lock
#include <linux/sched.h>    //find_task_by_pid_type
#include <linux/pinctrl/pinctrl.h>
#include <linux/workqueue.h>            // Required for workqueues
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/timekeeping.h>

#define CSI2_TX_START_ADDR      0xA0020000
#define CSI2_TX_END_ADDR        0xA0030000

#define CSI2TX_BUILD_ID             0x0000
#define CSI2TX_BUILD_DATE           0x0004
#define CSI2TX_BUILD_TIME           0x0008
#define CSI2TX_TX_SUB_SYSTM_RESET   0x0010
#define CSI2TX_SETTINGS             0x0014
#define CSI2TX_FRAME_START          0x0020
#define CSI2TX_NFRAMES_TRNSMIT      0x0030
#define CSI2TX_LINE_LEN             0x0034
#define CSI2TX_FRAME_LEN            0x0038
#define CSI2TX_VERT_LINE            0x003C
#define CSI2TX_FRAMES_TRNSMIT_CNT   0x0040
#define CSI2TX_FRAME_PTR            0x0044
#define CSI2TX_IRQ_STAT             0x0070
#define CSI2TX_IRQ_CLR              0x0074
#define CSI2TX_CAL_SUM_DBG1         0x00F0 
#define CSI2TX_CAL_SUM_DBG2         0x00F4 
#define CSI2TX_MEM_FRAME            0x00C0

#define CSI2TX_IRQ_OVERFLOW_STAT    0x4

#define BASIC_SETTINGS              0x0302
#define MAX_FRAME_BUFF_SIZE         0x4000000
#define MAX_LINELEN                 0xFFF8
#define MAX_FILE_LEN                512 /*USB SSD file size*/
/* Frame Transmit Status notification */
#define NO_FRAME_INT_TRIGGERED      0x00     /*No interrupt triggered*/
#define FRAME_INT_TRIGGERED         0x01     /*Frame complete interrupt has been asserted*/
#define TX_DMA_ERROR                0x02     /*TX DMA error*/
#define IN_OUT_ERROR                0x03     /*Input/Output Error During Read*/

static int pid; // Stores application PID in user space

struct csi2_tx_control_reg
{
    bool     status;
    int      iRet;
    uint32_t buildID;
    uint32_t buildDate;
    uint32_t buildTime;
    volatile uint32_t irqStat;
    uint32_t frameLen;
    uint32_t nframesTrans;
    uint32_t framesCnt;
    uint32_t offset;
    uint32_t setting;
    uint32_t vert_Lines;
    uint32_t line_Len;
    uint32_t calSum_dbg1;
    uint32_t calSum_dbg2;
    uint32_t vc0_lines;
};

struct metadata_header {
        uint32_t numOfModes;
        uint32_t radarCycleCount;
        uint32_t capture;
        uint32_t rfa1;
        uint32_t rfa2;
        uint32_t rfa3;
        uint32_t numOfChirps_1;
        uint32_t numOfSamples_1;
        uint32_t numOfChirps_2;
        uint32_t numOfSamples_2;
        uint32_t numOfChirps_3;
        uint32_t numOfSamples_3;
        uint32_t numOfChirps_4;
        uint32_t numOfSamples_4;
        uint32_t rfa4;
        uint32_t rfa5;

};

struct streamAdcInfo
{
        char filename[MAX_FILE_LEN];
        uint32_t msdelay;
};

#define IOCTL_REG_WRITE_TX          _IOWR('C', 1, struct csi2_tx_control_reg*)
#define IOCTL_FRAME_START_TX        _IO('C', 2)
#define IOCTL_FRAME_LEN_TX          _IOW('C', 3,struct csi2_tx_control_reg*)
#define IOCTL_IRQ_READ_TX           _IOWR('C', 4, struct csi2_tx_control_reg*)
#define IOCTL_IRQ_RESET_TX          _IO('C', 5)
#define IOCTL_BUILD_INFO_READ_TX    _IOWR('C', 6, struct csi2_tx_control_reg*)
#define IOCTL_CONF_SETTINGS_TX      _IOW('C', 7, struct csi2_tx_control_reg*)
#define IOCTL_DBG1_DBG2_READ_TX     _IOR('C', 8, struct csi2_tx_control_reg*)
#define IOCTL_METADATA_HEADER_TX    _IOR('C', 9, struct streamAdcInfo *)
#endif /* CSI2TX_IOCTL_H */


