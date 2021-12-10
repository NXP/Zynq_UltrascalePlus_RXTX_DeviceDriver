/*
* Copyright 2021 NXP
*
* SPDX-License-Identifier: GPL-2.0
* The GPL-2.0 license for this file can be found in the COPYING.GPL file
* included with this distribution or at http://www.gnu.org/licenses/gpl-2.0.html
*/
#ifndef CSI2RX_IOCTL_H
#define CSI2RX_IOCTL_H

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

#define CSI2_RX_START_ADDR      0xA0010000
#define CSI2_RX_END_ADDR        0xA0020000

#define CSI2RX_RESET            0x0100
#define CSI2RX_SETTINGS         0x0300
#define CSI2RX_FRAME_START      0x0400
#define CSI2RX_IRQ_CLR          0x2D00
#define CSI2RX_TEST_DATA_CH1    0x9000
#define CSI2RX_TEST_DATA_CH2    0x9100
#define CSI2RX_TEST_DATA_CH3    0x9200
#define CSI2RX_TEST_DATA_CH4    0x9300

#define CSI2RX_SAM_PER_FRAME    0X0800
#define CSI2RX_N_CHIRPS1        0X2100
#define CSI2RX_N_CHIRPS2        0X2200
#define CSI2RX_N_CHIRPS3        0X2300
#define CSI2RX_N_CHIRPS4        0X2400
#define CSI2RX_N_CYCLE          0X2000
#define CSI2RX_RXGEN_CTL_PATTERN  0XE000
#define CSI2RX_RXGEN_SIZE       0XE200

static int pid; // Stores application PID in user space

struct csi2_rx_control_reg
{
    bool bconf;
    int iRet;
    uint32_t offset;
    uint32_t patternID;
    uint32_t Nsample;
    uint32_t Ncycle;
    uint32_t Nchirps1;
    uint32_t Nchirps2;
    uint32_t Nchirps3;
    uint32_t Nchirps4;
    uint32_t Rxgen;
    uint32_t Frame_length;
    volatile uint32_t irq_stat;
    bool     test_flg;
    uint32_t TestData1;
    uint32_t TestData2;
    uint32_t TestData3;
    uint32_t TestData4;
    uint32_t setting;
} ;

#define IOCTL_REG_WRITE_RX      _IOWR('C', 1, struct csi2_rx_control_reg*)
#define IOCTL_FRAME_START_RX    _IO('C', 2)
#define IOCTL_FRAME_LEN_RX     _IOR('C', 3, uint32_t *)
#define IOCTL_IRQ_READ_RX       _IOWR('C', 4,struct csi2_rx_control_reg*)
#define IOCTL_IRQ_RESET_RX       _IO('C', 5)
#define IOCTL_FPGA_TRIGGER_RX    _IO('C', 6)
#define IOCTL_REG_PTRN_WRITE_RX _IOW('C', 7, struct csi2_rx_control_reg*)
#endif
