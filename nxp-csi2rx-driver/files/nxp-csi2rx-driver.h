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
#ifndef CSI2RX_IOCTL_H
#define CSI2RX_IOCTL_H

#include <linux/init.h>
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

#include <linux/workqueue.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/timekeeping.h>


/*Mixel regiters */
#define CSI2_RX_START_PHY_ADDR          0xA0040000
#define CSI2_RX_END_PHY_ADDR            0xA0080000

#define RX_CFG_MODE_OFFSET              0x00
#define CFG_FLUSH_COUNT_OFFSET          0x04
#define CFG_WATCHDOG_COUNT_OFFSET       0x08
#define BIT_ERR_OFFSET                  0x0C
#define PAYLOAD_CRC_OFFSET              0x10
#define RX_IRQ_STATUS_OFFSET            0x14
#define RX_IRQ_ENABLE_OFFSET            0x18
#define CFG_DISABLE_PAYLOAD_0_OFFSET    0x1C 
#define CFG_DISABLE_PAYLOAD_1_OFFSET    0x20
/*Enables user defined data types to be mapped according to other data type byte to pixel unpacking rules*/
#define CFG_RX_USERDEF_DT_0_OFFSET      0x24 
#define CFG_RX_USERDEF_DT_1_OFFSET      0x28
#define RX_CLOCK_LANE_OFFSET            0x2C
#define RX_LANE_0_OFFSET                0x30
#define RX_LANE_1_OFFSET                0x34
#define RX_LANE_2_OFFSET                0x38 /* Not applicable for SAF85xx*/ 
#define RX_LANE_3_OFFSET                0x3C /*Not applicable for SAF85xx */
#define EXTENDED_WC_OFFSET              0x40 /*RX Controller debug mode controls  */

/* default Mixel configurations */
#define DEFAULT_MIXEL_RX_CFG_MODE_CONFIG            0x00000002
#define DEFAULT_MIXEL_RX_IRQ_ENABLE_CONFIG          0x0000001F
#define DEFAULT_MIXEL_RX_CLOCK_LANE_CONFIG          0x80000000
#define DEFAULT_MIXEL_RX_LANE_0_CONFIG              0x80000000
#define DEFAULT_MIXEL_RX_LANE_1_CONFIG              0x81000000


#define CSI2RX_MODNAME                  "nxp_csi2rx_driver"

#define IOCTL_CMD_FUNC_NUM              20

/*Xilinx registers*/
#define CSI2_RX_START_ADDR      0xA0010000
#define CSI2_RX_END_ADDR        0xA0020000

#define VERSION_BIT_MASKING     0xFFFF0000
#define MIXEL_VERSION           0xEB000000
#define CSI2RX_VERSION          0x0000
#define CSI2RX_RESET            0x0100
#define CSI2RX_SETTINGS         0x0300
#define CSI2RX_FRAME_START      0x0400
#define CSI2RX_SAM_PER_FRAME    0X0800
#define CSI2RX_FRAME_LEN        0x1000
#define CSI2RX_N_CYCLE          0X2000
#define CSI2RX_N_CHIRPS1        0X2100
#define CSI2RX_N_CHIRPS2        0X2200
#define CSI2RX_N_CHIRPS3        0X2300
#define CSI2RX_N_CHIRPS4        0X2400
#define CSI2RX_IRQ_STATUS       0X2C00
#define CSI2RX_IRQ_CLR          0x2D00
#define CSI2RX_TEST_DATA_CH1    0x9000
#define CSI2RX_TEST_DATA_CH2    0x9100
#define CSI2RX_TEST_DATA_CH3    0x9200
#define CSI2RX_TEST_DATA_CH4    0x9300
#define CSI2RX_RXGEN_CTL_PATTERN  0XE000
#define CSI2RX_RXGEN_SIZE         0XE200

static int pid; // Stores application PID in user space

#define NXP_MD_BUF_LEN           		64   //MetaData Header Size
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

#define IOCTL_REG_WRITE_RX      _IOWR('C', 1, struct csi2_rx_control_reg*)
#define IOCTL_FRAME_START_RX    _IO('C', 2)
#define IOCTL_FRAME_LEN_RX     _IOR('C', 3, uint32_t *)
#define IOCTL_IRQ_READ_RX       _IOWR('C', 4,struct csi2_rx_control_reg*)
#define IOCTL_IRQ_RESET_RX       _IO('C', 5)
#define IOCTL_FPGA_TRIGGER_RX    _IO('C', 6)
#define IOCTL_REG_PTRN_WRITE_RX _IOW('C', 7, struct csi2_rx_control_reg*)
#define IOCTL_METADATA_HEADER _IOR('C', 8, struct metadata_header *)
#endif

