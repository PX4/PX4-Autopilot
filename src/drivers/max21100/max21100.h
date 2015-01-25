/*
 * max21100.h
 *
 *  Created on: 25 Jan 2015
 *      Author: matt
 */

#ifndef MAX21100_H_
#define MAX21100_H_

enum{
	MAX21100_BANK_COMMON = 0,
	MAX21100_BANK_0 = 1,
	MAX21100_BANK_1 = 2,
	MAX21100_BANK_2 = 3,
};

// MAX21100 registers

// Common bank registers
#define MAX21100_REG_WHOAMI			MAX21100_BANK_COMMON,0x20
#define MAX21100_REG_REVISION		MAX21100_BANK_COMMON,0x21
#define MAX21100_REG_BANK			MAX21100_BANK_COMMON,0x22
#define MAX21100_REG_STATUS			MAX21100_BANK_COMMON,0x23
#define MAX21100_REG_GYRO_X_H		MAX21100_BANK_COMMON,0x24
#define MAX21100_REG_GYRO_X_L		MAX21100_BANK_COMMON,0x25
#define MAX21100_REG_GYRO_Y_H		MAX21100_BANK_COMMON,0x26
#define MAX21100_REG_GYRO_Y_L		MAX21100_BANK_COMMON,0x27
#define MAX21100_REG_GYRO_Z_H		MAX21100_BANK_COMMON,0x28
#define MAX21100_REG_GYRO_Z_L		MAX21100_BANK_COMMON,0x29

#define MAX21100_REG_ACCEL_X_H		MAX21100_BANK_COMMON,0x2A
#define MAX21100_REG_ACCEL_X_L		MAX21100_BANK_COMMON,0x2B
#define MAX21100_REG_ACCEL_Y_H		MAX21100_BANK_COMMON,0x2C
#define MAX21100_REG_ACCEL_Y_L		MAX21100_BANK_COMMON,0x2D
#define MAX21100_REG_ACCEL_Z_H		MAX21100_BANK_COMMON,0x2E
#define MAX21100_REG_ACCEL_Z_L		MAX21100_BANK_COMMON,0x2F

#define MAX21100_REG_MAG_X_H		MAX21100_BANK_COMMON,0x30
#define MAX21100_REG_MAG_X_L		MAX21100_BANK_COMMON,0x31
#define MAX21100_REG_MAG_Y_H		MAX21100_BANK_COMMON,0x32
#define MAX21100_REG_MAG_Y_L		MAX21100_BANK_COMMON,0x33
#define MAX21100_REG_MAG_Z_H		MAX21100_BANK_COMMON,0x34
#define MAX21100_REG_MAG_Z_L		MAX21100_BANK_COMMON,0x35

#define MAX21100_REG_TEMP_H			MAX21100_BANK_COMMON,0x36
#define MAX21100_REG_TEMP_L			MAX21100_BANK_COMMON,0x37

#define MAX21100_REG_RFU1			MAX21100_BANK_COMMON,0x38
#define MAX21100_REG_RFU2			MAX21100_BANK_COMMON,0x39
#define MAX21100_REG_RFU3			MAX21100_BANK_COMMON,0x3A
#define MAX21100_REG_RFU4			MAX21100_BANK_COMMON,0x3B

#define MAX21100_REG_FIFO_COUNT		MAX21100_BANK_COMMON,0x3C
#define MAX21100_REG_FIFO_STATUS	MAX21100_BANK_COMMON,0x3D
#define MAX21100_REG_FIFO_DATA		MAX21100_BANK_COMMON,0x3E

#define MAX21100_REG_RST			MAX21100_BANK_COMMON,0x3F

// Register bank 0
#define MAX21100_REG_POWER_CFG		MAX21100_BANK_0,0x00
#define MAX21100_REG_GYRO_CFG1		MAX21100_BANK_0,0x01
#define MAX21100_REG_GYRO_CFG2		MAX21100_BANK_0,0x02
#define MAX21100_REG_GYRO_CFG3		MAX21100_BANK_0,0x03
#define MAX21100_REG_PWR_ACC_CFG	MAX21100_BANK_0,0x04
#define MAX21100_REG_ACC_CFG1		MAX21100_BANK_0,0x05
#define MAX21100_REG_ACC_CFG2		MAX21100_BANK_0,0x06
#define MAX21100_REGMAG__SLV_CFG	MAX21100_BANK_0,0x07
#define MAX21100_REG_MAG_SLV_ADD	MAX21100_BANK_0,0x08
#define MAX21100_REG_MAG_SLV_REG	MAX21100_BANK_0,0x09
#define MAX21100_REG_MAG_MAP_REG	MAX21100_BANK_0,0x0A
#define MAX21100_REG_I2C_MST_ADD	MAX21100_BANK_0,0x0B
#define MAX21100_REG_I2C_MST_DATA	MAX21100_BANK_0,0x0C

#define MAX21100_REG_MAG_OFS_X_MSB	MAX21100_BANK_0,0x0D
#define MAX21100_REG_MAG_OFS_X_LSB	MAX21100_BANK_0,0x0E
#define MAX21100_REG_MAG_OFS_Y_MSB	MAX21100_BANK_0,0x0F
#define MAX21100_REG_MAG_OFS_Y_LSB	MAX21100_BANK_0,0x10
#define MAX21100_REG_MAG_OFS_Z_MSB	MAX21100_BANK_0,0x11
#define MAX21100_REG_MAG_OFS_Z_LSB	MAX21100_BANK_0,0x12

#define MAX21100_REG_DR_CFG			MAX21100_BANK_0,0x13
#define MAX21100_REG_IO_CFG			MAX21100_BANK_0,0x14
#define MAX21100_REG_I2C_PAD		MAX21100_BANK_0,0x15
#define MAX21100_REG_I2C_CFG		MAX21100_BANK_0,0x16

#define MAX21100_REG_FIFO_THRESH	MAX21100_BANK_0,0x17
#define MAX21100_REG_FIFO_CFG		MAX21100_BANK_0,0x18

#define MAX21100_REG_RFU5			MAX21100_BANK_0,0x19

#define MAX21100_REG_DSYNC_CFG		MAX21100_BANK_0,0x1A
#define MAX21100_REG_DSYNC_CNT		MAX21100_BANK_0,0x1B
#define MAX21100_REG_ITF_OTP		MAX21100_BANK_0,0x1C

#define MAX21100_REG_RFU6			MAX21100_BANK_0,0x1D
#define MAX21100_REG_RFU7			MAX21100_BANK_0,0x1E
#define MAX21100_REG_RFU8			MAX21100_BANK_0,0x0F

// Register bank 1
#define MAX21100_REG_INT_REF_X		MAX21100_BANK_1,0x00
#define MAX21100_REG_INT_REF_Y		MAX21100_BANK_1,0x01
#define MAX21100_REG_INT_REF_Z		MAX21100_BANK_1,0x02
#define MAX21100_REG_INT_DEB_X		MAX21100_BANK_1,0x03
#define MAX21100_REG_INT_DEB_Y		MAX21100_BANK_1,0x04
#define MAX21100_REG_INT_DEB_Z		MAX21100_BANK_1,0x05
#define MAX21100_REG_INT_MSK_X		MAX21100_BANK_1,0x06
#define MAX21100_REG_INT_MSK_Y		MAX21100_BANK_1,0x07
#define MAX21100_REG_INT_MSK_Z		MAX21100_BANK_1,0x08

#define MAX21100_REG_INT_MASK_AO	MAX21100_BANK_1,0x09

#define MAX21100_REG_INT_CFG1		MAX21100_BANK_1,0x0A
#define MAX21100_REG_INT_CFG2		MAX21100_BANK_1,0x0B
#define MAX21100_REG_INT_TMO		MAX21100_BANK_1,0x0C
#define MAX21100_REG_INT_STS_UL		MAX21100_BANK_1,0x0D
#define MAX21100_REG_INT_STS		MAX21100_BANK_1,0x0E
#define MAX21100_REG_INT_MSK		MAX21100_BANK_1,0x0F

#define MAX21100_REG_RFU_1_0		MAX21100_BANK_1,0x10
#define MAX21100_REG_RFU_1_1		MAX21100_BANK_1,0x11
#define MAX21100_REG_RFU_1_2		MAX21100_BANK_1,0x12
#define MAX21100_REG_RFU_1_3		MAX21100_BANK_1,0x13
#define MAX21100_REG_RFU_1_4		MAX21100_BANK_1,0x14
#define MAX21100_REG_RFU_1_5		MAX21100_BANK_1,0x15
#define MAX21100_REG_RFU_1_6		MAX21100_BANK_1,0x16

#define MAX21100_REG_INT_SRC_SEL	MAX21100_BANK_1,0x17
#define MAX21100_REG_RFU_16			MAX21100_BANK_1,0x18
#define MAX21100_REG_RFU_17			MAX21100_BANK_1,0x19

#define MAX21100_REG_SERIAL_5		MAX21100_BANK_1,0x1A
#define MAX21100_REG_SERIAL_4		MAX21100_BANK_1,0x1B
#define MAX21100_REG_SERIAL_3		MAX21100_BANK_1,0x1C
#define MAX21100_REG_SERIAL_2		MAX21100_BANK_1,0x1D
#define MAX21100_REG_SERIAL_1		MAX21100_BANK_1,0x1E
#define MAX21100_REG_SERIAL_0		MAX21100_BANK_1,0x0F

// Register bank 2
#define MAX21100_REG_QUAT0_H		MAX21100_BANK_2,0x00
#define MAX21100_REG_QUAT0_L		MAX21100_BANK_2,0x01
#define MAX21100_REG_QUAT1_H		MAX21100_BANK_2,0x02
#define MAX21100_REG_QUAT1_L		MAX21100_BANK_2,0x03
#define MAX21100_REG_QUAT2_H		MAX21100_BANK_2,0x04
#define MAX21100_REG_QUAT2_L		MAX21100_BANK_2,0x05
#define MAX21100_REG_QUAT3_H		MAX21100_BANK_2,0x06
#define MAX21100_REG_QUAT3_L		MAX21100_BANK_2,0x07

#define MAX21100_REG_RFU_2_0		MAX21100_BANK_2,0x08
#define MAX21100_REG_RFU_2_1		MAX21100_BANK_2,0x09
#define MAX21100_REG_RFU_2_2		MAX21100_BANK_2,0x0A
#define MAX21100_REG_RFU_2_3		MAX21100_BANK_2,0x0B
#define MAX21100_REG_RFU_2_4		MAX21100_BANK_2,0x0C
#define MAX21100_REG_RFU_2_5		MAX21100_BANK_2,0x0D
#define MAX21100_REG_RFU_2_6		MAX21100_BANK_2,0x0E
#define MAX21100_REG_RFU_2_7		MAX21100_BANK_2,0x0F
#define MAX21100_REG_RFU_2_8		MAX21100_BANK_2,0x10
#define MAX21100_REG_RFU_2_9		MAX21100_BANK_2,0x11
#define MAX21100_REG_RFU_2_10		MAX21100_BANK_2,0x12

#define MAX21100_REG_BIAS_GYRO_X_H	MAX21100_BANK_2,0x13
#define MAX21100_REG_BIAS_GYRO_X_L	MAX21100_BANK_2,0x14
#define MAX21100_REG_BIAS_GYRO_Y_H	MAX21100_BANK_2,0x15
#define MAX21100_REG_BIAS_GYRO_Y_L	MAX21100_BANK_2,0x16
#define MAX21100_REG_BIAS_GYRO_Z_H	MAX21100_BANK_2,0x17
#define MAX21100_REG_BIAS_GYRO_Z_L	MAX21100_BANK_2,0x18

#define MAX21100_REG_BIAS_COMP_ACC_X	MAX21100_BANK_2,0x19
#define MAX21100_REG_BIAS_COMP_ACC_Y	MAX21100_BANK_2,0x1A
#define MAX21100_REG_BIAS_COMP_ACC_Z	MAX21100_BANK_2,0x1B

#define MAX21100_REG_FUS_CFG0		MAX21100_BANK_2,0x1C
#define MAX21100_REG_FUS_CFG1		MAX21100_BANK_2,0x1D

#define MAX21100_REG_RFU_2_11		MAX21100_BANK_2,0x1E
#define MAX21100_REG_GYR_ODR_TRIM	MAX21100_BANK_2,0x1F


// Register format definitions

typedef struct max21100_reg_bank{
	unsigned	bank_sel	: 4;
	unsigned	rfu			: 4;
} max21100_reg_bank;

typedef struct max21100_reg_system_status{
	unsigned	gyro_dr		: 1;
	unsigned	gyro_err	: 1;
	unsigned	acc_dr		: 1;
	unsigned	acc_err		: 1;
	unsigned	mag_dr		: 1;
	unsigned	mag_err		: 1;
	unsigned	quat_dr		: 1;
	unsigned	quat_err	: 1;
} max21100_reg_system_status;

typedef struct max21100_reg_system_status_t{
	unsigned	fifo_empty		: 1;
	unsigned	fifo_full		: 1;
	unsigned	fifo_th			: 1;
	unsigned	fifo_rd_empty	: 1;
	unsigned	fifo_wr_full	: 1;
	unsigned	rfu				: 3;
} max21100_reg_system_status;

typedef struct max21100_reg_rst_t{
	unsigned	hpf_g_rst		: 1;
	unsigned	hpf_a_rst		: 1;
	unsigned	par_rst			: 1;
	unsigned	unused			: 5;
} max21100_max21100_reg_rst;

typedef struct max21100_reg_power_cfg_t{
	unsigned	sns_en_x		: 1;
	unsigned	sns_en_y		: 1;
	unsigned	sns_en_z		: 1;
	unsigned	gyro_mode		: 2;
	unsigned 	accel_mode		: 2;
	unsigned	sel				: 1;
} max21100_reg_power_cfg;


enum{
	MAX21100_GYRO_MODE_POWERDOWN 	= 0,
	MAX21100_GYRO_MODE_SLEEP 		= 1,
	MAX21100_GYRO_MODE_SPOT 		= 2,
	MAX21100_GYRO_MODE_NORMAL 		= 3
};

typedef struct max21100_reg_gyro_cfg1_t{
	unsigned	dout_fsc	: 2;
	unsigned	lpf_cnf		: 4;
	unsigned	self_test	: 2;
} max21100_reg_gyro_cfg1;

enum{
	MAX21100_GYRO_LPF_BW_2HZ = 0,
	MAX21100_GYRO_LPF_BW_4HZ = 1,
	MAX21100_GYRO_LPF_BW_6HZ = 2,
	MAX21100_GYRO_LPF_BW_8HZ = 3,
};

typedef struct max21100_reg_gyro_cfg2_t{
	unsigned	odr		: 4;
	unsigned	hpf_en	: 1;
	unsigned	lpf_en	: 1;
	unsigned	rfu		: 2;
} max21100_reg_gyro_cfg2;

typedef struct max21100_reg_gyro_cfg3_t{
	unsigned	hpf_cfg		: 3;
	unsigned	data_mode	: 2;
	unsigned	rfu			: 3;
} max21100_reg_gyro_cfg3;

typedef struct max21100_reg_pwr_acc_t{
	unsigned	sns_en_x	: 1;
	unsigned	sns_en_y	: 1;
	unsigned	sns_en_z	: 1;
	unsigned	self_test	: 3;
	unsigned	acc_fsc		: 2;
} max21100_reg_pwr_acc;

enum{
	MAX21100_ACC_FS_16G = 0,
	MAX21100_ACC_FS_8G = 1,
	MAX21100_ACC_FS_4G = 2,
	MAX21100_ACC_FS_2G = 3,
};

typedef struct max21100_reg_acc_cfg1_t{
	unsigned	odr			: 4;
	unsigned	lpf_cnf		: 2;
	unsigned	hpf_cnf		: 2;
} max21100_reg_acc_cfg1;

enum{
	MAX21100_ACC_HPF_ODR_400 	= 0,
	MAX21100_ACC_HPF_ODR_200 	= 1,
	MAX21100_ACC_HPF_ODR_100 	= 2,
	MAX21100_ACC_HPF_ODR_50 	= 3,
};

typedef struct max21100_reg_acc_cfg2_t{
	unsigned	hpf_en		: 1;
	unsigned	mag_odr		: 3;
	unsigned	rfu			: 4;
} max21100_reg_acc_cfg2;

enum{
	MAX21100_ACC_HPF_ODR_400 	= 0,
	MAX21100_ACC_HPF_ODR_200 	= 1,
	MAX21100_ACC_HPF_ODR_100 	= 2,
	MAX21100_ACC_HPF_ODR_50 	= 3,
};

typedef struct max21100_reg_dr_cfg_t{
	unsigned	temp_en			: 1;
	unsigned	coarse_temp		: 1;
	unsigned	dr_rst_mode		: 2;
	unsigned	single_en		: 1;
	unsigned	rw_sel			: 1;
	unsigned	rfu				: 1;
	unsigned	byp_en			: 1;
} max21100_reg_dr_cfg;

typedef struct max21100_reg_io_cfg_t{
	unsigned	i2c_slave_pu_en		: 1;
	unsigned	i2c_master_pu_en	: 1;
	unsigned	int2_pu_en			: 1;
	unsigned	int2_pd_en			: 1;
	unsigned	int1_pu_en			: 1;
	unsigned	int1_pd_en			: 1;
	unsigned	dsync_pu_en			: 1;
	unsigned	dsync_pd_en			: 1;
} max21100_reg_io_cfg;

typedef struct max21100_reg_pad_t{
	unsigned	int_stg			: 1;
	unsigned	mst_stg			: 1;
	unsigned	slv_stg			: 1;
	unsigned	rfu				: 5;
} max21100_reg_pad;

typedef struct max21100_reg_i2c_cfg_t{
	unsigned	i2c_off			: 1;
	unsigned	endian			: 1;
	unsigned	spi_3_wire		: 1;
	unsigned	rfu				: 3;
	unsigned	mst_i2c_cfg		: 2;
} max21100_reg_i2c_cfg;

typedef struct max21100_reg_fifo_cfg_t{
	unsigned	fifo_store_gyr	: 1;
	unsigned	fifo_store_acc	: 1;
	unsigned	fifo_store_mag	: 1;
	unsigned	fifo_store_quat	: 1;
	unsigned	fifo_overrun	: 1;
	unsigned	fifo_int_sel	: 1;
	unsigned	fifo_mode		: 2;
} max21100_reg_fifo_cfg;

typedef struct max21100_reg_int_msk_xyz_t{
	unsigned	low_neg		: 1;
	unsigned	high_neg	: 1;
	unsigned	low_pos		: 1;
	unsigned	high_pos	: 1;
	unsigned	low_neg_en	: 1;
	unsigned	high_neg_en	: 1;
	unsigned	low_pos_en	: 1;
	unsigned	high_pos_en	: 1;
} max21100_reg_int_msk_xyz;




#define MAX21100_ACCEL_DEFAULT_RANGE_G			8
#define MAX21100_ACCEL_DEFAULT_RATE			1000
#define MAX21100_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define MAX21100_GYRO_DEFAULT_RANGE_G			8
#define MAX21100_GYRO_DEFAULT_RATE			1000
#define MAX21100_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define MAX21100_DEFAULT_ONCHIP_FILTER_FREQ		42


#endif /* MAX21100_H_ */
