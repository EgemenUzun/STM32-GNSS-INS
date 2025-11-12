/*
 * mpu6050.h
 *
 *  Created on: Mar 24, 2025
 *      Author: pilotpc
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32h7xx_hal.h"
#include <stdbool.h>

/* MPU6050 I2C Addresses */
#define MPU6050_ADDR_AD0_LOW     (0x68) << 1 // AD0 pin connected to GND (default)
#define MPU6050_ADDR_AD0_HIGH    (0x69) << 1 // AD0 pin connected to VCC

/* MPU6050 Register Addresses */
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_FIFO_EN      0x23
#define MPU6050_INT_PIN_CFG 	 0X37
#define MPU6050_REG_INT_ENABLE   0x38
#define MPU6050_REG_INT_STATUS   0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_USER_CTRL    0x6A
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

// Define interrupt pin and port
#define MPU6050_INT_PORT 		 GPIOB
#define MPU6050_INT_PIN 		 GPIO_PIN_7

/* Gyroscope full scale ranges */
typedef enum {
	MPU6050_GYRO_RANGE_250_DEG  = 0x00, // ±250 °/s
	MPU6050_GYRO_RANGE_500_DEG  = 0x08, // ±500 °/s
	MPU6050_GYRO_RANGE_1000_DEG = 0x10, // ±1000 °/s
	MPU6050_GYRO_RANGE_2000_DEG = 0x18  // ±2000 °/s
} MPU6050_GyroRange;

/* Accelerometer full scale ranges */
typedef enum {
	MPU6050_ACCEL_RANGE_2_G  = 0x00, // ±2g
	MPU6050_ACCEL_RANGE_4_G  = 0x08, // ±4g
	MPU6050_ACCEL_RANGE_8_G  = 0x10, // ±8g
	MPU6050_ACCEL_RANGE_16_G = 0x18  // ±16g
} MPU6050_AccelRange;

/* Digital Low Pass Filter bandwidths */
typedef enum {
	MPU6050_DLPF_BW_260_HZ = 0x00,
	MPU6050_DLPF_BW_184_HZ = 0x01,
	MPU6050_DLPF_BW_94_HZ  = 0x02,
	MPU6050_DLPF_BW_44_HZ  = 0x03,
	MPU6050_DLPF_BW_21_HZ  = 0x04,
	MPU6050_DLPF_BW_10_HZ  = 0x05,
	MPU6050_DLPF_BW_5_HZ   = 0x06
} MPU6050_DLPF_Bandwidth;

typedef struct {
	bool i2c_bypass_en;
	bool fsync_int_en;
	bool fsync_int_level;
	bool int_rd_clear;
	bool latch_int_en;
	bool int_open;
	bool int_level;
} INT_CONFIG_t;

typedef enum {
	I2C_BYPASS_EN = (0x01) << 1,
	FSYNC_INT_EN = (0x01) << 2,
	FSYNC_INT_LEVEL = (0x01) << 3,
	INT_RD_CLEAR = (0x01) << 4,
	LATCH_INT_EN = (0x01) << 5,
	INT_OPEN = (0x01) << 6,
	INT_LEVEL = (0x01) << 7,
} INT_CONFIG;

typedef struct {
	bool data_rdy; // Data Ready interrupt
	bool i2c_mst_int; // I2C Master interrupt
	bool fifo_offlow; // FIFO buffer overflow interrupt
	bool mot; // Motion Detection interrupt
} INT_ENABLE_t;

typedef enum {
	DATA_RDY_EN = (0x01) << 0,
	I2C_MST_INT_EN = (0x01) << 3,
	FIFO_OFLOW_EN = (0x01) << 4,
	MOT_EN = (0x01) << 6,
} INT_ENABLE;

typedef struct {
	bool xg_fifo_en;
	bool yg_fifo_en;
	bool zg_fifo_en;
	bool accel_fifo_en;
} FIFO_SELECTION_t;


/* MPU6050 data structure */
typedef struct {
	I2C_HandleTypeDef* hi2c;      // I2C handle pointer
	uint8_t address;              // I2C device address
	float accel_sensitivity;      // Conversion factor for accelerometer
	float gyro_sensitivity;       // Conversion factor for gyroscope

	// Raw sensor data
	int16_t accel_raw[3];         // Raw accelerometer data (x, y, z)
	int16_t gyro_raw[3];          // Raw gyroscope data (x, y, z)
	int16_t temp_raw;             // Raw temperature data

	// Calibration offsets
	int16_t accel_offset[3];      // Accelerometer offsets
	int16_t gyro_offset[3];       // Gyroscope offsets

	// Converted sensor data
	float accel[3];               // Accelerometer data in g (x, y, z)
	float gyro[3];                // Gyroscope data in degrees/s (x, y, z)
	float temp;                   // Temperature in degrees Celsius

	float gyro_angel[3];		  // Gyroscope data in degrees (x, y, z)

	float gyro_angel_f[3];		  // Gyroscope data in degrees (filtered) (x, y, z) except yaw angle
} MPU6050_t;

/**
 * @brief Initialize the MPU6050 sensor
 * @param mpu Pointer to MPU6050_t structure
 * @param hi2c Pointer to I2C handle
 * @param address I2C address of MPU6050 (MPU6050_ADDR_AD0_LOW or MPU6050_ADDR_AD0_HIGH)
 * @return HAL status (HAL_OK if successful)
 */
HAL_StatusTypeDef MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c, uint8_t address, INT_ENABLE_t *interrupts, INT_CONFIG_t *interrupt_config, volatile uint8_t *is_ready);

/**
 * @brief Set gyroscope range
 * @param mpu Pointer to MPU6050_t structure
 * @param range Gyroscope range (see MPU6050_GyroRange enum)
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_SetGyroRange(MPU6050_t *mpu, MPU6050_GyroRange range);

/**
 * @brief Set accelerometer range
 * @param mpu Pointer to MPU6050_t structure
 * @param range Accelerometer range (see MPU6050_AccelRange enum)
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_SetAccelRange(MPU6050_t *mpu, MPU6050_AccelRange range);

/**
 * @brief Set digital low pass filter bandwidth
 * @param mpu Pointer to MPU6050_t structure
 * @param bandwidth DLPF bandwidth (see MPU6050_DLPF_Bandwidth enum)
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_SetDLPFBandwidth(MPU6050_t *mpu, MPU6050_DLPF_Bandwidth bandwidth);

/**
 * @brief Set sample rate divider
 * @param mpu Pointer to MPU6050_t structure
 * @param divider Sample rate divider value (0-255)
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_SetSampleRateDivider(MPU6050_t *mpu, uint8_t divider);

/**
 * @brief Set FIFO buffer enable or disable
 * @param mpu Pointer to MPU6050_t structure
 * @param divider Sample rate divider value (0-255)
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_SetUserControl(MPU6050_t *mpu, bool enable_buffer, FIFO_SELECTION_t *fifo_selection);


/**
 * @brief Read accelerometer data
 * @param mpu Pointer to MPU6050_t structure
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_ReadAccelerometer(MPU6050_t *mpu);

/**
 * @brief Read gyroscope data
 * @param mpu Pointer to MPU6050_t structure
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_ReadGyroscope(MPU6050_t *mpu);

/**
 * @brief Read temperature data
 * @param mpu Pointer to MPU6050_t structure
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_ReadTemperature(MPU6050_t *mpu);

/**
 * @brief Read all sensor data (accelerometer, gyroscope, temperature)
 * @param mpu Pointer to MPU6050_t structure
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_ReadAllData(MPU6050_t *mpu);


/**
 * @brief enable selected interrupt
 * @param mpu Pointer to MPU6050_t structure
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_InitInterrupts(MPU6050_t *mpu, INT_ENABLE_t *interrupts, INT_CONFIG_t *interrupt_config);

/**
 * @brief Calibrate MPU6050 gyroscope and accelerometer
 * @param mpu Pointer to MPU6050_t structure
 * @param samples Number of samples to use for calibration
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_Calibrate(MPU6050_t *mpu, volatile uint8_t *is_ready, uint16_t samples);

uint8_t MPU6050_DataReady(void);


#endif /* INC_MPU6050_H_ */
