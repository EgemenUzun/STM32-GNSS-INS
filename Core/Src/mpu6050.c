/*
 * mpu6050.c
 *
 *  Created on: Mar 24, 2025
 *      Author: pilotpc
 */

#include "mpu6050.h"
#include <stdio.h>

HAL_StatusTypeDef MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c, uint8_t address, INT_ENABLE_t *interrupts, INT_CONFIG_t *interrupt_config, volatile uint8_t *is_ready) {
	uint8_t check, data;
	mpu->hi2c = hi2c;
	mpu->address = address;

	if(HAL_I2C_IsDeviceReady(mpu->hi2c, mpu->address, 1, HAL_MAX_DELAY) != HAL_OK) {
		printf("Device is not ready to use\n");
		return HAL_ERROR;
	}
	printf("Device is ready to use\n");
	// Check device ID
	if (HAL_I2C_Mem_Read(mpu->hi2c, mpu->address, MPU6050_REG_WHO_AM_I, 1, &check, 1, HAL_MAX_DELAY) != HAL_OK) {
		printf("Couldn't read WHO_AM_I register\n");
		return HAL_ERROR;
	}
	if (check != 0x68) {  // MPU6050 WHO_AM_I register should return 0x68
		printf("MPU6050 device address is not 0x68\n");
		return HAL_ERROR;
	}

	// Reset the whole module before initialization
	data = 0x01 << 7;
	if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		printf("MPU6050 is couldn't restart\n");
		return HAL_ERROR;
	}
	printf("MPU6050 is restart\n");
	HAL_Delay(50);

	// Wake up the sensor (clear sleep mode bit 6 in PWR_MGMT_1)
	data = 0x00;
	if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		printf("MPU6050 is couldn't wake up\n");
		return HAL_ERROR;
	}
	printf("MPU6050 is wake up\n");
	HAL_Delay(50);

	data = 0x09; // Update sample rate as 200HZ
	if(MPU6050_SetSampleRateDivider(mpu, data) != HAL_OK) {
		printf("MPU6050 sample rate couldn't update\n");
		return HAL_ERROR;
	}
	printf("MPU6050 sample rate is updated\n");
	HAL_Delay(50);

	// Set gyroscope configuration
	if (MPU6050_SetGyroRange(mpu, MPU6050_GYRO_RANGE_250_DEG) != HAL_OK) {
		printf("MPU6050 gyro config register is couldn't update\n");
		return HAL_ERROR;
	}
	printf("MPU6050 gyro config register is updated\n");
	HAL_Delay(50);

	// Set accelerometer configuration
	if (MPU6050_SetAccelRange(mpu, MPU6050_ACCEL_RANGE_2_G) != HAL_OK) {
		printf("MPU6050 accel config register is couldn't update\n");
		return HAL_ERROR;
	}
	printf("MPU6050 accel config register is updated\n");
	HAL_Delay(50);

	// Set Digital Low Pass Filter configuration
	if (MPU6050_SetDLPFBandwidth(mpu, MPU6050_DLPF_BW_44_HZ) != HAL_OK) {
		printf("MPU6050 digital low pass filter register is couldn't update\n");
		return HAL_ERROR;
	}
	printf("MPU6050 digital low pass filter register is updated\n");
	HAL_Delay(50);

	// Initialize interrupts
	if (MPU6050_InitInterrupts(mpu, interrupts, interrupt_config) != HAL_OK) {
		printf("MPU6050 interrupt config and enable register is couldn't update\n");
		return HAL_ERROR;
	}
	printf("MPU6050 interrupt config and enable register is updated\n");

	// Calibrate Gyroscope
	// if (MPU6050_Calibrate(mpu, is_ready, 2000) != HAL_OK) {
	// 	printf("Calibration is failed\n");
	// 	return HAL_ERROR;
	// }
	MPU6050_SetOffsets(mpu, -51, 386, -149, 160, 498, 1872);
	printf("Calibration is finished\n");
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_SetGyroRange(MPU6050_t *mpu, MPU6050_GyroRange range) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_REG_GYRO_CONFIG, 1, (uint8_t*)&range, 1, HAL_MAX_DELAY);
	if (status == HAL_OK) {
		/* Update sensitivity based on chosen range */
		switch (range) {
		case MPU6050_GYRO_RANGE_250_DEG:
			mpu->gyro_sensitivity = 131.0f; // LSB/(°/s)
			break;
		case MPU6050_GYRO_RANGE_500_DEG:
			mpu->gyro_sensitivity = 65.5f;
			break;
		case MPU6050_GYRO_RANGE_1000_DEG:
			mpu->gyro_sensitivity = 32.8f;
			break;
		case MPU6050_GYRO_RANGE_2000_DEG:
			mpu->gyro_sensitivity = 16.4f;
			break;
		}
	} else {
		printf("MPU6050 gyro config register couldn't updated\n");
	}
	return status;
}

HAL_StatusTypeDef MPU6050_SetAccelRange(MPU6050_t *mpu, MPU6050_AccelRange range) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_REG_ACCEL_CONFIG, 1, (uint8_t*)&range, 1, HAL_MAX_DELAY);

	if (status == HAL_OK) {
		/* Update sensitivity based on chosen range */
		switch (range) {
		case MPU6050_ACCEL_RANGE_2_G:
			mpu->accel_sensitivity = 16384.0f; // LSB/g
			break;
		case MPU6050_ACCEL_RANGE_4_G:
			mpu->accel_sensitivity = 8192.0f;
			break;
		case MPU6050_ACCEL_RANGE_8_G:
			mpu->accel_sensitivity = 4096.0f;
			break;
		case MPU6050_ACCEL_RANGE_16_G:
			mpu->accel_sensitivity = 2048.0f;
			break;
		}
	} else {
		printf("MPU6050 accel config register couldn't updated\n");
	}
	return status;
}

HAL_StatusTypeDef MPU6050_SetDLPFBandwidth(MPU6050_t *mpu, MPU6050_DLPF_Bandwidth bandwidth) {
	return HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_REG_CONFIG, 1, (uint8_t*)&bandwidth, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_SetSampleRateDivider(MPU6050_t *mpu, uint8_t divider) {
	return HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_REG_SMPLRT_DIV, 1, &divider, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_SetUserControl(MPU6050_t *mpu, bool enable_buffer, FIFO_SELECTION_t *fifo_selection) {
	uint8_t fifo_en_data = enable_buffer ? 0x40 : 0x00;
	if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_REG_USER_CTRL, 1, &fifo_en_data, 1, HAL_MAX_DELAY) != HAL_OK) {
		printf("User control register couldn't set\n");
		return HAL_ERROR;
	}
	uint8_t buffer_selection_data = 0x00;
	buffer_selection_data |=  fifo_selection->xg_fifo_en ? 0x01 << 6 : 0x00;
	buffer_selection_data |=  fifo_selection->yg_fifo_en ? 0x01 << 5 : 0x00;
	buffer_selection_data |=  fifo_selection->zg_fifo_en ? 0x01 << 4 : 0x00;
	buffer_selection_data |=  fifo_selection->accel_fifo_en ? 0x01 << 3 : 0x00;

	return HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_REG_FIFO_EN, 1, &buffer_selection_data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_ReadAccelerometer(MPU6050_t *mpu) {
	uint8_t data[6];
	if (HAL_I2C_Mem_Read(mpu->hi2c, mpu->address, MPU6050_REG_ACCEL_XOUT_H, 1, data, 6, HAL_MAX_DELAY) != HAL_OK) {
		printf("Couldn't read accelerometer data\n");
		return HAL_ERROR;
	}

	mpu->accel_raw[0] = (int16_t)(data[0] << 8 | data[1]);
	mpu->accel_raw[1] = (int16_t)(data[2] << 8 | data[3]);
	mpu->accel_raw[2] = (int16_t)(data[4] << 8 | data[5]);

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadGyroscope(MPU6050_t *mpu) {
	uint8_t data[6];
	if (HAL_I2C_Mem_Read(mpu->hi2c, mpu->address, MPU6050_REG_GYRO_XOUT_H, 1, data, 6, HAL_MAX_DELAY) != HAL_OK) {
		printf("Couldn't read gyroscope data\n");
		return HAL_ERROR;
	}

	mpu->gyro_raw[0] = (int16_t)(data[0] << 8 | data[1]);
	mpu->gyro_raw[1] = (int16_t)(data[2] << 8 | data[3]);
	mpu->gyro_raw[2] = (int16_t)(data[4] << 8 | data[5]);

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadTemperature(MPU6050_t *mpu) {
	uint8_t data[2];
	if (HAL_I2C_Mem_Read(mpu->hi2c, mpu->address, MPU6050_REG_TEMP_OUT_H, 1, data, 2, HAL_MAX_DELAY) != HAL_OK) {
		printf("Couldn't read temperature data\n");
		return HAL_ERROR;
	}

	mpu->temp_raw = (int16_t)(data[0] << 8 | data[1]);

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAllData(MPU6050_t *mpu) {
	uint8_t data[14];
	HAL_StatusTypeDef response = HAL_I2C_Mem_Read(mpu->hi2c, mpu->address, MPU6050_REG_ACCEL_XOUT_H, 1, data, 14, HAL_MAX_DELAY);
	if (response != HAL_OK) {
		printf("Couldn't read all data: %d\n", response);
		return HAL_ERROR;
	}

	mpu->accel_raw[0] = (int16_t)(data[0] << 8 | data[1]);
	mpu->accel_raw[1] = (int16_t)(data[2] << 8 | data[3]);
	mpu->accel_raw[2] = (int16_t)(data[4] << 8 | data[5]);

	mpu->temp_raw = (int16_t)(data[6] << 8 | data[7]);

	mpu->gyro_raw[0] = (int16_t)(data[8]  << 8 | data[9]);
	mpu->gyro_raw[1] = (int16_t)(data[10] << 8 | data[11]);
	mpu->gyro_raw[2] = (int16_t)(data[12] << 8 | data[13]);

	mpu->temp = (float)(mpu->temp_raw)/340+36.53;

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Calibrate(MPU6050_t *mpu, volatile uint8_t *is_ready, uint16_t samples) {
	// Recommended sample is 2000
	int64_t accel_offset[3] = {0}, gyro_offset[3] = {0};
	uint16_t i = 0;
	while (i < samples) {
		if (*is_ready) {
			*is_ready = 0;
			if (MPU6050_ReadAllData(mpu) == HAL_OK) {
				MPU6050_ClearInterrupt(mpu);
				printf("Offsets read: %d\n", i);
				for (uint8_t j = 0; j < 3; j++) {
					accel_offset[j] += mpu->accel_raw[j];
					gyro_offset[j] += mpu->gyro_raw[j];
				}
				i++;
			}
		}
	}

	mpu->gyro_offset[0] = gyro_offset[0] / samples;
    mpu->gyro_offset[1] = gyro_offset[1] / samples;
    mpu->gyro_offset[2] = gyro_offset[2] / samples;
    
    mpu->accel_offset[0] = accel_offset[0] / samples;
    mpu->accel_offset[1] = accel_offset[1] / samples;
    mpu->accel_offset[2] = (accel_offset[2] / samples) - (int16_t)mpu->accel_sensitivity;;

	printf("---IMU Offsets---\n");
	printf("GYROX | GYROY | GYROZ | ACCX | ACCY | ACCZ \n");
	printf("%d, %d , %d, %d, %d, %d\n", mpu->gyro_offset[0], mpu->gyro_offset[1], mpu->gyro_offset[2],
			mpu->accel_offset[0], mpu->accel_offset[1], mpu->accel_offset[2]);
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_InitInterrupts(MPU6050_t *mpu, INT_ENABLE_t *interrupts, INT_CONFIG_t *interrupt_config) {
	uint8_t enable_interrups = 0x00, config = 0x00;

	if (interrupt_config->i2c_bypass_en)
		config |= I2C_BYPASS_EN;
	if (interrupt_config->fsync_int_en)
		config |= FSYNC_INT_EN;
	if (interrupt_config->fsync_int_level)
		config |= FSYNC_INT_LEVEL;
	if (interrupt_config->int_rd_clear)
		config |= INT_RD_CLEAR;
	if (interrupt_config->latch_int_en)
		config |= LATCH_INT_EN;
	if (interrupt_config->int_open)
		config |= INT_OPEN;
	if (interrupt_config->int_level)
		config |= INT_LEVEL;
	if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_INT_PIN_CFG, 1, &config, 1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	if (interrupts->data_rdy)
		enable_interrups |= DATA_RDY_EN;
	if (interrupts->fifo_offlow)
		enable_interrups |= FIFO_OFLOW_EN;
	if (interrupts->i2c_mst_int)
		enable_interrups |= I2C_MST_INT_EN;
	if (interrupts->mot)
		enable_interrups |= MOT_EN;
	if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->address, MPU6050_REG_INT_ENABLE, 1, &enable_interrups, 1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ClearInterrupt(MPU6050_t *mpu) {
    uint8_t int_status;
    return HAL_I2C_Mem_Read(mpu->hi2c, mpu->address, MPU6050_REG_INT_STATUS, 1, &int_status, 1, HAL_MAX_DELAY);
}

uint8_t MPU6050_DataReady(void) {
	return HAL_GPIO_ReadPin(MPU6050_INT_PORT, MPU6050_INT_PIN);
}

void MPU6050_SetOffsets(MPU6050_t *mpu, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t accel_x, int16_t accel_y, int16_t accel_z) {
	mpu->gyro_offset[0] = gyro_x;
    mpu->gyro_offset[1] = gyro_y;
    mpu->gyro_offset[2] = gyro_z;
    
    mpu->accel_offset[0] = accel_x;
    mpu->accel_offset[1] = accel_y;
    mpu->accel_offset[2] = accel_z;
}
