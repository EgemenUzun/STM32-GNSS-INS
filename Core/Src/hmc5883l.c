#include "hmc5883l.h"
#include <stdint.h>
#include <stdio.h>

HAL_StatusTypeDef HMC5883L_Init(HMC5883L_t *hmc, I2C_HandleTypeDef *hi2c,
                                volatile uint8_t *is_ready) {
  hmc->hi2c = hi2c;
  uint8_t data;

  // Step 1: Config A Register (0x00)
  // 8-average, 15 Hz default, normal measurement
  data =
      (HMC5883L_AVG_SMP_8 << 5) | (HMC5883L_15HZ << 2) | (HMC5883L_MSM_MODE_PN);
  if (HAL_I2C_Mem_Write(hmc->hi2c, HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 1,
                        &data, 1, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }

  // Step 2 Config B Register (0x01)
  // Gain = +-1.3 Ga (Default)
  data = 0X00 | HMC5883L_1_3_GA << 5;
  if (HAL_I2C_Mem_Write(hmc->hi2c, HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 1,
                        &data, 1, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }

  // Step 3: Mode Register (0x02)
  // Continuous-measurement mode
  data = 0x00 | HMC5883L_CONTINUOUS;
  if (HAL_I2C_Mem_Write(hmc->hi2c, HMC5883L_ADDR, HMC5883L_REG_MODE, 1, &data,
                        1, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }

  HMC5883L_Calibration_Update(hmc, 200, is_ready);
  return HAL_OK;
}

HAL_StatusTypeDef HMC5883L_ReadRaw(HMC5883L_t *hmc) {
  uint8_t buffer[6];
  if (HAL_I2C_Mem_Read(hmc->hi2c, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB,
                       I2C_MEMADD_SIZE_8BIT, buffer, 6, 100) != HAL_OK) {
    return HAL_ERROR;
  }
  hmc->x = (int16_t)((buffer[0] << 8) | buffer[1]);
  hmc->z = (int16_t)((buffer[2] << 8) | buffer[3]);
  hmc->y = (int16_t)((buffer[4] << 8) | buffer[5]);

  return HAL_OK;
}

void HMC5883L_Calibration_Update(HMC5883L_t *hmc, uint16_t sample,
                                 volatile uint8_t *is_ready) {
  float minX = 99999, minY = 99999, minZ = 99999;
  float maxX = -99999, maxY = -99999, maxZ = -99999;
  uint16_t i = 0;
  while (i < sample) {
    if (*is_ready) {
      *is_ready = 0;
      if (HMC5883L_ReadRaw(hmc) == HAL_OK) {
        printf("Sample read: %d\n", i);
        if (hmc->x < minX)
          minX = hmc->x;
        if (hmc->y < minY)
          minY = hmc->y;
        if (hmc->z < minZ)
          minZ = hmc->z;

        if (hmc->x > maxX)
          maxX = hmc->x;
        if (hmc->y > maxY)
          maxY = hmc->y;
        if (hmc->z > maxZ)
          maxZ = hmc->z;
      }
      i++;
    }
  }

  // HARD IRON offset
  hmc->offset[0] = (maxX + minX) / 2.0f;
  hmc->offset[1] = (maxY + minY) / 2.0f;
  hmc->offset[2] = (maxZ + minZ) / 2.0f;

  float rangeX = maxX - minX;
  float rangeY = maxY - minY;
  float rangeZ = maxZ - minZ;

  float avgRange = (rangeX + rangeY + rangeZ) / 3.0f;

  // SOFT IRON scale
  hmc->scale[0] = avgRange / rangeX;
  hmc->scale[1] = avgRange / rangeY;
  hmc->scale[2] = avgRange / rangeZ;
}