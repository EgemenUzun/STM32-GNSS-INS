#include "hmc5883l.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"

HAL_StatusTypeDef HMC5883L_Init(HMC5883L_t *hmc, I2C_HandleTypeDef *hi2c) {
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

void HMC5883L_Find_Min_Max(HMC5883L_t *hmc, volatile uint8_t *is_ready) {
  printf("---Compass Calibration Start---\n");
  int minX = 32767, minY = 32767, minZ = 32767;
  int maxX = -32768, maxY = -32768, maxZ = -32768;
  bool changed = false;
  bool done = false;
  unsigned int t = 0;
  unsigned int c = HAL_GetTick();

  while (!done) {
    if (*is_ready) {
      *is_ready = 0;
      if (HMC5883L_ReadRaw(hmc) == HAL_OK) {
        changed = false;
        if (hmc->x < minX) {
          minX = hmc->x;
          changed = true;
        }
        if (hmc->y < minY) {
          minY = hmc->y;
          changed = true;
        }
        if (hmc->z < minZ) {
          minZ = hmc->z;
          changed = true;
        }

        if (hmc->x > maxX) {
          maxX = hmc->x;
          changed = true;
        }
        if (hmc->y > maxY) {
          maxY = hmc->y;
          changed = true;
        }
        if (hmc->z > maxZ) {
          maxZ = hmc->z;
          changed = true;
        }

        if(changed && !done){
          c = HAL_GetTick();
        }
        t = HAL_GetTick();
        
        if ((t - c > 10000) && !done) {
          done = true;
          	printf("---Compass Calibration Values---\n");
            printf("Max X: %d\n", maxX);
            printf("Min X: %d\n", minX);
            printf("Max Y: %d\n", maxY);
            printf("Min Y: %d\n", minY);
            printf("Max Z: %d\n", maxZ);
            printf("Min Z: %d\n", minZ);
        }
      }
    }

    hmc->max_x = maxX;
    hmc->min_x = minX;
    hmc->max_y = maxY;
    hmc->min_y = minY;
    hmc->max_z = maxZ;
    hmc->min_z = minZ;
  }
}

void HMC5883L_Calculate_Offsets_And_Scales(HMC5883L_t *hmc) {
  // HARD IRON offset
  hmc->offset[0] = (hmc->max_x + hmc->min_x) / 2.0f;
  hmc->offset[1] = (hmc->max_y + hmc->min_y) / 2.0f;
  hmc->offset[2] = (hmc->max_z + hmc->min_z) / 2.0f;

  float rangeX = hmc->max_x - hmc->min_x;
  float rangeY = hmc->max_y - hmc->min_y;
  float rangeZ = hmc->max_z - hmc->min_z;

  float avgRange = (rangeX + rangeY + rangeZ) / 3.0f;

  // SOFT IRON scale
  hmc->scale[0] = avgRange / rangeX;
  hmc->scale[1] = avgRange / rangeY;
  hmc->scale[2] = avgRange / rangeZ;
  printf("---Compass offsets and scales are defined---\n");
}

void HMC5883L_Set_Min_Max(HMC5883L_t *hmc, int maxX, int minX, int maxY, int minY, int maxZ, int minZ) {
    hmc->max_x = maxX;
    hmc->min_x = minX;
    hmc->max_y = maxY;
    hmc->min_y = minY;
    hmc->max_z = maxZ;
    hmc->min_z = minZ;
}