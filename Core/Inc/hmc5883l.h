#ifndef HMC5883L_H
#define HMC5883L_H

#include "stm32h7xx_hal.h"

#define HMC5883L_ADDR (0x1E) << 1

/* Register Addresses */
#define HMC5883L_REG_CONFIG_A       0x00
#define HMC5883L_REG_CONFIG_B       0x01
#define HMC5883L_REG_MODE           0x02
#define HMC5883L_REG_DATA_X_MSB     0x03
#define HMC5883L_REG_DATA_X_LSB     0x04
#define HMC5883L_REG_DATA_Z_MSB     0x05
#define HMC5883L_REG_DATA_Z_LSB     0x06
#define HMC5883L_REG_DATA_Y_MSB     0x07
#define HMC5883L_REG_DATA_Y_LSB     0x08
#define HMC5883L_REG_STATUS         0x09

typedef struct {
    I2C_HandleTypeDef* hi2c;
    
    int16_t x;
    int16_t y;
    int16_t z;

    float heading_deg;

    float offset[3];   // Hard iron offset
    float scale[3];    // Soft iron scale
} HMC5883L_t;

typedef enum {
    HMC5883L_0_75HZ = 0,
    HMC5883L_1_5HZ = 1,
    HMC5883L_3HZ = 2,
    HMC5883L_7_5HZ = 3,
    HMC5883L_15HZ = 4,
    HMC5883L_30HZ = 5,
    HMC5883L_75HZ = 6
} OUTPUT_RATE; // 2nd, 3rd and 4th bits

typedef enum {
    HMC5883L_AVG_SMP_1 = 0,
    HMC5883L_AVG_SMP_2 = 1,
    HMC5883L_AVG_SMP_4 = 2,
    HMC5883L_AVG_SMP_8 = 3
} AVG_SAMPLE; // 5th and 6th bits

typedef enum {
    HMC5883L_MSM_MODE_PN = 0,
    HMC5883L_MSM_MODE_P = 1,
    HMC5883L_MSM_MODE_N = 2
} MEASUREMENT_MODE; // 0th and 1st bits

typedef enum {
    HMC5883L_0_78_GA = 0,
    HMC5883L_1_3_GA = 1,
    HMC5883L_1_9_GA = 2,
    HMC5883L_2_5_GA = 3,
    HMC5883L_4_GA = 4,
    HMC5883L_4_7_GA = 5,
    HMC5883L_5_6_GA = 6,
    HMC5883L_8_1_GA = 7
} FIELD_RANGE; // 5th, 6th and 7th bits

typedef enum {
    HMC5883L_CONTINUOUS = 0,
    HMC5883L_SINGLE = 1,
    HMC5883L_IDLE_1 = 2,
    HMC5883L_IDLE_2 = 3,
} OPERATING_MODE; // 0th, 1st bits

HAL_StatusTypeDef HMC5883L_Init(HMC5883L_t *hmc, I2C_HandleTypeDef *hi2c, volatile uint8_t *is_ready);
HAL_StatusTypeDef HMC5883L_ReadRaw(HMC5883L_t *hmc);
void HMC5883L_Calibration_Update(HMC5883L_t *hmc, uint16_t sample, volatile uint8_t *is_ready);

#endif // HMC5883L_H