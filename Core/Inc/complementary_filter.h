/*
 * complementary_filter.h
 *
 *  Created on: Aug 13, 2025
 *      Author: pilotpc
 */

#ifndef INC_COMPLEMENTARY_FILTER_H_
#define INC_COMPLEMENTARY_FILTER_H_
#include "mpu6050.h"
#include "hmc5883l.h"

void Calculate_Euler_Angles(MPU6050_t *mpu6050, uint32_t tick, HMC5883L_t *hmc);
void HMC5883L_CalculateHeading(HMC5883L_t *hmc);

#endif /* INC_COMPLEMENTARY_FILTER_H_ */
