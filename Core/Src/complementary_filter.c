/*
 * complementary_filter.c
 *
 *  Created on: Aug 13, 2025
 *      Author: pilotpc
 */

#include "complementary_filter.h"
#include <stdio.h>

#define RAD_TO_DEG (180.0f / M_PI)

uint32_t timer = 0;
float acc_roll = 0, acc_pitch = 0;
float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
static float alpha = 0.98f;
double dt = 1/200;

void Calculate_Euler_Angles(MPU6050_t *mpu6050, uint32_t tick) {
	if (timer == 0) {
		dt = 0; // start time axis at 0
	} else {
		dt = (double)(tick - timer) / 1000;
	}
	timer = tick;
	// y axis is pitch and x is roll
	acc_roll = atan2(mpu6050->accel[1], sqrt(mpu6050->accel[0]*mpu6050->accel[0] + mpu6050->accel[2]*mpu6050->accel[2])) * RAD_TO_DEG;
    acc_pitch = atan2(-mpu6050->accel[0], sqrt(mpu6050->accel[1]*mpu6050->accel[1] + mpu6050->accel[2]*mpu6050->accel[2])) * RAD_TO_DEG;
	printf("dt: %f\n", dt);


	mpu6050->gyro_angel[0] += mpu6050->gyro[0] * dt;
	mpu6050->gyro_angel[1] += mpu6050->gyro[1] * dt;
	mpu6050->gyro_angel[2] += mpu6050->gyro[2] * dt;
	mpu6050->gyro_angel_f[0] = alpha * (mpu6050->gyro_angel_f[0] + mpu6050->gyro[0] * dt) + (1.0f - alpha) * acc_roll;
    mpu6050->gyro_angel_f[1] = alpha * (mpu6050->gyro_angel_f[1] + mpu6050->gyro[1] * dt) + (1.0f - alpha) * acc_pitch;
    mpu6050->gyro_angel_f[2] = mpu6050->gyro_angel[2];

	printf("gyro x: %f, gyro y: %f, gyro z: %f, accel x: %f, accel y: %f, accel z: %f, filtered_gyro x: %f, filtered_gyro y: %f, filtered_gyro z: %f, accel roll: %f, accel pitch: %f\n",
						mpu6050->gyro_angel[0], mpu6050->gyro_angel[1], mpu6050->gyro_angel[2], 
						mpu6050->accel[0],  mpu6050->accel[1],  mpu6050->accel[2], 
						mpu6050->gyro_angel_f[0], mpu6050->gyro_angel_f[1], mpu6050->gyro_angel_f[2],
						acc_roll, acc_pitch);
}
