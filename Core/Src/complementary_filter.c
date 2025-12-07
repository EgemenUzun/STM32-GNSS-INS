/*
 * complementary_filter.c
 *
 *  Created on: Aug 13, 2025
 *      Author: pilotpc
 */

#include "complementary_filter.h"
#include <stdio.h>

#define ANKARA_DECLINATION_ANGLE (6.2f * (M_PI / 180.0f))
#define RAD_TO_DEG (180.0f / M_PI )
#define DEG_TO_RAD (M_PI / 180.0f)
#define ANKARA_DECLINATION_RAD 0.1082f 

uint32_t timer = 0;
float acc_roll = 0, acc_pitch = 0;
float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
float alpha = 0.98f;
float alpha_heading = 0.97f;
double dt = 0;

void Calculate_Euler_Angles(MPU6050_t *mpu6050, uint32_t tick, HMC5883L_t *hmc) {
	if (timer == 0) {
		dt = 0; // start time axis at 0
	} else {
		dt = (double)(tick - timer) / 1000;
	}
	timer = tick;
	// y axis is pitch and x is roll
	acc_pitch = atan2(mpu6050->accel[1], mpu6050->accel[2]) * RAD_TO_DEG;
    acc_roll = atan2(-mpu6050->accel[0], mpu6050->accel[2]) * RAD_TO_DEG;
	printf("dt: %f\n", dt);

	mpu6050->gyro[0] = (float)(mpu6050->gyro_raw[0] - mpu6050->gyro_offset[0]) / mpu6050->gyro_sensitivity;
	mpu6050->gyro[1] = (float)(mpu6050->gyro_raw[1] - mpu6050->gyro_offset[1]) / mpu6050->gyro_sensitivity;
	mpu6050->gyro[2] = (float)(mpu6050->gyro_raw[2] - mpu6050->gyro_offset[2]) / mpu6050->gyro_sensitivity;

	mpu6050->accel[0] = (float)(mpu6050->accel_raw[0] - mpu6050->accel_offset[0]) / mpu6050->accel_sensitivity;
	mpu6050->accel[1] = (float)(mpu6050->accel_raw[1] - mpu6050->accel_offset[1]) / mpu6050->accel_sensitivity;
	mpu6050->accel[2] = (float)(mpu6050->accel_raw[2] - mpu6050->accel_offset[2]) / mpu6050->accel_sensitivity;

	mpu6050->gyro_angel[0] += mpu6050->gyro[0] * dt;
	mpu6050->gyro_angel[1] += mpu6050->gyro[1] * dt;
	mpu6050->gyro_angel[2] += mpu6050->gyro[2] * dt;
	mpu6050->gyro_angel[2] = fmod(mpu6050->gyro_angel[2], 360.0f);
	if (mpu6050->gyro_angel[2] < 0) {
    	mpu6050->gyro_angel[2] += 360.0f;
	}
	HMC5883L_CalculateHeading(hmc);

    mpu6050->gyro_angel_f[0] = alpha * (mpu6050->gyro_angel_f[0] + mpu6050->gyro[0] * dt) + (1.0f - alpha) * acc_pitch;
	mpu6050->gyro_angel_f[1] = alpha * (mpu6050->gyro_angel_f[1] + mpu6050->gyro[1] * dt) + (1.0f - alpha) * acc_roll;
    mpu6050->gyro_angel_f[2] = alpha_heading * (mpu6050->gyro_angel_f[2] + mpu6050->gyro[2] * dt) + (1.0f - alpha_heading) * hmc->heading_deg;

	printf("gyro x: %f, gyro y: %f, gyro z: %f, accel x: %f, accel y: %f, accel z: %f, filtered_gyro x: %f, filtered_gyro y: %f, filtered_gyro z: %f, accel roll: %f, accel pitch: %f\n",
						mpu6050->gyro_angel[0], mpu6050->gyro_angel[1], mpu6050->gyro_angel[2], 
						mpu6050->accel[0],  mpu6050->accel[1],  mpu6050->accel[2], 
						mpu6050->gyro_angel_f[0], mpu6050->gyro_angel_f[1], mpu6050->gyro_angel_f[2],
						acc_roll, acc_pitch);
}

void HMC5883L_CalculateHeading_Avionics(HMC5883L_t *hmc, float pitch_rad, float roll_rad)
{
    float mx = (float)hmc->x;
    float my = (float)hmc->y;
    float mz = (float)hmc->z;

    // ---------- 1. HARD IRON CORRECTION ----------
    mx -= hmc->offset[0];
    my -= hmc->offset[1];
    mz -= hmc->offset[2];

    // ---------- 2. SOFT IRON CORRECTION ----------
    mx *= hmc->scale[0];
    my *= hmc->scale[1];
    mz *= hmc->scale[2];

    // ---------- 3. TILT COMPENSATION ----------
    
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);

    // Tilt compensated X ve Y
    float Xh = mx * cos_pitch + mz * sin_pitch; 

    float Yh = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

    // ---------- 4. HEADING Calculation ----------
    float heading = atan2f(Yh, Xh);

    // ---------- 5. DECLINATION CORRECTION (Ankara) ----------
    heading += ANKARA_DECLINATION_ANGLE;

    // ---------- 6. Normalization (0 - 2PI) ----------
    if (heading < 0)
        heading += 2.0f * M_PI;
    if (heading > 2.0f * M_PI)
        heading -= 2.0f * M_PI;

    hmc->heading_deg = heading * (180.0f / M_PI);

    printf("Heading (Ankara/Compensated): %.2f\n", hmc->heading_deg);
}


void Calculate_AHRS(MPU6050_t *mpu6050, HMC5883L_t *hmc, uint32_t tick) 
{
    // -------------------------------------------------------------------------
    // 1. ZAMAN FARKI (DT) HESAPLAMA
    // -------------------------------------------------------------------------
    if (timer == 0) {
        dt = 0; 
        timer = tick;
        return; // İlk döngüde dt hatalı olmasın diye çıkıyoruz
    } else {
        dt = (double)(tick - timer) / 1000.0;
    }
    timer = tick;

    // -------------------------------------------------------------------------
    // 2. RAW VERİYİ FİZİKSEL BİRİME ÇEVİRME (ÖNCE BUNU YAPMALIYIZ)
    // -------------------------------------------------------------------------
    for(int i=0; i<3; i++) {
        mpu6050->gyro[i] = (float)(mpu6050->gyro_raw[i] - mpu6050->gyro_offset[i]) / mpu6050->gyro_sensitivity;
        mpu6050->accel[i] = (float)(mpu6050->accel_raw[i] - mpu6050->accel_offset[i]) / mpu6050->accel_sensitivity;
    }

	mpu6050->gyro_angel[0] += mpu6050->gyro[0] * dt;
	mpu6050->gyro_angel[1] += mpu6050->gyro[1] * dt;
	mpu6050->gyro_angel[2] += mpu6050->gyro[2] * dt;
	mpu6050->gyro_angel[2] = fmod(mpu6050->gyro_angel[2], 360.0f);
	if (mpu6050->gyro_angel[2] < 0) {
    	mpu6050->gyro_angel[2] += 360.0f;
	}

    // -------------------------------------------------------------------------
    // 3. ACCELEROMETER İLE PITCH & ROLL HESABI (Trigonometri)
    // -------------------------------------------------------------------------
    // Standart Havacılık (NED): X-İleri, Y-Sağ, Z-Aşağı
    // acc_roll (phi): X ekseni etrafında dönüş.
    // acc_pitch (theta): Y ekseni etrafında dönüş.

	float acc_pitch = atan2(mpu6050->accel[1], mpu6050->accel[2]) * RAD_TO_DEG;
    float acc_roll = atan2(-mpu6050->accel[0], mpu6050->accel[2]) * RAD_TO_DEG;
    

    // -------------------------------------------------------------------------
    // 4. GYRO INTEGRASYONU & COMPLEMENTARY FILTER (PITCH & ROLL)
    // -------------------------------------------------------------------------
    // Formül: Filtreli_Açı = alpha * (Eski_Açı + Gyro_Hızı * dt) + (1-alpha) * İvmeölçer_Açısı
    
	mpu6050->gyro_angel_f[0] = alpha * (mpu6050->gyro_angel_f[0] + mpu6050->gyro[0] * dt) + (1.0f - alpha) * acc_pitch;
	mpu6050->gyro_angel_f[1] = alpha * (mpu6050->gyro_angel_f[1] + mpu6050->gyro[1] * dt) + (1.0f - alpha) * acc_roll;

    // -------------------------------------------------------------------------
    // 5. TILT COMPENSATED MAGNETOMETER (EĞİM TELAFİLİ PUSULA)
    // -------------------------------------------------------------------------
    // Filtrelenmiş Pitch ve Roll değerlerini kullanıyoruz (Radyana çevirerek)
    // Bu işlem uçağın/dronun eğilmesine rağmen pusulanın düzgün çalışmasını sağlar.
    
    float pitch_rad = mpu6050->gyro_angel_f[0] * DEG_TO_RAD;
    float roll_rad = mpu6050->gyro_angel_f[1] * DEG_TO_RAD;

    // Manyetometre Kalibrasyon (Hard Iron)
    float mx = (float)hmc->x - hmc->offset[0]; 
    float my = (float)hmc->y - hmc->offset[1];
    float mz = (float)hmc->z - hmc->offset[2];
    
    // Eğim Telafisi (Rotation to Horizontal Plane)
    // Pitch (Yunuslama) düzeltmesi:
    float Xh = mx * cos(pitch_rad) + mz * sin(pitch_rad);
    
    // Roll (Yuvarlanma) düzeltmesi:
    float Yh = mx * sin(roll_rad) * sin(pitch_rad) + my * cos(roll_rad) - mz * sin(roll_rad) * cos(pitch_rad);

    // Baş Yönü (Heading) Hesabı
    float mag_heading = atan2(Yh, Xh);

    // Ankara Manyetik Sapma Eklemesi (+6.2 derece)
    mag_heading += ANKARA_DECLINATION_RAD;

    // 0-2PI Normalizasyonu
    if (mag_heading < 0) mag_heading += 2.0f * M_PI;
    if (mag_heading > 2.0f * M_PI) mag_heading -= 2.0f * M_PI;

    // Dereceye çevirip struct içine kaydet (Debug/Log için)
    float mag_heading_deg = mag_heading * RAD_TO_DEG;
    hmc->heading_deg = mag_heading_deg; 

    // -------------------------------------------------------------------------
    // 6. YAW FUSION (WRAP-AROUND PROTECTED)
    // -------------------------------------------------------------------------
    // Bu kısım çok kritiktir. 359 dereceden 1 dereceye geçerken filtrenin sapıtmaması gerekir.
    
    // Gyro ile anlık tahmini yaw pozisyonu (Tahmin)
    float predicted_yaw = mpu6050->gyro_angel_f[2] + (mpu6050->gyro[2] * dt);

    // Manyetometre ile Gyro Tahmini arasındaki fark (Hata)
    float yaw_error = mag_heading_deg - predicted_yaw;

    // Farkı -180 ile +180 arasına sıkıştır (En kısa dönüş yolunu bul)
    // Örnek: Pusula 1 derece, Gyro 359 derece ise -> Fark -358 değil, +2 derece olmalı.
    while (yaw_error > 180.0f)  yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    // Filtreyi uygula: Yeni_Yaw = Tahmin + (Hata * (1 - Güven))
    mpu6050->gyro_angel_f[2] = predicted_yaw + (yaw_error * (1.0f - alpha_heading));

    // Sonuç 0-360 aralığında kalsın
    if (mpu6050->gyro_angel_f[2] < 0.0f) mpu6050->gyro_angel_f[2] += 360.0f;
    if (mpu6050->gyro_angel_f[2] >= 360.0f) mpu6050->gyro_angel_f[2] -= 360.0f;

    // -------------------------------------------------------------------------
    // 7. DEBUG ÇIKTISI
    // -------------------------------------------------------------------------
    // Seri portu çok meşgul etmemek için sadece filtrelenmiş veriyi basıyoruz.
    // İhtiyaç olursa diğerlerini açabilirsiniz.
    
    printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | Mag_Heading: %6.2f\n", 
           mpu6050->gyro_angel_f[1], 
           mpu6050->gyro_angel_f[0], 
           mpu6050->gyro_angel_f[2],
           mag_heading_deg);
}