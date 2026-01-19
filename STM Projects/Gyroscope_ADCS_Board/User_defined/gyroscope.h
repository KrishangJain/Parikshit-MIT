/*
 * gyroscope.h
 *
 *  Created on: July 12, 2023
 *      Author: SRISHTI SINGH
 */

#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_

#include "stm32h7xx_hal.h"
#include "stdlib.h"
#include "math.h"

#define SELF_TEST_X 0x0D
#define SELF_TEST_Y 0x0E
#define SELF_TEST_Z 0x0F
#define SELF_TEST_A 0x10
#define TIMER_TIMEOUT_I2C 0x000FFFFF
#define MPU6050_ADDR 0xD0     //device address for MPU6050

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
typedef struct {
	float_t acc_x;
	float_t acc_y;
	float_t acc_z;
	float_t temperature;
	float_t gyr_x;
	float_t gyr_y;
	float_t gyr_z;
} ADCS_GyroData_t;

ErrorStatus ADCS_Gyro_Init(I2C_HandleTypeDef *hi2c);
ErrorStatus ADCS_Gyro_SelfTest(I2C_HandleTypeDef *hi2c);
ErrorStatus ADCS_Gyro_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *value);
ErrorStatus ADCS_Gyro_ReadAll(I2C_HandleTypeDef *hi2c, ADCS_GyroData_t *data);

#endif /* GYROSCOPE_H_ */
