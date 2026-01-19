/*
 * gyroscope.c
 *
 *  Created on: July 12, 2023
 *      Author: SRISHTI SINGH
 *
 *  Modified on: January 12, 2026
 *  		 By: KRISHANG JAIN
 */

#include "gyroscope.h"

/**
 * @brief  Gyroscope Initialization Function
 * @param  hi2c pointer to I2c structure
 * @retval ErrorStatus
 *         ERROR-     0
 *         SUCCESS-   1
 */
ErrorStatus ADCS_Gyro_Init(I2C_HandleTypeDef *hi2c) {

	(hi2c->Devaddress) = MPU6050_ADDR; //device address
	(hi2c->State) = HAL_I2C_STATE_READY;
	(hi2c->Mode) = HAL_I2C_MODE_MASTER;
	(hi2c->XferSize) = 1;
	(hi2c->ErrorCode) = HAL_I2C_ERROR_NONE;

	return SUCCESS;

	/* USER CODE END I2C1_Init */

}

/**
 * @brief  Self testing of gyroscope
 * @param  hi2c pointer to I2c handle.
 * @retval ErrorStatus
 *         ERROR-     0
 *         SUCCESS-   1
 */

ErrorStatus ADCS_Gyro_SelfTest(I2C_HandleTypeDef *hi2c) {
	uint8_t buf_1 = 0;
	float_t factoryTrim[6];
	float_t destination[6];
	uint32_t TIMER_Simulator_Var;
	uint8_t rawData[4];
	float_t selfTest[6];

	(hi2c->Devaddress) = MPU6050_ADDR; //device address
	(hi2c->Memaddress) = 0x1B;

	buf_1 = 0xE0;
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &buf_1, 1, 50)
			!= HAL_OK) { /* I2C bus or peripheral is not able to start communication: Error management */
	}
	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}

	(hi2c->Memaddress) = 0x1C;

	buf_1 = 0xF0;
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &buf_1, 1, 50)
			!= HAL_OK) { /* I2C bus or peripheral is not able to start communication: Error management */
	}
	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}

	(hi2c->Memaddress) = SELF_TEST_X;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, SELF_TEST_X, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}
	/* Wait for end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	rawData[0] = buf_1;

	(hi2c->Memaddress) = SELF_TEST_Y;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, SELF_TEST_Y, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}
	/* Wait for end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	rawData[1] = buf_1;

	(hi2c->Memaddress) = SELF_TEST_Z;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, SELF_TEST_Z, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}
	/* Wait for end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	rawData[2] = buf_1;

	(hi2c->Memaddress) = SELF_TEST_A;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, SELF_TEST_A, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}
	/* Wait for end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	rawData[3] = buf_1;

	// Extract the acceleration test results first
	selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4; // XA_TEST result is a five-bit unsigned integer
	selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2; // YA_TEST result is a five-bit unsigned integer
	selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0; // ZA_TEST result is a five-bit unsigned integer
	// Extract the gyration test results first
	selfTest[3] = rawData[0] & 0x1F; // XG_TEST result is a five-bit unsigned integer
	selfTest[4] = rawData[1] & 0x1F; // YG_TEST result is a five-bit unsigned integer
	selfTest[5] = rawData[2] & 0x1F; // ZG_TEST result is a five-bit unsigned integer

	// Process results to allow final comparison with factory set values
	factoryTrim[0] = (float_t) (4096.0 * 0.34)
			* (float_t) (powf((0.92 / 0.34),
					(((float_t) selfTest[0] - 1.0) / 30.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float_t) (4096.0 * 0.34)
			* (float_t) (powf((0.92 / 0.34),
					(((float_t) selfTest[1] - 1.0) / 30.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float_t) (4096.0 * 0.34)
			* (float_t) (powf((0.92 / 0.34),
					(((float_t) selfTest[2] - 1.0) / 30.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float_t) (25.0 * 131.0)
			* (float_t) (powf(1.046, ((float_t) selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float_t) (-25.0 * 131.0)
			* (float_t) (powf(1.046, ((float_t) selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float_t) (25.0 * 131.0)
			* (float_t) (powf(1.046, ((float_t) selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

	for (uint8_t i = 0; i < 6; i++)
		destination[i] = 100.0
				+ (100.0 * ((float_t) selfTest[i] - factoryTrim[i])
						/ factoryTrim[i]); // Report percent differences

	if (destination[0] < 1 && destination[1] < 1 && destination[2] < 1
			&& destination[3] < 1 && destination[4] < 1 && destination[5] < 1)
		return SUCCESS;
	return ERROR;
}

/**
 * @brief  Reads the value in a register
 * @param  hi2c Pointer to I2C handle
 * @param  addr Register address to read
 * @param  data_buffer Pointer to store read value
 * @retval ErrorStatus
 *         ERROR-     0
 *         SUCCESS-   1
 */

ErrorStatus ADCS_Gyro_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t addr,
		uint8_t *data_buffer) {
	uint8_t buf_1 = 0;
	uint32_t TIMER_Simulator_Var;

	(hi2c->Memaddress) = addr;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, addr, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}
	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}

	*data_buffer = buf_1;

	return SUCCESS;
}

/**
 * @brief  Reads the values of gyroscope and accelerometer, in x, y & z direction and temperature
 * @param  hi2c Pointer to I2C handle
 * @param  data Pointer to gyroscope data structure
 * @retval ErrorStatus
 *         ERROR-     0
 *         SUCCESS-   1
 */

ErrorStatus ADCS_Gyro_ReadAll(I2C_HandleTypeDef *hi2c, ADCS_GyroData_t *data) {
	uint8_t buf_1 = 0;
	int16_t acc_x = 0, acc_y = 0, acc_z = 0;
	int16_t gyr_x = 0, gyr_y = 0, gyr_z = 0;
	int16_t temp = 0;
	uint32_t TIMER_Simulator_Var;

	/*******/

	(hi2c->Devaddress) = MPU6050_ADDR; //device address
	(hi2c->Memaddress) = 0x6B;    // power management 1 register

	buf_1 = 0x00; // for setting bit 6 of this register to 0
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, 1, &buf_1, 1, 50)
			!= HAL_OK) { /* I2C bus or peripheral is not able to start communication: Error management */
	}
	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}

	(hi2c->Memaddress) = 0x1B;  //gyroscope configuration register

	buf_1 = 0x00; // for writing 00000010 into configuration register
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &buf_1, 1, 50)
			!= HAL_OK) { /* I2C bus or peripheral is not able to start communication: Error management */
	}
	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}

	(hi2c->Memaddress) = 0x1C;  //accelerometer configuration register

	buf_1 = 0x00; // for writing 00000000 into configuration register
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &buf_1, 1, 50)
			!= HAL_OK) { /* I2C bus or peripheral is not able to start communication: Error management */
	}
	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	/*************/

	(hi2c->Memaddress) = 0x41;  //contains temperature value

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x41, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}
	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	temp = temp | buf_1;
	temp = temp << 8;

	(hi2c->Memaddress) = 0x42;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x42, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	temp = temp | buf_1;

	(hi2c->Memaddress) = 0x43;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x43, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}
	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	gyr_x = gyr_x | buf_1;
	gyr_x = gyr_x << 8;

	(hi2c->Memaddress) = 0x44;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x44, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	gyr_x = gyr_x | buf_1;

	(hi2c->Memaddress) = 0x45;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x45, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	gyr_y = gyr_y | buf_1;
	gyr_y = gyr_y << 8;

	(hi2c->Memaddress) = 0x46;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x46, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */
	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	gyr_y = gyr_y | buf_1;

	(hi2c->Memaddress) = 0x47;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x47, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	gyr_z = gyr_z | buf_1;
	gyr_z = gyr_z << 8;

	(hi2c->Memaddress) = 0x48;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x48, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */
	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	gyr_z = gyr_z | buf_1;

	(hi2c->Memaddress) = 0x3B;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3B, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */
	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	acc_x = acc_x | buf_1;
	acc_x = acc_x << 8;

	(hi2c->Memaddress) = 0x3C;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3C, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	acc_x = acc_x | buf_1;

	(hi2c->Memaddress) = 0x3D;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3D, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/*Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	acc_y = acc_y | buf_1;
	acc_y = acc_y << 8;

	(hi2c->Memaddress) = 0x3E;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3E, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	acc_y = acc_y | buf_1;

	(hi2c->Memaddress) = 0x3F;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3F, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */

	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	acc_z = acc_z | buf_1;
	acc_z = acc_z << 8;

	(hi2c->Memaddress) = 0x40;

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x40, 1, &buf_1, 1, 50)
			!= HAL_OK) {
		return ERROR;
	}

	/* Wait the end of transfer */
	TIMER_Simulator_Var = 0;
	while ((hi2c->State) != HAL_I2C_STATE_READY
			&& (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C))
		;
	if (TIMER_Simulator_Var == TIMER_TIMEOUT_I2C) {
		return ERROR;
	}
	acc_z = acc_z | buf_1;

	/*(hi2c->Memaddress)=0x68;

	 buf_1=0x07; // for writing 00000111 into configuration register
	 if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &buf_1, 1, 50) != HAL_OK){
	 return ERROR;
	 }
	 //     Wait the end of transfer

	 TIMER_Simulator_Var = 0;
	 while((hi2c->State) !=  HAL_I2C_STATE_READY && (++TIMER_Simulator_Var < TIMER_TIMEOUT_I2C));
	 if(TIMER_Simulator_Var == TIMER_TIMEOUT_I2C)
	 {
	 return ERROR;
	 }
	 addr = buf_1;
	 */

	data->acc_x = ((float_t) acc_x) / 16384.0;
	data->acc_y = ((float_t) acc_y) / 16384.0;
	data->acc_z = ((float_t) acc_z) / 16384.0;
	data->temperature = temp / 340.0 + 36.53;
	data->gyr_x = gyr_x / 131.0;
	data->gyr_y = gyr_y / 131.0;
	data->gyr_z = gyr_z / 131.0;

	return SUCCESS;
}

