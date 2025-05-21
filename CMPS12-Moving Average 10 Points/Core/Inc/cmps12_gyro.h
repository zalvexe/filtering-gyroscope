/*
 * cmps12_gyro.h
 *
 *  Created on: Oct 11, 2024
 *      Author: zalv
 */

#ifndef INC_CMPS12_GYRO_H_
#define INC_CMPS12_GYRO_H_

#define I2C_ADDRESS 0x60
#define ACCEL_MA_WINDOW 10

#define REGISTER_GYRO_X 0x12
#define REGISTER_GYRO_Y 0x14
#define REGISTER_GYRO_Z 0x16
#define BEARING_Register 0x02
#define CALLIB_GYRO 0x1E

#define PITCH_Register 0x04
#define ROLL_Register 0x05

#define ACCELERO_X_Register 0x0C
#define ACCELERO_Y_Register 0x0E
#define ACCELERO_Z_Register 0x10

#define MAGNET_X_Register 0x06
#define MAGNET_Y_Register 0x08
#define MAGNET_Z_Register 0x0A

#define CONTROL_Register 0

#define TWO_BYTES 2
#define ONE_BYTE 1

float integrateGyro(int16_t gyro_dps);
void calibrateGyro();
int16_t getGyroX();
int16_t getGyroY();
int16_t getGyroZ();
int16_t getAcceleroX();
int16_t getAcceleroY();
int16_t getAcceleroZ();
int16_t getMagnetX();
int16_t getMagnetY();
int16_t getMagnetZ();
int16_t getBearing();
uint8_t getPitch();
uint8_t getRoll();
uint8_t getCalibrationState();
void updateAccelBuffers(void);
int16_t getAcceleroX_MA();
int16_t getAcceleroY_MA();
int16_t getAcceleroZ_MA();

void changeAddress(uint8_t i2cAddress, uint8_t newi2cAddress);

#endif /* INC_CMPS12_GYRO_H_ */
