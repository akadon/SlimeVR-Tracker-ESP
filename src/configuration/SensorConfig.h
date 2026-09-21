/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 TheDevMinerTV

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#ifndef SLIMEVR_CONFIGURATION_SENSORCONFIG_H
#define SLIMEVR_CONFIGURATION_SENSORCONFIG_H

#include <stdint.h>

#include "consts.h"

namespace SlimeVR::Configuration {
struct BMI160SensorConfig {
	// accelerometer offsets and correction matrix
	float A_B[3];
	float A_Ainv[3][3];

	// magnetometer offsets and correction matrix
	float M_B[3];
	float M_Ainv[3][3];

	// raw offsets, determined from gyro at rest
	float G_off[3];

	// calibration temperature for dynamic compensation
	float temperature;
};

struct SoftFusionSensorConfig {
	SensorTypeID ImuType;
	uint16_t MotionlessDataLen;

	// accelerometer offsets and correction matrix
	float A_B[3];
	float A_Ainv[3][3];

	// magnetometer offsets and correction matrix
	float M_B[3];
	float M_Ainv[3][3];

	// raw offsets, determined from gyro at rest
	float G_off[3];

	// calibration temperature for dynamic compensation
	float temperature;

	// real measured sensor sampling rate
	float A_Ts;
	float G_Ts;
	float M_Ts;

	// gyro sensitivity multiplier
	float G_Sens[3];

	uint8_t MotionlessData[60];

	// temperature sampling rate (placed at the end to not break existing configs)
	float T_Ts;
};

struct RuntimeCalibrationSensorConfig {
	SensorTypeID ImuType;
	uint16_t MotionlessDataLen;

	bool sensorTimestepsCalibrated;
	float A_Ts;
	float G_Ts;
	float M_Ts;
	float T_Ts;

	bool motionlessCalibrated;
	uint8_t MotionlessData[60];

	uint8_t gyroPointsCalibrated;
	float gyroMeasurementTemperature1;
	float G_off1[3];
	float gyroMeasurementTemperature2;
	float G_off2[3];

	bool accelCalibrated[3];
	float A_off[3];

	// Magnetometer hard-iron offset, in raw counts, in the IMU frame -- the same
	// frame and units as the driver's MagHardIron constant, i.e. remapped and
	// signed but before the subtraction. magCalibrated false means the driver's
	// compiled-in default is still in use.
	//
	// That default is one chip's offset applied to every board, and the offsets
	// are not close: two trackers a metre apart report fields of 381 and 224
	// counts, and the second one's earth-frame dip comes out at +5 degrees where
	// the real one is -68. A wrong offset is subtracted in the body frame, so it
	// rotates with the tracker and tilts the field direction by an amount that
	// depends on orientation -- which is what trips VQF's disturbance rejection
	// and, when it lasts, makes the filter adopt the distorted field as its new
	// reference and hold a heading that is wrong by over a hundred degrees.
	//
	// Written from the host with `SET MAGOFF`, fitted from a rotation capture.
	bool magCalibrated;
	float M_off[3];
};

// The magnetometer fields above must not grow this struct past the union's
// largest member: loadSensors() rejects any calibration file whose size is not
// sizeof(SensorConfig), so growing the union silently discards every stored
// calibration on the next boot.
static_assert(
	sizeof(RuntimeCalibrationSensorConfig) <= sizeof(SoftFusionSensorConfig),
	"RuntimeCalibrationSensorConfig grew past SoftFusionSensorConfig, which "
	"changes sizeof(SensorConfig) and invalidates every stored calibration"
);

struct MPU6050SensorConfig {
	// accelerometer offsets and correction matrix
	float A_B[3];

	// raw offsets, determined from gyro at rest
	float G_off[3];
};

struct MPU9250SensorConfig {
	// accelerometer offsets and correction matrix
	float A_B[3];
	float A_Ainv[3][3];

	// magnetometer offsets and correction matrix
	float M_B[3];
	float M_Ainv[3][3];

	// raw offsets, determined from gyro at rest
	float G_off[3];
};

struct ICM20948SensorConfig {
	// gyroscope bias
	int32_t G[3];

	// accelerometer bias
	int32_t A[3];

	// compass bias
	int32_t C[3];
};

struct ICM42688SensorConfig {
	// accelerometer offsets and correction matrix
	float A_B[3];
	float A_Ainv[3][3];

	// magnetometer offsets and correction matrix
	float M_B[3];
	float M_Ainv[3][3];

	// raw offsets, determined from gyro at rest
	float G_off[3];
};

struct BNO0XXSensorConfig {
	bool magEnabled;
};

enum class SensorConfigType {
	NONE,
	BMI160,
	MPU6050,
	MPU9250,
	ICM20948,
	SFUSION,
	BNO0XX,
	RUNTIME_CALIBRATION,
};

const char* calibrationConfigTypeToString(SensorConfigType type);

struct SensorConfig {
	SensorConfigType type;

	union {
		BMI160SensorConfig bmi160;
		SoftFusionSensorConfig sfusion;
		MPU6050SensorConfig mpu6050;
		MPU9250SensorConfig mpu9250;
		ICM20948SensorConfig icm20948;
		BNO0XXSensorConfig bno0XX;
		RuntimeCalibrationSensorConfig runtimeCalibration;
	} data;
};

struct SensorConfigBits {
	bool magEnabled : 1;
	bool magSupported : 1;
	bool calibrationEnabled : 1;
	bool calibrationSupported : 1;
	bool tempGradientCalibrationEnabled : 1;
	bool tempGradientCalibrationSupported : 1;

	// Remove if the above fields exceed a byte, necessary to make the struct 16
	// bit
	uint8_t padding;

	bool operator==(const SensorConfigBits& rhs) const;
	bool operator!=(const SensorConfigBits& rhs) const;
};

// If this fails, you forgot to do the above
static_assert(sizeof(SensorConfigBits) == 2);

}  // namespace SlimeVR::Configuration

#endif
