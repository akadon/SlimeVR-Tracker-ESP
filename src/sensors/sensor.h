/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#ifndef SLIMEVR_SENSOR_H_
#define SLIMEVR_SENSOR_H_

#include <Arduino.h>
#include <quat.h>
#include <vector3.h>

#include <memory>

#include "PinInterface.h"
#include "SensorToggles.h"
#include "configuration/Configuration.h"
#include "globals.h"
#include "logging/Logger.h"
#include "sensorinterface/RegisterInterface.h"
#include "sensorinterface/SensorInterface.h"
#include "sensorinterface/i2cimpl.h"
#include "status/TPSCounter.h"
#include "utils.h"

#define DATA_TYPE_NORMAL 1
#define DATA_TYPE_CORRECTION 2

enum class SensorStatus : uint8_t {
	SENSOR_OFFLINE = 0,
	SENSOR_OK = 1,
	SENSOR_ERROR = 2
};

// VQF's own view of its magnetic heading correction and rest detection,
// surfaced for the serial INFO dump. The question this answers is whether the
// corrections are engaging at all while the tracker sits still: VQF only accepts
// a magnetic field reference while the tracker is moving (magNewMinGyr), so a
// tracker that never moves can leave magRefNorm at zero and magDistDetected
// stuck true, which silently halves the heading correction gain -- and if rest
// is never detected, the gyroscope bias is never learned.
struct MagDiagnostics {
	bool magDistDetected = true;
	bool restDetected = false;
	// Zero until a magnetic field reference has been accepted.
	float magRefNorm = 0.0f;
	float magRefDipDeg = 0.0f;
	// Heading correction angle VQF is currently applying, in degrees.
	float deltaDeg = 0.0f;
	// Relative rest deviations; rest requires both to stay below 1.
	float restDevGyr = 0.0f;
	float restDevAcc = 0.0f;
	// Estimated gyroscope bias, in degrees per second.
	float gyroBias[3] = {0.0f, 0.0f, 0.0f};
	// Magnetometer hard-iron offset being subtracted, in raw counts, in the frame
	// the driver applies it in. perUnit false means the compiled-in default.
	float magOffset[3] = {0.0f, 0.0f, 0.0f};
	bool magOffsetPerUnit = false;
	// Gyroscope temperature and the bias actually being subtracted at it, in raw
	// counts. The calibration stores a bias at each of two temperatures and only
	// the first one used to be applied, so the pair below is what shows whether
	// the chip's own heating is being followed or ignored.
	float gyroTemperatureC = 0.0f;
	float appliedGyroBias[3] = {0.0f, 0.0f, 0.0f};
	// What the magnetometer poll costs and what it actually delivers.
	//
	// The read is a blocking run of aux I2C transactions inside the motion loop,
	// and the fusion's magnetometer timestep is compiled in from the rate the
	// driver *asks* the chip for rather than from the rate it produces. Neither is
	// visible from outside, so both are measured here: the sample rate is the
	// on-device check of the timestep, and the duration is the cost of the poll
	// against the sensor FIFO it is sharing the loop with.
	float magRateHz = 0.0f;
	float magPollUs = 0.0f;
	float magPollMaxUs = 0.0f;
	uint32_t magPollFailures = 0;
	// Magnitude of the last field handed to the fusion, in counts, and how many
	// samples were withheld for reading a field that cannot be Earth's.
	//
	// The IST8306 reports 0.3 uT/LSB, so Earth's 22-67 uT is 73-223 counts.
	// Measured on one of these trackers lying on a desk: 585 counts, 176 uT, with
	// the fusion dragging the heading at 0.4 deg/min towards the heading that
	// field implies, while the gyroscope and accelerometer held the attitude to
	// 0.01 deg over the same 100 s. VQF cannot catch that on its own -- it only
	// compares a reading against its own reference, never against a physically
	// possible magnitude -- and its disturbance rejection is not a substitute,
	// because past magMaxRejectionTime it resumes correcting at reduced gain, so
	// a permanently wrong reading drags the heading permanently.
	float magFieldNorm = 0.0f;
	uint32_t magFieldRejected = 0;
	// Inclination of the last corrected sample, in degrees below the horizontal,
	// and how many samples were withheld for not having the inclination a
	// geomagnetic field has at the tracker's latitude. Length and inclination are
	// the two independent things a field can be wrong in and they fail separately:
	// this one moves when the length is fine and the direction is not, which is
	// what a field distorted by what the tracker is sitting on, or rotated by a
	// wrong offset, looks like -- and it is the one that matters, because a field
	// steers the heading through its direction alone.
	float magDipDeg = 0.0f;
	uint32_t magDipRejected = 0;
	// What the tracker has worked out about its own offset without being told to
	// calibrate: the fit that is waiting to be confirmed by a second one, and
	// whether the offset in use was learned this boot rather than stored by an
	// earlier calibration. Both are here because "why is it still reading 85 uT"
	// is answered by "the capture has not covered enough directions yet", and
	// that is not visible from the field magnitude alone.
	bool magAutoCandidatePending = false;
	float magAutoCandidate[3] = {0.0f, 0.0f, 0.0f};
	bool magAutoLearned = false;
};

// A hard-iron offset fitted on the tracker itself, from a capture of its own
// magnetometer, plus what the fit is worth.
//
// The compiled-in constant is one board's offset applied to every unit, and the
// error is not small: on one of these trackers a rigid tumble moves the field
// *magnitude* the driver reads from 209 to 601 counts, where a correct offset
// holds it constant to within the chip's noise. That is a property of the
// individual board, so it can only be solved on the board.
//
// `valid` is the capture's coverage test, not the fit's: a sphere is only
// constrained in directions the samples actually reach, and a tracker sitting
// on a desk produces thousands of samples of a single direction. `bins` counts
// the direction cells the samples cover out of a 4x4x4 grid, and `span` is how
// far the raw field reaches along each axis in counts.
struct MagCalibration {
	bool valid = false;
	// The fit has to beat the compiled constant on this capture by a margin
	// before it is worth storing, so a capture that constrains little cannot be
	// mistaken for one that constrains a lot.
	bool improves = false;
	int samples = 0;
	int bins = 0;
	float span[3] = {0.0f, 0.0f, 0.0f};
	// Fitted hard-iron offset, in raw counts in the IMU frame -- the frame and
	// units of the driver's MagHardIron, and of the stored per-unit offset.
	float centre[3] = {0.0f, 0.0f, 0.0f};
	// Field magnitude the fit implies, in counts. Sanity check on the capture:
	// a radius far from the ~150-220 counts a 25-65 uT field gives at this
	// chip's sensitivity means the capture was distorted, not that the field is.
	float radius = 0.0f;
	// What the offset already in use -- the stored per-unit one, or the compiled
	// constant when there is none -- leaves over the very same capture, in counts
	// rms, next to what the fitted one leaves. This is the whole argument for
	// replacing one with the other, measured on the tracker. Comparing against the
	// offset in use rather than against the compiled constant is what makes the
	// automatic fit safe to run unattended: an offset that is already right leaves
	// nothing to improve, so nothing is replaced.
	float residualInUse = 0.0f;
	float residualFitted = 0.0f;
};

class Sensor {
public:
	Sensor(
		const char* sensorName,
		SensorTypeID type,
		uint8_t id,
		SlimeVR::Sensors::RegisterInterface& registerInterface,
		float rotation,
		SlimeVR::SensorInterface* sensorInterface = nullptr
	)
		: m_hwInterface(sensorInterface)
		, m_RegisterInterface(registerInterface)
		, sensorId(id)
		, sensorType(type)
		, sensorOffset({Quat(Vector3(IMU_ROTATION_AXIS), rotation)})
		, m_Logger(SlimeVR::Logging::Logger(sensorName)) {
		char buf[4];
		sprintf(buf, "%u", id);
		m_Logger.setTag(buf);
		addr = registerInterface.getAddress();
	}

	virtual ~Sensor(){};
	virtual void motionSetup(){};
	virtual void postSetup(){};
	virtual void motionLoop(){};
	virtual void sendData();
	virtual void setAcceleration(Vector3 a);
	virtual void setFusedRotation(Quat r);
	virtual void startCalibration(int calibrationType){};
	virtual SensorStatus getSensorState();
	virtual void printTemperatureCalibrationState();
	virtual void printDebugTemperatureCalibrationState();
	virtual void resetTemperatureCalibrationState();
	virtual void saveTemperatureCalibration();
	// TODO: currently only for softfusionsensor, bmi160 and others should get
	// an overload too
	virtual const char* getAttachedMagnetometer() const;
	// The most recent raw magnetometer sample, in whatever frame the mag chip is
	// mounted in. Used by the serial INFO dump to work out how the mag's axes
	// relate to the IMU's -- a wrong mapping produces a heading worse than no
	// magnetometer at all, and the mapping is a property of the board layout,
	// not of anything the firmware can know. False when there is no mag or
	// nothing has been read yet.
	virtual bool getRawMagSample(float out[3]) const {
		(void)out;
		return false;
	}
	// VQF's magnetic heading correction state, for diagnosing drift at rest.
	// False when the sensor has no fusion to report from.
	virtual bool getMagDiagnostics(MagDiagnostics& out) const {
		(void)out;
		return false;
	}
	// Fits a hard-iron offset from the orientation capture this sensor has been
	// collecting, and reports the fit and the coverage it is based on. False
	// when the sensor has no magnetometer; `valid` in the result says whether
	// the capture was good enough to use. Read-only: `SET MAGCAL APPLY` stores
	// what this returns.
	virtual bool getMagCalibration(MagCalibration& out) const {
		(void)out;
		return false;
	}
	// Throws away the current capture and starts a new one.
	virtual void resetMagCalibration() {}
	// Stores a per-unit hard-iron offset, in raw counts in the IMU frame -- the
	// frame the driver subtracts in, so a fitted centre can be handed straight
	// over. Goes in the sensor's runtime calibration file, which is the only
	// per-sensor storage this firmware has. False when there is nowhere to put
	// it; takes effect from the next boot either way, because the running filter
	// has already adopted a heading reference shaped by the old offset.
	virtual bool storeMagOffset(const float offset[3]) {
		(void)offset;
		return false;
	}
	// Drops the per-unit offset, putting the driver's compiled-in default back
	// in use from the next boot.
	virtual bool clearMagOffset() { return false; }
	// Direct register access to the magnetometer, for the serial console. The
	// setup sequence is only evidence about the chip if the writes actually land,
	// and nothing else in the firmware can tell whether they did: the driver
	// writes and never reads back, and a chip left in its power-on defaults
	// reports data that looks like data.
	virtual int magReadRegister(uint8_t reg) {
		(void)reg;
		return -1;
	}
	virtual bool magWriteRegister(uint8_t reg, uint8_t value) {
		(void)reg;
		(void)value;
		return false;
	}
	// TODO: realistically each sensor should print its own state instead of
	// having 15 getters for things only the serial commands use
	bool isWorking() { return working; };
	bool getHadData() const { return hadData; };
	bool isValid() { return m_hwInterface != nullptr; };
	uint8_t getSensorId() { return sensorId; };
	SensorTypeID getSensorType() { return sensorType; };
	const Vector3& getAcceleration() { return acceleration; };
	const Quat& getFusedRotation() { return fusedRotation; };
	bool hasNewDataToSend() { return newFusedRotation || newAcceleration; };
	inline bool hasCompletedRestCalibration() { return restCalibrationComplete; }
	void setFlag(SensorToggles toggle, bool state);
	[[nodiscard]] virtual bool isFlagSupported(SensorToggles toggle) const {
		return false;
	}
	// The stored state of a toggle, as opposed to setFlag's "apply this now".
	// Exposed so the serial INFO dump can show whether the server's magnetometer
	// toggle is actually being honoured, in both directions.
	[[nodiscard]] bool getFlagState(SensorToggles toggle) const {
		return toggles.getToggle(toggle);
	}
	SlimeVR::Configuration::SensorConfigBits getSensorConfigData();

	virtual SensorDataType getDataType() {
		return SensorDataType::SENSOR_DATATYPE_ROTATION;
	};

	SensorPosition getSensorPosition() { return m_SensorPosition; };

	void setSensorInfo(SensorPosition sensorPosition) {
		m_SensorPosition = sensorPosition;
	};

	TPSCounter m_tpsCounter;
	TPSCounter m_dataCounter;
	SlimeVR::SensorInterface* m_hwInterface = nullptr;
	virtual void deinit() {}
	virtual bool isAtRest() { return false; }

protected:
	SlimeVR::Sensors::RegisterInterface& m_RegisterInterface;
	uint8_t addr;
	uint8_t sensorId = 0;
	SensorTypeID sensorType = SensorTypeID::Unknown;
	bool working = false;
	bool hadData = false;
	uint8_t calibrationAccuracy = 0;
	/**
	 * Apply sensor offset to align it with tracker's axises
	 * (Y to top of the tracker, Z to front, X to left)
	 */
	Quat sensorOffset;

	bool newFusedRotation = false;
	Quat fusedRotation{};
	Quat lastFusedRotationSent{};

	bool newAcceleration = false;
	Vector3 acceleration{};

	SensorPosition m_SensorPosition = SensorPosition::POSITION_NO;

	SensorToggleState toggles;

	void markRestCalibrationComplete(bool completed = true);

	mutable SlimeVR::Logging::Logger m_Logger;

private:
	void printTemperatureCalibrationUnsupported();

	bool restCalibrationComplete = false;
};

const char* getIMUNameByType(SensorTypeID imuType);

#endif  // SLIMEVR_SENSOR_H_
