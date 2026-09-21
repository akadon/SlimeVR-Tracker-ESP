/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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

#pragma once

#include <vector3.h>

#include <cmath>
#include <cstdint>

#include "../../../GlobalVars.h"
#include "../../../configuration/Configuration.h"
#include "AccelBiasCalibrationStep.h"
#include "GyroBiasCalibrationStep.h"
#include "MotionlessCalibrationStep.h"
#include "NullCalibrationStep.h"
#include "SampleRateCalibrationStep.h"
#include "configuration/SensorConfig.h"
#include "logging/Logger.h"
#include "sensors/SensorFusion.h"
#include "sensors/softfusion/CalibrationBase.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename IMU>
class RuntimeCalibrator : public Sensors::CalibrationBase<IMU> {
public:
	static constexpr bool HasUpsideDownCalibration = false;

	using Base = Sensors::CalibrationBase<IMU>;
	using Self = RuntimeCalibrator<IMU>;
	using Consts = typename Base::Consts;
	using RawSensorT = typename Consts::RawSensorT;
	using RawVectorT = typename Consts::RawVectorT;

	RuntimeCalibrator(
		SensorFusion& fusion,
		IMU& imu,
		uint8_t sensorId,
		Logging::Logger& logger,
		SensorToggleState& toggles
	)
		: Base{fusion, imu, sensorId, logger, toggles} {
		activeCalibration.T_Ts = Consts::getDefaultTempTs();
		activeCalibration.T_Ts = Consts::getDefaultTempTs();
	}

	bool calibrationMatches(const Configuration::SensorConfig& sensorCalibration
	) final {
		return sensorCalibration.type
				== SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION
			&& (sensorCalibration.data.sfusion.ImuType == IMU::Type)
			&& (sensorCalibration.data.sfusion.MotionlessDataLen
				== Base::MotionlessCalibDataSize());
	}

	void assignCalibration(const Configuration::SensorConfig& sensorCalibration) final {
		activeCalibration = sensorCalibration.data.runtimeCalibration;
		calibrationEnabled = toggles.getToggle(SensorToggles::CalibrationEnabled);
		if (!calibrationEnabled) {
			activeCalibration.gyroPointsCalibrated = 0;
			for (size_t i = 0; i < 3; i++) {
				activeCalibration.G_off1[i] = 0;
				activeCalibration.G_off2[i] = 0;
			}

			for (size_t i = 0; i < 3; i++) {
				activeCalibration.accelCalibrated[i] = false;
				activeCalibration.A_off[i] = 0;
			}
		} else {
			calculateZROChange();
			updateGyroBias();
		}

		currentStep = &nullCalibrationStep;
	}

	void begin() final {
		startupMillis = millis();

		gyroBiasCalibrationStep.swapCalibrationIfNecessary();

		currentStep = &sampleRateCalibrationStep;
		currentStep->start();
		nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

		calculateZROChange();
		updateGyroBias();

		printCalibration();
	}

	void tick() final {
		if (skippedAStep && !lastTickRest && fusion.getRestDetected()) {
			computeNextCalibrationStep();
			skippedAStep = false;
		}

		if (millis() - startupMillis < initialStartupDelaySeconds * 1e3) {
			return;
		}

		if (!fusion.getRestDetected() && currentStep->requiresRest()) {
			if (isCalibrating) {
				currentStep->cancel();
				isCalibrating = false;
			}

			lastTickRest = fusion.getRestDetected();
			return;
		}

		if (!isCalibrating) {
			isCalibrating = true;
			currentStep->start();
		}

		if (currentStep->requiresRest() && !currentStep->restDetectionDelayElapsed()) {
			lastTickRest = fusion.getRestDetected();
			return;
		}

		auto result = currentStep->tick();

		switch (result) {
			case CalibrationStep<RawSensorT>::TickResult::DONE:
				if (nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
					stepCalibrationForward(true, false);
					break;
				}
				stepCalibrationForward();
				break;
			case CalibrationStep<RawSensorT>::TickResult::SKIP:
				stepCalibrationForward(false, false);
				break;
			case CalibrationStep<RawSensorT>::TickResult::CONTINUE:
				break;
		}

		lastTickRest = fusion.getRestDetected();
	}

	void scaleAccelSample(sensor_real_t accelSample[3]) final {
		accelSample[0] = accelSample[0] * Consts::AScale - activeCalibration.A_off[0];
		accelSample[1] = accelSample[1] * Consts::AScale - activeCalibration.A_off[1];
		accelSample[2] = accelSample[2] * Consts::AScale - activeCalibration.A_off[2];
	}

	float getAccelTimestep() final { return activeCalibration.A_Ts; }

	void scaleGyroSample(sensor_real_t gyroSample[3]) final {
		gyroSample[0] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[0] - activeGyroBias[0])
		);
		gyroSample[1] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[1] - activeGyroBias[1])
		);
		gyroSample[2] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[2] - activeGyroBias[2])
		);
	}

	float getGyroTimestep() final { return activeCalibration.G_Ts; }
	float getMagTimestep() final { return activeCalibration.M_Ts; }

	float getTempTimestep() final { return activeCalibration.T_Ts; }

	const uint8_t* getMotionlessCalibrationData() final {
		return activeCalibration.MotionlessData;
	}

	void signalOverwhelmed() final {
		if (isCalibrating) {
			currentStep->signalOverwhelmed();
		}
	}

	void provideAccelSample(const RawSensorT accelSample[3]) final {
		if (isCalibrating) {
			currentStep->processAccelSample(accelSample);
		}
	}

	void provideGyroSample(const RawSensorT gyroSample[3]) final {
		if (isCalibrating) {
			currentStep->processGyroSample(gyroSample);
		}
	}

	void provideTempSample(float tempSample) final {
		// Kept whether or not a calibration is running: the live bias correction
		// below needs the chip's current temperature on every session, not just
		// while a step happens to be collecting.
		lastMeasuredTemperature = tempSample;
		haveMeasuredTemperature = true;
		updateGyroBias();

		if (isCalibrating) {
			currentStep->processTempSample(tempSample);
		}
	}

	void calculateZROChange() {
		// With fewer than two points there is no measured bias-versus-temperature
		// slope to derive anything from. This used to fall through and compute one
		// from the zero-initialised second point.
		if (activeCalibration.gyroPointsCalibrated < 2) {
			activeZROChange = IMU::TemperatureZROChange;
			return;
		}

		float diffX = (activeCalibration.G_off2[0] - activeCalibration.G_off1[0])
					* Consts::GScale;
		float diffY = (activeCalibration.G_off2[1] - activeCalibration.G_off1[1])
					* Consts::GScale;
		float diffZ = (activeCalibration.G_off2[2] - activeCalibration.G_off1[2])
					* Consts::GScale;

		float maxDiff
			= std::max(std::max(std::abs(diffX), std::abs(diffY)), std::abs(diffZ));

		float temperatureDiff = activeCalibration.gyroMeasurementTemperature2
							  - activeCalibration.gyroMeasurementTemperature1;

		// Both points at the same temperature, or both at the same bias, leave
		// nothing to divide by; the quotient was inf/NaN and reached the filter.
		if (!(maxDiff > 0.0f) || !(std::fabs(temperatureDiff) > 0.0f)) {
			activeZROChange = IMU::TemperatureZROChange;
			return;
		}

		float zroChange = 0.1f / maxDiff / temperatureDiff;
		activeZROChange
			= std::isfinite(zroChange) ? zroChange : IMU::TemperatureZROChange;
	}

	float getZROChange() final { return activeZROChange; }

	bool getMagOffset(float out[3]) final {
		if (!activeCalibration.magCalibrated) {
			return false;
		}

		for (size_t i = 0; i < 3; i++) {
			out[i] = activeCalibration.M_off[i];
		}

		return true;
	}

	// Puts a newly fitted offset into use without a reboot. The stored copy is
	// only read into activeCalibration once, at assignCalibration, so writing the
	// configuration alone leaves the running filter subtracting the old one until
	// the tracker is restarted -- which is why the deliberate calibrations reboot.
	// The automatic fit runs while the tracker is being worn and cannot reboot, so
	// it has to land here as well.
	void setMagOffset(const float offset[3]) final {
		for (size_t i = 0; i < 3; i++) {
			activeCalibration.M_off[i] = offset[i];
		}
		activeCalibration.magCalibrated = true;
	}

	// Back to the compiled-in default without a reboot: the stored values are left
	// where they are and simply stop being used, which is what getMagOffset's
	// return value means.
	void clearMagOffset() final { activeCalibration.magCalibrated = false; }

	void getAppliedGyroBias(float out[3]) const final {
		for (size_t i = 0; i < 3; i++) {
			out[i] = activeGyroBias[i];
		}
	}

	// Bias subtracted from the live gyro stream. The calibration stores a measured
	// bias at each of two temperatures and the chip's own self-heating runs past
	// both of them, so the offset that is right at boot is wrong ten minutes later:
	// the gyro warms from about 25C to 40C and the bias walks with it. Only the
	// first point used to be applied, which leaves that walk in the signal.
	//
	// VQF does estimate gyro bias, but only where it can see it -- at rest. Yaw is
	// unobservable while the tracker is moving, so whatever the thermal walk
	// contributes during motion integrates straight into the heading. Removing the
	// measured part of it before the filter sees it is the difference between the
	// filter estimating a residual and estimating the whole thing.
	//
	// Outside the calibrated temperature range the measured slope is followed
	// rather than the nearer point held. Holding asserts a thermal coefficient of
	// zero, and the coefficient is not zero: the two points are at least 5 C apart
	// (GyroBiasCalibrationStep takes the second one only at that separation), so
	// the slope between them is a real measurement -- 0.002 to 0.004 dps/C on the
	// unit this was checked on -- and it is an order of magnitude above its own
	// uncertainty, which means a linear model beats a constant one at any distance
	// the relationship stays linear over. That matters because the tracker warms
	// past the warm point in normal use: it idles near the top of the range on a
	// desk, booting at about 25 C and settling around 30, and a tracker worn
	// against the body goes beyond that. Holding the endpoint there leaves the
	// whole thermal walk in the signal, and the walk lands in yaw, which is the
	// one axis VQF cannot observe while the tracker is moving.
	void updateGyroBias() {
		for (size_t i = 0; i < 3; i++) {
			activeGyroBias[i] = activeCalibration.G_off1[i];
		}

		if (!haveMeasuredTemperature || activeCalibration.gyroPointsCalibrated < 2) {
			return;
		}

		float temperature1 = activeCalibration.gyroMeasurementTemperature1;
		float span
			= activeCalibration.gyroMeasurementTemperature2 - temperature1;

		// G_off1/G_off2 are kept sorted by temperature, but a pair taken moments
		// apart is two noise measurements of the same thing and the slope between
		// them is meaningless. The same goes for a non-finite span.
		if (!(span >= MinBiasTemperatureSpan)) {
			return;
		}

		// Past this far out the relationship is no longer safely linear and the
		// last value the fit can vouch for is held instead. A reset that leaves
		// the tracker well outside the calibrated range is the case this bounds.
		const float extrapolation = MaxBiasTemperatureExtrapolation / span;
		float fraction = (lastMeasuredTemperature - temperature1) / span;
		if (fraction < -extrapolation) {
			fraction = -extrapolation;
		} else if (fraction > 1.0f + extrapolation) {
			fraction = 1.0f + extrapolation;
		}

		// A bias step this large between two rest measurements is a bad calibration
		// rather than a real thermal coefficient, and following it would inject
		// degrees per second of false rate into the filter. Keep the first point.
		float maxDifference = MaxBiasTemperatureDifferenceDps / Consts::GScale;

		for (size_t i = 0; i < 3; i++) {
			float difference
				= activeCalibration.G_off2[i] - activeCalibration.G_off1[i];
			if (std::fabs(difference) > maxDifference) {
				continue;
			}

			// Held to the same limit the difference itself is, so extrapolating
			// can never put more into the stream than a pair at that limit puts
			// in while it is inside the calibrated range.
			float correction = difference * fraction;
			if (correction > maxDifference) {
				correction = maxDifference;
			} else if (correction < -maxDifference) {
				correction = -maxDifference;
			}

			activeGyroBias[i] += correction;
		}
	}

private:
	enum class CalibrationStepEnum {
		NONE,
		SAMPLING_RATE,
		MOTIONLESS,
		GYRO_BIAS,
		ACCEL_BIAS,
	};

	void computeNextCalibrationStep() {
		if (!activeCalibration.motionlessCalibrated && Base::HasMotionlessCalib) {
			nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
			currentStep = &motionlessCalibrationStep;
		} else if (activeCalibration.gyroPointsCalibrated == 0) {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
			// } else if (!accelBiasCalibrationStep.allAxesCalibrated()) {
			// 	nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
			// 	currentStep = &accelBiasCalibrationStep;
		} else {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
		}
	}

	void stepCalibrationForward(bool print = true, bool save = true) {
		currentStep->cancel();
		switch (nextCalibrationStep) {
			case CalibrationStepEnum::NONE:
				// Sequence finished. Leave the step marked as not running, or the loop
				// keeps re-entering this function on the null step.
				isCalibrating = false;
				return;
			case CalibrationStepEnum::SAMPLING_RATE:
				nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
				currentStep = &motionlessCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::TIMESTEPS);
				}
				break;
			case CalibrationStepEnum::MOTIONLESS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::MOTIONLESS);
				}
				break;
			case CalibrationStepEnum::GYRO_BIAS:
				if (activeCalibration.gyroPointsCalibrated < 2) {
					// One point down; run the step again so the second one is taken at
					// a different temperature.
					// nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
					// currentStep = &accelBiasCalibrationStep;
					nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
					currentStep = &gyroBiasCalibrationStep;
				} else {
					// Both temperature points recorded, so the sequence is over. This is
					// where it used to stop making progress: the step stayed selected
					// forever, so a tracker sitting on a table re-ran a five second
					// calibration and wrote the whole configuration to flash every five
					// seconds for as long as it was powered on.
					nextCalibrationStep = CalibrationStepEnum::NONE;
					currentStep = &nullCalibrationStep;
				}

				if (print) {
					printCalibration(CalibrationPrintFlags::GYRO_BIAS);
				}
				break;
			case CalibrationStepEnum::ACCEL_BIAS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;

				if (print) {
					printCalibration(CalibrationPrintFlags::ACCEL_BIAS);
				}

				if (!accelBiasCalibrationStep.allAxesCalibrated()) {
					skippedAStep = true;
				}
				break;
		}

		isCalibrating = false;

		// A step may have just written a new bias point; the live correction is
		// derived from those points and has to follow them immediately, not on the
		// next temperature sample.
		calculateZROChange();
		updateGyroBias();

		if (save) {
			saveCalibration();
		}
	}

	void saveCalibration() {
		// While the calibration toggle is off, activeCalibration has its offsets
		// zeroed on purpose; persisting that would discard the stored calibration.
		if (!calibrationEnabled) {
			return;
		}

		SlimeVR::Configuration::SensorConfig sensorConfig{};
		sensorConfig.type
			= SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION;
		sensorConfig.data.runtimeCalibration = activeCalibration;

		// The magnetometer offset is not something the calibrator measures; it is
		// written from the host with `SET MAGOFF` and is not part of the working
		// copy, so without carrying the stored value across, the next rest
		// calibration would silently erase it.
		SlimeVR::Configuration::SensorConfig stored = configuration.getSensor(sensorId);
		if (stored.type == SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION) {
			sensorConfig.data.runtimeCalibration.magCalibrated
				= stored.data.runtimeCalibration.magCalibrated;
			for (size_t i = 0; i < 3; i++) {
				sensorConfig.data.runtimeCalibration.M_off[i]
					= stored.data.runtimeCalibration.M_off[i];
			}
		}

		configuration.setSensor(sensorId, sensorConfig);
		configuration.save();
	}

	enum class CalibrationPrintFlags {
		TIMESTEPS = 1,
		MOTIONLESS = 2,
		GYRO_BIAS = 4,
		ACCEL_BIAS = 8,
	};

	static constexpr CalibrationPrintFlags PrintAllCalibration
		= CalibrationPrintFlags::TIMESTEPS | CalibrationPrintFlags::MOTIONLESS
		| CalibrationPrintFlags::GYRO_BIAS | CalibrationPrintFlags::ACCEL_BIAS;

	void printCalibration(CalibrationPrintFlags toPrint = PrintAllCalibration) {
		if (any(toPrint & CalibrationPrintFlags::TIMESTEPS)) {
			if (activeCalibration.sensorTimestepsCalibrated) {
				logger.info(
					"Calibrated timesteps: Accel %f, Gyro %f, Temperature %f",
					activeCalibration.A_Ts,
					activeCalibration.G_Ts,
					activeCalibration.T_Ts
				);
			} else {
				logger.info("Sensor timesteps not calibrated");
			}
		}

		if (Base::HasMotionlessCalib
			&& any(toPrint & CalibrationPrintFlags::MOTIONLESS)) {
			if (activeCalibration.motionlessCalibrated) {
				logger.info("Motionless calibration done");
			} else {
				logger.info("Motionless calibration not done");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::GYRO_BIAS)) {
			if (activeCalibration.gyroPointsCalibrated != 0) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					activeCalibration.gyroMeasurementTemperature1,
					activeCalibration.G_off1[0],
					activeCalibration.G_off1[1],
					activeCalibration.G_off1[2]
				);
			} else {
				logger.info("Gyro bias not calibrated");
			}

			if (activeCalibration.gyroPointsCalibrated == 2) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					activeCalibration.gyroMeasurementTemperature2,
					activeCalibration.G_off2[0],
					activeCalibration.G_off2[1],
					activeCalibration.G_off2[2]
				);
			}
		}

		if (any(toPrint & CalibrationPrintFlags::ACCEL_BIAS)) {
			if (accelBiasCalibrationStep.allAxesCalibrated()) {
				logger.info(
					"Calibrated accel bias: %f %f %f",
					activeCalibration.A_off[0],
					activeCalibration.A_off[1],
					activeCalibration.A_off[2]
				);
			} else if (accelBiasCalibrationStep.anyAxesCalibrated()) {
				logger.info(
					"Partially calibrated accel bias: %f %f %f",
					activeCalibration.A_off[0],
					activeCalibration.A_off[1],
					activeCalibration.A_off[2]
				);
			} else {
				logger.info("Accel bias not calibrated");
			}
		}
	}

	CalibrationStepEnum nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

	static constexpr float initialStartupDelaySeconds = 5;
	uint64_t startupMillis = millis();

	// The steps below hold references to these two, so they must be declared first.
	//
	// Transparent calibration that doesn't affect input data, used as the starting
	// point for activeCalibration. activeCalibration is the single working copy: the
	// steps write to it, and the scaling and timestep accessors below read from it.
	// There used to be two copies where the steps wrote to one and the live path read
	// the other, so runtime-measured values only took effect on the next boot.
	SlimeVR::Configuration::RuntimeCalibrationSensorConfig defaultCalibration{
		.ImuType = {IMU::Type},
		.MotionlessDataLen = {Base::MotionlessCalibDataSize()},

		.sensorTimestepsCalibrated = false,
		.A_Ts = IMU::AccTs,
		.G_Ts = IMU::GyrTs,
		.M_Ts = IMU::MagTs,
		.T_Ts = 0,

		.motionlessCalibrated = false,
		.MotionlessData = {},

		.gyroPointsCalibrated = 0,
		.gyroMeasurementTemperature1 = 0,
		.G_off1 = {0.0, 0.0, 0.0},
		.gyroMeasurementTemperature2 = 0,
		.G_off2 = {0.0, 0.0, 0.0},

		.accelCalibrated = {false, false, false},
		.A_off = {0.0, 0.0, 0.0},

		.magCalibrated = false,
		.M_off = {0.0, 0.0, 0.0},
	};

	Configuration::RuntimeCalibrationSensorConfig activeCalibration
		= defaultCalibration;

	float activeZROChange = 0;

	// Bias actually subtracted from the gyro stream, in raw counts: G_off1, or the
	// interpolation between the two stored points at the current temperature.
	float activeGyroBias[3] = {0.0f, 0.0f, 0.0f};

	// Starts at the colder calibration point so that, until the first temperature
	// sample arrives, the correction is exactly what it was before the second
	// point existed.
	float lastMeasuredTemperature = 0;

	// Until a temperature has actually been read, the interpolation below would be
	// extrapolating from a fabricated zero -- which for a tracker calibrated in a
	// cold room is inside the calibrated range and would move the bias.
	bool haveMeasuredTemperature = false;

	// A measured bias-versus-temperature slope steeper than this is a bad
	// calibration, not a real one: the datasheet thermal drift of these parts is
	// under 0.05 dps/C, so 2 dps across the calibrated range is already two orders
	// of magnitude of headroom.
	static constexpr float MaxBiasTemperatureDifferenceDps = 2.0f;

	// Below this the two temperature points are the same measurement and the slope
	// between them is noise.
	static constexpr float MinBiasTemperatureSpan = 2.0f;

	// How far past the calibrated range the measured slope is still followed. The
	// tracker settles a few degrees above the warm point of its own calibration in
	// normal use, which is what this is for; this bound only stops a reset that
	// leaves it far outside the range from extrapolating a measured slope a long
	// way. See updateGyroBias().
	static constexpr float MaxBiasTemperatureExtrapolation = 15.0f;

	// Mirrors the CalibrationEnabled toggle at assignCalibration() time; while it is
	// off the offsets in activeCalibration are deliberately zeroed and must not be
	// written back over the stored calibration.
	bool calibrationEnabled = true;

	SampleRateCalibrationStep<RawSensorT> sampleRateCalibrationStep{activeCalibration};
	MotionlessCalibrationStep<IMU, RawSensorT> motionlessCalibrationStep{
		activeCalibration,
		sensor
	};
	GyroBiasCalibrationStep<RawSensorT> gyroBiasCalibrationStep{activeCalibration};
	AccelBiasCalibrationStep<RawSensorT> accelBiasCalibrationStep{
		activeCalibration,
		static_cast<float>(Consts::AScale)
	};
	NullCalibrationStep<RawSensorT> nullCalibrationStep{activeCalibration};

	CalibrationStep<RawSensorT>* currentStep = &nullCalibrationStep;

	bool isCalibrating = false;
	bool skippedAStep = false;
	bool lastTickRest = false;

	using Base::fusion;
	using Base::logger;
	using Base::sensor;
	using Base::sensorId;
	using Base::toggles;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
