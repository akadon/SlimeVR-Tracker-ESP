/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Tailsy13 & SlimeVR Contributors

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

#include <PinInterface.h>

#include <cstdint>
#include <cstring>

#include "../../GlobalVars.h"
#include "../../sensorinterface/SensorInterface.h"
#include "../RestCalibrationDetector.h"
#include "../sensor.h"
#include "TempGradientCalculator.h"
#include "imuconsts.h"
#include "motionprocessing/types.h"
#include "sensors/SensorFusion.h"
#include "sensors/softfusion/magdriver.h"

namespace SlimeVR::Sensors {

template <typename SensorType, template <typename IMU> typename Calibrator>
class SoftFusionSensor : public Sensor {
	using Consts = IMUConsts<SensorType>;
	using RawSensorT = typename Consts::RawSensorT;

	using Calib = Calibrator<SensorType>;
	static constexpr auto UpsideDownCalibrationInit = Calib::HasUpsideDownCalibration;

	float lastReadTemperature = 0;
	uint32_t lastTempPollTime = micros();

	bool detected() const {
		const auto value
			= m_sensor.m_RegisterInterface.readReg(SensorType::Regs::WhoAmI::reg);
		if constexpr (requires { SensorType::Regs::WhoAmI::values.size(); }) {
			for (auto possible : SensorType::Regs::WhoAmI::values) {
				if (value == possible) {
					return true;
				}
			}
			// this assumes there are only 2 values in the array
			m_Logger.error(
				"Sensor not detected, expected reg 0x%02x = [0x%02x, 0x%02x] but got "
				"0x%02x",
				SensorType::Regs::WhoAmI::reg,
				SensorType::Regs::WhoAmI::values[0],
				SensorType::Regs::WhoAmI::values[1],
				value
			);
			return false;
		} else {
			if (value == SensorType::Regs::WhoAmI::value) {
				return true;
			}
			m_Logger.error(
				"Sensor not detected, expected reg 0x%02x = 0x%02x but got 0x%02x",
				SensorType::Regs::WhoAmI::reg,
				SensorType::Regs::WhoAmI::value,
				value
			);
			return false;
		}
	}

	void sendData() final {
		Sensor::sendData();
		sendTempIfNeeded();
	}

	void sendTempIfNeeded() {
		uint32_t now = micros();
		constexpr float maxSendRateHz = 2.0f;
		constexpr uint32_t sendInterval = 1.0f / maxSendRateHz * 1e6;
		uint32_t elapsed = now - m_lastTemperaturePacketSent;
		if (elapsed >= sendInterval) {
			m_lastTemperaturePacketSent = now;
			networkConnection.sendTemperature(sensorId, lastReadTemperature);
		}
	}

	// Bounds for the temperature-driven bias forgetting time. Unbounded, a vanishing
	// temperature gradient sends it to infinity, which stops the gyro bias estimate
	// adapting at all (and reaches the filter coefficients as NaN); a very small one
	// would make the estimate chase noise. Within the band the estimate still
	// adapts, just more slowly the steadier the temperature is.
	//
	// The ceiling is what governs a tracker lying still: there the gradient is small
	// enough that the forgetting time sits on it, and it sets the resting estimator's
	// time constant (VQF's rest update settles at tau = 0.405 * forgettingTime, so
	// 1000 s was 405 s). Measured on a tracker at rest for 40 minutes: the bias
	// estimate still walked 0.005 dps while the 6D heading drifted 0.5 deg/h, which
	// is that time constant lagging a bias that is still moving. The estimate can
	// afford to track it harder: its own noise comes out far below the
	// biasSigmaRest VQF assumes for the rest measurement, and what it contributes to
	// the heading is a small fraction of a degree per hour.
	static constexpr float MinBiasForgettingTime = 10.0f;
	static constexpr float MaxBiasForgettingTime = 300.0f;

	TemperatureGradientCalculator tempGradientCalculator{[&](float gradient) {
		float magnitude = std::fabs(gradient);
		float forgettingTime = (magnitude < 1e-6f)
								 ? MaxBiasForgettingTime
								 : calibrator.getZROChange() / magnitude;

		if (!std::isfinite(forgettingTime)
			|| forgettingTime > MaxBiasForgettingTime) {
			forgettingTime = MaxBiasForgettingTime;
		} else if (forgettingTime < MinBiasForgettingTime) {
			forgettingTime = MinBiasForgettingTime;
		}

		m_fusion.updateBiasForgettingTime(forgettingTime);
	}};

	void processAccelSample(const RawSensorT xyz[3], const sensor_real_t timeDelta) {
		sensor_real_t accelData[]
			= {static_cast<sensor_real_t>(xyz[0]),
			   static_cast<sensor_real_t>(xyz[1]),
			   static_cast<sensor_real_t>(xyz[2])};

		calibrator.scaleAccelSample(accelData);

		m_fusion.updateAcc(accelData, calibrator.getAccelTimestep());

		calibrator.provideAccelSample(xyz);
	}

	void processGyroSample(const RawSensorT xyz[3], const sensor_real_t timeDelta) {
		sensor_real_t gyroData[]
			= {static_cast<sensor_real_t>(xyz[0]),
			   static_cast<sensor_real_t>(xyz[1]),
			   static_cast<sensor_real_t>(xyz[2])};
		calibrator.scaleGyroSample(gyroData);
		m_fusion.updateGyro(gyroData, calibrator.getGyroTimestep());

		calibrator.provideGyroSample(xyz);
	}

	void
	processTempSample(const int16_t rawTemperature, const sensor_real_t timeDelta) {
		if constexpr (!Consts::DirectTempReadOnly) {
			const float scaledTemperature
				= SensorType::TemperatureBias
				+ static_cast<float>(rawTemperature)
					  * (1.0 / SensorType::TemperatureSensitivity);

			lastReadTemperature = scaledTemperature;
			if (toggles.getToggle(SensorToggles::TempGradientCalibrationEnabled)) {
				tempGradientCalculator.feedSample(
					lastReadTemperature,
					calibrator.getTempTimestep()
				);
			}

			calibrator.provideTempSample(lastReadTemperature);
		}
	}

public:
	static constexpr auto TypeID = SensorType::Type;
	static constexpr uint8_t Address = SensorType::Address;

	SoftFusionSensor(
		uint8_t id,
		RegisterInterface& registerInterface,
		float rotation,
		SlimeVR::SensorInterface* sensorInterface = nullptr,
		PinInterface* intPin = nullptr,
		uint8_t = 0
	)
		: Sensor(
			SensorType::Name,
			SensorType::Type,
			id,
			registerInterface,
			rotation,
			sensorInterface
		)
		, m_fusion(
			  SensorType::SensorVQFParams,
			  SensorType::GyrTs,
			  SensorType::AccTs,
			  SensorType::MagTs
		  )
		, m_sensor(registerInterface, m_Logger) {}
	~SoftFusionSensor() override = default;

	void checkSensorTimeout() {
		constexpr uint32_t sensorTimeoutMillis = 2e3;  // 2 seconds

		uint32_t now = millis();
		if (m_lastRotationUpdateMillis + sensorTimeoutMillis > now) {
			return;
		}

		working = false;
		m_status = SensorStatus::SENSOR_ERROR;
		m_Logger.error(
			"Sensor timeout I2C Address 0x%02x delaytime: %d ms",
			addr,
			now - m_lastRotationUpdateMillis
		);
		networkConnection.sendSensorError(
			this->sensorId,
			static_cast<uint8_t>(PacketErrorCode::WATCHDOG_TIMEOUT)
		);
	}

	void motionLoop() final {
		calibrator.tick();

		// read fifo updating fusion
		uint32_t now = micros();

		if constexpr (Consts::DirectTempReadOnly) {
			uint32_t tempElapsed = now - lastTempPollTime;
			if (tempElapsed >= Consts::DirectTempReadTs * 1e6) {
				lastTempPollTime
					= now
					- (tempElapsed
					   - static_cast<uint32_t>(Consts::DirectTempReadTs * 1e6));
				lastReadTemperature = m_sensor.getDirectTemp();

				calibrator.provideTempSample(lastReadTemperature);

				if (toggles.getToggle(SensorToggles::TempGradientCalibrationEnabled)) {
					tempGradientCalculator.feedSample(
						lastReadTemperature,
						Consts::DirectTempReadTs
					);
				}
			}
		}

		if (toggles.getToggle(SensorToggles::TempGradientCalibrationEnabled)) {
			tempGradientCalculator.tick();
		}

		// send new fusion values when time is up
		// (A 6 ms FIFO poll deadline used to be computed here into m_lastPollTime,
		// which nothing ever read, so it had no effect on how often bulkRead ran.)
		now = micros();
		constexpr float maxSendRateHz = 100.0f;
		constexpr uint32_t sendInterval = 1.0f / maxSendRateHz * 1e6f;
		const uint32_t elapsed = now - m_lastRotationPacketSent;
		if (elapsed >= sendInterval) {
			auto overwhelmed = m_sensor.bulkRead({
				[&](const auto sample[3], float AccTs) {
					processAccelSample(sample, AccTs);
				},
				[&](const auto sample[3], float GyrTs) {
					processGyroSample(sample, GyrTs);
				},
				[&](int16_t sample, float TempTs) {
					processTempSample(sample, TempTs);
				},
			});
			if (overwhelmed) {
				calibrator.signalOverwhelmed();
			}
			if (!m_fusion.isUpdated()) {
				checkSensorTimeout();
				return;
			}
			hadData = true;
			m_lastRotationUpdateMillis = millis();
			m_fusion.clearUpdated();

			m_lastRotationPacketSent = now - (elapsed - sendInterval);

			setFusedRotation(m_fusion.getQuaternionQuat());
			setAcceleration(m_fusion.getLinearAccVec());
			optimistic_yield(100);
		}

		if (calibrationDetector.update(m_fusion)) {
			markRestCalibrationComplete();
		}

		pollMag();
	}

	void motionSetup() final {
		if (!detected()) {
			m_status = SensorStatus::SENSOR_ERROR;
			return;
		}

		SlimeVR::Configuration::SensorConfig sensorCalibration
			= configuration.getSensor(sensorId);

		toggles = configuration.getSensorToggles(sensorId);

		// If no compatible calibration data is found, the calibration data will just be
		// zero-ed out
		if (calibrator.calibrationMatches(sensorCalibration)) {
			calibrator.assignCalibration(sensorCalibration);
		} else if (sensorCalibration.type == SlimeVR::Configuration::SensorConfigType::NONE) {
			m_Logger.warn(
				"No calibration data found for sensor %d, ignoring...",
				sensorId
			);
			m_Logger.info("Calibration is advised");
		} else {
			m_Logger.warn(
				"Incompatible calibration data found for sensor %d, ignoring...",
				sensorId
			);
			m_Logger.info("Please recalibrate");
		}

		calibrator.begin();

		bool initResult = false;

		if constexpr (Calib::HasMotionlessCalib) {
			typename SensorType::MotionlessCalibrationData calibData;
			std::memcpy(
				&calibData,
				calibrator.getMotionlessCalibrationData(),
				sizeof(calibData)
			);
			initResult = m_sensor.initialize(calibData);
		} else {
			initResult = m_sensor.initialize();
		}

		if (!initResult) {
			m_Logger.error("Sensor failed to initialize!");
			m_status = SensorStatus::SENSOR_ERROR;
			return;
		}

		m_status = SensorStatus::SENSOR_OK;
		working = true;

		calibrator.checkStartupCalibration();

		if constexpr (Consts::SupportsMags) {
			magDriver.init(
				SoftFusion::MagInterface{
					.readByte
					= [&](uint8_t address) { return m_sensor.readAux(address); },
					.writeByte
					= [&](uint8_t address, uint8_t value) {
						  m_sensor.writeAux(address, value);
					  },
					.setDeviceId
					= [&](uint8_t deviceId) { m_sensor.setAuxId(deviceId); },
					.startPolling
					= [&](uint8_t dataReg, SoftFusion::MagDataWidth dataWidth
					  ) { m_sensor.startAuxPolling(dataReg, dataWidth); },
					.stopPolling = [&]() { m_sensor.stopAuxPolling(); },
				},
				Consts::Supports9ByteMag
			);

			if (toggles.getToggle(SensorToggles::MagEnabled)) {
				magDriver.startPolling();
			}

			// No onToggleChange callback here on purpose: nothing calls
			// SensorToggleState::emitToggleChange, so such a callback would never
			// run. The toggle takes effect because pollMag() re-reads it every
			// pass, which switches the mag off (and back on) immediately.
		}
	}

	void startCalibration(int calibrationType) final {
		calibrator.startCalibration(calibrationType);
	}

	[[nodiscard]] bool isFlagSupported(SensorToggles toggle) const final {
		if (toggle == SensorToggles::CalibrationEnabled
			|| toggle == SensorToggles::TempGradientCalibrationEnabled) {
			return true;
		}
		// Advertised to the server as "this tracker has a magnetometer", which is
		// what decides whether the magnetometer toggle is offered for it at all.
		// Without this, magSupported went out as false: the toggle was greyed out
		// server-side, and setFlag() rejected any attempt to change it, so the mag
		// could never be turned on -- nor, just as importantly, turned back off.
		if constexpr (Consts::SupportsMags) {
			return toggle == SensorToggles::MagEnabled && magDriver.hasMag();
		}
		return false;
	}

	SensorStatus getSensorState() final { return m_status; }

	SensorFusion m_fusion;
	SensorType m_sensor;
	Calib calibrator{m_fusion, m_sensor, sensorId, m_Logger, toggles};

	SensorStatus m_status = SensorStatus::SENSOR_OFFLINE;
	uint32_t m_lastRotationUpdateMillis = 0;
	uint32_t m_lastRotationPacketSent = 0;
	uint32_t m_lastTemperaturePacketSent = 0;

	RestCalibrationDetector calibrationDetector;

	void deinit() final {
		m_sensor.deinit();
		// magDriver.deinit();
	}
	bool isAtRest() final { return m_fusion.getRestDetected(); }

	SoftFusion::MagDriver magDriver;

	// --- Magnetometer ---------------------------------------------------------
	//
	// The IMU's hardware aux-FIFO streaming is unimplemented on this driver
	// family (see startAuxPolling in icm45base.h), so the mag is read directly
	// through the IMU's I2C master instead, in one burst per poll so a sample
	// cannot be spliced together from two different measurements.
	//
	// Gated on the "use magnetometer on trackers" toggle, which the server sets
	// per tracker, so it can be switched off without reflashing.

	static constexpr uint8_t MagSampleBytes = 6;
	// The IST8306 runs at 20 Hz, so polling slightly faster than that is enough
	// to never miss a sample; duplicates are dropped below. The two rates will
	// not stay in step -- the chip times itself off its own oscillator and this
	// off the ESP32's crystal -- so the margin is what keeps a sample from being
	// overwritten between two polls.
	static constexpr uint32_t MagPollIntervalMicros = 40000;

	// Index of the raw mag axis feeding each IMU axis, and its sign.
	//
	// Measured, not guessed: a 100 s capture of paired (attitude, raw mag)
	// samples was fitted against  m_imu = b + R^T c  over all 24 signed
	// permutations. (2, 0, 1) won every ranking by a wide margin, including a
	// yaw-free test that uses only the component along gravity -- which matters,
	// because VQF's heading is unobservable in 6D and drifts between runs, and
	// an earlier attempt to fit a single world field was defeated by exactly
	// that drift.
	//
	// Yaw is the one thing that capture could not pin down, so the remap below
	// has to be read against the rotation the tracker actually reports rather
	// than against the fusion's own frame. Checked that way it lands in the
	// *reported* frame: raw minus offset, turned by the reported attitude, comes
	// out at the local inclination -- refnorm 169.3 counts, dip 68.5 deg against
	// ~67 for central Europe, on a unit whose per-unit offset is (-22.0, 3.7,
	// 69.6) -- where a right angle either side of it does not come close. That
	// reading is a property of the spot, not of the code: this room is
	// magnetically dirty and the same tracker in the same build reads 46 degrees
	// a metre away, which is the environment, not a frame error.
	//
	// The fusion thinks in a different body frame, and the mapping between them
	// runs the other way round from the one the fit's wording suggests.
	// setFusedRotation stores `r * sensorOffset`, so the reported attitude is the
	// fusion's own composed with sensorOffset on the right, and coordinates go
	// back the other way: v_fusion = sensorOffset * v_reported. A field solved in
	// the reported frame therefore has to be turned *by* sensorOffset before VQF
	// sees it -- not by its inverse. The leftover is a yaw, which is why feeding
	// this remap straight in left the filter correcting towards a field a quarter
	// turn from the real one. The call site below applies the turn.
	//
	// sensorOffset is Quat(Z, IMU_ROTATION), and IMU_ROTATION is DEG_270 -- but
	// DEG_X(270) is +90 deg, not 270 (the macro subtracts from 360), so this is
	// a quarter turn about z, not three quarters. The magnetometer axes are
	// remapped into the tracker's frame by hand above; this is the same board
	// rotation, so it is applied by hand too rather than folded into the map.
	//
	// That the turn is the right one is checked against VQF's own state rather
	// than against a fit. updateMag rotates the sample with the heading-free 6D
	// attitude and drives atan2(magEarth[0], magEarth[1]) - delta to zero, so at
	// convergence delta *is* the azimuth of the sample the filter was handed.
	// Replayed over a live capture: turning the sample by sensorOffset reproduces
	// the printed delta to within 0.8 deg, where the unturned sample is 92 deg
	// out, the opposite turn 175, and a 180 deg turn 87. Only the turned sample
	// leaves the correction pulling towards the field the tracker reports.
	static constexpr int8_t MagAxisMap[3] = {2, 0, 1};
	static constexpr int8_t MagAxisSign[3] = {1, -1, -1};

	// Hard-iron offset in raw counts, in the IMU frame -- i.e. subtracted after
	// the remap above, which is the frame the fit solved it in. It is large
	// (|b| ~ 122 counts against a total field of ~202) and is not a fitting
	// artefact: letting each axis take a free scale as well moves the residual
	// only from 7.5 to 6.3 counts, and the scale it picks is uniform, which is
	// the model's exact scale degeneracy rather than a real soft-iron term.
	static constexpr float MagHardIron[3] = {-89.5f, -81.7f, -16.4f};

	// Whether the mag is allowed to correct VQF's heading.
	//
	// VQF needs the mag in the IMU's own frame, and the mapping above plus the
	// hard-iron offset are properties of how the mag chip sits on the board --
	// not something the firmware can derive. While they are wrong, every sample
	// fed in drags the heading toward a false direction with tauMag's 9 s time
	// constant, which is worse than running 6D. It also corrupts the tracker's
	// own attitude, which is what the mapping has to be measured against, so a
	// capture taken with the feed on cannot be solved -- which is why this was
	// off while the constants above were being measured.
	//
	// With them measured, the feed is on: it is what removes the heading drift
	// that 6D has no way to observe. It stays gated on the server's per-tracker
	// magnetometer toggle, so it can still be switched off without reflashing.
	static constexpr bool MagFeedFusion = true;

	uint32_t m_nextMagPollMicros = 0;
	float m_lastRawMag[3] = {0.0f, 0.0f, 0.0f};
	float m_lastSentMag[3] = {0.0f, 0.0f, 0.0f};
	bool m_haveMagSample = false;
	bool m_haveSentMag = false;
	// Poll cost and observed sample rate. The rate is the only check on the
	// mag timestep handed to the fusion, which is 1/20 to match the continuous
	// rate the driver's setup sequence asks the chip for. The window is
	// MagRateWindowMicros of time actually spent between polls, not wall-clock
	// time, so a window during which the mag was toggled off cannot dilute the
	// rate.
	static constexpr uint32_t MagRateWindowMicros = 2000000;
	// The first window closes on MagRateWarmupMicros instead, so that the line
	// has a real rate to print within about half a second of the mag coming on
	// rather than the 0.0 it would otherwise read until a full window has run.
	static constexpr uint32_t MagRateWarmupMicros = MagPollIntervalMicros * 12;
	// A gap longer than this is the polling having been off, not a slow poll,
	// and is not counted into the window at all.
	static constexpr uint32_t MagPollGapLimitMicros = MagPollIntervalMicros * 4;
	uint32_t m_magPollMicrosSum = 0;
	uint32_t m_magPollCount = 0;
	uint32_t m_magPollMicrosMax = 0;
	uint32_t m_magPollFailures = 0;
	uint32_t m_magFreshCount = 0;
	uint32_t m_magRateWindowMicros = 0;
	uint32_t m_lastMagPollMicros = 0;
	float m_magRateHz = 0.0f;
	// What the field handed to the fusion has to look like to be believed, in
	// counts of the IST8306's 0.3 uT/LSB: Earth's field is 22-67 uT anywhere on
	// the planet, so 73-223 counts, and this band is that with a small margin.
	// Wide enough that no real geomagnetic field is ever withheld, narrow enough
	// that a tracker sitting against something ferrous -- 585 counts measured, a
	// factor of 3.7 -- is.
	static constexpr float MagFieldMinCounts = 60.0f;
	static constexpr float MagFieldMaxCounts = 240.0f;
	// The same test in the other dimension, and the one that catches a field whose
	// length is right and whose direction is not. Earth's field is inclined by an
	// amount that belongs to the latitude rather than to the tracker, 67 deg here,
	// and a corrected sample anywhere within MagDipToleranceDeg of that is
	// believable while anything near the horizontal is not: measured on this
	// tracker on its desk, 52 uT at 3-4 deg from the horizontal, which passes the
	// magnitude band above. Feeding that puts the fusion's reference 63 deg from
	// the true field and holds the heading 93 deg from where the gyroscope would
	// have had it -- delta, measured while this was -- which is a heading that
	// moves whenever the tracker does: a distortion fixed in the room and one
	// fixed in the tracker each stop being fixed in the other's frame. That is the
	// yaw that runs up while the tracker is only being handled. VQF cannot catch
	// it for the reason the magnitude band exists at all: it compares a reading
	// only against its own reference, and that reference was itself adopted from
	// this field.
	//
	// The inclination is the corrected sample's angle from gravity, measured in
	// the world frame the tracker reports, so it holds whatever the attitude
	// estimate's yaw is doing.
	//
	// Like MagHardIron above, the nominal value is a property of where the tracker
	// is used, so moving it to another latitude means moving this constant; near
	// the magnetic equator, where the real dip is under the tolerance, a real field
	// would be withheld and the heading would fall back to the gyroscope. A value
	// stuck at this desk's 3-4 deg after carrying the tracker somewhere else says
	// the error is in the tracker's own mag path -- a remap or an offset that
	// rotates the field -- rather than in the room.
	static constexpr float MagDipNominalDeg = 67.0f;
	static constexpr float MagDipToleranceDeg = 22.0f;
	float m_magDipDeg = 0.0f;
	uint32_t m_magDipRejected = 0;
	float m_magFieldNorm = 0.0f;
	uint32_t m_magFieldRejected = 0;
	// Offset actually subtracted from the remapped sample: the calibrated per-unit
	// one when this tracker has one stored, and the shared constant otherwise.
	float m_magOffset[3] = {MagHardIron[0], MagHardIron[1], MagHardIron[2]};
	bool m_magOffsetPerUnit = false;

	// Re-read on every poll rather than latched once: the stored calibration is
	// rewritten whenever a rest calibration completes, and a stale offset here is
	// a heading pulled in a false direction for as long as the tracker is on.
	void refreshMagOffset() {
		float offset[3] = {MagHardIron[0], MagHardIron[1], MagHardIron[2]};
		m_magOffsetPerUnit = calibrator.getMagOffset(offset);

		for (int axis = 0; axis < 3; axis++) {
			m_magOffset[axis] = offset[axis];
		}
	}
	// Set once the magnetic field reference has been adopted, so it is seeded
	// exactly once and later disturbances are still detected against it.
	bool m_seededMagRef = false;

	// On-device hard-iron capture and fit.
	//
	// The compiled-in MagHardIron is one board's offset applied to every unit,
	// and the error on that unit is large: a rigid tumble of COM12 moves the
	// field magnitude the driver reads from 209 to 601 counts, where a correct
	// offset holds it constant to within the chip's noise. The offset is a
	// property of the individual board, so the fit has to run on the board.
	//
	// Samples are only kept while they open up *directions*: the sphere's
	// bounding cube is divided into a 4x4x4 grid and each cell keeps at most
	// MagCalPerBin samples, so a tracker sitting on a desk fills one cell and
	// stops, while turning it through all orientations fills the grid. The grid
	// is also the acceptance test -- a sphere is only constrained in the
	// directions the samples reach.
	static constexpr int MagCalBins = 64;
	// Measured against a real 95 s hand tumble: it covered 40 of the 64 cells
	// and kept 215 of its 949 samples at this cap. The cap is what decides how
	// many samples the fit gets; the cell count is what decides whether it is
	// allowed to store anything.
	static constexpr int MagCalPerBin = 16;
	static constexpr int MagCalMaxSamples = MagCalBins * MagCalPerBin;
	// A rotate-in-place fills 10-16 cells and leaves one axis at almost no span;
	// a real tumble filled 31-40. Both tests are applied.
	static constexpr int MagCalMinBins = 24;
	static constexpr int MagCalMinSamples = 96;
	// Every axis has to be reached by the capture, or the fit is a line, not a
	// sphere. A genuine tumble covers each axis to within about a factor of two
	// of the widest, a rotate-in-place covers one axis barely at all.
	static constexpr float MagCalMinAxisRatio = 0.35f;
	// Widest axis span a capture has to reach, in counts. A tumble covers about
	// twice the field magnitude; the earth is 25-65 uT and this chip is a few
	// tenths of a uT per count, so even the weakest field gives ~80 counts. This
	// only catches captures of a tracker that was barely moved.
	static constexpr float MagCalMinSpan = 80.0f;
	// Samples are only taken while the tracker is being turned, measured as a
	// multiple of VQF's own rest threshold (its rest deviation is 1 at that
	// threshold). At rest the observed deviation is 0.02-0.14, so this is far
	// above it and far below the 20 deg/s the fusion needs to adopt a reference;
	// a slow, careful tumble still counts. Without this a tracker sitting on a
	// desk fills the direction cells with the directions of its own noise.
	static constexpr float MagCalMinMotion = 2.0f;
	// The fitted offset has to beat the compiled one on the very capture it came
	// from, by a margin. A capture that spans little constrains little, and a
	// fit of it can look like an improvement while being dozens of counts off;
	// this refuses to store anything that cannot show its work.
	static constexpr float MagCalMinImprovement = 0.7f;

	// Learning the offset without being asked to.
	//
	// The capture above already runs on every fresh sample, gated only on the
	// tracker being turned, so a tracker that is worn normally fills it just by
	// being used -- and the fit it produces is then the same one the button
	// gesture produces, from a longer and calmer window of motion. So instead of
	// waiting for a deliberate calibration, the fit is attempted periodically and
	// stored if it passes the same acceptance test.
	//
	// Two things make this safe to run unattended. First, `improves` is measured
	// against the offset actually in use: a unit whose compiled constant is
	// already right, or whose stored offset is, leaves nothing to improve and is
	// never touched. Second, a fit is only stored once a second fit, taken at least
	// this interval later over a capture that has had new samples added to it,
	// lands within MagAutoStableCounts of the first -- one window of motion can be
	// distorted in a way its own residual does not show, two consecutive ones
	// agreeing is much harder to fake.
	static constexpr uint32_t MagAutoIntervalMicros = 30000000;
	// Noise alone would put two fits of the same board ~2.7 counts apart, since
	// the IST8306 gives 0.8 uT rms and this is 0.3 uT/LSB, but they are fitted
	// over different windows, so they differ by how much of the sphere each window
	// covered as well. Tumbles recorded from these trackers were split into two
	// windows and refitted: disjoint halves of the same tumble land 11-24 counts
	// apart on every capture that was distorted enough for the difference to
	// matter, and the clean ones could not be split at all, because each half on
	// its own failed the coverage test. So the scale of a real disagreement is
	// tens of counts, and this is set below the smallest one seen. Nine degrees of
	// heading is 15 counts of offset at 170.
	static constexpr float MagAutoStableCounts = 15.0f;
	// And the fit has to explain the capture at all, in counts rms -- the spread
	// of |m - b| it leaves, which for a correct offset is the chip's noise plus
	// whatever the room's field does across the space the tracker was moved
	// through. Measured on those same tumbles: 1.9-8.2 counts where the offset is
	// right, 39-54 where the capture was taken next to something ferrous. 20
	// counts is 6 uT, twice the worst good capture seen and half the best bad one.
	//
	// This is the test the deliberate calibrations do not have -- and should not,
	// since a person watching the residuals can simply tumble it again -- but the
	// automatic one stores without anyone watching, and the relative test alone
	// passes a distorted capture as often as not: on one of those tumbles the fit
	// beat the compiled offset by 2.4x while being 45 counts away from the offset
	// that the same board's clean captures say is right.
	static constexpr float MagAutoMaxResidual = 20.0f;

	uint32_t m_magAutoNextMicros = 0;
	bool m_magAutoCandidatePending = false;
	float m_magAutoCandidate[3] = {0.0f, 0.0f, 0.0f};
	int m_magAutoCandidateSamples = 0;
	bool m_magAutoLearned = false;

	int16_t m_magCalSample[MagCalMaxSamples][3] = {};
	uint8_t m_magCalBinCount[MagCalBins] = {};
	int m_magCalSamples = 0;
	int m_magCalBins = 0;
	float m_magCalMean[3] = {0.0f, 0.0f, 0.0f};
	float m_magCalMin[3] = {0.0f, 0.0f, 0.0f};
	float m_magCalMax[3] = {0.0f, 0.0f, 0.0f};
	uint32_t m_magCalSeen = 0;

	// Offered every fresh sample, in the IMU frame and before the offset is
	// subtracted, so the fit sees the raw field the offset has to be measured
	// from.
	void collectMagCalibration(const float field[3]) {
		sensor_real_t restDev[2];
		m_fusion.getRelativeRestDeviations(restDev);
		if (restDev[0] < MagCalMinMotion) {
			return;
		}

		// Running mean, used only as the reference the sample directions are
		// measured from -- the fit never sees it.
		m_magCalSeen++;
		const float n = static_cast<float>(m_magCalSeen);
		for (int axis = 0; axis < 3; axis++) {
			m_magCalMean[axis] += (field[axis] - m_magCalMean[axis]) / n;
		}

		float dir[3];
		float norm = 0.0f;
		for (int axis = 0; axis < 3; axis++) {
			dir[axis] = field[axis] - m_magCalMean[axis];
			norm += dir[axis] * dir[axis];
		}
		norm = sqrtf(norm);
		if (norm < 1.0f) {
			// Too close to the mean to have a direction yet. Happens for the
			// first sample, when the mean is still the sample itself.
			return;
		}

		int bin = 0;
		for (int axis = 0; axis < 3; axis++) {
			int cell = static_cast<int>((dir[axis] / norm + 1.0f) * 2.0f);
			if (cell > 3) {
				cell = 3;
			}
			bin = bin * 4 + cell;
		}
		if (m_magCalBinCount[bin] >= MagCalPerBin) {
			return;
		}
		if (m_magCalBinCount[bin] == 0) {
			m_magCalBins++;
		}
		m_magCalBinCount[bin]++;

		if (m_magCalSamples == 0) {
			for (int axis = 0; axis < 3; axis++) {
				m_magCalMin[axis] = m_magCalMax[axis] = field[axis];
			}
		}
		for (int axis = 0; axis < 3; axis++) {
			m_magCalSample[m_magCalSamples][axis] = static_cast<int16_t>(lroundf(field[axis]));
			if (field[axis] < m_magCalMin[axis]) {
				m_magCalMin[axis] = field[axis];
			}
			if (field[axis] > m_magCalMax[axis]) {
				m_magCalMax[axis] = field[axis];
			}
		}
		m_magCalSamples++;
	}

	// Algebraic sphere fit of the kept samples: |p|^2 = 2*c.p + (R^2 - |c|^2),
	// which is linear in the four unknowns, so one 4x4 solve. Then both offsets'
	// residuals over the same samples, which is the only number that says
	// whether the fitted one is better than the compiled one here.
	[[nodiscard]] MagCalibration fitMagCalibration() const {
		MagCalibration out;
		out.samples = m_magCalSamples;
		out.bins = m_magCalBins;
		for (int axis = 0; axis < 3; axis++) {
			out.span[axis] = m_magCalMax[axis] - m_magCalMin[axis];
		}

		if (m_magCalSamples < MagCalMinSamples || m_magCalBins < MagCalMinBins) {
			return out;
		}
		float widest = 0.0f;
		for (int axis = 0; axis < 3; axis++) {
			if (out.span[axis] > widest) {
				widest = out.span[axis];
			}
		}
		if (widest <= 0.0f) {
			return out;
		}
		if (widest < MagCalMinSpan) {
			return out;
		}
		for (int axis = 0; axis < 3; axis++) {
			if (out.span[axis] < MagCalMinAxisRatio * widest) {
				return out;
			}
		}

		double m[4][4] = {};
		double rhs[4] = {};
		for (int i = 0; i < m_magCalSamples; i++) {
			const double x = m_magCalSample[i][0];
			const double y = m_magCalSample[i][1];
			const double z = m_magCalSample[i][2];
			const double row[4] = {2.0 * x, 2.0 * y, 2.0 * z, 1.0};
			const double value = x * x + y * y + z * z;
			for (int r = 0; r < 4; r++) {
				for (int c = 0; c < 4; c++) {
					m[r][c] += row[r] * row[c];
				}
				rhs[r] += row[r] * value;
			}
		}

		double solution[4];
		if (!solveMagCal(m, rhs, solution)) {
			return out;
		}
		// The first three unknowns are the centre itself, not twice it: the row
		// below carries 2x, 2y, 2z against the model's 2*c.p, so the factor of two
		// is already in the matrix. Halving here would store half the board's hard
		// iron and leave the other half to swing the field with orientation, and it
		// would leave radiusSquared below with |c|^2/4 where it needs |c|^2 -- which
		// for a centre wider than the field makes it negative and refuses a fit that
		// is otherwise good.
		for (int axis = 0; axis < 3; axis++) {
			out.centre[axis] = static_cast<float>(solution[axis]);
		}
		const double radiusSquared
			= solution[3]
			  + static_cast<double>(out.centre[0]) * out.centre[0]
			  + static_cast<double>(out.centre[1]) * out.centre[1]
			  + static_cast<double>(out.centre[2]) * out.centre[2];
		if (!(radiusSquared > 0.0)) {
			return out;
		}
		out.radius = static_cast<float>(sqrt(radiusSquared));

		out.residualInUse = magCalResidual(m_magOffset);
		out.residualFitted = magCalResidual(out.centre);
		out.valid = true;
		out.improves = out.residualFitted < MagCalMinImprovement * out.residualInUse;
		return out;
	}

	// RMS of |sample - offset| over the capture: constant when the offset is the
	// board's real hard iron, whatever the tracker was doing.
	[[nodiscard]] float magCalResidual(const float offset[3]) const {
		if (m_magCalSamples == 0) {
			return 0.0f;
		}
		// Mean removed: the spread of |m - b| is what is being judged, not its
		// magnitude, which is the field and differs per tracker and per room.
		double mean = 0.0;
		for (int i = 0; i < m_magCalSamples; i++) {
			double d[3];
			for (int axis = 0; axis < 3; axis++) {
				d[axis] = static_cast<double>(m_magCalSample[i][axis]) - offset[axis];
			}
			mean += sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
		}
		mean /= m_magCalSamples;
		double variance = 0.0;
		for (int i = 0; i < m_magCalSamples; i++) {
			double d[3];
			for (int axis = 0; axis < 3; axis++) {
				d[axis] = static_cast<double>(m_magCalSample[i][axis]) - offset[axis];
			}
			const double r = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) - mean;
			variance += r * r;
		}
		return static_cast<float>(sqrt(variance / m_magCalSamples));
	}

	// Runs the fit over the capture that ordinary use has filled, and stores it
	// once a second fit agrees with the first. See MagAutoIntervalMicros.
	void learnMagOffset(uint32_t now) {
		if (static_cast<int32_t>(m_magAutoNextMicros - now) > 0) {
			return;
		}
		m_magAutoNextMicros = now + MagAutoIntervalMicros;

		const MagCalibration fit = fitMagCalibration();
		if (!fit.valid || !fit.improves || fit.residualFitted > MagAutoMaxResidual) {
			// The capture has not covered enough directions yet, which is the normal
			// state of a tracker that was just switched on; or the fit does not beat
			// the offset already in use, which is the normal state of one that has
			// nothing to learn; or it fits worse than a capture of a rigid board can
			// be fitted, which means the room moved the field rather than the board
			// having an offset. All three are silent: they are the expected answers.
			return;
		}

		if (m_magAutoCandidatePending) {
			// A second fit only says anything about the first if the capture has
			// moved on since: the same samples refitted give the same centre, which
			// would confirm nothing. A capture that has stopped growing -- the cap
			// reached, or the tracker put down -- has nothing more to add either way,
			// and the fit that passed is the best this data can give, so it is
			// taken rather than waited on.
			const bool grew = m_magCalSamples > m_magAutoCandidateSamples;
			float distance = 0.0f;
			for (int axis = 0; axis < 3; axis++) {
				const float delta = fit.centre[axis] - m_magAutoCandidate[axis];
				distance += delta * delta;
			}
			distance = sqrtf(distance);
			if (!grew || distance <= MagAutoStableCounts) {
				if (!storeMagOffset(fit.centre)) {
					m_Logger.error(
						"mag: no runtime calibration to store the fitted offset in"
					);
					m_magAutoCandidatePending = false;
					return;
				}
				m_magAutoLearned = true;
				m_magAutoCandidatePending = false;
				refreshMagOffset();
				// The filter's magnetic reference was built from the field the old
				// offset implied, so it has to be rebuilt rather than carried over:
				// it is the step that turns a reading into a heading, and a stale one
				// spends the next minute pulling the heading back towards where the
				// wrong field said the tracker was pointing.
				m_seededMagRef = false;
				// Start a new capture: this offset is the best this data can give, and
				// a later one that covers more directions can only improve on it if it
				// is fitted from samples this fit has not already seen.
				resetMagCalibration();
				m_Logger.info(
					"mag: learned offset %.1f %.1f %.1f counts, residual %.1f counts "
					"against %.1f for the offset it replaces",
					fit.centre[0],
					fit.centre[1],
					fit.centre[2],
					fit.residualFitted,
					fit.residualInUse
				);
				return;
			}
			// Disagrees with the previous fit, so that one was not the board: the
			// capture moved under it. Take this one as the new candidate.
		} else {
			m_Logger.info(
				"mag: fitted offset %.1f %.1f %.1f counts from %d samples over %d "
				"directions, residual %.1f counts against %.1f in use",
				fit.centre[0],
				fit.centre[1],
				fit.centre[2],
				fit.samples,
				fit.bins,
				fit.residualFitted,
				fit.residualInUse
			);
		}

		m_magAutoCandidatePending = true;
		m_magAutoCandidateSamples = m_magCalSamples;
		for (int axis = 0; axis < 3; axis++) {
			m_magAutoCandidate[axis] = fit.centre[axis];
		}
	}

	// Gauss-Jordan with partial pivoting on the 4x4 normal equations. The
	// singularity test is relative to the matrix: the entries scale with the
	// square of the field in counts, so an absolute threshold would mean
	// different things for different chips.
	static bool solveMagCal(double m[4][4], double rhs[4], double out[4]) {
		double scale = 0.0;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (fabs(m[i][j]) > scale) {
					scale = fabs(m[i][j]);
				}
			}
		}
		if (scale <= 0.0) {
			return false;
		}

		for (int col = 0; col < 4; col++) {
			int pivot = col;
			for (int row = col + 1; row < 4; row++) {
				if (fabs(m[row][col]) > fabs(m[pivot][col])) {
					pivot = row;
				}
			}
			if (fabs(m[pivot][col]) < 1e-9 * scale) {
				return false;
			}
			if (pivot != col) {
				for (int c = 0; c < 4; c++) {
					const double t = m[col][c];
					m[col][c] = m[pivot][c];
					m[pivot][c] = t;
				}
				const double t = rhs[col];
				rhs[col] = rhs[pivot];
				rhs[pivot] = t;
			}
			for (int row = 0; row < 4; row++) {
				if (row == col) {
					continue;
				}
				const double factor = m[row][col] / m[col][col];
				for (int c = col; c < 4; c++) {
					m[row][c] -= factor * m[col][c];
				}
				rhs[row] -= factor * rhs[col];
			}
		}
		for (int i = 0; i < 4; i++) {
			out[i] = rhs[i] / m[i][i];
		}
		return true;
	}

	[[nodiscard]] bool magWanted() const {
		return toggles.getToggle(SensorToggles::MagEnabled) && magDriver.hasMag();
	}

	void pollMag() {
		// Everything below is discarded for drivers with no aux master, which is
		// also what keeps readAuxBurst from being instantiated for them.
		if constexpr (Consts::SupportsMags) {
			if (!magWanted()) {
				// Drop the filter back to 6D. Turning the toggle off stops the
				// reads and the calls to updateMag, but on its own that leaves
				// SensorFusion::magExist set, so getQuaternion() would keep
				// reporting the 9D attitude with VQF's last heading correction
				// frozen into it. Clearing it here makes "magnetometer off"
				// mean gyro+accel only, which is what the toggle promises.
				if (m_haveMagSample) {
					m_fusion.disableMag();
					m_haveMagSample = false;
					m_haveSentMag = false;
					// The field may well have changed while the mag was off, so
					// re-adopt it on the next rest rather than trusting the old
					// reference.
					m_seededMagRef = false;
					m_lastRawMag[0] = m_lastRawMag[1] = m_lastRawMag[2] = 0.0f;
				}
				return;
			}

			uint32_t now = micros();
			if (static_cast<int32_t>(m_nextMagPollMicros - now) > 0) {
				return;
			}
			m_nextMagPollMicros = now + MagPollIntervalMicros;
			// Window bookkeeping for the rate below: only the time actually spent
			// between polls is counted, so the window cannot be diluted by periods
			// when the toggle had the polling switched off. A gap well beyond the
			// poll interval is a gap in the polling rather than a slow poll, and is
			// counted as one interval instead of its real length -- otherwise the
			// first poll after a long off period would close the window on its own.
			uint32_t pollGapMicros = MagPollIntervalMicros;
			if (m_lastMagPollMicros != 0) {
				pollGapMicros = now - m_lastMagPollMicros;
				if (pollGapMicros > MagPollGapLimitMicros) {
					pollGapMicros = MagPollIntervalMicros;
				}
			}
			m_lastMagPollMicros = now;
			m_magRateWindowMicros += pollGapMicros;
			refreshMagOffset();

			uint8_t bytes[MagSampleBytes];
			const uint32_t pollStart = micros();
			const bool pollOk
				= m_sensor.readAuxBurst(magDriver.getDataReg(), bytes, MagSampleBytes);
			const uint32_t pollMicros = micros() - pollStart;

			// Timed here because this is the one blocking read in the motion loop
			// that is not the IMU's own FIFO: with the aux pins bridged onto the
			// host bus it is a single I2C transaction, and without that it is
			// several indirect transactions plus a bounded wait on the IMU's aux
			// master. Worth knowing what it costs either way.
			m_magPollMicrosSum += pollMicros;
			m_magPollCount++;
			if (pollMicros > m_magPollMicrosMax) {
				m_magPollMicrosMax = pollMicros;
			}
			if (!pollOk) {
				m_magPollFailures++;
				return;
			}

			for (int axis = 0; axis < 3; axis++) {
				const auto raw = static_cast<int16_t>(
					static_cast<uint16_t>(bytes[axis * 2])
					| (static_cast<uint16_t>(bytes[axis * 2 + 1]) << 8)
				);
				m_lastRawMag[axis] = static_cast<float>(raw);
			}
			m_haveMagSample = true;

			if constexpr (MagFeedFusion) {
				// Polling faster than the mag produces samples means a share of
				// these reads return the previous one. Handing VQF the same
				// measurement twice would let it apply the heading correction
				// twice for one interval, so only feed it when the sample
				// actually changed.
				const bool duplicate = m_haveSentMag
									&& m_lastRawMag[0] == m_lastSentMag[0]
									&& m_lastRawMag[1] == m_lastSentMag[1]
									&& m_lastRawMag[2] == m_lastSentMag[2];

				// Count the changes over a window: this is the only on-device
				// measurement of the rate the chip is really running at, and the
				// timestep the fusion applies the heading correction with (MagTs)
				// is compiled in from what the setup sequence asks for. If the chip
				// runs at another rate, every correction is applied at the wrong
				// gain, and nothing else in the firmware would notice.
				if (!duplicate) {
					m_magFreshCount++;
				}
				// The window is polled time, not wall-clock time. The mag toggle can
				// stop the polling part-way through a window, and dividing a handful
				// of fresh samples by the whole off-period reports a rate the chip
				// never ran at -- which reads as "0.0 Hz" whenever the mag is mostly
				// off, i.e. exactly when this number gets looked at.
				if (m_magRateWindowMicros
					>= (m_magRateHz > 0.0f ? MagRateWindowMicros : MagRateWarmupMicros)) {
					const float windowMicros = static_cast<float>(m_magRateWindowMicros);
					const float rate = m_magFreshCount * 1e6f / windowMicros;
					m_magRateHz
						= m_magRateHz > 0.0f ? 0.75f * m_magRateHz + 0.25f * rate : rate;
					m_magFreshCount = 0;
					m_magRateWindowMicros = 0;
					// The poll cost is averaged over the same window rather than
					// since boot: the first poll after boot waits on the IMU's
					// aux master while its FIFO drains and can take hundreds of
					// milliseconds, which would otherwise sit in a since-boot
					// maximum forever and hide what the poll costs once running.
					// The window starts at this poll, whose cost is already in
					// the totals being cleared.
					m_magPollMicrosSum = pollMicros;
					m_magPollCount = 1;
					m_magPollMicrosMax = pollMicros;
				}

				if (duplicate) {
					return;
				}

				sensor_real_t mag[3];
				float field[3];
				for (int axis = 0; axis < 3; axis++) {
					const int source = MagAxisMap[axis];
					// Remap first, then subtract: the offset is expressed in the
					// IMU frame, which is the frame the fit solved it in.
					field[axis] = static_cast<float>(m_lastRawMag[source])
								* static_cast<float>(MagAxisSign[axis]);
					mag[axis] = static_cast<sensor_real_t>(field[axis])
							  - static_cast<sensor_real_t>(m_magOffset[axis]);
				}

				// Fed the remapped value with no offset subtracted, so the fit
				// measures the board rather than agreeing with whatever offset
				// happens to be in use. Only fresh samples: polling faster than
				// the chip produces means a share of reads hand back the one
				// before.
				//
				// Deliberately not gated on the field magnitude below: the capture
				// is what the offset is fitted *from*, so requiring a plausible
				// offset-corrected magnitude would require the calibration it is
				// being collected to produce. A capture taken in a distorted field
				// is caught by the fit's own residual and coverage tests instead.
				collectMagCalibration(field);
				learnMagOffset(now);

				// A magnetometer reading is only evidence about heading if it is a
				// reading of the geomagnetic field. The chip gives 0.3 uT/LSB, so
				// Earth's 22-67 uT is 73-223 counts, and a tracker lying on
				// something ferrous reads a multiple of that: measured here, 585
				// counts -- 176 uT -- while the fusion dragged the heading 0.4
				// deg/min towards the heading that field implies, with the gyro and
				// accelerometer holding the attitude to 0.01 deg over the same 100
				// s. VQF cannot tell: it compares a reading only against its own
				// reference, never against a magnitude that is physically possible,
				// and past magMaxRejectionTime it resumes correcting at reduced
				// gain, so a permanently wrong reading drags the heading
				// permanently instead of being rejected forever. Withholding it
				// leaves the heading to the gyroscope, which is the smaller error
				// by two orders of magnitude.
				float fieldNorm = 0.0f;
				for (int axis = 0; axis < 3; axis++) {
					fieldNorm += static_cast<float>(mag[axis]) * static_cast<float>(mag[axis]);
				}
				fieldNorm = sqrtf(fieldNorm);
				m_magFieldNorm = fieldNorm;
				const bool fieldPlausible = fieldNorm >= MagFieldMinCounts
										 && fieldNorm <= MagFieldMaxCounts;
				if (!fieldPlausible) {
					m_magFieldRejected++;
				}

				// Inclination of the same sample, in the world frame the tracker
				// reports: the corrected field turned by the reported attitude, and
				// the angle it makes below the horizontal there. The pair has to be
				// the reported one on both sides -- getFusedRotation() is the
				// fusion's quaternion composed with sensorOffset, Sensor.cpp, which
				// is exactly the composition the sample below is missing -- and then
				// this is the same world vector VQF computes, from its own
				// quaternion with the turned sample, to within a degree.
				float dipDeg = 0.0f;
				if (fieldNorm > 1.0f) {
					const Vector3 magWorld = getFusedRotation().xform(
						Vector3(
							static_cast<float>(mag[0]),
							static_cast<float>(mag[1]),
							static_cast<float>(mag[2])
						)
					);
					const float sinDip = fmaxf(-1.0f, fminf(1.0f, magWorld.z / fieldNorm));
					dipDeg = -asinf(sinDip) * 180.0f / static_cast<float>(M_PI);
				}
				m_magDipDeg = dipDeg;
				const bool dipPlausible
					= fabsf(dipDeg - MagDipNominalDeg) <= MagDipToleranceDeg;
				if (!dipPlausible) {
					m_magDipRejected++;
				}

				if (fieldPlausible && dipPlausible) {
					// The remap and offset above are solved in the frame the
					// tracker reports. VQF's body frame is that one composed with
					// sensorOffset (`fusedRotation = r * sensorOffset`), so
					// coordinates come the other way, v_fusion = sensorOffset *
					// v_reported, and the sample is turned by sensorOffset here.
					// Without the turn it arrives a quarter turn out and the
					// correction drags the heading -- and the attitude with it --
					// toward a field 92 deg from the one the tracker reports;
					// with it, the filter's own delta lands on the azimuth of
					// that sample to under a degree. See the constants above.
					const Vector3 magInFusionFrame = sensorOffset.xform(
						Vector3(
							static_cast<float>(mag[0]),
							static_cast<float>(mag[1]),
							static_cast<float>(mag[2])
						)
					);
					sensor_real_t magFusion[3];
					for (int axis = 0; axis < 3; axis++) {
						magFusion[axis] = static_cast<sensor_real_t>(magInFusionFrame[axis]);
					}
					m_fusion.updateMag(magFusion);

					// VQF acquires its magnetic field reference only while the
					// tracker is moving above magNewMinGyr (20 deg/s), which a
					// tracker that lives on a table never produces. Measured with
					// the tracker flat and still: magRefNorm stayed 0.0 and
					// magDistDetected stayed true indefinitely, which pins the
					// heading correction at half gain and leaves disturbance
					// rejection comparing against a zero reference, so it cannot
					// tell a real disturbance from the normal field.
					//
					// At rest the observed field is stable and is the environment
					// the tracker actually operates in, so adopt it as the
					// reference. Done once, so a later genuine disturbance is still
					// caught against it -- and only from a field that passed the
					// magnitude test above, since adopting a reference is the step
					// that turns a bad reading into a permanent heading error.
					if (!m_seededMagRef && m_fusion.getRestDetected()) {
						m_seededMagRef = m_fusion.seedMagRef();
					}
				}

				for (int axis = 0; axis < 3; axis++) {
					m_lastSentMag[axis] = m_lastRawMag[axis];
				}
				m_haveSentMag = true;
			}
		}
	}

	bool getMagCalibration(MagCalibration& out) const final {
		out = fitMagCalibration();
		return true;
	}

	void resetMagCalibration() final {
		for (int i = 0; i < MagCalBins; i++) {
			m_magCalBinCount[i] = 0;
		}
		for (int i = 0; i < 3; i++) {
			m_magCalMean[i] = 0.0f;
		}
		m_magCalSamples = 0;
		m_magCalBins = 0;
		m_magCalSeen = 0;
	}

	bool storeMagOffset(const float offset[3]) final {
		SlimeVR::Configuration::SensorConfig config = configuration.getSensor(sensorId);
		if (config.type != SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION) {
			return false;
		}

		for (int i = 0; i < 3; i++) {
			config.data.runtimeCalibration.M_off[i] = offset[i];
		}
		config.data.runtimeCalibration.magCalibrated = true;
		configuration.setSensor(sensorId, config);
		configuration.save();
		// The running filter reads its offset from the calibrator's own copy, which
		// is loaded from this file at setup and not watched afterwards, so the file
		// alone would only take effect on the next boot.
		calibrator.setMagOffset(offset);
		return true;
	}

	bool clearMagOffset() final {
		SlimeVR::Configuration::SensorConfig config = configuration.getSensor(sensorId);
		if (config.type != SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION) {
			return false;
		}

		config.data.runtimeCalibration.magCalibrated = false;
		configuration.setSensor(sensorId, config);
		configuration.save();
		calibrator.clearMagOffset();
		return true;
	}

	int magReadRegister(uint8_t reg) final {
		(void)reg;
		if constexpr (Consts::SupportsMags) {
			if (magDriver.hasMag()) {
				return m_sensor.readAux(reg);
			}
		}
		return -1;
	}

	bool magWriteRegister(uint8_t reg, uint8_t value) final {
		(void)reg;
		(void)value;
		if constexpr (Consts::SupportsMags) {
			if (magDriver.hasMag()) {
				m_sensor.writeAux(reg, value);
				return true;
			}
		}
		return false;
	}

	bool getRawMagSample(float out[3]) const final {
		if (!m_haveMagSample) {
			return false;
		}
		for (int i = 0; i < 3; i++) {
			out[i] = m_lastRawMag[i];
		}
		return true;
	}

	bool getMagDiagnostics(MagDiagnostics& out) const final {
		out.magDistDetected = m_fusion.getMagDistDetected();
		out.restDetected = m_fusion.getRestDetected();

		sensor_real_t restDev[2];
		m_fusion.getRelativeRestDeviations(restDev);
		out.restDevGyr = restDev[0];
		out.restDevAcc = restDev[1];

		out.magRefNorm = m_fusion.getMagRefNorm();
		out.magRefDipDeg
			= static_cast<float>(m_fusion.getMagRefDip() * 180.0 / M_PI);
		out.deltaDeg = static_cast<float>(m_fusion.getDelta() * 180.0 / M_PI);

		sensor_real_t bias[3];
		m_fusion.getBiasEstimate(bias);
		for (int i = 0; i < 3; i++) {
			out.gyroBias[i] = static_cast<float>(bias[i] * 180.0 / M_PI);
		}

		for (int i = 0; i < 3; i++) {
			out.magOffset[i] = m_magOffset[i];
		}
		out.magOffsetPerUnit = m_magOffsetPerUnit;
		out.magAutoLearned = m_magAutoLearned;
		out.magAutoCandidatePending = m_magAutoCandidatePending;
		for (int i = 0; i < 3; i++) {
			out.magAutoCandidate[i] = m_magAutoCandidate[i];
		}

		out.magRateHz = m_magRateHz;
		out.magPollUs = m_magPollCount > 0
						  ? static_cast<float>(m_magPollMicrosSum)
								/ static_cast<float>(m_magPollCount)
						  : 0.0f;
		out.magPollMaxUs = static_cast<float>(m_magPollMicrosMax);
		out.magPollFailures = m_magPollFailures;
		out.magFieldNorm = m_magFieldNorm;
		out.magFieldRejected = m_magFieldRejected;
		out.magDipDeg = m_magDipDeg;
		out.magDipRejected = m_magDipRejected;

		calibrator.getAppliedGyroBias(out.appliedGyroBias);
		out.gyroTemperatureC = lastReadTemperature;
		return true;
	}

	static bool checkPresent(const RegisterInterface& imuInterface) {
		I2Cdev::readTimeout = 100;
		auto value = imuInterface.readReg(SensorType::Regs::WhoAmI::reg);
		I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;
		if constexpr (requires { SensorType::Regs::WhoAmI::values.size(); }) {
			for (auto possible : SensorType::Regs::WhoAmI::values) {
				if (value == possible) {
					return true;
				}
			}
			return false;
		} else {
			if (value == SensorType::Regs::WhoAmI::value) {
				return true;
			}
			return false;
		}
	}

	const char* getAttachedMagnetometer() const final {
		return magDriver.getAttachedMagName();
	}
};

}  // namespace SlimeVR::Sensors
