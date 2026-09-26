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

#include <algorithm>
#include <array>
#include <cstdint>

#include <Wire.h>

#include "../../../sensorinterface/RegisterInterface.h"
#include "callbacks.h"
#include "sensors/softfusion/magdriver.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 32g
// and gyroscope range at 4000dps
// using high resolution mode
// Uses 32.768kHz clock
// Gyroscope ODR = 204.8Hz, accel ODR = 102.4Hz
// Timestamps reading not used, as they're useless (constant predefined increment)

struct ICM45Base {
	static constexpr uint8_t Address = 0x68;

	static constexpr float GyrTs = 1.0 / 204.8;
	static constexpr float AccTs = 1.0 / 102.4;
	static constexpr float TempTs = 1.0 / 409.6;

	// The IST8306 is configured for continuous measurement at 20 Hz (its setup
	// sequence in magdriver.cpp writes 0x04 to CTRL2), so the magnetometer
	// timestep handed to VQF has to match that: kMag is one minus the exponential
	// of magTs/tauMag, and half the timestep would apply each correction at half
	// the gain it was designed for. This previously claimed 100 Hz, which would
	// have had VQF integrate every heading correction ten times too fast --
	// harmless only because no mag sample ever reached it.
	static constexpr float MagTs = 1.0 / 20;

	static constexpr float GyroSensitivity = 131.072f;
	static constexpr float AccelSensitivity = 16384.0f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 128.0f;

	static constexpr float TemperatureZROChange = 20.0f;

	// The magnetometer hangs off the IMU's aux pins, so reaching it normally means
	// driving the IMU's own I2C master through indirect banked registers: ten
	// indirect accesses per poll, each one a separate I2C transaction to the IMU.
	// The same pins can instead be bridged onto the tracker's own bus, which makes
	// the mag an ordinary device on it and turns the poll into one read. That is
	// what the flag asks for; m_auxPassThrough is what was actually possible once
	// initializeBase asked the register interface what kind of host bus this is.
	static constexpr bool PreferAuxPassThrough = true;
	bool m_auxPassThrough = false;
	// Slave address given by MagDriver::init through setAuxId. In pass-through it
	// travels with every host transaction; without it the IMU's DEV_PROFILE
	// register holds it and it is not needed here.
	uint8_t m_auxId = 0;

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;
	ICM45Base(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger) {}

	struct BaseRegs {
		static constexpr uint8_t TempData = 0x0c;

		struct DeviceConfig {
			static constexpr uint8_t reg = 0x7f;
			static constexpr uint8_t valueSwReset = 0b11;
		};

		struct GyroConfig {
			static constexpr uint8_t reg = 0x1c;
			static constexpr uint8_t value
				= (0b0000 << 4) | 0b1000;  // 4000dps, odr=204.8Hz
		};

		struct AccelConfig {
			static constexpr uint8_t reg = 0x1b;
			static constexpr uint8_t value
				= (0b000 << 4) | 0b1001;  // 32g, odr = 102.4Hz
		};

		struct FifoConfig0 {
			static constexpr uint8_t reg = 0x1d;
			static constexpr uint8_t value
				= (0b01 << 6) | (0b011111);  // stream to FIFO mode, FIFO depth
											 // 8k bytes <-- this disables all APEX
											 // features, but we don't need them
		};

		struct FifoConfig3 {
			static constexpr uint8_t reg = 0x21;
			static constexpr uint8_t value = (0b1 << 0) | (0b1 << 1) | (0b1 << 2)
										   | (0b1 << 3);  // enable FIFO,
														  // enable accel,
														  // enable gyro,
														  // enable hires mode
		};

		struct PwrMgmt0 {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value
				= 0b11 | (0b11 << 2);  // accel in low noise mode, gyro in low noise
		};

		static constexpr uint8_t FifoCount = 0x12;
		static constexpr uint8_t FifoData = 0x14;

		// Indirect Register Access

		static constexpr uint32_t IRegWaitTimeMicros = 4;

		enum class Bank : uint8_t {
			IMemSram = 0x00,
			IPregBar = 0xa0,
			IPregSys1 = 0xa4,
			IPregSys2 = 0xa5,
			IPregTop1 = 0xa2,
		};

		static constexpr uint8_t IRegAddr = 0x7c;
		static constexpr uint8_t IRegData = 0x7e;

		// Mag Support

		struct IOCPadScenarioAuxOvrd {
			static constexpr uint8_t reg = 0x30;
			// Bits 3:2 say what drives the aux pins: 1 leaves them to the IMU's own
			// I2C master, 2 bridges them straight onto the host bus.
			static constexpr uint8_t value = (0b1 << 4)  // Enable AUX1 override
										   | (0b01 << 2)  // Aux pins to the I2C master
										   | (0b1 << 1)  // Enable AUX1 enable override
										   | (0b1 << 0);  // Enable AUX1
			static constexpr uint8_t valuePassThrough
				= (0b1 << 4)  // Enable AUX1 override
				| (0b10 << 2)  // Aux pins bridged to the host bus
				| (0b1 << 1)  // Enable AUX1 enable override
				| (0b1 << 0);  // Enable AUX1
		};

		struct I2CMCommand0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x06;
		};

		struct I2CMDevProfile0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x0e;
		};

		struct I2CMDevProfile1 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x0f;
		};

		struct I2CMWrData0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x33;
		};

		struct I2CMWrData1 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x34;
		};

		struct I2CMRdData0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x1b;
		};

		struct DmpExtSenOdrCfg {
			// TODO: todo
		};

		struct I2CMControl {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x16;
		};

		struct I2CMStatus {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x18;
			static constexpr uint8_t SDAErr = 0b1 << 5;
			static constexpr uint8_t SCLErr = 0b1 << 4;
			static constexpr uint8_t SRSTErr = 0b1 << 3;
			static constexpr uint8_t TimeoutErr = 0b1 << 2;
			static constexpr uint8_t Done = 0b1 << 1;
			static constexpr uint8_t Busy = 0b1 << 0;
		};
	};

#pragma pack(push, 1)
	struct FifoEntryAligned {
		int16_t accel[3];
		int16_t gyro[3];
		uint16_t temp;
		uint16_t timestamp;
		uint8_t lsb[3];
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	void softResetIMU() {
		m_RegisterInterface.writeReg(
			BaseRegs::DeviceConfig::reg,
			BaseRegs::DeviceConfig::valueSwReset
		);
		delay(35);
	}

	bool initializeBase() {
		// perform initialization step
		m_RegisterInterface.writeReg(
			BaseRegs::GyroConfig::reg,
			BaseRegs::GyroConfig::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::AccelConfig::reg,
			BaseRegs::AccelConfig::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig0::reg,
			BaseRegs::FifoConfig0::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig3::reg,
			BaseRegs::FifoConfig3::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::PwrMgmt0::reg,
			BaseRegs::PwrMgmt0::value
		);

		// Pass-through needs a host bus to bridge the aux pins onto, so it is only
		// available when the host interface is I2C: an SPI-hosted IMU would lose
		// the magnetometer instead of gaining a cheaper read.
		m_auxPassThrough = PreferAuxPassThrough && hostInterfaceIsI2C();
		m_RegisterInterface.writeReg(
			BaseRegs::IOCPadScenarioAuxOvrd::reg,
			m_auxPassThrough ? BaseRegs::IOCPadScenarioAuxOvrd::valuePassThrough
							 : BaseRegs::IOCPadScenarioAuxOvrd::value
		);
		// Says which of the two aux transports is live. Both produce working mag
		// reads, and nothing else on the device reports which one is in use --
		// the difference only shows in the poll cost and in whether the pass-
		// through-only registers (0x31, 0x41) hold what was written.
		m_Logger.info(
			"IMU %s: aux pins %s",
			m_RegisterInterface.toString().c_str(),
			m_auxPassThrough ? "bridged onto the host bus" : "driven by the IMU's own master"
		);

		read_buffer.resize(FullFifoEntrySize * MaxReadings);

		delay(1);

		return true;
	}

	static constexpr size_t MaxReadings = 8;
	// Allocate on heap so that it does not take up stack space, which can result in
	// stack overflow and panic
	std::vector<uint8_t> read_buffer;

	bool bulkRead(DriverCallbacks<int32_t>&& callbacks) {
		constexpr int16_t InvalidReading = -32768;

		size_t fifo_packets = m_RegisterInterface.readReg16(BaseRegs::FifoCount);

		if (fifo_packets <= 1) {
			return false;
		}

		// AN-000364
		// 2.16 FIFO EMPTY EVENT IN STREAMING MODE CAN CORRUPT FIFO DATA
		//
		// Description: When in FIFO streaming mode, a FIFO empty event
		// (caused by host reading the last byte of the last FIFO frame) can
		// cause FIFO data corruption in the first FIFO frame that arrives
		// after the FIFO empty condition. Once the issue is triggered, the
		// FIFO state is compromised and cannot recover. FIFO must be set in
		// bypass mode to flush out the wrong state
		//
		// When operating in FIFO streaming mode, if FIFO threshold
		// interrupt is triggered with M number of FIFO frames accumulated
		// in the FIFO buffer, the host should only read the first M-1
		// number of FIFO frames. This prevents the FIFO empty event, that
		// can cause FIFO data corruption, from happening.
		--fifo_packets;

		auto packets_to_read = std::min(fifo_packets, MaxReadings);

		size_t bytes_to_read = packets_to_read * FullFifoEntrySize;
		m_RegisterInterface
			.readBytes(BaseRegs::FifoData, bytes_to_read, read_buffer.data());

		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			uint8_t header = read_buffer[i];
			bool has_gyro = header & (1 << 5);
			bool has_accel = header & (1 << 6);

			FifoEntryAligned entry;
			memcpy(
				&entry,
				&read_buffer[i + 0x1],
				sizeof(FifoEntryAligned)
			);  // skip fifo header

			if (has_gyro && entry.gyro[0] != InvalidReading) {
				const int32_t gyroData[3]{
					static_cast<int32_t>(entry.gyro[0]) << 4 | (entry.lsb[0] & 0xf),
					static_cast<int32_t>(entry.gyro[1]) << 4 | (entry.lsb[1] & 0xf),
					static_cast<int32_t>(entry.gyro[2]) << 4 | (entry.lsb[2] & 0xf),
				};
				callbacks.processGyroSample(gyroData, GyrTs);
			}

			if (has_accel && entry.accel[0] != InvalidReading) {
				const int32_t accelData[3]{
					static_cast<int32_t>(entry.accel[0]) << 4
						| (static_cast<int32_t>((entry.lsb[0]) & 0xf0) >> 4),
					static_cast<int32_t>(entry.accel[1]) << 4
						| (static_cast<int32_t>((entry.lsb[1]) & 0xf0) >> 4),
					static_cast<int32_t>(entry.accel[2]) << 4
						| (static_cast<int32_t>((entry.lsb[2]) & 0xf0) >> 4),
				};
				callbacks.processAccelSample(accelData, AccTs);
			}

			if (entry.temp != 0x8000) {
				callbacks.processTempSample(static_cast<int16_t>(entry.temp), TempTs);
			}
		}

		return fifo_packets > MaxReadings;
	}

	template <typename Reg>
	uint8_t readBankRegister() {
		uint8_t buffer;
		readBankRegister<Reg>(&buffer, sizeof(buffer));
		return buffer;
	}

	template <typename Reg, typename T>
	void readBankRegister(T* buffer, size_t length) {
		uint8_t data[] = {
			static_cast<uint8_t>(Reg::bank),
			Reg::reg,
		};

		auto* bufferBytes = reinterpret_cast<uint8_t*>(buffer);
		m_RegisterInterface.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		for (size_t i = 0; i < length * sizeof(T); i++) {
			bufferBytes[i] = m_RegisterInterface.readReg(BaseRegs::IRegData);
			delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		}
	}

	template <typename Reg>
	void writeBankRegister() {
		writeBankRegister<Reg>(&Reg::value, sizeof(Reg::value));
	}

	template <typename Reg, typename T>
	void writeBankRegister(T* buffer, size_t length) {
		auto* bufferBytes = reinterpret_cast<uint8_t*>(buffer);

		uint8_t data[] = {
			static_cast<uint8_t>(Reg::bank),
			Reg::reg,
			bufferBytes[0],
		};

		m_RegisterInterface.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		for (size_t i = 1; i < length * sizeof(T); i++) {
			m_RegisterInterface.writeReg(BaseRegs::IRegData, bufferBytes[i]);
			delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		}
	}

	template <typename Reg>
	void writeBankRegister(uint8_t value) {
		writeBankRegister<Reg>(&value, sizeof(value));
	}

	// I2CImpl names itself "I2C(0x68)" and SPIImpl "SPI". This only decides which
	// way the aux pins are driven, so a string check costs nothing once at init
	// and does not need RTTI to be enabled.
	bool hostInterfaceIsI2C() const {
		return m_RegisterInterface.toString().rfind("I2C", 0) == 0;
	}

	void setAuxId(uint8_t deviceId) {
		// The aux master reads the address out of DEV_PROFILE1 before every
		// transaction; pass-through has no such state, because the mag is just
		// another device on the host bus and the address goes in each transaction.
		m_auxId = deviceId;
		if (!m_auxPassThrough) {
			writeBankRegister<typename BaseRegs::I2CMDevProfile1>(deviceId);
		}
	}

	// Bounded wait for the aux I2C master to finish a transaction. The
	// unbounded spin this replaces would hang the whole tracker on a wedged or
	// NAKing aux bus, which is a far worse failure than dropping one
	// magnetometer byte and retrying on the next pass.
	static constexpr uint32_t AuxTimeoutMicros = 5000;

	// Pass-through reads carry their own timeout rather than inheriting I2Cdev's
	// 1000 ms default: a wedged mag must not stall the motion loop for a second
	// per poll, which is the failure the bounded aux wait above exists to avoid.
	static constexpr uint16_t AuxPassThroughTimeoutMs = 20;

	bool waitForAux() {
		uint32_t start = micros();
		uint8_t status;
		while ((status = readBankRegister<typename BaseRegs::I2CMStatus>())
			   & BaseRegs::I2CMStatus::Busy) {
			if (micros() - start > AuxTimeoutMicros) {
				m_Logger.error("Aux transaction timed out");
				return false;
			}
		}

		if (status != BaseRegs::I2CMStatus::Done) {
			m_Logger.error("Aux transaction returned status 0x%02x", status);
			return false;
		}

		return true;
	}

	bool readAuxChecked(uint8_t address, uint8_t& out) {
		writeBankRegister<typename BaseRegs::I2CMDevProfile0>(address);

		writeBankRegister<typename BaseRegs::I2CMCommand0>(
			(0b1 << 7)  // Last transaction
			| (0b0 << 6)  // Channel 0
			| (0b01 << 4)  // Read with register
			| (0b0001 << 0)  // Read 1 byte
		);
		writeBankRegister<typename BaseRegs::I2CMControl>(
			(0b0 << 6)  // No restarts
			| (0b0 << 3)  // Fast mode
			| (0b1 << 0)  // Start transaction
		);

		if (!waitForAux()) {
			return false;
		}

		out = readBankRegister<typename BaseRegs::I2CMRdData0>();
		return true;
	}

	// Whether anything answers at the aux device's address at all.
	//
	// Asked before the who-am-I read, because that read has a chip that is not
	// fitted as one of its normal answers: MagDriver::init walks a list of
	// candidates and a board carries one of them, so reading a register at the
	// address of an absent chip is a step of detection rather than a fault. On the
	// host bus it does not look like one -- the core prints two errors for each
	// absent candidate, an ESP_ERR_INVALID_STATE from i2c_master_transmit_receive
	// and the Wire error that reports it -- so every boot of every board carried
	// errors about a chip it was never going to have. A transmit of no bytes is
	// the same question in the one form the core logs only at verbose: it runs the
	// IDF device probe, which reports a missing device through its return code.
	bool auxDevicePresent() {
		if (!m_auxPassThrough) {
			// Through the IMU's own aux master a missing device only sets the
			// status register that waitForAux polls, and nothing is printed either
			// way, so the who-am-I read can be the question there.
			return true;
		}

		Wire.beginTransmission(m_auxId);
		return Wire.endTransmission() == 0;
	}

	uint8_t readAux(uint8_t address) {
		uint8_t value = 0;
		// Through the aux master this is a read op with the register address in
		// DEV_PROFILE0; in pass-through the register is the read's own address
		// byte and the answer comes straight back off the host bus.
		const bool ok = m_auxPassThrough
			? I2Cdev::readByte(m_auxId, address, &value, AuxPassThroughTimeoutMs) == 1
			: readAuxChecked(address, value);
		if (!ok) {
			m_Logger.error("Aux read from address 0x%02x failed", address);
		}
		return value;
	}

	// Reads `length` consecutive aux registers in a single transaction, so the
	// bytes all come from the same sample. Reading them one transaction at a
	// time lets the sensor update mid-way through, splicing two samples into one
	// vector -- and a spliced magnetometer vector is a heading error, not just
	// noise, because VQF trusts it.
	bool readAuxBurst(uint8_t address, uint8_t* out, uint8_t length) {
		if (length == 0 || length > 6) {
			return false;
		}

		if (m_auxPassThrough) {
			// One host read of `length` consecutive registers, which carries the
			// same one-sample-per-read guarantee as the aux burst below for one
			// I2C transaction instead of ten indirect banked accesses and a wait
			// on the IMU's aux master.
			return I2Cdev::readBytes(m_auxId, address, length, out, AuxPassThroughTimeoutMs)
				== length;
		}

		writeBankRegister<typename BaseRegs::I2CMDevProfile0>(address);

		writeBankRegister<typename BaseRegs::I2CMCommand0>(
			(0b1 << 7)  // Last transaction
			| (0b0 << 6)  // Channel 0
			| (0b01 << 4)  // Read with register
			| (length & 0x0f)  // Read `length` bytes
		);
		writeBankRegister<typename BaseRegs::I2CMControl>(
			(0b0 << 6)  // No restarts
			| (0b0 << 3)  // Fast mode
			| (0b1 << 0)  // Start transaction
		);

		if (!waitForAux()) {
			return false;
		}

		// I2CM_RD_DATA0..RD_DATA20 are contiguous, and the indirect access port
		// auto-increments, so one address write covers the whole burst.
		uint8_t data[] = {
			static_cast<uint8_t>(BaseRegs::Bank::IPregTop1),
			BaseRegs::I2CMRdData0::reg,
		};
		m_RegisterInterface.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		for (uint8_t i = 0; i < length; i++) {
			out[i] = m_RegisterInterface.readReg(BaseRegs::IRegData);
			delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		}

		return true;
	}

	void writeAux(uint8_t address, uint8_t value) {
		if (m_auxPassThrough) {
			if (!I2Cdev::writeByte(m_auxId, address, value)) {
				m_Logger.error("Aux write to address 0x%02x failed", address);
			}
			return;
		}

		// A write op sends its payload as raw bytes -- I2CM_DEV_PROFILE0 is
		// named rd_address_0 and is only consulted for read ops. So the
		// register address has to travel as the first byte of the write data,
		// with the value as the second, and the burst length covering both.
		// Writing the address to DEV_PROFILE0 and a length of 1 instead (which
		// is what this did) completes without error and changes nothing, so
		// every attempt to configure the magnetometer was silently dropped and
		// it stayed suspended with zeroed data registers.
		writeBankRegister<typename BaseRegs::I2CMWrData0>(address);
		writeBankRegister<typename BaseRegs::I2CMWrData1>(value);
		writeBankRegister<typename BaseRegs::I2CMCommand0>(
			(0b1 << 7)  // Last transaction
			| (0b0 << 6)  // Channel 0
			| (0b00 << 4)  // Write op
			| (0b0010 << 0)  // Register address + 1 value byte
		);
		writeBankRegister<typename BaseRegs::I2CMControl>(
			(0b0 << 6)  // No restarts
			| (0b0 << 3)  // Fast mode
			| (0b1 << 0)  // Start transaction
		);

		if (!waitForAux()) {
			m_Logger.error("Aux write to address 0x%02x failed", address);
		}
	}

	// The ICM's hardware aux-FIFO streaming (I2C master polling the mag on its
	// own schedule and dropping timestamped frames into the FIFO) is not
	// implemented for this driver family -- it needs the aux routing and ODR
	// registers configured exactly right, and a wrong bit there can wedge the
	// aux bus. Instead the sensor reads the mag's data registers itself, either
	// through the host bus when the aux pins are bridged to it or through the
	// aux transactions above when they are not, so these two have nothing to do.
	// Kept because MagInterface still calls them.
	void startAuxPolling(uint8_t dataReg, MagDataWidth dataWidth) {}

	void stopAuxPolling() {}

	void deinit() { softResetIMU(); }
};

};  // namespace SlimeVR::Sensors::SoftFusion::Drivers
