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

#include "serialcommands.h"

#include <CmdCallback.hpp>

#include <cmath>
#include <cstdlib>

#include "GlobalVars.h"
#include "base64.hpp"
#include "batterymonitor.h"
#include "logging/Logger.h"
#include "sensorinterface/I2CWireSensorInterface.h"
#include "utils.h"

#if defined(CONFIG_IDF_TARGET_ESP32C3)
#include "soc/rtc_cntl_reg.h"
#endif

#ifdef EXT_SERIAL_COMMANDS
#define CALLBACK_SIZE 7  // Increase callback size to allow for debug commands
#include "i2cscan.h"
#endif

#ifndef CALLBACK_SIZE
#define CALLBACK_SIZE 6  // Default callback size
#endif

#if defined(VENDOR_URL) && defined(VENDOR_NAME) && defined(PRODUCT_NAME) \
	&& defined(UPDATE_ADDRESS) && defined(UPDATE_NAME)
constexpr const char* FULL_VENDOR_STR
	= "Vendor: " VENDOR_NAME " (" VENDOR_URL "), product: " PRODUCT_NAME
	  ", firmware update url: " UPDATE_ADDRESS ", name: " UPDATE_NAME;
#elif defined(VENDOR_URL) && defined(VENDOR_NAME) && defined(PRODUCT_NAME)
constexpr const char* FULL_VENDOR_STR
	= "Vendor: " VENDOR_NAME " (" VENDOR_URL "), product: " PRODUCT_NAME;
#elif defined(VENDOR_NAME) && defined(PRODUCT_NAME)
constexpr const char* FULL_VENDOR_STR
	= "Vendor: " VENDOR_NAME ", product: " PRODUCT_NAME;
#else
constexpr const char* FULL_VENDOR_STR = "Vendor: Unknown, product: Unknown";
#endif

static const char sCMDFlashmdoe[] PROGMEM
	= "Entering flashing mode.\r\n"
	  "You can now close the serial monitor\r\n"
	  "and go to the firmware flasher to flash\r\n"
	  "your tracker over USB.\r\n"
	  "If you entered the flashing mode by accident,\r\n"
	  "turn your tracker off and on.";

namespace SerialCommands {
SlimeVR::Logging::Logger logger("SerialCommands");

CmdCallback<CALLBACK_SIZE> cmdCallbacks;
CmdParser cmdParser;
CmdBuffer<256> cmdBuffer;

bool lengthCheck(
	const char* const text,
	unsigned int length,
	const char* const cmd,
	const char* const name
) {
	size_t l = text != nullptr ? strlen(text) : 0;
	if ((l > length)) {
		logger.error(
			"%s ERROR: %s is longer than %d bytes / Characters",
			cmd,
			name,
			length
		);
		return false;
	}
	return true;
}

unsigned int
decode_base64_length_null(const char* const b64char, unsigned int* b64ssidlength) {
	if (b64char == NULL) {
		return 0;
	}
	*b64ssidlength = (unsigned int)strlen(b64char);
	return decode_base64_length((unsigned char*)b64char, *b64ssidlength);
}

// Stores this tracker's own magnetometer hard-iron offset, in raw counts, in the
// frame the driver subtracts it in -- that is, after the axis remap and sign, so
// it can be pasted straight in from a fit. It lives in the sensor's runtime
// calibration file, which is the only per-sensor storage this firmware has.
//
// Takes effect immediately: the running filter is handed the new offset, though
// the magnetic reference it has already adopted was shaped by the old one, so the
// first minutes after a change are the filter re-centring on the new field.
// Storing without restarting is what the automatic fit needs, since it runs while
// the tracker is being worn.
void setMagOffset(CmdParser* parser) {
	auto& sensors = sensorManager.getSensors();
	if (sensors.empty()) {
		logger.error("CMD SET MAGOFF ERROR: no sensors");
		return;
	}
	Sensor* sensor = sensors[0].get();

	if (parser->getParamCount() == 3 && parser->equalCmdParam(2, "DEFAULT")) {
		if (!sensor->clearMagOffset()) {
			logger.error(
				"CMD SET MAGOFF ERROR: sensor 0 has no runtime calibration to store it in"
			);
			return;
		}
		logger.info(
			"CMD SET MAGOFF OK: default offset restored"
		);
		return;
	}

	if (parser->getParamCount() != 5) {
		logger.error("CMD SET MAGOFF ERROR: expected three values");
		logger.info("Syntax: SET MAGOFF <x> <y> <z>");
		logger.info("        SET MAGOFF DEFAULT");
		return;
	}

	float offset[3];
	for (int i = 0; i < 3; i++) {
		const char* value = parser->getCmdParam(2 + i);
		offset[i] = value != nullptr ? strtof(value, nullptr) : 0.0f;
		if (value == nullptr || !std::isfinite(offset[i])) {
			logger.error("CMD SET MAGOFF ERROR: argument %d is not a number", i + 1);
			return;
		}
	}

	if (!sensor->storeMagOffset(offset)) {
		logger.error(
			"CMD SET MAGOFF ERROR: sensor 0 has no runtime calibration to store it in"
		);
		return;
	}

	logger.info(
		"CMD SET MAGOFF OK: %.1f %.1f %.1f",
		offset[0],
		offset[1],
		offset[2]
	);
}

// Fits this tracker's own magnetometer hard-iron offset from the capture it has
// been collecting, and stores it in the same per-unit slot `SET MAGOFF` writes
// to, taking effect at once. The magnetic reference the filter has already
// adopted was shaped by the old offset, so the heading is only right again after
// it has re-seeded -- the button gesture reboots for exactly that reason, and
// this is the same fit without the reboot.
//
// The point of doing this on the tracker rather than offline is that the offset
// belongs to the board, and the compiled-in constant is one board's value for
// every board. What makes it usable is the reporting: the fit prints the
// coverage it was based on and the residual both offsets leave over that same
// capture, so a bad capture is visible instead of being stored as truth.
void setMagCalibration(CmdParser* parser) {
	auto& sensors = sensorManager.getSensors();
	if (sensors.empty()) {
		logger.error("CMD SET MAGCAL ERROR: no sensors");
		return;
	}
	Sensor* sensor = sensors[0].get();

	if (parser->getParamCount() == 3 && parser->equalCmdParam(2, "RESET")) {
		sensor->resetMagCalibration();
		logger.info("CMD SET MAGCAL OK: capture cleared, start turning the tracker");
		return;
	}

	const bool apply = parser->getParamCount() == 3 && parser->equalCmdParam(2, "APPLY");
	if (parser->getParamCount() != 2 && !apply) {
		logger.error("CMD SET MAGCAL ERROR: unrecognized argument");
		logger.info("Syntax: SET MAGCAL          report the capture and what it fits");
		logger.info("        SET MAGCAL APPLY    store the fitted offset");
		logger.info("        SET MAGCAL RESET    start a new capture");
		return;
	}

	MagCalibration fit;
	if (!sensor->getMagCalibration(fit)) {
		logger.error("CMD SET MAGCAL ERROR: sensor 0 has no magnetometer");
		return;
	}

	logger.info(
		"CMD SET MAGCAL: capture %d samples over %d/64 directions, spans %.0f %.0f %.0f counts",
		fit.samples,
		fit.bins,
		fit.span[0],
		fit.span[1],
		fit.span[2]
	);

	if (!fit.valid) {
		logger.error(
			"CMD SET MAGCAL ERROR: the capture does not cover enough orientations; "
			"a sphere is only constrained where the samples reach"
		);
		logger.info(
			"        Turn the tracker slowly through all orientations, then APPLY"
		);
		return;
	}

	logger.info(
		"CMD SET MAGCAL fit: centre %.1f %.1f %.1f, |B| %.0f counts",
		fit.centre[0],
		fit.centre[1],
		fit.centre[2],
		fit.radius
	);
	logger.info(
		"CMD SET MAGCAL residual: in use %.1f counts rms, fitted %.1f counts rms",
		fit.residualInUse,
		fit.residualFitted
	);

	if (!fit.improves) {
		logger.error(
			"CMD SET MAGCAL ERROR: the fit does not beat the offset in use on this "
			"capture, so the capture is distorted rather than the offset wrong"
		);
		logger.info(
			"        Turn the tracker through the full sphere away from metal, "
			"and try again"
		);
		return;
	}

	if (!apply) {
		logger.info("CMD SET MAGCAL: not stored, run 'SET MAGCAL APPLY' to keep it");
		return;
	}

	if (!sensor->storeMagOffset(fit.centre)) {
		logger.error(
			"CMD SET MAGCAL ERROR: sensor 0 has no runtime calibration to store it in"
		);
		return;
	}

	logger.info(
		"CMD SET MAGCAL OK: %.1f %.1f %.1f stored, in use from now",
		fit.centre[0],
		fit.centre[1],
		fit.centre[2]
	);
}

// One register of the magnetometer, through the IMU's I2C master. Diagnostic:
// the driver's setup sequence writes and never reads back, so whether those
// writes landed -- and what the chip is actually configured as -- is otherwise
// not observable from the tracker at all. With no argument this dumps the low
// registers, which is the whole control and data area of the parts this
// firmware drives.
void getMagRegister(CmdParser* parser) {
	auto& sensors = sensorManager.getSensors();
	if (sensors.empty()) {
		logger.error("CMD GET MAGREG ERROR: no sensors");
		return;
	}
	Sensor* sensor = sensors[0].get();

	if (parser->getParamCount() == 2) {
		for (int base = 0; base < 0x50; base += 8) {
			char line[3 * 8 + 8];
			int used = 0;
			for (int i = 0; i < 8; i++) {
				const int value = sensor->magReadRegister(static_cast<uint8_t>(base + i));
				used += snprintf(
					line + used,
					sizeof(line) - used,
					"%02x ",
					value < 0 ? 0 : value & 0xff
				);
			}
			logger.info("CMD GET MAGREG %02x: %s", base, line);
		}
		return;
	}

	if (parser->getParamCount() != 3) {
		logger.error("CMD GET MAGREG ERROR: expected at most one register");
		logger.info("Syntax: GET MAGREG          dump 0x00-0x4f");
		logger.info("        GET MAGREG <reg>    one register");
		return;
	}

	const long reg = strtol(parser->getCmdParam(2), nullptr, 0);
	if (reg < 0 || reg > 0xff) {
		logger.error("CMD GET MAGREG ERROR: register must be 0-255");
		return;
	}
	const int value = sensor->magReadRegister(static_cast<uint8_t>(reg));
	if (value < 0) {
		logger.error("CMD GET MAGREG ERROR: sensor 0 has no magnetometer");
		return;
	}
	logger.info("CMD GET MAGREG 0x%02x = 0x%02x", static_cast<int>(reg), value & 0xff);
}

// Writes one magnetometer register and reads it straight back, so the reply
// says both what was asked for and what the chip took. Writes here can fail
// silently on this path -- the aux bus reports success for a transaction the
// chip never saw -- and a register that reads back unchanged is how that shows.
void setMagRegister(CmdParser* parser) {
	auto& sensors = sensorManager.getSensors();
	if (sensors.empty()) {
		logger.error("CMD SET MAGREG ERROR: no sensors");
		return;
	}
	if (parser->getParamCount() != 4) {
		logger.error("CMD SET MAGREG ERROR: expected a register and a value");
		logger.info("Syntax: SET MAGREG <reg> <value>");
		return;
	}

	const long reg = strtol(parser->getCmdParam(2), nullptr, 0);
	const long value = strtol(parser->getCmdParam(3), nullptr, 0);
	if (reg < 0 || reg > 0xff || value < 0 || value > 0xff) {
		logger.error("CMD SET MAGREG ERROR: register and value must be 0-255");
		return;
	}

	Sensor* sensor = sensors[0].get();
	if (!sensor->magWriteRegister(static_cast<uint8_t>(reg), static_cast<uint8_t>(value))) {
		logger.error("CMD SET MAGREG ERROR: sensor 0 has no magnetometer");
		return;
	}
	const int readBack = sensor->magReadRegister(static_cast<uint8_t>(reg));
	logger.info(
		"CMD SET MAGREG OK: 0x%02x wrote 0x%02x, reads back 0x%02x",
		static_cast<int>(reg),
		static_cast<int>(value),
		readBack < 0 ? 0 : readBack & 0xff
	);
}

// Applies the server's magnetometer toggle from the serial console. The server
// is the only writer that normally sets it, and while its own per-tracker state
// says the mag is unwanted it re-applies that state within seconds, so this is
// for the bench: it is the only way to run the firmware with the mag on without
// the GUI, and it is what the field and rate diagnostics below are read against.
void setMagFlag(CmdParser* parser) {
	auto& sensors = sensorManager.getSensors();
	if (sensors.empty()) {
		logger.error("CMD SET MAGFLAG ERROR: no sensors");
		return;
	}
	if (parser->getParamCount() != 3) {
		logger.error("CMD SET MAGFLAG ERROR: too many arguments");
		logger.info("Syntax: SET MAGFLAG <0|1>");
		return;
	}

	const long state = strtol(parser->getCmdParam(2), nullptr, 0);
	if (state != 0 && state != 1) {
		logger.error("CMD SET MAGFLAG ERROR: expected 0 or 1");
		return;
	}

	Sensor* sensor = sensors[0].get();
	if (!sensor->isFlagSupported(SensorToggles::MagEnabled)) {
		logger.error("CMD SET MAGFLAG ERROR: sensor 0 has no magnetometer");
		return;
	}

	sensor->setFlag(SensorToggles::MagEnabled, state != 0);
	logger.info(
		"CMD SET MAGFLAG OK: mag toggle now %s",
		sensor->getFlagState(SensorToggles::MagEnabled) ? "on" : "off"
	);
}

void cmdSet(CmdParser* parser) {
	if (parser->getParamCount() != 1) {
		if (parser->equalCmdParam(1, "WIFI")) {
			if (parser->getParamCount() < 3) {
				logger.error("CMD SET WIFI ERROR: Too few arguments");
				logger.info("Syntax: SET WIFI \"<SSID>\" \"<PASSWORD>\"");
			} else {
				const char* sc_ssid = parser->getCmdParam(2);
				const char* sc_pw = parser->getCmdParam(3);

				if (!lengthCheck(sc_ssid, 32, "CMD SET WIFI", "SSID")
					&& !lengthCheck(sc_pw, 64, "CMD SET WIFI", "Password")) {
					return;
				}

				wifiNetwork.setWiFiCredentials(sc_ssid, sc_pw);
				logger.info("CMD SET WIFI OK: New wifi credentials set, reconnecting");
			}
		} else if (parser->equalCmdParam(1, "BWIFI")) {
			if (parser->getParamCount() < 3) {
				logger.error("CMD SET BWIFI ERROR: Too few arguments");
				logger.info("Syntax: SET BWIFI <B64SSID> <B64PASSWORD>");
			} else {
				const char* b64ssid = parser->getCmdParam(2);
				const char* b64pass = parser->getCmdParam(3);
				unsigned int b64ssidlength = 0;
				unsigned int b64passlength = 0;
				unsigned int ssidlength
					= decode_base64_length_null(b64ssid, &b64ssidlength);
				unsigned int passlength
					= decode_base64_length_null(b64pass, &b64passlength);

				// alloc the strings and set them to 0 (null terminating)
				char ssid[ssidlength + 1];
				memset(ssid, 0, ssidlength + 1);
				char pass[passlength + 1];
				memset(pass, 0, passlength + 1);
				// make a pointer to pass
				char* ppass = pass;
				decode_base64(
					(const unsigned char*)b64ssid,
					b64ssidlength,
					(unsigned char*)ssid
				);
				if (!lengthCheck(ssid, 32, "CMD SET BWIFI", "SSID")) {
					return;
				}

				if ((b64pass != NULL) && (b64passlength > 0)) {
					decode_base64(
						(const unsigned char*)b64pass,
						b64passlength,
						(unsigned char*)pass
					);
					if (!lengthCheck(pass, 64, "CMD SET BWIFI", "Password")) {
						return;
					}
				} else {
					// set the pointer for pass to null for no password
					ppass = NULL;
				}
				wifiNetwork.setWiFiCredentials(ssid, ppass);
				logger.info("CMD SET BWIFI OK: New wifi credentials set, reconnecting");
			}
		} else if (parser->equalCmdParam(1, "FLASHMODE")) {
#if ESP8266
			logger.info(sCMDFlashmdoe);
			delay(1000);
			ESP.rebootIntoUartDownloadMode();
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
			logger.info(sCMDFlashmdoe);
			delay(1000);
			// from https://esp32.com/viewtopic.php?t=33180
			REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
			esp_restart();
#else
			logger.error(PSTR("Flashmode is not supported on this device"));
#endif
		} else if (parser->equalCmdParam(1, "MAGOFF")) {
			setMagOffset(parser);
		} else if (parser->equalCmdParam(1, "MAGCAL")) {
			setMagCalibration(parser);
		} else if (parser->equalCmdParam(1, "MAGREG")) {
			setMagRegister(parser);
		} else if (parser->equalCmdParam(1, "MAGFLAG")) {
			setMagFlag(parser);
#if EXT_SERIAL_COMMANDS
		} else if (parser->equalCmdParam(1, "CRASH")) {			// well target of this function is to crash the tracker
			// used for debuging/testing
			while (true) {
				int* ptr = NULL;
				*ptr = 0;
			}
#endif
		} else {
			logger.error("CMD SET ERROR: Unrecognized variable to set");
		}
	} else {
		logger.error("CMD SET ERROR: No variable to set");
	}
}

void printState() {
	logger.info(
		"SlimeVR Tracker, board: %d, hardware: %d, protocol: %d, firmware: %s, "
		"address: %s, mac: %s, imu rotation: %.0f deg, status: %d, wifi state: %d",
		BOARD,
		HARDWARE_MCU,
		PROTOCOL_VERSION,
		FIRMWARE_VERSION,
		wifiNetwork.getAddress().toString().c_str(),
		WiFi.macAddress().c_str(),
		// The rotation this build bakes in, so a tracker reported as reading
		// inverted can be checked against what the firmware was built with.
		IMU_ROTATION * RAD_TO_DEG,
		statusManager.getStatus(),
		static_cast<int>(wifiNetwork.getWiFiState())
	);

	logger.info("%s", FULL_VENDOR_STR);

	for (auto& sensor : sensorManager.getSensors()) {
		logger.info(
			"Sensor[%d]: %s (%.7f %.7f %.7f %.7f) is working: %s, had data: %s",
			sensor->getSensorId(),
			getIMUNameByType(sensor->getSensorType()),
			UNPACK_QUATERNION(sensor->getFusedRotation()),
			sensor->isWorking() ? "true" : "false",
			sensor->getHadData() ? "true" : "false"
		);
		const char* mag = sensor->getAttachedMagnetometer();
		// Logged unconditionally, including while the toggle is off: the point of
		// the toggle is that the mag can be switched off without a reflash, so the
		// line has to still say something when it is off.
		logger.info(
			"Sensor[%d] magnetometer: %s, toggle: %s, supported: %s",
			sensor->getSensorId(),
			mag ? mag : "none",
			sensor->getFlagState(SensorToggles::MagEnabled) ? "on" : "off",
			sensor->isFlagSupported(SensorToggles::MagEnabled) ? "yes" : "no"
		);

		// Raw mag alongside the attitude, so the mag's axis mapping can be
		// measured against a known rotation rather than guessed. The correct
		// mapping makes the mag vector constant once rotated into the world
		// frame by the quaternion on the line above.
		//
		// The acceleration is printed for one reason: it is the only independent
		// measure of which way is down. The angle between the field and gravity is
		// the field's inclination, and both vectors are read by the same chip in
		// the same body frame, so that angle holds whatever the attitude estimate
		// is doing -- a field at 54 uT but 35 degrees from the horizontal, where the
		// geomagnetic field at this latitude is 67, is a distorted field or a wrong
		// offset, and no amount of staring at the tracker's own dip can tell those
		// two apart. Note the frames differ: setAcceleration sandwiches the
		// acceleration with the same sensorOffset the rotation carries, so this
		// vector is in the reported frame while the mag below is in the IMU frame.
		const Vector3& accel = sensor->getAcceleration();
		logger.info(
			"Sensor[%d] accel: %.3f %.3f %.3f",
			sensor->getSensorId(),
			accel.x,
			accel.y,
			accel.z
		);
		float rawMag[3];
		if (sensor->getRawMagSample(rawMag)) {
			logger.info(
				"Sensor[%d] raw mag: %.0f %.0f %.0f",
				sensor->getSensorId(),
				rawMag[0],
				rawMag[1],
				rawMag[2]
			);
		}

		// Whether VQF is actually correcting heading, and whether it thinks it is
		// at rest. Both matter most when the tracker is sitting still: a magRefNorm
		// of zero means no magnetic reference was ever accepted, and rest never
		// detected means the gyroscope bias is never learned.
		MagDiagnostics diag;
		if (sensor->getMagDiagnostics(diag)) {
			logger.info(
				"Sensor[%d] fusion: rest %s (dev gyr %.2f acc %.2f), magdist %s, "
				"refnorm %.1f dip %.1f, delta %.2f deg, bias %.3f %.3f %.3f dps",
				sensor->getSensorId(),
				diag.restDetected ? "yes" : "no",
				diag.restDevGyr,
				diag.restDevAcc,
				diag.magDistDetected ? "yes" : "no",
				diag.magRefNorm,
				diag.magRefDipDeg,
				diag.deltaDeg,
				diag.gyroBias[0],
				diag.gyroBias[1],
				diag.gyroBias[2]
			);
			logger.info(
				"Sensor[%d] mag offset: %.1f %.1f %.1f (%s%s)",
				sensor->getSensorId(),
				diag.magOffset[0],
				diag.magOffset[1],
				diag.magOffset[2],
				diag.magOffsetPerUnit ? "per-unit" : "default",
				diag.magAutoLearned ? ", learned this boot" : ""
			);
			// The field magnitude the fusion is being handed, against the 73-223
			// counts a geomagnetic field gives at this chip's 0.3 uT/LSB. A
			// magnitude outside that is not a reading of the Earth's field, so it
			// is withheld rather than allowed to steer the heading -- this line is
			// the answer to "why is the magnetometer not doing anything", in both
			// directions.
			logger.info(
				"Sensor[%d] mag field: %.0f counts (%.0f uT, %s), %u samples withheld",
				sensor->getSensorId(),
				diag.magFieldNorm,
				diag.magFieldNorm * 0.3f,
				diag.magFieldNorm >= 60.0f && diag.magFieldNorm <= 240.0f ? "plausible"
																		  : "rejected",
				static_cast<unsigned>(diag.magFieldRejected)
			);
			// The other half of the same test. A field of the right length can still
			// point nowhere near the geomagnetic field, and direction is the half
			// that steers the heading, so this line is the answer to "the magnitude
			// reads plausible, why is the magnetometer still not doing anything".
			// 67 deg is this latitude's inclination and 22 deg the margin the gate
			// allows it; both are in MagDipNominalDeg/MagDipToleranceDeg. A value
			// stuck near the horizontal wherever the tracker is carried says the
			// error is in the tracker, not in the room.
			logger.info(
				"Sensor[%d] mag inclination: %.0f deg (%s, %u samples), nominal 67 +- 22",
				sensor->getSensorId(),
				diag.magDipDeg,
				fabsf(diag.magDipDeg - 67.0f) <= 22.0f ? "plausible" : "rejected",
				static_cast<unsigned>(diag.magDipRejected)
			);
			// What the poll costs against the FIFO it shares the motion loop with,
			// and the rate the chip is actually producing against the timestep the
			// fusion was compiled with (MagTs = 1/20).
			logger.info(
				"Sensor[%d] mag poll: %.0f us avg, %.0f us max, %u failed, %.1f Hz samples",
				sensor->getSensorId(),
				diag.magPollUs,
				diag.magPollMaxUs,
				static_cast<unsigned>(diag.magPollFailures),
				diag.magRateHz
			);
			// Progress of the on-device hard-iron capture. Reported whether or
			// not it is ready, because the answer to "why won't it apply" is
			// always in these three numbers. This capture is the same one the
			// tracker learns from on its own, so "keep turning" here is also the
			// answer to why an uncalibrated tracker is still reading the
			// compiled-in offset.
			MagCalibration magCal;
			if (sensor->getMagCalibration(magCal)) {
				logger.info(
					"Sensor[%d] mag cal: %d samples, %d/64 directions, spans %.0f %.0f %.0f (%s)",
					sensor->getSensorId(),
					magCal.samples,
					magCal.bins,
					magCal.span[0],
					magCal.span[1],
					magCal.span[2],
					diag.magAutoCandidatePending  ? "fit held, awaiting a second one"
					: magCal.valid && magCal.improves ? "ready"
													  : "keep turning"
				);
			}
			// In raw counts, so it can be read straight against the two stored
			// calibration points the firmware logs at boot.
			logger.info(
				"Sensor[%d] gyro %.2fC, offset applied %.2f %.2f %.2f counts",
				sensor->getSensorId(),
				diag.gyroTemperatureC,
				diag.appliedGyroBias[0],
				diag.appliedGyroBias[1],
				diag.appliedGyroBias[2]
			);
		}
	}
	logger.info(
		"Battery voltage: %.3f, level: %.1f%%",
		battery.getVoltage(),
		battery.getLevel() * 100
	);
	// Every sensor shares one Wire bus; each swapIn() to a different pin pair
	// tears it down and starts it again. A count that stays at its boot value
	// means nothing is thrashing the bus.
	logger.info("I2C bus rebuilds: %u", static_cast<unsigned>(SlimeVR::i2cBusRebuilds));
}

#ifdef ESP32
String getEncryptionTypeName(wifi_auth_mode_t type) {
	switch (type) {
		case WIFI_AUTH_OPEN:
			return "OPEN";
		case WIFI_AUTH_WEP:
			return "WEP";
		case WIFI_AUTH_WPA_PSK:
			return "WPA_PSK";
		case WIFI_AUTH_WPA2_PSK:
			return "WPA2_PSK";
		case WIFI_AUTH_WPA_WPA2_PSK:
			return "WPA_WPA2_PSK";
		case WIFI_AUTH_WPA2_ENTERPRISE:
			return "WPA2_ENTERPRISE";
		case WIFI_AUTH_WPA3_PSK:
			return "WPA3_PSK";
		case WIFI_AUTH_WPA2_WPA3_PSK:
			return "WPA2_WPA3_PSK";
		case WIFI_AUTH_WAPI_PSK:
			return "WAPI_PSK";
		case WIFI_AUTH_WPA3_ENT_192:
			return "WPA3_ENT_192";
	}
#else
String getEncryptionTypeName(uint8_t type) {
	switch (type) {
		case ENC_TYPE_NONE:
			return "OPEN";
		case ENC_TYPE_WEP:
			return "WEP";
		case ENC_TYPE_TKIP:
			return "WPA_PSK";
		case ENC_TYPE_CCMP:
			return "WPA2_PSK";
		case ENC_TYPE_AUTO:
			return "WPA_WPA2_PSK";
	}
#endif
	return "UNKNOWN";
}

void cmdGet(CmdParser* parser) {
	if (parser->getParamCount() < 2) {
		return;
	}

	if (parser->equalCmdParam(1, "INFO")) {
		printState();

		// We don't want to print this on every timed state output
		logger.info("Git commit: %s", GIT_REV);
	}

	if (parser->equalCmdParam(1, "CONFIG")) {
		Serial.printf(sSlVRPrInfo);
	}

	if (parser->equalCmdParam(1, "MAGREG")) {
		getMagRegister(parser);
	}

	if (parser->equalCmdParam(1, "TEST")) {
		logger.info(
			"[TEST] Board: %d, hardware: %d, protocol: %d, firmware: %s, address: %s, "
			"mac: %s, status: %d, wifi state: %d",
			BOARD,
			HARDWARE_MCU,
			PROTOCOL_VERSION,
			FIRMWARE_VERSION,
			wifiNetwork.getAddress().toString().c_str(),
			WiFi.macAddress().c_str(),
			statusManager.getStatus(),
			static_cast<int>(wifiNetwork.getWiFiState())
		);
		auto& sensor0 = sensorManager.getSensors()[0];
		sensor0->motionLoop();
		logger.info(
			"[TEST] Sensor[0]: %s (%.7f %.7f %.7f %.7f) is working: %s, had data: %s",
			getIMUNameByType(sensor0->getSensorType()),
			UNPACK_QUATERNION(sensor0->getFusedRotation()),
			sensor0->isWorking() ? "true" : "false",
			sensor0->getHadData() ? "true" : "false"
		);

		const char* mag = sensor0->getAttachedMagnetometer();
		if (mag) {
			logger.info("[TEST] Sensor[0] magnetometer: %s", mag);
		} else {
			logger.info("[TEST] Sensor[0] has no magnetometer attached");
		}

		if (!sensor0->getHadData()) {
			logger.error("[TEST] Sensor[0] didn't send any data yet!");
		} else {
			logger.info("[TEST] Sensor[0] sent some data, looks working.");
		}
	}

	if (parser->equalCmdParam(1, "WIFISCAN")) {
		// Which network the tracker would aim at, next to what the radio can
		// hear: the scan alone cannot show a stored SSID that is a byte off, and
		// that looks identical to an access point that is out of range.
		logger.info(
			"[WSCAN] Credentials in use: saved '%s' / hardcoded '%s'",
			wifiNetwork.getSSID().c_str(),
#if defined(WIFI_CREDS_SSID)
			WIFI_CREDS_SSID
#else
			"(none)"
#endif
		);
		logger.info("[WSCAN] Scanning for WiFi networks...");

		// Scan would fail if connecting, stop connecting before scan
		if (WiFi.status() != WL_CONNECTED) {
			WiFi.disconnect();
		}
		if (wifiProvisioning.isProvisioning()) {
			wifiProvisioning.stopProvisioning();
		}

		WiFi.scanNetworks();

		int scanRes = WiFi.scanComplete();
		if (scanRes >= 0) {
			logger.info("[WSCAN] Found %d networks:", scanRes);
			// The columns are named because this listing is read against the
			// credentials line above it, and a bare number between the index and
			// the name invites a wrong reading -- which is what happened to the
			// SSID length that used to be printed here, and was read as a channel.
			logger.info("[WSCAN] idx\tchannel\trssi\tname\tencryption");
			for (int i = 0; i < scanRes; i++) {
				logger.info(
					"[WSCAN] %d:\t%d\t%d dBm\t'%s'\t%s",
					i,
					WiFi.channel(i),
					WiFi.RSSI(i),
					WiFi.SSID(i).c_str(),
					getEncryptionTypeName(WiFi.encryptionType(i)).c_str()
				);
			}
			WiFi.scanDelete();
		} else {
			logger.info("[WSCAN] Scan failed!");
		}

		// Restore conencting state
		if (WiFi.status() != WL_CONNECTED) {
			// Only worth asking for if there is an SSID to ask with: with an
			// empty station config this fails with ESP_ERR_WIFI_SSID, which
			// reads as a scan problem and is not one.
			if (wifiNetwork.getSSID().length() > 0) {
				WiFi.begin();
			} else {
				logger.info("[WSCAN] No stored SSID, leaving the WiFi state alone");
			}
		}
	}
}

void cmdReboot(CmdParser* parser) {
	logger.info("REBOOT");
	ESP.restart();
}

void cmdFactoryReset(CmdParser* parser) {
	logger.info("FACTORY RESET");
	configuration.factoryReset();
	// No return here factoryReset will reboot the tracker.
}

void cmdTemperatureCalibration(CmdParser* parser) {
	if (parser->getParamCount() > 1) {
		if (parser->equalCmdParam(1, "PRINT")) {
			for (auto& sensor : sensorManager.getSensors()) {
				sensor->printTemperatureCalibrationState();
			}
			return;
		} else if (parser->equalCmdParam(1, "DEBUG")) {
			for (auto& sensor : sensorManager.getSensors()) {
				sensor->printDebugTemperatureCalibrationState();
			}
			return;
		} else if (parser->equalCmdParam(1, "RESET")) {
			for (auto& sensor : sensorManager.getSensors()) {
				sensor->resetTemperatureCalibrationState();
			}
			return;
		} else if (parser->equalCmdParam(1, "SAVE")) {
			for (auto& sensor : sensorManager.getSensors()) {
				sensor->saveTemperatureCalibration();
			}
			return;
		}
	}
	logger.info("Usage:");
	logger.info("  TCAL PRINT: print current temperature calibration config");
	logger.info(
		"  TCAL DEBUG: print debug values for the current temperature calibration "
		"profile"
	);
	logger.info(
		"  TCAL RESET: reset current temperature calibration in RAM (does not delete "
		"already saved)"
	);
	logger.info("  TCAL SAVE: save current temperature calibration to persistent flash"
	);
	logger.info("Note:");
	logger.info(
		"  Temperature calibration config saves automatically when calibration percent "
		"is at 100%%"
	);
}

void cmdDeleteCalibration(CmdParser* parser) {
	logger.info("ERASE CALIBRATION");

	configuration.eraseSensors();
}

#if EXT_SERIAL_COMMANDS
void cmdScanI2C(CmdParser* parser) {
	logger.info("Forcing I2C scan...");
	I2CSCAN::scani2cports();
}
#endif

void setUp() {
	cmdCallbacks.addCmd("SET", &cmdSet);
	cmdCallbacks.addCmd("GET", &cmdGet);
	cmdCallbacks.addCmd("FRST", &cmdFactoryReset);
	cmdCallbacks.addCmd("REBOOT", &cmdReboot);
	cmdCallbacks.addCmd("DELCAL", &cmdDeleteCalibration);
	cmdCallbacks.addCmd("TCAL", &cmdTemperatureCalibration);
#if EXT_SERIAL_COMMANDS
	cmdCallbacks.addCmd("SCANI2C", &cmdScanI2C);
#endif
}

void update() { cmdCallbacks.updateCmdProcessing(&cmdParser, &cmdBuffer, &Serial); }
}  // namespace SerialCommands
