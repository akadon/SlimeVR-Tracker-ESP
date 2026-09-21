/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

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

#include "MagCalibrationMode.h"

#include <Arduino.h>

#include "GlobalVars.h"
#include "SensorToggles.h"
#include "logging/Logger.h"

namespace SlimeVR {

MagCalibrationMode MagCalibrationMode::instance;

MagCalibrationMode& MagCalibrationMode::getInstance() { return instance; }

namespace {
Logging::Logger logger("MagCal");
}

void MagCalibrationMode::toggle() {
	if (m_Capturing) {
		cancel();
		return;
	}

	start();
}

void MagCalibrationMode::start() {
	auto& sensors = sensorManager.getSensors();
	if (sensors.empty()) {
		fail("no sensors to calibrate");
		return;
	}

	Sensor* sensor = sensors[0].get();
	if (!sensor->isFlagSupported(SensorToggles::MagEnabled)) {
		fail("this tracker has no magnetometer");
		return;
	}
	// Without the toggle the sensor does not read the chip at all, so there
	// would be nothing to collect. Turning it on here instead would be a
	// promise the firmware cannot keep: SlimeVR sets it back from its own
	// configuration on the next change.
	if (!sensor->getFlagState(SensorToggles::MagEnabled)) {
		fail("the magnetometer is switched off in SlimeVR");
		return;
	}

	sensor->resetMagCalibration();

	m_Capturing = true;
	m_StartedMillis = millis();
	ledManager.setOwned(true);

	logger.info(
		"magnetometer calibration: turn the tracker through every orientation for %u s",
		static_cast<unsigned>(WindowMillis / 1000)
	);
}

void MagCalibrationMode::cancel() {
	m_Capturing = false;
	ledManager.setOwned(false);
	ledManager.update();
	logger.info("magnetometer calibration cancelled, nothing stored");
}

void MagCalibrationMode::tick() {
	if (!m_Capturing) {
		return;
	}

	const uint32_t now = millis();
	if (now - m_StartedMillis < WindowMillis) {
		blink(now);
		return;
	}

	auto& sensors = sensorManager.getSensors();
	if (sensors.empty()) {
		fail("the sensor went away");
		return;
	}

	closeWindow(*sensors[0].get());
}

void MagCalibrationMode::closeWindow(Sensor& sensor) {
	MagCalibration fit;
	if (!sensor.getMagCalibration(fit)) {
		fail("this tracker has no magnetometer");
		return;
	}

	logger.info(
		"magnetometer calibration window closed: %d samples over %d/64 directions, "
		"spans %.0f %.0f %.0f counts",
		fit.samples,
		fit.bins,
		fit.span[0],
		fit.span[1],
		fit.span[2]
	);

	// The same gate `SET MAGCAL APPLY` uses -- and the same one the tracker applies
	// to itself, unattended, from the capture ordinary use fills. Coverage that
	// constrains a sphere, and a fit that beats the offset in use on its own
	// samples. One definition of a good enough capture, so the button, the console
	// and the automatic fit cannot disagree about what they are willing to store --
	// and a capture that fails it is not stored at all, because an offset fitted
	// from directions the tracker never reached is worse than the one it would
	// replace.
	if (!fit.valid) {
		fail("the capture did not cover enough orientations");
		return;
	}
	if (!fit.improves) {
		fail("the fit does not beat the offset in use on this capture");
		return;
	}

	logger.info(
		"magnetometer calibration fit: centre %.1f %.1f %.1f, |B| %.0f counts, "
		"residual %.1f counts rms against %.1f for the offset in use",
		fit.centre[0],
		fit.centre[1],
		fit.centre[2],
		fit.radius,
		fit.residualFitted,
		fit.residualInUse
	);

	if (!sensor.storeMagOffset(fit.centre)) {
		fail("nowhere to store the offset");
		return;
	}

	m_Capturing = false;
	logger.info(
		"magnetometer offset %.1f %.1f %.1f stored, rebooting to apply it",
		fit.centre[0],
		fit.centre[1],
		fit.centre[2]
	);

	// Three quick blinks, then dark, then the reboot. The offset only takes
	// effect from a boot, and a tracker that reboots with no warning reads as a
	// tracker that crashed.
	for (int i = 0; i < 3; i++) {
		ledManager.blink(100);
		delay(120);
	}
	ledManager.off();
	delay(400);

	ESP.restart();
}

void MagCalibrationMode::fail(const char* reason) {
	m_Capturing = false;
	logger.error("magnetometer calibration not stored: %s", reason);

	// Five short blinks: the wearer gets told the gesture did not take, and the
	// log gets the reason. Blocking, because the pattern is the point.
	for (int i = 0; i < 5; i++) {
		ledManager.blink(60);
		delay(120);
	}

	ledManager.setOwned(false);
	ledManager.update();
}

void MagCalibrationMode::blink(uint32_t now) {
	if ((now - m_StartedMillis) % BlinkPeriodMillis < BlinkOnMillis) {
		ledManager.on();
	} else {
		ledManager.off();
	}
}

}  // namespace SlimeVR
