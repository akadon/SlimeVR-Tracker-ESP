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

#pragma once

#include <cstdint>

#include "sensor.h"

namespace SlimeVR {

// The button-and-LED way to run the on-device hard-iron calibration, so that a
// tracker can be calibrated without a serial console.
//
// Holding the button for five seconds starts a capture window and the LED
// blinks for as long as it is open. Move the tracker around through every
// orientation while it blinks; at fifteen seconds the window closes on its own.
// If the capture is good enough to beat the compiled-in offset on its own
// samples, the fit is stored and the tracker reboots into it -- the offset only
// takes effect from a boot, because the running filter has already adopted a
// heading reference shaped by the old one. If the capture is not good enough to
// store, nothing is written and the LED says so.
//
// Measured over the sweeps in the capture files, a fifteen second window of
// real tumbling covers 34-42 of the 64 direction cells and 105-118 samples,
// while a window of the tracker sitting still covers as many cells out of its
// own noise but reaches 11-32 counts of field span instead of 90-400. The
// difference the acceptance tests read is that span: turning the tracker
// through large changes of orientation is what fills a capture, and small
// wiggles in place are what the tests turn away.
//
// Three quick blinks and a reboot: stored. Five quick blinks: not stored, and
// the log has the reason -- the capture did not constrain enough orientations,
// or the fit did not beat the compiled offset on it, or the magnetometer is
// switched off in SlimeVR, or this tracker has none. The same five second hold
// cancels a window that is running.
class MagCalibrationMode {
public:
	static MagCalibrationMode& getInstance();

	// Opens a capture window, or cancels the one that is open.
	void toggle();

	// Drives the LED and closes the window when its time is up. Called every
	// loop.
	void tick();

	[[nodiscard]] bool isActive() const { return m_Capturing; }

private:
	MagCalibrationMode() = default;
	static MagCalibrationMode instance;

	// How long the window stays open. The capture needs the tracker turned
	// through a spread of orientations, and this is long enough for that, short
	// enough that the gesture has a known end.
	static constexpr uint32_t WindowMillis = 15000;
	// The blink, at a fixed rate: it says the window is open, nothing more. The
	// outcome is the only thing the LED reports after it closes.
	static constexpr uint32_t BlinkPeriodMillis = 400;
	static constexpr uint32_t BlinkOnMillis = 100;

	void start();
	void cancel();
	void closeWindow(Sensor& sensor);
	void fail(const char* reason);
	void blink(uint32_t now);

	bool m_Capturing = false;
	uint32_t m_StartedMillis = 0;
};

}  // namespace SlimeVR
