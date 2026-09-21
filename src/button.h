/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors

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

#include <Arduino.h>

#include <cstdint>
#include <functional>
#include <vector>

#include "GlobalVars.h"

#ifdef ON_OFF_BUTTON_PIN

class OnOffButton {
public:
	void setup();
	void tick();
	void onBeforeSleep(std::function<void()> callback);

	// Runs once per press, at the moment the press has been held for
	// holdSeconds. A click does nothing on this tracker -- it is already on --
	// so the hold is free to mean something else, and it is long enough that
	// putting a tracker on cannot set it off.
	void onHold(std::function<void()> callback);

	// Resets the disconnect timer. Call this on every loop where the tracker is
	// connected to SlimeVR; it powers itself off once it has been without a
	// connection for BUTTON_DISCONNECT_SLEEP_SECONDS.
	void signalTrackerConnected();

	static OnOffButton& getInstance();

private:
	OnOffButton() = default;
	static OnOffButton instance;

	// How long the button has to be held to run the hold gesture.
	static constexpr float holdSeconds = 5.0f;

	// How long the battery may sit below BUTTON_BATTERY_VOLTAGE_THRESHOLD before
	// the tracker gives up on it.
	static constexpr float batteryBadTimeoutSeconds = 10.0f;

	bool getButton();
	void emitOnBeforeSleep();
	void emitHold();
	void goToSleep(const char* reason);

	uint64_t buttonCircularBuffer = 0;
	std::vector<std::function<void()>> callbacks;
	std::vector<std::function<void()>> holdCallbacks;
	// Press tracking for the hold above. `holdFired` keeps one long press to one
	// gesture: the button stays down for as long as the calibration it started
	// takes, and re-firing would restart it over and over.
	bool holding = false;
	bool holdFired = false;
	uint64_t holdStartedMillis = 0;
	bool batteryBad = false;
	uint64_t batteryBadSinceMillis = 0;
	bool wasReleasedInitially = false;

	uint64_t disconnectedSinceMillis = 0;
	// Starts true so that a tracker which boots and never reaches SlimeVR still
	// powers itself back off instead of draining itself flat waiting.
	bool wasDisconnected = true;
	// Whether SlimeVR has been heard from since the last tick.
	bool connected = false;

	friend void IRAM_ATTR buttonInterruptHandler();
};

#endif
