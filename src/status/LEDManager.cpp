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

#include "LEDManager.h"

#include "../GlobalVars.h"

namespace SlimeVR {
namespace {
// Half-period of the blink shown while the tracker is powered but not connected.
constexpr unsigned long LED_BLINK_MS = 500;
}  // namespace

void LEDManager::setup() {
	if (m_Enabled) {
		pinMode(m_Pin, OUTPUT);
	}

	// Do the initial pull of the state
	update();
}

void LEDManager::on() {
	if (m_Enabled) {
		digitalWrite(m_Pin, m_On);
	}
}

void LEDManager::off() {
	if (m_Enabled) {
		digitalWrite(m_Pin, m_Off);
	}
}

void LEDManager::blink(unsigned long time) {
	on();
	delay(time);
	off();
}

void LEDManager::pattern(unsigned long timeon, unsigned long timeoff, int times) {
	for (int i = 0; i < times; i++) {
		blink(timeon);
		delay(timeoff);
	}
}

void LEDManager::setOwned(bool owned) { m_Owned = owned; }

void LEDManager::update() {
	// The LED says whether SlimeVR has this tracker: a steady glow once the
	// server handshake has gone through, a blink while the tracker is on but has
	// not got there yet -- still searching, or dropped off. Dark means it is not
	// running.
	//
	// It reports the server rather than WiFi, because the server connection is
	// what the tracker is for: a tracker with the network but not the server
	// tracks nothing, and glowing for it would read as working.
	//
	// Written unconditionally rather than only when the state changes, because
	// the calibration routines drive the LED directly through on()/off(). A
	// change-only write would leave whatever they last set in place.
	//
	// While one of those routines owns the LED it is showing something with it,
	// and repainting from here would put the two meanings on the same light at
	// the same time.
	if (m_Owned) {
		return;
	}

	if (networkConnection.isConnected()) {
		// Leave the blink starting from a lit phase, so the tracker is seen to
		// be blinking as soon as the server drops it rather than sitting dark
		// for half a period first.
		m_BlinkLit = false;
		m_LastBlinkToggle = millis();
		on();
		return;
	}

	// millis() based rather than blink()/pattern(), which delay(): update() runs
	// once per loop, and blocking here would stall the tracker's main loop.
	const unsigned long now = millis();
	if (now - m_LastBlinkToggle >= LED_BLINK_MS) {
		m_LastBlinkToggle = now;
		m_BlinkLit = !m_BlinkLit;
		if (m_BlinkLit) {
			on();
		} else {
			off();
		}
	}
}
}  // namespace SlimeVR
