/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Eiren Rain & SlimeVR Contributors

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

#include "I2CWireSensorInterface.h"

#include <optional>

std::optional<uint8_t> activeSCLPin;
std::optional<uint8_t> activeSDAPin;
bool isI2CActive = false;

namespace SlimeVR {
// main.cpp brings Wire up on the board's IMU pins before any sensor exists, so
// by the time the first sensor calls swapIn() the bus is already live on the
// pins that sensor asks for. Recording that here keeps the first swapIn() from
// tearing down and rebuilding a bus that is already in use -- the teardown is
// both pointless and the one operation on this bus that can leave the driver's
// semaphores and event queue behind in a bad state.
void markI2CActive(uint8_t sclPin, uint8_t sdaPin) {
	activeSCLPin = sclPin;
	activeSDAPin = sdaPin;
	isI2CActive = true;
}

// Counts teardowns and restarts of the bus. swapIn() runs every loop iteration,
// so this is not logged as it happens: it belongs in GET INFO, where it is read
// on demand and a number that keeps climbing is the signal that something is
// re-pointing a bus it does not need to.
uint32_t i2cBusRebuilds = 0;

void swapI2C(uint8_t sclPin, uint8_t sdaPin) {
	if (sclPin != activeSCLPin || sdaPin != activeSDAPin || !isI2CActive) {
		i2cBusRebuilds++;
		Wire.flush();
#ifdef ESP32
		// Landing the bus on a different pair of pins means tearing it down and
		// starting it again: the driver_ng API behind Wire has no re-point call,
		// and Wire.begin() deliberately refuses to touch a bus that is already
		// up. The legacy i2c_set_pin() that used to serve the second case
		// cannot be used here at all -- referencing it drags the legacy driver
		// into the link, and that driver's startup constructor aborts the whole
		// boot on ESP-IDF 5.x the moment anything else uses driver_ng, which is
		// what arduino-esp32 3.x's Wire does. Wire.end() is a no-op when the
		// bus is not up, so this covers both cases.
		Wire.end();

		if (activeSCLPin && activeSCLPin) {
			// Disconnect pins from HWI2C
			gpio_set_direction((gpio_num_t)*activeSCLPin, GPIO_MODE_INPUT);
			gpio_set_direction((gpio_num_t)*activeSDAPin, GPIO_MODE_INPUT);
		}

		Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin), I2C_SPEED);
		Wire.setTimeOut(150);
#else
		Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin));
#endif

		activeSCLPin = sclPin;
		activeSDAPin = sdaPin;
		isI2CActive = true;
	}
}

void disconnectI2C() {
	Wire.flush();
	isI2CActive = false;
#ifdef ESP32
	Wire.end();
#endif
}
}  // namespace SlimeVR
