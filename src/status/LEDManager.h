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
#ifndef SLIMEVR_LEDMANAGER_H
#define SLIMEVR_LEDMANAGER_H

#include <Arduino.h>

#include "../globals.h"
#include "../logging/Logger.h"

namespace SlimeVR {

class LEDManager {
public:
	void setup();

	/*!
	 *  @brief Turns the LED on
	 */
	void on();

	/*!
	 *  @brief Turns the LED off
	 */
	void off();

	/*!
	 *  @brief Blink the LED for [time]ms. *Can* cause lag
	 *  @param time Amount of ms to turn the LED on
	 */
	void blink(unsigned long time);

	/*!
	 *  @brief Show a pattern on the LED. *Can* cause lag
	 *  @param timeon Amount of ms to turn the LED on
	 *  @param timeoff Amount of ms to turn the LED off
	 *  @param times Amount of times to display the pattern
	 */
	void pattern(unsigned long timeon, unsigned long timeoff, int times);

	/*!
	 *  @brief Drives the LED from the connection state: lit for as long as
	 *  SlimeVR is connected, dark at every other moment.
	 */
	void update();

	/*!
	 *  @brief Hands the LED to a routine that drives it through on()/off().
	 *  While owned, update() leaves the LED alone rather than repainting it
	 *  from the connection state on every loop, which is what a routine that
	 *  runs across many loops instead of blocking needs.
	 */
	void setOwned(bool owned);

private:
	uint8_t m_Pin = LED_PIN;
	bool m_Enabled = m_Pin >= 0 && m_Pin < LED_OFF;
	bool m_On = LED_INVERTED ? LOW : HIGH;
	bool m_Off = !m_On;
	bool m_Owned = false;

	Logging::Logger m_Logger = Logging::Logger("LEDManager");
};
}  // namespace SlimeVR

#endif
