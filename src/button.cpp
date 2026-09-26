#include "button.h"

#include <climits>

#if ESP32
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#endif

#include "GlobalVars.h"

#ifdef ON_OFF_BUTTON_PIN

void IRAM_ATTR buttonInterruptHandler() {
	// The button only powers the tracker on: the ESP32 wakes from deep sleep on
	// the GPIO edge and the firmware boots. Everything after that is handled by
	// OnOffButton::tick(), which powers the tracker back off once SlimeVR has
	// been gone for BUTTON_DISCONNECT_SLEEP_SECONDS.
}

void OnOffButton::setup() {
#if ESP8266
	digitalWrite(D0, LOW);
	pinMode(D0, OUTPUT);
	pinMode(ON_OFF_BUTTON_PIN, INPUT);
#endif

#if ESP32
	pinMode(
		ON_OFF_BUTTON_PIN,
		BUTTON_ACTIVE_LEVEL == 0 ? INPUT_PULLUP : INPUT_PULLDOWN
	);

	esp_deep_sleep_enable_gpio_wakeup(
		1 << ON_OFF_BUTTON_PIN,
		BUTTON_ACTIVE_LEVEL == 0 ? ESP_GPIO_WAKEUP_GPIO_LOW : ESP_GPIO_WAKEUP_GPIO_HIGH
	);

#ifdef BUTTON_IMU_ENABLE_PIN
	pinMode(BUTTON_IMU_ENABLE_PIN, OUTPUT);
	gpio_hold_dis(static_cast<gpio_num_t>(BUTTON_IMU_ENABLE_PIN));
	digitalWrite(BUTTON_IMU_ENABLE_PIN, BUTTON_IMU_ENABLE_ACTIVE_LEVEL);
#endif

	gpio_deep_sleep_hold_en();
#endif

	attachInterrupt(
		ON_OFF_BUTTON_PIN,
		buttonInterruptHandler,
		BUTTON_ACTIVE_LEVEL == 0 ? FALLING : RISING
	);
}

void OnOffButton::tick() {
#if defined(BUTTON_BATTERY_VOLTAGE_THRESHOLD) && BATTERY_MONITOR == BAT_EXTERNAL
	if (battery.getVoltage() >= BUTTON_BATTERY_VOLTAGE_THRESHOLD) {
		batteryBad = false;
	} else if (!batteryBad) {
		batteryBad = true;
		batteryBadSinceMillis = millis();
	}

	if (batteryBad
		&& millis() - batteryBadSinceMillis >= batteryBadTimeoutSeconds * 1e3) {
		goToSleep("battery too low");
	}
#endif

#ifdef BUTTON_DISCONNECT_SLEEP_SECONDS
	// Start the countdown on the connected -> disconnected edge, never on the
	// disconnection check itself: resetting every loop would keep the timer at
	// zero and the tracker would never sleep.
	//
	// The flag has to be cleared again while connected, or it latches on the first
	// disconnection the tracker ever sees and `disconnectedSinceMillis` keeps that
	// one timestamp for the rest of the session -- so the next missed packet, which
	// is all a server restart or a brief dropout looks like, finds the countdown
	// already expired and sleeps the tracker on the spot.
	if (connected) {
		wasDisconnected = false;
	} else if (!wasDisconnected) {
		wasDisconnected = true;
		disconnectedSinceMillis = millis();
	} else if (millis() - disconnectedSinceMillis
			   >= BUTTON_DISCONNECT_SLEEP_SECONDS * 1e3) {
		goToSleep("no SlimeVR connection");
	}
#endif

	// The button reads as pressed for a moment after the tracker wakes, because
	// it is the same press that powered it on. Wait for the release before arming
	// anything, so that waking cannot immediately be read as another click.
	const bool pressed = getButton();
	if (!wasReleasedInitially) {
		if (!pressed) {
			wasReleasedInitially = true;
		}
		return;
	}

	// Nothing to do on a click: the tracker is already on, and it is the
	// disconnect timeout above that decides when it goes back off.

	// A press held past holdSeconds runs the hold gesture instead, once per
	// press. Released and pressed again is a fresh press, and so a fresh
	// gesture.
	if (pressed) {
		if (!holding) {
			holding = true;
			holdFired = false;
			holdStartedMillis = millis();
		} else if (!holdFired && millis() - holdStartedMillis >= holdSeconds * 1e3) {
			holdFired = true;
			emitHold();
		}
	} else {
		holding = false;
		holdFired = false;
	}

	// Only a live report from this tick counts as being connected, so that a
	// dropped connection is noticed on the very next one.
	connected = false;
}

void OnOffButton::onBeforeSleep(std::function<void()> callback) {
	callbacks.push_back(callback);
}

void OnOffButton::onHold(std::function<void()> callback) {
	holdCallbacks.push_back(callback);
}

void OnOffButton::signalTrackerConnected() { connected = true; }

OnOffButton& OnOffButton::getInstance() { return instance; }

bool OnOffButton::getButton() {
	static constexpr uint8_t circularBufferBitCount
		= sizeof(buttonCircularBuffer) * CHAR_BIT;

	bool isPressed = digitalRead(ON_OFF_BUTTON_PIN) == BUTTON_ACTIVE_LEVEL;
	buttonCircularBuffer = buttonCircularBuffer << 1 | isPressed;

	auto popCount = __builtin_popcount(buttonCircularBuffer);
	return popCount >= circularBufferBitCount / 2;
}

void OnOffButton::emitOnBeforeSleep() {
	for (auto& callback : callbacks) {
		callback();
	}
}

void OnOffButton::emitHold() {
	for (auto& callback : holdCallbacks) {
		callback();
	}
}

void OnOffButton::goToSleep(const char* reason) {
	printf("Going to sleep: %s\n", reason);

	emitOnBeforeSleep();

#if defined(BUTTON_IMU_ENABLE_PIN) && ESP32
	digitalWrite(BUTTON_IMU_ENABLE_PIN, LOW);
	gpio_hold_en(static_cast<gpio_num_t>(BUTTON_IMU_ENABLE_PIN));
#endif

	// Three quick blinks, so that powering down is visibly deliberate rather
	// than the tracker appearing to just die. This blocks, and pattern() leaves
	// the LED off after its final blink, so the tracker goes dark before the
	// power is cut.
	static constexpr uint32_t flashOnMillis = 100;
	static constexpr uint32_t flashOffMillis = 100;
	static constexpr int flashCount = 3;

	ledManager.pattern(flashOnMillis, flashOffMillis, flashCount);

#if ESP8266
	ESP.deepSleep(0);
#elif ESP32
	esp_deep_sleep_start();
#endif
}

OnOffButton OnOffButton::instance;

#endif
