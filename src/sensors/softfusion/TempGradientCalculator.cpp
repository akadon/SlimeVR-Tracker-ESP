#include "TempGradientCalculator.h"

#include <functional>

TemperatureGradientCalculator::TemperatureGradientCalculator(
	const std::function<void(float gradient)>& callback
)
	: callback{callback} {}

void TemperatureGradientCalculator::feedSample(float sample, float timeStep) {
	tempSum += sample * timeStep;
}

void TemperatureGradientCalculator::tick() {
	uint32_t now = millis();

	// Unsigned subtraction, so this stays correct across the millis() wrap.
	if (now - lastAverageSentMillis
		< static_cast<uint32_t>(AveragingTimeSeconds * 1e3)) {
		return;
	}

	// Restart the averaging window from now rather than from a fixed schedule, so a
	// stall (or the wrap) does not leave a backlog of periods to fire off at once.
	lastAverageSentMillis = now;
	float average = tempSum / AveragingTimeSeconds;
	callback((average - lastTempAverage) / AveragingTimeSeconds);
	lastTempAverage = average;
	tempSum = 0;
}
