#include <Swerve/DoublePot.h>

double DoublePot::PIDGet() {
	double originalReading = AnalogPotentiometer::PIDGet();
	double delta = originalReading - lastValue;
	double range = max - min;
	double sum = 0;

	if (delta > 300) {
		originalReading = lastValue;
	}

	lastValue = originalReading;
	originalReading += offset;

	if (originalReading > max) {
		originalReading -= range;
	}

	if (originalReading < min) {
		originalReading += range;
	}

	for (int i = OVERSAMPLE - 1; i > 0; i--) {
		data[i - 1] = data[i];
	}

	data[OVERSAMPLE - 1] = originalReading;
	oversampleAccumulator++;

	char buff[1024];
	char temp[32];
	sprintf(buff, "Oversample");

	if (oversampleAccumulator >= OVERSAMPLE) {
		for (int i = 0; i < OVERSAMPLE; i++) {
			sum += data[i];
			sprintf(temp, "[%i]=%f,", i, data[i]);
			strcat(buff, temp);
		}

		return (sum / OVERSAMPLE);
	}

	return originalReading;
}

double DoublePot::GetAverage() {
	double sum = 0;

	for (int i = 0; i < OVERSAMPLE; i++) {
		sum += data[i];
	}

	return (sum / OVERSAMPLE);
}
