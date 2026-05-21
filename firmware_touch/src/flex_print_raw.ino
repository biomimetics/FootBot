
#include <Trill.h>

Trill trillSensor;

void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	int ret;
	while((ret = trillSensor.setup(Trill::TRILL_FLEX))) {
		Serial.println("failed to initialise trillSensor");
		Serial.print("Error code: ");
		Serial.println(ret);
	}
  // when the slider is connected we increase the
  // prescaler to deal with the increased baseline
  // capacitance it brings
  trillSensor.setPrescaler(3);
  delay(10);
  trillSensor.setNoiseThreshold(0);
  delay(10);
  // after any prescaler change, it's always good to update
  // the baseline, too.
  trillSensor.updateBaseline();
}

void loop() {
	// put your main code here, to run repeatedly:
	delay(100);
	trillSensor.requestRawData();

	while(trillSensor.rawDataAvailable() > 0) {
		int data = trillSensor.rawDataRead();
		if(data < 1000)
			Serial.print(0);
		if(data < 100)
			Serial.print(0);
		if(data < 10)
			Serial.print(0);
		Serial.print(data);
		Serial.print(" ");
	}
	Serial.println("");
}