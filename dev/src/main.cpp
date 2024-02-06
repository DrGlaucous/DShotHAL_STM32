

#include <Arduino.h>
#include <dshot.h>


unsigned long MillisecondTicks{};
unsigned long MicrosecondTicks{};
unsigned long LastMillisecondTicks{};//previous values
unsigned long LastMicrosecondTicks{};


//System clock
void GetTicks(void)
{
	LastMillisecondTicks = MillisecondTicks;
	LastMicrosecondTicks = MicrosecondTicks;

	MillisecondTicks = millis();
	MicrosecondTicks = micros();

}

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);

}


void loop() {

    static bool LED_STATE = false;

    digitalWrite(LED_BUILTIN, LED_STATE);
    LED_STATE != LED_STATE;

    delay(500);



}

