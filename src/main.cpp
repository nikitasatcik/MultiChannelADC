#include <Arduino.h>

#include <MultiChannelADC.h>

unsigned long start_time, stop_time;

MultiChannelADC adc;


void setup()
{
  Serial.begin(115200);
  adc.init();
}

void loop()
{
  start_time = micros();
  adc.ADC_read();
  stop_time = micros() - start_time;
  Serial.print("Elapsed time, us:");
  Serial.println(stop_time);
  delay(100);
  adc.ADC_print_data();
  Serial.println("selected channel");
  Serial.println(adc.ADC_get_channel_value(adc.ADC_CHANNEL_1));
}