/*
  Multichannel interrupted-driven ADC library for Arduino.
  Each channel is independant and can asynchronously read ADC value.
  Median filter (Phil Ekstrom’s method) is used for impulse noise reduction.
  Circular buffer is used to store ADC values from 8 channels.

  Copyright (c) 2022 Kolchiba Mykyta.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MULTICHANNELADC_H
#define MULTICHANNELADC_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "MedianFilterLib.h"
#include <CircularBuffer.h>

#define ADC_NOISE_VALUE 20

// Define the ADC channels used.  ADLAR will be zero.
#define ADCH0 ((1 << REFS0) | 0)
#define ADCH1 ((1 << REFS0) | 1)
#define ADCH2 ((1 << REFS0) | 2)
#define ADCH3 ((1 << REFS0) | 3)
#define ADCH4 ((1 << REFS0) | 4)
#define ADCH5 ((1 << REFS0) | 5)
#define ADCH6 ((1 << REFS0) | 6)
#define ADCH7 ((1 << REFS0) | 7)

enum adc_filter_size
{
  filter_size_5 = 5,  // runtime = 2 ms
  filter_size_7 = 7,  // runtime = 3 ms
  filter_size_9 = 9,  // runtime = 4.3 ms
  filter_size_11 = 11 // runtime = 5.8 ms
};

// Select buffer size according to the size of filter
const int buffer_size = filter_size_5;
MedianFilter<int> medianFilter(buffer_size);

CircularBuffer<int, buffer_size> ADC0_buffer;
CircularBuffer<int, buffer_size> ADC1_buffer;
CircularBuffer<int, buffer_size> ADC2_buffer;
CircularBuffer<int, buffer_size> ADC3_buffer;
CircularBuffer<int, buffer_size> ADC4_buffer;
CircularBuffer<int, buffer_size> ADC5_buffer;
CircularBuffer<int, buffer_size> ADC6_buffer;
CircularBuffer<int, buffer_size> ADC7_buffer;

// Number of samples for the first each channel.
uint16_t ADC_channel_samples;
volatile int ADC_channel;
volatile int adc_output[8];

class MultiChannelADC
{
public:
  enum adc_channel
  {
    ADC_CHANNEL_0 = 0,
    ADC_CHANNEL_1 = 1,
    ADC_CHANNEL_2 = 2,
    ADC_CHANNEL_3 = 3,
    ADC_CHANNEL_4 = 4,
    ADC_CHANNEL_5 = 5,
    ADC_CHANNEL_6 = 6,
    ADC_CHANNEL_7 = 7
  };
  static int buff_size;
  void init() const;
  void ADC_read();
  void ADC_print_data();
  // void ADC_set_filter_size(int size);
  int ADC_get_channel_value(const enum adc_channel adc_ch);
};

// void MultiChannelADC::ADC_set_filter_size(int size){
//   buff_size = size;
// }

void MultiChannelADC::init() const
{
  for (int i = 0; i < 8; i++)
    digitalWrite(A0 + i, HIGH);                                                     // internal pull-up for ADC pins
  ADCSRA |= (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0) | (1 << ADEN); // Prescale 64. Time = 60us  Freq = 19ksps.
  ADCSRB = 0;                                                                       // Clear register.
  _delay_ms(100);                                                                   // Enabling ADC takes 13 ADC clocks to execute
  sei();                                                                            // Enable global interrupts.
  ADCSRA |= (1 << ADSC);                                                            // Set ADSC to start ADC conversion.
}

// helper to read raw data form ADC channel
int read_channel(int raw_val)
{
  raw_val = ADCL | (ADCH << 8);
  return raw_val < ADC_NOISE_VALUE ? 0 : raw_val;
}

ISR(ADC_vect) // ADC interrupt in single conversion mode
{
  ADC_channel = ADMUX & 7; // Select ADC inputs: A0-A7
  ADMUX = ADCH0;           // Select ADC Channel 0.
  volatile int raw_val;
  switch (ADC_channel)
  {
  case 0:
    ADC0_buffer.push(read_channel(raw_val));
    ADC_channel_samples++; // Count ADC samples from A0, because other channels will have same or higher number of samples.
    ADMUX = ADCH1;         // Select ADC Channel 1.
    break;
  case 1:
    ADC1_buffer.push(read_channel(raw_val));
    ADMUX = ADCH2; // Select ADC Channel 2.
    break;
  case 2:
    ADC2_buffer.push(read_channel(raw_val));
    ADMUX = ADCH3; // Select ADC channel 3.
    break;
  case 3:
    ADC3_buffer.push(read_channel(raw_val));
    ADMUX = ADCH4; // Select ADC channel 4.
    break;
  case 4:
    ADC4_buffer.push(read_channel(raw_val));
    ADMUX = ADCH5; // Select ADC channel 5.
    break;
  case 5:
    ADC5_buffer.push(read_channel(raw_val));
    ADMUX = ADCH6; // Select ADC channel 6.
    break;
  case 6:
    ADC6_buffer.push(read_channel(raw_val));
    ADMUX = ADCH7; // Select ADC channel 7.
    break;
  case 7:
    ADC7_buffer.push(read_channel(raw_val));
    ADMUX = ADCH0; // Select ADC channel 0.
    break;
  }
  // Set ADSC to start the next ADC conversion.
  ADCSRA |= (1 << ADSC);
}

void MultiChannelADC::ADC_read()
{
  if (ADC_channel_samples >= buffer_size)
  {
    using index_t = decltype(ADC0_buffer)::index_t;
    for (index_t i = 0; i < buffer_size; i++)
    {
      medianFilter.AddValue(ADC0_buffer[i]);
    }
    adc_output[0] = medianFilter.GetFiltered();

    for (index_t i = 0; i < buffer_size; i++)
    {
      medianFilter.AddValue(ADC1_buffer[i]);
    }
    adc_output[1] = medianFilter.GetFiltered();

    for (index_t i = 0; i < buffer_size; i++)
    {
      medianFilter.AddValue(ADC2_buffer[i]);
    }
    adc_output[2] = medianFilter.GetFiltered();

    for (index_t i = 0; i < buffer_size; i++)
    {
      medianFilter.AddValue(ADC3_buffer[i]);
    }
    adc_output[3] = medianFilter.GetFiltered();

    for (index_t i = 0; i < buffer_size; i++)
    {
      medianFilter.AddValue(ADC4_buffer[i]);
    }
    adc_output[4] = medianFilter.GetFiltered();

    for (index_t i = 0; i < buffer_size; i++)
    {
      medianFilter.AddValue(ADC5_buffer[i]);
    }
    adc_output[5] = medianFilter.GetFiltered();

    for (index_t i = 0; i < buffer_size; i++)
    {
      medianFilter.AddValue(ADC6_buffer[i]);
    }
    adc_output[6] = medianFilter.GetFiltered();

    for (index_t i = 0; i < buffer_size; i++)
    {
      medianFilter.AddValue(ADC7_buffer[i]);
    }
    adc_output[7] = medianFilter.GetFiltered();

    ADC_channel_samples = 0;
  }
}

void MultiChannelADC::ADC_print_data()
{
  for (int i = 0; i < 8; i++)
  {
    Serial.print(F("ADC_"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(adc_output[i]);
  }
}

int MultiChannelADC::ADC_get_channel_value(const enum adc_channel adc_ch)
{
  return adc_ch < 8 ? adc_output[adc_ch] : 0;
}

#endif