volatile unsigned long Duration[MaxProductCount];
volatile unsigned long PulseCount[MaxProductCount];
uint32_t LastPulse[MaxProductCount];

unsigned long TimedCounts[MaxProductCount];
uint32_t RateInterval[MaxProductCount];
uint32_t RateTimeLast[MaxProductCount];
uint32_t PWMTimeLast[MaxProductCount];

unsigned long CurrentCount;
uint32_t CurrentDuration;

unsigned long PPM[MaxProductCount];		// pulse per minute * 100
unsigned long Osum[MaxProductCount];
unsigned long Omax[MaxProductCount];
unsigned long Omin[MaxProductCount];
byte Ocount[MaxProductCount];
float Oave[MaxProductCount];

static unsigned long PulseTimeISR0= micros();
static unsigned long PulseTimeISR1= micros();

void ISR0()
{
	unsigned long micronow;
	unsigned long dur;
	micronow = micros();
	if (PulseTimeISR0 > micronow)
	{
		dur = micronow - PulseTimeISR0;
	}
	else
	{
		dur = 0xFFFFFFFF + micronow - PulseTimeISR0;
	}

	if (dur > MDL.Debounce * 1000)
	{
		Duration[0] = dur;
		PulseTimeISR0 = micronow;
		PulseCount[0]++;
    State = !State;
    digitalWrite(12, State);
	}
}



void ISR1()
{
	unsigned long micronow;
	unsigned long dur;
	micronow = micros();
	if (PulseTimeISR1 > micronow)
	{
		dur = micronow - PulseTimeISR1;
	}
	else
	{
		dur = 0xFFFFFFFF + micronow - PulseTimeISR1;
	}

	if (dur > MDL.Debounce * 1000)
	{
		Duration[1] = dur;
		PulseTimeISR1 = micronow;
		PulseCount[1]++;
	}
}

void GetUPM()
{
	for (int i = 0; i < MDL.ProductCount; i++)
	{
		if (Sensor[i].ControlType == 3)
		{
			// use weight
			Sensor[i].UPM = Sensor[i].MeterCal * (double)Sensor[i].pwmSetting;
		}
		else
		{
			// use flow meter
			GetUPMflow(i);
		}
	}
}

void GetUPMflow(int ID)
{
	if (PulseCount[ID])
	{
		noInterrupts();
		CurrentCount = PulseCount[ID];
		PulseCount[ID] = 0;
		CurrentDuration = Duration[ID];
		interrupts();

		if (Sensor[ID].UseMultiPulses)
		{
			// low ms/pulse, use pulses over time
			TimedCounts[ID] += CurrentCount;
			RateInterval[ID] = millis() - RateTimeLast[ID];
			if (RateInterval[ID] > 500)
			{
				RateTimeLast[ID] = millis();
				PPM[ID] = (6000000 * TimedCounts[ID]) / RateInterval[ID];	// 100 X actual
				TimedCounts[ID] = 0;
			}
		}
		else
		{
			// high ms/pulse, use time for one pulse
			if (CurrentDuration == 0)
			{
				PPM[ID] = 0;
			}
			else
			{
				PPM[ID] = 6000000000 / CurrentDuration;	// 100 X actual
			}
		}


		LastPulse[ID] = millis();
		Sensor[ID].TotalPulses += CurrentCount;
	}

	if (millis() - LastPulse[ID] > 4000)	PPM[ID] = 0;	// check for no flow

	// olympic average
	Osum[ID] += PPM[ID];
	if (Omax[ID] < PPM[ID]) Omax[ID] = PPM[ID];
	if (Omin[ID] > PPM[ID]) Omin[ID] = PPM[ID];

	Ocount[ID]++;
	if (Ocount[ID] > 4)
	{
		Osum[ID] -= Omax[ID];
		Osum[ID] -= Omin[ID];
 		Oave[ID] = (float)Osum[ID] / 300.0;	// divide by 3 samples and divide by 100 for decimal place
		Osum[ID] = 0;
		Omax[ID] = 0;
		Omin[ID] = 5000000000;
		Ocount[ID] = 0;
	}

	// units per minute
	if (Sensor[ID].MeterCal > 0)
	{
		Sensor[ID].UPM = Oave[ID] / Sensor[ID].MeterCal;
	}
	else
	{
		Sensor[ID].UPM = 0;
	}
}
