#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 200 Hz

* 0 Hz - 5 Hz
  gain = 1
  desired ripple = 1 dB
  actual ripple = 0.22371862902938705 dB

* 20 Hz - 100 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -50.525736377847124 dB

*/

#define SAMPLEFILTER_TAP_NUM 31

typedef struct
{
  double history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter *f);
void SampleFilter_put(SampleFilter *f, double input);
double SampleFilter_get(SampleFilter *f);

#endif