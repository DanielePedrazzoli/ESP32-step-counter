#include "SampleFilter.h"

#define SAMPLEFILTER_TAP_NUM 31
static double filter_taps[SAMPLEFILTER_TAP_NUM] = {
    -0.00331215337212421,
    -0.004893665831885696,
    -0.007382909493024512,
    -0.009459493179836465,
    -0.01023182275178142,
    -0.008688817195888928,
    -0.003890892483447335,
    0.004813805536345832,
    0.017588775688595674,
    0.03397167839705332,
    0.05283726430563065,
    0.07248918864962421,
    0.09088182134607507,
    0.10591088798141066,
    0.11575772030475916,
    0.11918498836813025,
    0.11575772030475916,
    0.10591088798141066,
    0.09088182134607507,
    0.07248918864962421,
    0.05283726430563065,
    0.03397167839705332,
    0.017588775688595674,
    0.004813805536345832,
    -0.003890892483447335,
    -0.008688817195888928,
    -0.01023182275178142,
    -0.009459493179836465,
    -0.007382909493024512,
    -0.004893665831885696,
    -0.00331215337212421,
};

void SampleFilter_init(SampleFilter *f)
{
    int i;
    for (i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
        f->history[i] = 0;
    f->last_index = 0;
}

void SampleFilter_put(SampleFilter *f, double input)
{
    f->history[f->last_index++] = input;
    if (f->last_index == SAMPLEFILTER_TAP_NUM)
        f->last_index = 0;
}

double SampleFilter_get(SampleFilter *f)
{
    double acc = 0;
    int index = f->last_index, i;
    for (i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
    {
        index = index != 0 ? index - 1 : SAMPLEFILTER_TAP_NUM - 1;
        acc += f->history[index] * filter_taps[i];
    };
    return acc;
}