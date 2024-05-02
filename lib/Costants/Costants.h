#ifndef COSTANTS
#define COSTANTS

#define N_VALUE_PER_READING 84
#define N_AXES 3
#define BYTE_PER_AXES 2
#define ACCELEROMETER_PRESENT 1
#define GYROSCOPE_PRESENT 1
#define N_SAMPLE_PER_READING ((uint16_t)(N_VALUE_PER_READING * N_AXES * BYTE_PER_AXES * (ACCELEROMETER_PRESENT + GYROSCOPE_PRESENT)))

enum AXES
{
    X,
    Y,
    Z
};

#define SAMPLE_FILTER_TAP_NUM 21
const short ACC_FILTER_TAPS[SAMPLE_FILTER_TAP_NUM] = {
    -355,
    -475,
    -579,
    -461,
    -6,
    829,
    1986,
    3293,
    4506,
    5365,
    5675,
    5365,
    4506,
    3293,
    1986,
    829,
    -6,
    -461,
    -579,
    -475,
    -355,
};
#endif