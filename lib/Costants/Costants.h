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

#define SAMPLE_FILTER_TAP_NUM 13

/* */
static double FILTER_TAPS[SAMPLE_FILTER_TAP_NUM] = {
    -0.012376630905497659,
    -0.011997855910257257,
    0.009822484849634577,
    0.06447193455373151,
    0.14148310471140452,
    0.21060464400018522,
    0.2384776071880643,
    0.21060464400018522,
    0.14148310471140452,
    0.06447193455373151,
    0.009822484849634577,
    -0.011997855910257257,
    -0.012376630905497659,
};

union float2bytes
{
    float f;
    uint8_t b[sizeof(float)];
};

#endif

/*
    0-5, 20-50, 1,-40  samp = 100Hz, 13 tap
    -0.012376630905497659,
    -0.011997855910257257,
    0.009822484849634577,
    0.06447193455373151,
    0.14148310471140452,
    0.21060464400018522,
    0.2384776071880643,
    0.21060464400018522,
    0.14148310471140452,
    0.06447193455373151,
    0.009822484849634577,
    -0.011997855910257257,
    -0.012376630905497659,
*/

/*
   0-5, 10-50, 0.1,-40  samp = 100Hz, 45 tap
 -0.006035202093124089,
  -0.0021214779471192763,
  -0.0009185793079305108,
  0.0014354409383040695,
  0.004535325774584334,
  0.0075845667263980355,
  0.009520010410639433,
  0.009283559976078708,
  0.006147237627867569,
  0.00003562243666405604,
  -0.00826040562814116,
  -0.01705478790749873,
  -0.023972035109763246,
  -0.02637021659400638,
  -0.021912527321237326,
  -0.009166387897164957,
  0.011925275582553121,
  0.03982187247588656,
  0.07148949807746896,
  0.10282609826912409,
  0.12938049566799986,
  0.14716065816253898,
  0.15342592464984162,
  0.14716065816253898,
  0.12938049566799986,
  0.10282609826912409,
  0.07148949807746896,
  0.03982187247588656,
  0.011925275582553121,
  -0.009166387897164957,
  -0.021912527321237326,
  -0.02637021659400638,
  -0.023972035109763246,
  -0.01705478790749873,
  -0.00826040562814116,
  0.00003562243666405604,
  0.006147237627867569,
  0.009283559976078708,
  0.009520010410639433,
  0.0075845667263980355,
  0.004535325774584334,
  0.0014354409383040695,
  -0.0009185793079305108,
  -0.0021214779471192763,
  -0.006035202093124089
*/

/*

   0-5, 10-50, 0.1,-20  samp = 100Hz, 4315 tap
  0.02502443181452652,
  -0.016252637657547142,
  -0.025686227974788964,
  -0.0024036175500719344,
  0.0008544107817295869,
  0.02740951299654984,
  0.02092513389169697,
  0.015895188512537836,
  -0.022642475685715386,
  -0.04207473258141669,
  -0.053682517273733056,
  -0.009185394183995239,
  0.05959055375642114,
  0.16022449633522862,
  0.2302841903425285,
  0.2669081530481276,
  0.2302841903425285,
  0.16022449633522862,
  0.05959055375642114,
  -0.009185394183995239,
  -0.053682517273733056,
  -0.04207473258141669,
  -0.022642475685715386,
  0.015895188512537836,
  0.02092513389169697,
  0.02740951299654984,
  0.0008544107817295869,
  -0.0024036175500719344,
  -0.025686227974788964,
  -0.016252637657547142,
  0.02502443181452652
*/