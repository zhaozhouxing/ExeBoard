#ifndef _TS_CALIBRATE_H_
#define _TS_CALIBRATE_H_

#include "touch.h"

typedef struct {
    int x[5], xfb[5];
    int y[5], yfb[5];
    int a[7];
} calibration;

void tslib_linear_calibrate(TOUCH_EVENT *samp, int a[]);
void tslib_get_sample (calibration *cal,int index, int x, int y,TOUCH_EVENT *samp);
int  tslib_perform_calibration(calibration *cal);

#endif
