#include    <ucos_ii.h>

#include    <string.h>
#include    <stdio.h>

#include "stm32_eval.h"

#include "LCD.h"

#include "Tscalibrate.h"

#define XORMODE 0x80000000

void tslib_linear_calibrate(TOUCH_EVENT *samp, int a[])
{
    int xtemp,ytemp;

    //sprintf (cal_buffer,"%d %d %d %d %d %d %d",
    //     cal.a[1], cal.a[2], cal.a[0],
    //     cal.a[4], cal.a[5], cal.a[3], cal.a[6]);

    xtemp = samp->usX; ytemp = samp->usY;
    //samp->usX =   ( a[2] +
    //        a[0]*xtemp + 
    //        a[1]*ytemp ) / a[6];
    //samp->usY =   ( a[5] +
    //        a[3]*xtemp +
    //        a[4]*ytemp ) / a[6];

    samp->usX =   ( a[0] +
            a[1]*xtemp + 
            a[2]*ytemp ) / a[6];
    samp->usY =   ( a[3] +
            a[4]*xtemp +
            a[5]*ytemp ) / a[6];
    

}


int tslib_perform_calibration(calibration *cal) {
    int j;
    float n, x, y, x2, y2, xy, z, zx, zy;
    float det, a, b, c, e, f, i;
    float scaling = 65536.0;

// Get sums for matrix
    n = x = y = x2 = y2 = xy = 0;
    for(j=0;j<5;j++) {
        n += 1.0;
        x += (float)cal->x[j];
        y += (float)cal->y[j];
        x2 += (float)(cal->x[j]*cal->x[j]);
        y2 += (float)(cal->y[j]*cal->y[j]);
        xy += (float)(cal->x[j]*cal->y[j]);
    }

// Get determinant of matrix -- check if determinant is too small
    det = n*(x2*y2 - xy*xy) + x*(xy*y - x*y2) + y*(x*xy - y*x2);
    if(det < 0.1 && det > -0.1) {
        printf("ts_calibrate: determinant is too small -- %f\n",det);
        return 0;
    }

// Get elements of inverse matrix
    a = (x2*y2 - xy*xy)/det;
    b = (xy*y - x*y2)/det;
    c = (x*xy - y*x2)/det;
    e = (n*y2 - y*y)/det;
    f = (x*y - n*xy)/det;
    i = (n*x2 - x*x)/det;

// Get sums for x calibration
    z = zx = zy = 0;
    for(j=0;j<5;j++) {
        z += (float)cal->xfb[j];
        zx += (float)(cal->xfb[j]*cal->x[j]);
        zy += (float)(cal->xfb[j]*cal->y[j]);
    }

// Now multiply out to get the calibration for framebuffer x coord
    cal->a[0] = (int)((a*z + b*zx + c*zy)*(scaling));
    cal->a[1] = (int)((b*z + e*zx + f*zy)*(scaling));
    cal->a[2] = (int)((c*z + f*zx + i*zy)*(scaling));

    printf("%f %f %f\n",(a*z + b*zx + c*zy),
                (b*z + e*zx + f*zy),
                (c*z + f*zx + i*zy));

// Get sums for y calibration
    z = zx = zy = 0;
    for(j=0;j<5;j++) {
        z += (float)cal->yfb[j];
        zx += (float)(cal->yfb[j]*cal->x[j]);
        zy += (float)(cal->yfb[j]*cal->y[j]);
    }

// Now multiply out to get the calibration for framebuffer y coord
    cal->a[3] = (int)((a*z + b*zx + c*zy)*(scaling));
    cal->a[4] = (int)((b*z + e*zx + f*zy)*(scaling));
    cal->a[5] = (int)((c*z + f*zx + i*zy)*(scaling));

    printf("%f %f %f\n",(a*z + b*zx + c*zy),
                (b*z + e*zx + f*zy),
                (c*z + f*zx + i*zy));

// If we got here, we're OK, so assign scaling to a[6] and return
    cal->a[6] = (int)scaling;
    return 1;
/*  
// This code was here originally to just insert default values
    for(j=0;j<7;j++) {
        c->a[j]=0;
    }
    c->a[1] = c->a[5] = c->a[6] = 1;
    return 1;
*/

}

void tslib_get_sample (calibration *cal,int index, int x, int y,TOUCH_EVENT *samp)
{
    cal->x[index]    = samp->usX;
    cal->y[index]    = samp->usY;
    cal->xfb [index] = x;
    cal->yfb [index] = y;
}


