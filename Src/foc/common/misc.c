/*
 * misc.c
 *
 *  Created on: May 10, 2024
 *      Author: xange
 */
#include "foc/common/misc.h"







float _normalizeAngle(float angle){
	float a = fmod(angle,_2PI);
	return a >= 0 ? a : (a+_2PI);
}

float _electricalAngle(float shaft_angle, int pole_pairs){
	return (shaft_angle * pole_pairs);
}


// fast_atan2 based on https://math.stackexchange.com/a/1105038/81278
// Via Odrive project
// https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/utils.cpp
// This function is MIT licenced, copyright Oskar Weigl/Odrive Robotics
// The origin for Odrive atan2 is public domain. Thanks to Odrive for making
// it easy to borrow.
__attribute__((weak)) float _atan2(float y, float x) {
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    // inject FLT_MIN in denominator to avoid division by zero
    float a = fmin(abs_x, abs_y) / (fmax(abs_x, abs_y));
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r =
        ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if (abs_y > abs_x) r = 1.57079637f - r;
    // if x < 0 then r := 3.14159274 - r
    if (x < 0.0f) r = 3.14159274f - r;
    // if y < 0 then r := -r
    if (y < 0.0f) r = -r;

    return r;
  }


// square root approximation function using
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
__attribute__((weak)) float _sqrtApprox(float number) {//low in fat
  union {
    float    f;
    uint32_t i;
    uint16_t fd;
  } y = { .f = number };
  y.i = 0x5f375a86 - ( y.i >> 1 );
  return number * y.f;
}
