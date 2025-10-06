/*
 * Camera Control API (public) – can be reused by other apps
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Exposure (milliseconds)
int camera_get_exposure_ms(double *ms_out);
int camera_set_exposure_ms(double ms_in, double *applied_ms_out);

// Analogue gain (raw integer per V4L2_CID_ANALOGUE_GAIN)
int camera_get_analog_gain(long *val_out);
int camera_set_analog_gain(long val_in, long *applied_out);
// Analogue gain in multiplier units (e.g., 1.0 .. ~16.0)
int camera_get_analog_gain_mult(double *mult_out);
int camera_set_analog_gain_mult(double mult_in, double *applied_mult_out);

// Digital gain (raw code per V4L2_CID_DIGITAL_GAIN; code/256 = multiplier)
int camera_get_digital_gain(long *code_out);
int camera_set_digital_gain(long code_in, long *applied_out);
// Digital gain in multiplier units (code = mult*256)
int camera_get_digital_gain_mult(double *mult_out);
int camera_set_digital_gain_mult(double mult_in, double *applied_mult_out);

// 3A Lock (bitmask via V4L2_CID_3A_LOCK): bit0=AWB, bit1=AE, bit2=AF
int camera_get_3a_lock(int32_t *mask_out);
int camera_set_3a_lock(bool ae_on, bool awb_on, bool af_on, int32_t *applied_mask_out);

// Focus (absolute)
int camera_get_focus(long *pos_out);
int camera_set_focus(long pos_in, long *applied_out);

// Image flip (horizontal/vertical)
int camera_get_flip(bool *h_on, bool *v_on);
int camera_set_flip(bool h_on, bool v_on);

#ifdef __cplusplus
}
#endif
