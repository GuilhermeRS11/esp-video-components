/*
 * Camera Control API (implementation)
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "linux/videodev2.h"
#include "linux/v4l2-controls.h"
#include "camera_control_api.h"
#if CONFIG_CAMERA_IMX708
#include "imx708.h"
#endif

#define VIDEO_DEVICE "/dev/video0"

static int open_video(void)
{
    int fd = open(VIDEO_DEVICE, O_RDWR);
    return fd;
}

static int query_range(int fd, uint32_t id, int32_t *min, int32_t *max, int32_t *step)
{
    struct v4l2_query_ext_ctrl q = {0};
    q.id = id;
    if (ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q) != 0) return -1;
    if (min) *min = (int32_t)q.minimum;
    if (max) *max = (int32_t)q.maximum;
    if (step) *step = (int32_t)q.step;
    return 0;
}

int camera_get_exposure_ms(double *ms_out)
{
    if (!ms_out) return -1;
    int fd = open_video(); if (fd < 0) return -1;
    struct v4l2_ext_controls ctrls = {0};
    struct v4l2_ext_control  ctrl  = {0};
    ctrls.ctrl_class = V4L2_CID_CAMERA_CLASS; ctrls.count = 1; ctrls.controls = &ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    int ret = ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls);
    if (ret == 0) *ms_out = ctrl.value / 10.0; // 100us -> ms
    close(fd);
    return ret;
}

int camera_set_exposure_ms(double ms_in, double *applied_ms_out)
{
    int fd = open_video(); if (fd < 0) return -1;
    int32_t min=0, max=0, step=1; if (query_range(fd, V4L2_CID_EXPOSURE_ABSOLUTE, &min, &max, &step)!=0){ close(fd); return -1; }
    if (step <= 0) step = 1;
    long req_units = (long)(ms_in * 10.0 + 0.5);
    long k = (req_units + step/2) / step;
    long units_rounded = k * step;
    if (units_rounded < min) units_rounded = ((min + step - 1) / step) * step;
    if (units_rounded > max) units_rounded = (max / step) * step;
    struct v4l2_ext_controls ctrls = {0};
    struct v4l2_ext_control  ctrl  = {0};
    ctrls.ctrl_class = V4L2_CID_CAMERA_CLASS; ctrls.count = 1; ctrls.controls = &ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE; ctrl.value = (int32_t)units_rounded;
    int ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);
    if (ret == 0 && applied_ms_out) *applied_ms_out = units_rounded / 10.0;
    close(fd);
    return ret;
}

int camera_get_analog_gain(long *val_out)
{
    if (!val_out) return -1;
    int fd = open_video(); if (fd < 0) return -1;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id = V4L2_CID_ANALOGUE_GAIN; 
    int ret = ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls);
    if (ret==0) {
        *val_out = ctrl.value;
    }
    close(fd);
    return ret;
}

int camera_get_analog_gain_mult(double *mult_out)
{
    if (!mult_out) return -1;
#if CONFIG_CAMERA_IMX708
    uint8_t gh=0, gl=0;
    if (imx708_hw_read_register(0x0204, &gh)==ESP_OK && imx708_hw_read_register(0x0205, &gl)==ESP_OK) {
        int code8 = gl;
        double mult = 1.0 + (code8 * (15.0/255.0));
        *mult_out = mult;
        return 0;
    }
#endif
    long raw=0; if (camera_get_analog_gain(&raw)==0) { *mult_out = (double)raw; return 0; }
    return -1;
}

int camera_set_analog_gain_mult(double mult_in, double *applied_mult_out)
{
#if CONFIG_CAMERA_IMX708
    if (mult_in < 1.0) {
        mult_in = 1.0;
    }
    if (mult_in > 16.0) {
        mult_in = 16.0;
    }
    int code8 = (int)((mult_in - 1.0) * (255.0/15.0) + 0.5);
    if (code8 < 0) {
        code8 = 0;
    }
    if (code8 > 255) {
        code8 = 255;
    }
    esp_err_t r = imx708_hw_write_register(0x0204, 0x00);
    r |= imx708_hw_write_register(0x0205, (uint8_t)code8);
    if (r == ESP_OK) { if (applied_mult_out) *applied_mult_out = 1.0 + (code8 * (15.0/255.0)); return 0; }
    return -1;
#else
    (void)mult_in; (void)applied_mult_out; return -1;
#endif
}

int camera_set_analog_gain(long val_in, long *applied_out)
{
    int fd = open_video(); if (fd < 0) return -1;
    int32_t min=0,max=0,step=1; 
    if (query_range(fd, V4L2_CID_ANALOGUE_GAIN, &min, &max, &step)!=0){ 
        close(fd); 
        return -1; 
    }
    if (step<=0) step=1; 
    if (val_in<min) val_in=min; 
    if (val_in>max) val_in=max;
    long k=(val_in-min+step/2)/step; long val_rounded=min+k*step;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id=V4L2_CID_ANALOGUE_GAIN; 
    int ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);
    if (ret==0 && applied_out) {
        *applied_out = val_rounded;
    }
    close(fd);
    return ret;
}

int camera_get_digital_gain(long *code_out)
{
    if (!code_out) return -1;
    int fd = open_video(); if (fd < 0) return -1;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id=V4L2_CID_DIGITAL_GAIN; 
    int ret=ioctl(fd, VIDIOC_G_EXT_CTRLS,&ctrls);
    if (ret==0) {
        *code_out = ctrl.value;
    }
    close(fd);
    return ret;
}

int camera_get_digital_gain_mult(double *mult_out)
{
    if (!mult_out) return -1;
    long code=0; if (camera_get_digital_gain(&code)==0) { *mult_out = code / 256.0; return 0; }
#if CONFIG_CAMERA_IMX708
    uint8_t dh=0, dl=0; if (imx708_hw_read_register(0x020E,&dh)==ESP_OK && imx708_hw_read_register(0x020F,&dl)==ESP_OK) {
        long c = ((long)dh<<8)|dl; *mult_out = c / 256.0; return 0; }
#endif
    return -1;
}

int camera_set_digital_gain(long code_in, long *applied_out)
{
    int fd = open_video(); if (fd < 0) return -1;
    int32_t min=0,max=0,step=1; 
    if (query_range(fd, V4L2_CID_DIGITAL_GAIN, &min, &max, &step)!=0){ 
        close(fd); 
        return -1; 
    }
    if (step<=0) step=1; 
    if (code_in<min) code_in=min; 
    if (code_in>max) code_in=max;
    long k=(code_in-min+step/2)/step; long code_rounded=min+k*step;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id=V4L2_CID_DIGITAL_GAIN; 
    int ret=ioctl(fd, VIDIOC_S_EXT_CTRLS,&ctrls);
    if (ret==0 && applied_out) {
        *applied_out=code_rounded;
    }
    close(fd);
    return ret;
}

int camera_set_digital_gain_mult(double mult_in, double *applied_mult_out)
{
    if (mult_in < 1.0) {
        mult_in = 1.0;
    }
    if (mult_in > 16.0) {
        mult_in = 16.0;
    }
    long code = (long)(mult_in * 256.0 + 0.5);
    long applied=0; if (camera_set_digital_gain(code, &applied)==0) { if (applied_mult_out) *applied_mult_out = applied/256.0; return 0; }
#if CONFIG_CAMERA_IMX708
    if (code < 0x0100) {
        code = 0x0100;
    }
    if (code > 0x0FFF) {
        code = 0x0FFF;
    }
    uint8_t dh=(code>>8)&0xFF, dl=code&0xFF; esp_err_t r=ESP_OK; r|=imx708_hw_write_register(0x020E,dh); r|=imx708_hw_write_register(0x020F,dl);
    if (r==ESP_OK) { if (applied_mult_out) *applied_mult_out = code/256.0; return 0; }
#endif
    return -1;
}

int camera_get_3a_lock(int32_t *mask_out)
{
    if (!mask_out) return -1;
    int fd = open_video(); if (fd < 0) return -1;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id=V4L2_CID_3A_LOCK; 
    int ret=ioctl(fd, VIDIOC_G_EXT_CTRLS,&ctrls);
    if (ret==0) {
        *mask_out = ctrl.value;
    }
    close(fd);
    return ret;
}

int camera_set_3a_lock(bool ae_on, bool awb_on, bool af_on, int32_t *applied_mask_out)
{
    int32_t mask=0; if(awb_on)mask|=1; if(ae_on)mask|=2; if(af_on)mask|=4;
    int fd = open_video(); if (fd < 0) return -1;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id=V4L2_CID_3A_LOCK; ctrl.value=mask; 
    int ret=ioctl(fd, VIDIOC_S_EXT_CTRLS,&ctrls);
    if (ret==0 && applied_mask_out) {
        *applied_mask_out=mask;
    }
    close(fd);
    return ret;
}

int camera_get_focus(long *pos_out)
{
    if (!pos_out) return -1;
    int fd = open_video(); if (fd < 0) return -1;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id=V4L2_CID_FOCUS_ABSOLUTE; 
    int ret=ioctl(fd, VIDIOC_G_EXT_CTRLS,&ctrls);
    if (ret==0) {
        *pos_out = ctrl.value;
    }
    close(fd);
    return ret;
}

int camera_set_focus(long pos_in, long *applied_out)
{
    if (pos_in<0) pos_in=0; 
    if (pos_in>1023) pos_in=1023;
    int fd = open_video(); if (fd < 0) return -1;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id=V4L2_CID_FOCUS_ABSOLUTE; ctrl.value=(int32_t)pos_in; 
    int ret=ioctl(fd, VIDIOC_S_EXT_CTRLS,&ctrls);
    if (ret==0 && applied_out) {
        *applied_out=pos_in;
    }
    close(fd);
    return ret;
}

int camera_get_flip(bool *h_on, bool *v_on)
{
    int fd = open_video(); if (fd < 0) return -1; int ret=0;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id=V4L2_CID_HFLIP; ret|=ioctl(fd, VIDIOC_G_EXT_CTRLS,&ctrls); if (h_on) *h_on = (ret==0)? (ctrl.value!=0):false;
    ctrl.id=V4L2_CID_VFLIP; ret|=ioctl(fd, VIDIOC_G_EXT_CTRLS,&ctrls); if (v_on) *v_on = (ret==0)? (ctrl.value!=0):false;
    close(fd); return ret;
}

int camera_set_flip(bool h_on, bool v_on)
{
    int fd = open_video(); if (fd < 0) return -1; int ret=0;
    struct v4l2_ext_controls ctrls={0}; struct v4l2_ext_control ctrl={0};
    ctrls.ctrl_class=V4L2_CID_CAMERA_CLASS; ctrls.count=1; ctrls.controls=&ctrl;
    ctrl.id=V4L2_CID_HFLIP; ctrl.value=h_on?1:0; ret|=ioctl(fd, VIDIOC_S_EXT_CTRLS,&ctrls);
    ctrl.id=V4L2_CID_VFLIP; ctrl.value=v_on?1:0; ret|=ioctl(fd, VIDIOC_S_EXT_CTRLS,&ctrls);
    close(fd); return ret;
}
