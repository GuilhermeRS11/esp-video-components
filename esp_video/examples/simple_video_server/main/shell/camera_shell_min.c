/*
 * Minimal camera shell using public Camera Control API
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_console.h"
#include "camera_control_api.h"
#if CONFIG_CAMERA_IMX708
#include "imx708.h"
#endif

static int cmd_cam_exposure(int argc, char **argv)
{
    if (argc < 2) { printf("Usage:\n  cam_exposure <milliseconds>\n  cam_exposure current\n"); return 0; }
    if (strcmp(argv[1], "current") == 0) {
        double ms=0.0; if (camera_get_exposure_ms(&ms)==0) printf("Exposure: %.1f ms\n", ms); else printf("Failed to get exposure\n");
        return 0;
    }
    long ms_in = strtol(argv[1], NULL, 0); double applied=0.0;
    if (camera_set_exposure_ms((double)ms_in, &applied)==0) printf("Set exposure to %.1f ms (requested %ld ms)\n", applied, ms_in);
    else printf("Failed to set exposure\n");
    return 0;
}

static int cmd_cam_gaina(int argc, char **argv)
{
    // Direct register method (like sensor_gain_direct) for IMX708
    if (argc == 1) {
        printf("Analogue Gain Control (direct)\n");
        printf("Usage: cam_gaina <mult> | current | auto <enable|disable>\n");
        printf("  <mult>: human-friendly multiplier (e.g., 1, 1.5, 2 … up to ~16)\n");
        return 0;
    }
#if CONFIG_CAMERA_IMX708
    const char *param = argv[1];
    if (strcmp(param, "current") == 0) {
        double mult=0.0; if (camera_get_analog_gain_mult(&mult)==0) printf("Analogue Gain: ~%.2fx\n", mult); else printf("Failed to get analogue gain\n");
        return 0;
    }
    if (strcmp(param, "auto") == 0) {
        if (argc != 3) { printf("Usage: cam_gaina auto <enable|disable>\n"); return 1; }
        bool en = (strcmp(argv[2], "enable")==0);
        esp_err_t r = imx708_hw_write_register(0x3200, en?0x01:0x00);
        if (r==ESP_OK) printf("Sensor Auto Gain %s\n", en?"ENABLED":"DISABLED");
        else printf("Failed to set Auto Gain\n");
        return (r==ESP_OK)?0:1;
    }
    // Accept multiplier formats like "1", "1x", "1.5" (defaults to multiplier)
    double mult = 0.0; char *endp = NULL; mult = strtod(param, &endp);
    if (endp && (*endp=='x' || *endp=='X')) { /* ignore 'x' suffix */ }
    double applied=0.0; if (camera_set_analog_gain_mult(mult, &applied)==0) printf("Analogue gain set to ~%.2fx\n", applied); else printf("Failed to set analogue gain\n");
    return 0;
#else
    printf("cam_gaina not supported on this sensor\n");
    return 1;
#endif
}

static int cmd_cam_gaind(int argc, char **argv)
{
    // Direct register method (like sensor_dgain_direct) for IMX708
    if (argc == 1) {
        printf("Digital Gain Control (direct)\n");
        printf("Usage: cam_gaind <mult>|current\n");
        printf("  <mult>: human-friendly multiplier (e.g., 1, 1.5, 2 …)\n");
        return 0;
    }
#if CONFIG_CAMERA_IMX708
    const char *param = argv[1];
    if (strcmp(param, "current")==0) {
        double mult=0.0; if (camera_get_digital_gain_mult(&mult)==0) printf("Digital gain: %.2fx\n", mult); else printf("Failed to get digital gain\n"); return 0;
    }
    // Treat input as multiplier (1 -> ~1x, 2 -> ~2x)
    double mult=0.0; char *endp=NULL; mult=strtod(param,&endp); if (endp && (*endp=='x'||*endp=='X')) {}
    if (mult < 1.0) {
        mult = 1.0;
    }
    if (mult > 16.0) {
        mult = 16.0;
    }
    double applied_mult=0.0; if (camera_set_digital_gain_mult(mult, &applied_mult)==0) printf("Digital gain set to %.2fx\n", applied_mult); else printf("Failed to set digital gain\n"); return 0;
#else
    printf("cam_gaind not supported on this sensor\n");
    return 1;
#endif
}

static int cmd_cam_3a_lock(int argc, char **argv)
{
    if (argc < 2) { printf("Usage:\n  cam_3a_lock ae|awb|af on|off\n  cam_3a_lock status\n"); return 0; }
    int32_t cur=0; if (camera_get_3a_lock(&cur)!=0) { printf("Failed to get 3A lock\n"); return 1; }
    if (strcmp(argv[1], "status") == 0) { printf("3A lock bits: 0x%02x (AWB=1, AE=2, AF=4)\n", (unsigned)cur); return 0; }
    if (argc < 3) { printf("Invalid usage. Example: cam_3a_lock ae off\n"); return 1; }
    bool awb = (cur & 1) != 0, ae = (cur & 2) != 0, af = (cur & 4) != 0;
    bool on = (strcmp(argv[2], "on")==0);
    if      (strcmp(argv[1], "awb")==0) awb = on;
    else if (strcmp(argv[1], "ae")==0)  ae  = on;
    else if (strcmp(argv[1], "af")==0)  af  = on;
    else { printf("Invalid component: %s (use ae|awb|af)\n", argv[1]); return 1; }
    int32_t applied=0; if (camera_set_3a_lock(ae, awb, af, &applied)==0) printf("Set 3A lock to 0x%02x\n", (unsigned)applied); else printf("Failed to set 3A lock\n");
    return 0;
}

static int cmd_cam_focus(int argc, char **argv)
{
    if (argc < 2) { printf("Usage:\n  cam_focus <position> (0..1023)\n  cam_focus current\n"); return 0; }
    if (strcmp(argv[1], "current")==0) { long pos=0; if (camera_get_focus(&pos)==0) printf("Focus position: %ld\n", pos); else printf("Failed to get focus position\n"); return 0; }
    long pos = strtol(argv[1], NULL, 0); long applied=0; if (camera_set_focus(pos, &applied)==0) printf("Set focus position to %ld\n", applied); else printf("Failed to set focus position\n"); return 0;
}

static int cmd_cam_flip(int argc, char **argv)
{
    if (argc<2) { printf("Usage:\n  cam_flip status\n  cam_flip h on|off\n  cam_flip v on|off\n"); return 0; }
    if (strcmp(argv[1],"status")==0) { bool h=false,v=false; if(camera_get_flip(&h,&v)==0) printf("Flip: H=%s V=%s\n", h?"on":"off", v?"on":"off"); else printf("Failed to get flip\n"); return 0; }
    if (argc<3) { printf("Usage: cam_flip h|v on|off\n"); return 1; }
    bool h=false,v=false; camera_get_flip(&h,&v);
    bool on = (strcmp(argv[2],"on")==0);
    if (strcmp(argv[1],"h")==0) h=on; else if (strcmp(argv[1],"v")==0) v=on; else { printf("Use h|v\n"); return 1; }
    if (camera_set_flip(h,v)==0) printf("Flip set: H=%s V=%s\n", h?"on":"off", v?"on":"off"); else printf("Failed to set flip\n");
    return 0;
}

void camera_shell_register_commands_minimal(void)
{
    const esp_console_cmd_t commands[] = {
        { .command = "cam_exposure", .help = "Set/Get exposure in milliseconds", .hint = NULL, .func = &cmd_cam_exposure },
        { .command = "cam_gaina",    .help = "Set/Get analogue gain", .hint = NULL, .func = &cmd_cam_gaina },
        { .command = "cam_gaind",    .help = "Set/Get digital gain (code/256 = multiplier)", .hint = NULL, .func = &cmd_cam_gaind },
        { .command = "cam_3a_lock",  .help = "Lock/unlock AE/AWB/AF (V4L2_CID_3A_LOCK)", .hint = NULL, .func = &cmd_cam_3a_lock },
        { .command = "cam_focus",    .help = "Set/Get motor absolute focus position", .hint = NULL, .func = &cmd_cam_focus },
        { .command = "cam_flip",     .help = "Set/Get image flip (H/V)", .hint = NULL, .func = &cmd_cam_flip },
    };
    for (int i = 0; i < (int)(sizeof(commands)/sizeof(commands[0])); i++) {
        esp_console_cmd_register(&commands[i]);
    }
}
