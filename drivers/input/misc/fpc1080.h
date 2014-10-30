#ifndef LINUX_SPI_FPC1080_H
#define LINUX_SPI_FPC1080_H

struct fpc1080_nav_settings {
    // Added by Joshua on 2012-10-29 Mon PM  1:22
    int sum_x;
    int sum_y;
    u8 p_multiplier_x;
    u8 p_multiplier_y;
    u8 p_sensitivity_key;
    u8 multiplier_key_accel;
    u8 threshold_key_accel;
};

struct fpc1080_adc_setup {
    u8 gain;
    u8 offset;
    u8 pxl_setup;
    u8 finger_detect_thr;
    u8 finger_lost_thr;
};

struct fpc1080_platform_data {
    int irq_gpio;
    int reset_gpio;
    struct fpc1080_adc_setup adc_setup;
    struct fpc1080_nav_settings nav;
};

#endif
