idf_component_register(
    SRCS
        "main.c"
        "motor.c"
        "encoder.c"
        "drive.c"
        "camera_vision.c"
        "vision_line.c"
        "vision_follow.c"
        "ultrasonic.c"
        "obstacle_avoidance.c"
        "lcd.c"
        "lcd_monitor.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_spi
             esp_timer esp_psram usb_stream esp_jpeg
)
