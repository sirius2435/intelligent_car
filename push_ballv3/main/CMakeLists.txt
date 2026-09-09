idf_component_register(
    SRCS
        "main.c"
        "motor.c"
        "encoder.c"
        "drive.c"
        "line_follow.c"
        "pseudo_infrared.c"
        "camera_vision.c"
        "camera_stream.cpp"
        "camera_gimbal.c"
        "ultrasonic.c"
        "obstacle_avoidance.c"
        "ball_vision.c"
        "ball_push.c"
        "lcd.c"
        "lcd_monitor.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_spi
             esp_timer esp_psram usb_stream esp_jpeg
             esp_wifi esp_netif esp_event esp_http_server nvs_flash
)
