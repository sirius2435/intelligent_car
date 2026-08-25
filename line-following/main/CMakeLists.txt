idf_component_register(
    SRCS
        "main.c"
        "infrared_sensor.c"
        "motor.c"
        "drive.c"
        "line_follow.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc
)
