idf_component_register(
    SRCS
        "main.c"
        "infrared_sensor.c"
        "motor.c"
        "encoder.c"
        "drive.c"
        "line_follow.c"
        "ultrasonic.c"
        "obstacle_avoidance.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_timer
)
