idf_component_register(
    SRCS "main.c" "infrared_sensor.c" "status_led.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio led_strip
)
