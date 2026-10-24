idf_component_register(SRCS "pen_servo.cpp" INCLUDE_DIRS "../../../include"
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_timer)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
