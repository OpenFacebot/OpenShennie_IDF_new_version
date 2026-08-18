idf_component_register(SRCS "main.cpp"
                            "servo_config.cpp"
                            "servo_task.cpp"
                            "micro_ros_task.cpp"
                            "usjt_transport.c"
                    INCLUDE_DIRS "."
                    REQUIRES ds_servo micro_ros)