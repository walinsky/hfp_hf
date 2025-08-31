idf_component_register(SRCS "app_hf_msg_set.c"
                            "bt_app_core.c"
                            "bt_app_hf.c"
                            "gpio_pcm_config.c"
                            "bt_i2s.c"
                        #     "bt_app_pbac.c"
                            "main.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_driver_i2s console esp_ringbuf esp_audio_codec 
                    INCLUDE_DIRS ".")
