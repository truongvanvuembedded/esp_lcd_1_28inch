idf_component_register(SRCS "Drv_Display.c"
                    INCLUDE_DIRS "."
                    REQUIRES lvgl__lvgl
                    PRIV_REQUIRES driver esp_lcd espressif__esp_lvgl_port espressif__esp_lcd_gc9a01 )