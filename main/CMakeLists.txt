#set(srcs "app_main.c"
        #  "esp_hid_gap.c")
# set(include_dirs ".")
# 
idf_component_register(SRCS "app_main.c"
                    "esp_hid_gap.c"
                    INCLUDE_DIRS ".")
# idf_component_register(SRCS "${srcs}"
                    #    INCLUDE_DIRS "."
                    #    REQUIRES esp_hid)
