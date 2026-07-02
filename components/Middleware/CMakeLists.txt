
set(src_dirs "." 
"PID"
"Filter"
"Receive_Data"
"FreeRTOS")

set(include_dirs "." 
"PID"
"Filter"
"Receive_Data"
"FreeRTOS")

set(requires
    driver
    nvs_flash
    esp_wifi
    esp_event
    esp_netif
    mqtt
    log
    Debug
    Hardware
)

idf_component_register(
    SRC_DIRS ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)
component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)