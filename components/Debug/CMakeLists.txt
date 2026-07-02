set(src_dirs
    Config
    Printf_Debug)

set(include_dirs
    Config
    Printf_Debug)

set(requires
    driver
    nvs_flash
    esp_wifi
    esp_event
    esp_netif
    mqtt
    log
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)