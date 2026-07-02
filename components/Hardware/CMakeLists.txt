set(src_dirs
    Wireless
    IMU
    GPS
    DXLora
    Motor
    Servo
    Key
    LED)

set(include_dirs
    Wireless
    GPS
    IMU
    DXLora
    Motor
    Key
    Servo
    LED)

set(requires
    driver
    nvs_flash
    esp_wifi
    esp_event
    esp_netif
    mqtt
    log
    Middleware
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)