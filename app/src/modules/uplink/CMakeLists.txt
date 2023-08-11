#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

if (CONFIG_DEVICE_GATEWAY_UPPER) 
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/uplink_mqtt.c)
elseif(CONFIG_DEVICE_GATEWAY_LOWER)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/uplink_spi.c)
elseif(CONFIG_DEVICE_EDGE)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/uplink_bt.c)
else() # QEMU
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/uplink_mock.c)
endif ()

# zephyr_library_sources_ifdef(CONFIG_UPLINK_SPI ${CMAKE_CURRENT_SOURCE_DIR}/uplink_spi.c)
# zephyr_library_sources_ifdef(CONFIG_UPLINK_MQTT ${CMAKE_CURRENT_SOURCE_DIR}/uplink_mqtt.c)
# zephyr_library_sources_ifdef(CONFIG_UPLINK_BT ${CMAKE_CURRENT_SOURCE_DIR}/uplink_bt.c)
# zephyr_library_sources_ifdef(CONFIG_UPLINK_MOCK ${CMAKE_CURRENT_SOURCE_DIR}/uplink_mock.c)

# zephyr_include_directories(include)