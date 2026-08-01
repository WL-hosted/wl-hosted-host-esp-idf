set(wlh_sources
    "app_main.c"
    "console_commands.c"
    "iperf2.c"
    "iperf_controller.c"
    "network.c"
    "ota_command.c"
    "sdio_transport.c"
)

idf_component_register(
    SRCS ${wlh_sources}
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        console
        esp_driver_gpio
        esp_driver_sdmmc
        esp_driver_uart
        esp_driver_usb_serial_jtag
        esp_event
        esp_http_client
        esp_netif
        esp_timer
        iperf
        joltwallet__littlefs
        lwip
        mbedtls
        nvs_flash
        vfs
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Werror)

set(BUILD_TESTING OFF)
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/host-core
    ${CMAKE_BINARY_DIR}/core-host-build
    EXCLUDE_FROM_ALL
)
target_link_libraries(${COMPONENT_LIB} PUBLIC wlh_host_core)

wlh_common_enable_freertos_osal()
target_link_libraries(${COMPONENT_LIB} PUBLIC wlh::freertos_osal)

wlh_common_enable_log(BACKEND ESP)
target_link_libraries(${COMPONENT_LIB} PUBLIC wlh::log)

foreach(wlh_embedded_target
        wlh_host_core
        wlh_protocol
        wlh_nanopb_runtime
        wlh_protocol_nanopb
        wlh_sim_nanopb
        wlh_freertos_osal
        wlh_log_backend)
    if(TARGET ${wlh_embedded_target})
        set_target_properties(
            ${wlh_embedded_target} PROPERTIES POSITION_INDEPENDENT_CODE OFF
        )
    endif()
endforeach()
