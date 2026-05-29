set(srcs "")
set(requires "")
set(include_dirs "")

list(APPEND srcs         "src/espnow/src/espnow.c"
                         "src/espnow/src/espnow_group.c")
list(APPEND include_dirs "src/espnow/include")

list(APPEND srcs         "src/security/src/espnow_security.c")
list(APPEND include_dirs "src/security/include" "src/security/include/protocomm/security")
list(APPEND requires "mbedtls")

list(APPEND srcs         "src/utils/src/espnow_storage.c"
                         "src/utils/src/espnow_mem.c"
                         "src/utils/src/espnow_utils.c")
list(APPEND include_dirs "src/utils/include")

list(APPEND srcs         "src/debug/src/espnow_log_wrap.c")

list(APPEND requires "spi_flash" "nvs_flash" "app_update" "esp_timer" "esp_wifi")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES "${requires}")

set(WRAP_FUNCTIONS esp_log_write esp_log_writev)
foreach(func ${WRAP_FUNCTIONS})
     target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=${func}")
endforeach()

# Force esp_wifi to appear later than esp-now in link line since esp-now depends on esp_wifi.
idf_component_get_property(esp_wifi esp_wifi COMPONENT_LIB)
idf_build_get_property(build_components BUILD_COMPONENTS)

if("espressif__esp-now" IN_LIST build_components)
     idf_component_get_property(esp-now espressif__esp-now COMPONENT_LIB)
else()
     idf_component_get_property(esp-now esp-now COMPONENT_LIB)
endif()

target_link_libraries(${COMPONENT_LIB} INTERFACE $<TARGET_FILE:${esp_wifi}> $<TARGET_FILE:${esp-now}>)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-pointer-sign -Wno-format)

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
