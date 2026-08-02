cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(DEPENDENCIES_LOCK dependencies.lock.${IDF_TARGET})
project(wlh_host_esp_idf C)

# Post-build: create an SHA256-prefixed copy of the project ELF.
idf_build_get_property(python PYTHON)
add_custom_command(
    TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${python} ${CMAKE_SOURCE_DIR}/../wl-hosted-tools/add_elf_with_sha256.py $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating SHA256-suffixed ELF copy"
)
