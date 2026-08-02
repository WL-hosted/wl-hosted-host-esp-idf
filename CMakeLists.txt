cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(DEPENDENCIES_LOCK dependencies.lock.${IDF_TARGET})
project(wlh_host_esp_idf C)

# Post-build: create an SHA256-prefixed copy of the project ELF. The helper
# script lives in the sibling wl-hosted-tools repo; skip when it is absent.
set(WLH_ELF_SHA256_SCRIPT ${CMAKE_SOURCE_DIR}/../wl-hosted-tools/add_elf_with_sha256.py)
if(EXISTS ${WLH_ELF_SHA256_SCRIPT})
    idf_build_get_property(python PYTHON)
    add_custom_command(
        TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${python} ${WLH_ELF_SHA256_SCRIPT} $<TARGET_FILE:${PROJECT_NAME}.elf>
        COMMENT "Generating SHA256-suffixed ELF copy"
    )
else()
    message(STATUS "wlh: add_elf_with_sha256.py not found, skipping SHA256-suffixed ELF copy")
endif()
