add_subdirectory(hal_libraries/ch32v003_hal)
add_subdirectory(hal_libraries/bmsis)
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/Ld/Link.ld)
target_link_options(${TARGET_ELF} PUBLIC -T${LINKER_SCRIPT})
target_link_options(${TARGET_ELF} PUBLIC -Wl,-Map=$<TARGET_FILE:${TARGET_ELF}>.map)
