target_sources(${TARGET_ELF} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/source/I2C.c
    ${CMAKE_CURRENT_SOURCE_DIR}/source/ik_ina219.c
    ${CMAKE_CURRENT_SOURCE_DIR}/source/oled.c
    ${CMAKE_CURRENT_SOURCE_DIR}/include/I2C.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ik_ina219.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/oled.h
)

target_include_directories(${TARGET_ELF} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)