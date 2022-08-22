include(ExternalProject)

set(LIBOPENCM3_DIR ${CMAKE_SOURCE_DIR}/libopencm3)

if(NOT EXISTS ${LIBOPENCM3_DIR}/Makefile)
    message(FATAL_ERROR "libopencm3 not found")
endif()

if("${LIBOPENCM3_TARGET}" STREQUAL "")
    message(FATAL_ERROR "LIBOPENCM3_TARGET should not be empty")
endif()

set(LIBOPENCM3_LIB ${LIBOPENCM3_DIR}/lib/libopencm3_stm32${LIBOPENCM3_TARGET}.a)
set(LIBOPENCM3_INCLUDE_DIR ${LIBOPENCM3_DIR}/include)

add_custom_command(
    OUTPUT ${LIBOPENCM3_LIB}
    COMMAND make -j8 TARGETS=stm32/${LIBOPENCM3_TARGET} > /dev/null
    WORKING_DIRECTORY ${LIBOPENCM3_DIR}
)

add_custom_target(
    opencm3_build
    DEPENDS ${LIBOPENCM3_LIB}
)

add_library(opencm3 STATIC IMPORTED)
set_property(TARGET opencm3 PROPERTY IMPORTED_LOCATION ${LIBOPENCM3_LIB})
target_include_directories(opencm3 INTERFACE ${LIBOPENCM3_INCLUDE_DIR})
add_dependencies(opencm3 opencm3_build)
