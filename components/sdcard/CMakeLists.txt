# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# set(COMPONENT_ADD_INCLUDEDIRS "include")

set(component_srcs "sdcard.c")

# Need fatfs for the sdcard functions
idf_component_register(SRCS "${component_srcs}"
                    REQUIRES generic_i2c sdmmc
                    PRIV_REQUIRES driver fatfs
                    INCLUDE_DIRS "." PRIV_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include" )

component_compile_options(-Wno-error=format= -Wno-format)
                    