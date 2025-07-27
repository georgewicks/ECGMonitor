
# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# GW: Note! TODO - probably want to make Kconfig items for every component. Since I2C devices
# require the pin assignments for SDA and SCL, we should be using common definitions
# across the board, so to speak

set(component_srcs "generic_i2c_register.c")

idf_component_register(SRCS "${component_srcs}"
                    PRIV_REQUIRES driver
                    INCLUDE_DIRS ".")

# In the CMakeLists.txt of the 'helper' component
#idf_component_register(
#    SRCS "helper.c"
#    INCLUDE_DIRS "include" # Directory containing public headers like helper.h
#)


component_compile_options(-Wno-error=format= -Wno-format)
