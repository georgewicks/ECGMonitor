
# Original
# set(COMPONENT_SRCS main.c ds3231.c i2cdev.c)
# set(COMPONENT_ADD_INCLUDEDIRS "")

# register_component()

# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# GW: I don't thin that we need the following, so commenting out for now:
# set(COMPONENT_ADD_INCLUDEDIRS "include")

set(component_srcs "ds3231.c" "i2cdev.c" )


idf_component_register(SRCS "${component_srcs}"
                    PRIV_REQUIRES driver
                    INCLUDE_DIRS "."
                    PRIV_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include")

component_compile_options(-Wno-error=format= -Wno-format)
