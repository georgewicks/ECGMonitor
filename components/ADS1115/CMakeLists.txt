
# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# GW: Note! TODO - probably want to make Kconfig items for every component. Since I2C devices
# require the pin assignments for SDA and SCL, we should be using common definitions
# across the board, so to speak

set(component_srcs "ADS1115.c")

    # In the CMakeLists.txt of the component that needs to include headers
    # Example: 'main' component needs headers from 'helper' component
#    idf_component_register(
#        SRCS "main.c"
#        REQUIRES "helper"  # Public dependency, headers of 'helper' are exposed to components that depend on 'main'
#        # or
#        # PRIV_REQUIRES "helper" # Private dependency, headers of 'helper' are only available within 'main'
#    )

idf_component_register(SRCS "${component_srcs}"
                    REQUIRES generic_i2c
                    PRIV_REQUIRES driver
                    INCLUDE_DIRS ".")

component_compile_options(-Wno-error=format= -Wno-format)
