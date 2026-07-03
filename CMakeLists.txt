cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME R1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME} Src/main.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  Middlewares/Third_Party/XHU_RRC_LIB_V1_4_mini/Src/Critical_Section.c
  Middlewares/Third_Party/XHU_RRC_LIB_V1_4_mini/Src/DJI_Motor.c
  Middlewares/Third_Party/XHU_RRC_LIB_V1_4_mini/Src/HEAP.c
  Middlewares/Third_Party/XHU_RRC_LIB_V1_4_mini/Src/PID.c
  Drivers/BSP/Src/bsp_led.c
  Drivers/BSP/Src/can_receive.c
  Drivers/BSP/Src/debug_assistant.c
  Src/chassis.c
  Src/platform.c
  Src/remote_ctrl.c
  Src/robot_arm.c
  Src/safe_task.c
  Src/user_keymapping.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
  Middlewares/Third_Party/XHU_RRC_LIB_V1_4_mini/Conf
  Middlewares/Third_Party/XHU_RRC_LIB_V1_4_mini/Inc
  Inc
  Drivers/BSP/Inc
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
  ARM_MATH_CM4
  XHU_RRC_LIB_ENABLE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
