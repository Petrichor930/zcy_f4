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
set(CMAKE_PROJECT_NAME a)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# 关闭指定警告
add_compile_options(-Wno-unused-variable)
add_compile_options(-Wno-unused-but-set-variable)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Wno-unused-function)
add_compile_options(-Wno-unused-value)
add_compile_options(-Wno-sign-compare)
add_compile_options(-Wno-missing-braces)
add_compile_options(-Wno-sequence-point)
add_compile_options(-Wno-pedantic)
add_compile_options(-Wno-deprecated-volatile)
add_compile_options(-Wno-missing-field-initializers)
# Create an executable object type
# 添加.c文件
file(GLOB_RECURSE SOURCES "robot_core/*.*")
add_executable(${CMAKE_PROJECT_NAME}  ${SOURCES} )

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)



# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
macro(FIND_INCLUDE_DIR result curdir)
  file(GLOB_RECURSE children "${curdir}/*.hpp" "${curdir}/*.h")
  set(dirlist "")
  foreach(child ${children})
    string(REGEX REPLACE "(.*)/.*" "\\1" LIB_NAME ${child})
    if(IS_DIRECTORY ${LIB_NAME})
      list(FIND dirlist ${LIB_NAME} list_index)

      if(${list_index} LESS 0)
        list(APPEND dirlist ${LIB_NAME})
      endif()
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()


find_include_dir(INCLUDE_DIR_LIST robot_core)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${INCLUDE_DIR_LIST})

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
