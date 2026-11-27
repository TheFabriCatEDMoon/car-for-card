# -----------------------------------------------------------------------------
# File: gcc-arm-none-eabi.cmake
#
# GCC ARM embedded toolchain configuration for STM32 projects.
# This file configures the compiler, linker, and library
# options for STM32 targets using the arm-none-eabi toolchain.
#
# NOTE:
#   This file is generated only once,
#   User is free to modify the file as much as necessary.
# -----------------------------------------------------------------------------
set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

# 只写普通变量的话 CMake 不会把 CMAKE_<LANG>_COMPILER 放进 CMakeCache.txt，
# 而 VS Code CMake Tools 生成调试配置时要读缓存里的编译器，缺了就报
# "在缓存文件中找不到编译器"。这里解析成绝对路径并存成缓存变量。
find_program(TOOLCHAIN_C_COMPILER   NAMES ${TOOLCHAIN_PREFIX}gcc REQUIRED)
find_program(TOOLCHAIN_CXX_COMPILER NAMES ${TOOLCHAIN_PREFIX}g++ REQUIRED)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_C_COMPILER}   CACHE FILEPATH "")
set(CMAKE_ASM_COMPILER              ${TOOLCHAIN_C_COMPILER}   CACHE FILEPATH "")
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_CXX_COMPILER} CACHE FILEPATH "")
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m3 ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

# 2.0 的 EIDE 工程用的是 -Og（EIDE 的 "level-debug"），换成 -O0 会多出 11 KB 代码，
# 64 KB Flash 只剩 4% 余量。这里跟 2.0 和 starm-clang.cmake 保持一致，仍可单步调试。
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F103xx_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
