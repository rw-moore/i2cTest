# CMake file to setup rules for using the ARM cross compiler

# Prevent compiling executables for tests
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# Use the ARM gcc compiler in standard rules
set(toolchain_prefix arm-none-eabi-)

# setup the compilers
set(CMAKE_C_COMPILER ${toolchain_prefix}gcc)
set(CMAKE_CXX_COMPILER ${toolchain_prefix}g++)
set(CMAKE_ASM_COMPILER ${toolchain_prefix}gcc)

# Setup other tools
set(CMAKE_OBJCOPY ${toolchain_prefix}objcopy)
set(CMAKE_AR ${toolchain_prefix}ar)
set(CMAKE_RANLIB ${toolchain_prefix}ranlib)
set(CMAKE_OBJCOPY ${toolchain_prefix}objcopy)
set(CMAKE_SIZE ${toolchain_prefix}size)

# Setup the compiler flags
set(common_flags "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb ")
set(CMAKE_C_FLAGS "${common_flags}")
set(CMAKE_ASM_FLAGS "-x assembler-with-cpp ${common_flags}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-specs=nano.specs -Wl,--gc-sections -Wl,--gc-sections -lc -lm -lnosys")
set(CMAKE_EXE_LINKER_FLAGS "-specs=nano.specs -T${PROJECT_SOURCE_DIR}/STM32H743ZITx_FLASH.ld -Wl,--gc-sections -Wl,--gc-sections -lc -lm -lnosys")

# Enable assembly language compilation
enable_language(ASM)

# Create function to compile firmware
function(make_firmware INPUT)
    add_custom_command(TARGET ${INPUT}
            COMMAND ${CMAKE_OBJCOPY} -O binary ${INPUT} ${INPUT}.bin
            COMMENT "Extracting binary from ${INPUT} to make compatible firmware")
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${INPUT}.bin)
endfunction(make_firmware)
