cmake_minimum_required(VERSION 3.9.1)

# Set the cross compilation systems
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

project(sensors)
include(ARMcompiler.cmake)

# Set the STM32CUBE root directory
set(stmcube_root /home/moore/STM32Cube_FW_H7_V1.4.0)

# STM32CUBE HAL driver directory
set(stmcube_hal ${stmcube_root}/Drivers/STM32H7xx_HAL_Driver)

# STM Network middleware directory
set(stmcube_net ${stmcube_root}/Middlewares/Third_Party/LwIP/src)

# General compiler options
add_compile_options(-Wall)
add_compile_options(-fdata-sections)
add_compile_options(-ffunction-sections)

# Compiler definitions
add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DSTM32H743xx)

# Control searching since we are cross compiling
# See: https://cmake.org/cmake/help/v3.6/manual/cmake-toolchains.7.html
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Include directories
include_directories(
        inc
        ${stmcube_net}/include
        ${stmcube_root}/Middlewares/Third_Party/LwIP/system
        ${stmcube_hal}/Inc
        ${stmcube_hal}/Inc/Legacy
        ${stmcube_root}/Drivers/BSP/Components/lan8742
        ${stmcube_root}/Drivers/BSP/STM32H7xx_Nucleo_144
        ${stmcube_net}/include/netif/ppp
        ${stmcube_root}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
        ${stmcube_net}/include/lwip
        ${stmcube_net}/include/lwip/apps
        ${stmcube_net}/include/lwip/priv
        ${stmcube_net}/include/lwip/prot
        ${stmcube_net}/include/netif
        ${stmcube_net}/include/posix
        ${stmcube_net}/include/posix/sys
        ${stmcube_root}/Middlewares/Third_Party/LwIP/system/arch
        ${stmcube_root}/Drivers/CMSIS/Include
        ${stmcube_root}/Drivers/CMSIS/Include
        )

add_library(hal STATIC
        ${stmcube_hal}/Src/stm32h7xx_hal_eth.c
        ${stmcube_hal}/Src/stm32h7xx_hal_eth_ex.c
        ${stmcube_hal}/Src/stm32h7xx_hal_cortex.c
        ${stmcube_hal}/Src/stm32h7xx_hal_tim.c
        ${stmcube_hal}/Src/stm32h7xx_hal_tim_ex.c
        ${stmcube_hal}/Src/stm32h7xx_hal_uart.c
        ${stmcube_hal}/Src/stm32h7xx_hal_uart_ex.c
        ${stmcube_hal}/Src/stm32h7xx_hal_pcd.c
        ${stmcube_hal}/Src/stm32h7xx_hal_pcd_ex.c
        ${stmcube_hal}/Src/stm32h7xx_ll_usb.c
        ${stmcube_hal}/Src/stm32h7xx_hal_rcc.c
        ${stmcube_hal}/Src/stm32h7xx_hal_rcc_ex.c
        ${stmcube_hal}/Src/stm32h7xx_hal_flash.c
        ${stmcube_hal}/Src/stm32h7xx_hal_flash_ex.c
        ${stmcube_hal}/Src/stm32h7xx_hal_gpio.c
        ${stmcube_hal}/Src/stm32h7xx_hal_hsem.c
        ${stmcube_hal}/Src/stm32h7xx_hal_dma.c
        ${stmcube_hal}/Src/stm32h7xx_hal_dma_ex.c
        ${stmcube_hal}/Src/stm32h7xx_hal_mdma.c
        ${stmcube_hal}/Src/stm32h7xx_hal_pwr.c
        ${stmcube_hal}/Src/stm32h7xx_hal_pwr_ex.c
        ${stmcube_hal}/Src/stm32h7xx_hal.c
        ${stmcube_hal}/Src/stm32h7xx_hal_i2c.c
        ${stmcube_hal}/Src/stm32h7xx_hal_i2c_ex.c
        inc/lps22hb.h inc/lis3mdl.h src/lps22hb.c)

add_library(net STATIC
        ${stmcube_net}/netif/ppp/auth.c
        ${stmcube_net}/netif/ppp/ccp.c
        ${stmcube_net}/netif/ppp/chap_ms.c
        ${stmcube_net}/netif/ppp/chap-md5.c
        ${stmcube_net}/netif/ppp/chap-new.c
        ${stmcube_net}/netif/ppp/demand.c
        ${stmcube_net}/netif/ppp/eap.c
        ${stmcube_net}/netif/ppp/eui64.c
        ${stmcube_net}/netif/ppp/fsm.c
        ${stmcube_net}/netif/ppp/ipcp.c
        ${stmcube_net}/netif/ppp/ipv6cp.c
        ${stmcube_net}/netif/ppp/lcp.c
        ${stmcube_net}/netif/ppp/magic.c
        ${stmcube_net}/netif/ppp/mppe.c
        ${stmcube_net}/netif/ppp/multilink.c
        ${stmcube_net}/netif/ppp/ppp.c
        ${stmcube_net}/netif/ppp/pppapi.c
        ${stmcube_net}/netif/ppp/pppcrypt.c
        ${stmcube_net}/netif/ppp/pppoe.c
        ${stmcube_net}/netif/ppp/pppol2tp.c
        ${stmcube_net}/netif/ppp/pppos.c
        ${stmcube_net}/netif/ppp/upap.c
        ${stmcube_net}/netif/ppp/utils.c
        ${stmcube_net}/netif/ppp/vj.c
        ${stmcube_net}/netif/ethernet.c
        ${stmcube_net}/netif/slipif.c
        ${stmcube_net}/netif/lowpan6.c
        ${stmcube_net}/netif/ppp/ecp.c
        ${stmcube_net}/api/api_lib.c
        ${stmcube_net}/api/api_msg.c
        ${stmcube_net}/api/err.c
        ${stmcube_net}/api/netbuf.c
        ${stmcube_net}/api/netdb.c
        ${stmcube_net}/api/netifapi.c
        ${stmcube_net}/api/sockets.c
        ${stmcube_net}/api/tcpip.c
        ${stmcube_net}/core/def.c
        ${stmcube_net}/core/dns.c
        ${stmcube_net}/core/inet_chksum.c
        ${stmcube_net}/core/init.c
        ${stmcube_net}/core/ip.c
        ${stmcube_net}/core/mem.c
        ${stmcube_net}/core/memp.c
        ${stmcube_net}/core/netif.c
        ${stmcube_net}/core/pbuf.c
        ${stmcube_net}/core/raw.c
        ${stmcube_net}/core/stats.c
        ${stmcube_net}/core/sys.c
        ${stmcube_net}/core/tcp.c
        ${stmcube_net}/core/tcp_in.c
        ${stmcube_net}/core/tcp_out.c
        ${stmcube_net}/core/timeouts.c
        ${stmcube_net}/core/udp.c
        ${stmcube_net}/core/ipv4/autoip.c
        ${stmcube_net}/core/ipv4/dhcp.c
        ${stmcube_net}/core/ipv4/etharp.c
        ${stmcube_net}/core/ipv4/icmp.c
        ${stmcube_net}/core/ipv4/igmp.c
        ${stmcube_net}/core/ipv4/ip4.c
        ${stmcube_net}/core/ipv4/ip4_addr.c
        ${stmcube_net}/core/ipv4/ip4_frag.c
        ${stmcube_net}/core/ipv6/dhcp6.c
        ${stmcube_net}/core/ipv6/ethip6.c
        ${stmcube_net}/core/ipv6/icmp6.c
        ${stmcube_net}/core/ipv6/inet6.c
        ${stmcube_net}/core/ipv6/ip6.c
        ${stmcube_net}/core/ipv6/ip6_addr.c
        ${stmcube_net}/core/ipv6/ip6_frag.c
        ${stmcube_net}/core/ipv6/mld6.c
        ${stmcube_net}/core/ipv6/nd6.c
        ${stmcube_net}/apps/mqtt/mqtt.c
        )

add_executable(ledTest
        src/main.c
        src/tcp_io.c
        src/uart_io.c
        src/comms.c
        src/lwip.c
        src/ethernetif.c
        src/stm32h7xx_it.c
        src/stm32h7xx_hal_msp.c
        ${stmcube_root}/Drivers/BSP/Components/lan8742/lan8742.c
        ${stmcube_root}/Drivers/BSP/STM32H7xx_Nucleo_144/stm32h7xx_nucleo_144.c
        src/system_stm32h7xx.c
        startup_stm32h743xx.s
        src/i2c.c src/adxl355.c)
target_link_libraries(ledTest hal net c m nosys)
make_firmware(ledTest)
