##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.1.0] date: [Tue Mar 26 13:58:38 CDT 2019]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = testProj


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# STM32CUBE directory
STMCUBE_DIR=/mnt/d/moore/STM32Cube/Repository/STM32Cube_FW_H7_V1.4.0

# STM HAL driver directory
STM_HAL_DRIVER_DIR=$(STMCUBE_DIR)/Drivers/STM32H7xx_HAL_Driver/Src

# STM Network middleware directory
STM_NET_DIR=$(STMCUBE_DIR)/Middlewares/Third_Party/LwIP/src

STM_HAL_DRIVERS= \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_eth.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_eth_ex.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_cortex.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_tim.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_tim_ex.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_uart.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_uart_ex.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_pcd.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_pcd_ex.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_ll_usb.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_rcc.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_rcc_ex.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_flash.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_flash_ex.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_gpio.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_hsem.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_dma.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_dma_ex.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_mdma.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_pwr.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_pwr_ex.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_i2c.c \
$(STM_HAL_DRIVER_DIR)/stm32h7xx_hal_i2c_ex.c

STM_NET= \
$(STM_NET_DIR)/netif/ppp/auth.c \
$(STM_NET_DIR)/netif/ppp/ccp.c \
$(STM_NET_DIR)/netif/ppp/chap_ms.c \
$(STM_NET_DIR)/netif/ppp/chap-md5.c \
$(STM_NET_DIR)/netif/ppp/chap-new.c \
$(STM_NET_DIR)/netif/ppp/demand.c \
$(STM_NET_DIR)/netif/ppp/eap.c \
$(STM_NET_DIR)/netif/ppp/eui64.c \
$(STM_NET_DIR)/netif/ppp/fsm.c \
$(STM_NET_DIR)/netif/ppp/ipcp.c \
$(STM_NET_DIR)/netif/ppp/ipv6cp.c \
$(STM_NET_DIR)/netif/ppp/lcp.c \
$(STM_NET_DIR)/netif/ppp/magic.c \
$(STM_NET_DIR)/netif/ppp/mppe.c \
$(STM_NET_DIR)/netif/ppp/multilink.c \
$(STM_NET_DIR)/netif/ppp/ppp.c \
$(STM_NET_DIR)/netif/ppp/pppapi.c \
$(STM_NET_DIR)/netif/ppp/pppcrypt.c \
$(STM_NET_DIR)/netif/ppp/pppoe.c \
$(STM_NET_DIR)/netif/ppp/pppol2tp.c \
$(STM_NET_DIR)/netif/ppp/pppos.c \
$(STM_NET_DIR)/netif/ppp/upap.c \
$(STM_NET_DIR)/netif/ppp/utils.c \
$(STM_NET_DIR)/netif/ppp/vj.c \
$(STM_NET_DIR)/netif/ethernet.c \
$(STM_NET_DIR)/netif/slipif.c \
$(STM_NET_DIR)/netif/lowpan6.c \
$(STM_NET_DIR)/netif/ppp/ecp.c \
$(STM_NET_DIR)/api/api_lib.c \
$(STM_NET_DIR)/api/api_msg.c \
$(STM_NET_DIR)/api/err.c \
$(STM_NET_DIR)/api/netbuf.c \
$(STM_NET_DIR)/api/netdb.c \
$(STM_NET_DIR)/api/netifapi.c \
$(STM_NET_DIR)/api/sockets.c \
$(STM_NET_DIR)/api/tcpip.c \
$(STM_NET_DIR)/core/def.c \
$(STM_NET_DIR)/core/dns.c \
$(STM_NET_DIR)/core/inet_chksum.c \
$(STM_NET_DIR)/core/init.c \
$(STM_NET_DIR)/core/ip.c \
$(STM_NET_DIR)/core/mem.c \
$(STM_NET_DIR)/core/memp.c \
$(STM_NET_DIR)/core/netif.c \
$(STM_NET_DIR)/core/pbuf.c \
$(STM_NET_DIR)/core/raw.c \
$(STM_NET_DIR)/core/stats.c \
$(STM_NET_DIR)/core/sys.c \
$(STM_NET_DIR)/core/tcp.c \
$(STM_NET_DIR)/core/tcp_in.c \
$(STM_NET_DIR)/core/tcp_out.c \
$(STM_NET_DIR)/core/timeouts.c \
$(STM_NET_DIR)/core/udp.c \
$(STM_NET_DIR)/core/ipv4/autoip.c \
$(STM_NET_DIR)/core/ipv4/dhcp.c \
$(STM_NET_DIR)/core/ipv4/etharp.c \
$(STM_NET_DIR)/core/ipv4/icmp.c \
$(STM_NET_DIR)/core/ipv4/igmp.c \
$(STM_NET_DIR)/core/ipv4/ip4.c \
$(STM_NET_DIR)/core/ipv4/ip4_addr.c \
$(STM_NET_DIR)/core/ipv4/ip4_frag.c \
$(STM_NET_DIR)/core/ipv6/dhcp6.c \
$(STM_NET_DIR)/core/ipv6/ethip6.c \
$(STM_NET_DIR)/core/ipv6/icmp6.c \
$(STM_NET_DIR)/core/ipv6/inet6.c \
$(STM_NET_DIR)/core/ipv6/ip6.c \
$(STM_NET_DIR)/core/ipv6/ip6_addr.c \
$(STM_NET_DIR)/core/ipv6/ip6_frag.c \
$(STM_NET_DIR)/core/ipv6/mld6.c \
$(STM_NET_DIR)/core/ipv6/nd6.c \
$(STM_NET_DIR)/apps/mqtt/mqtt.c  

# C sources
C_SOURCES =  \
src/main.c \
src/tcp_io.c \
src/uart_io.c \
src/comms.c \
src/lwip.c \
src/ethernetif.c \
src/stm32h7xx_it.c \
src/stm32h7xx_hal_msp.c \
$(STMCUBE_DIR)/Drivers/BSP/Components/lan8742/lan8742.c \
$(STMCUBE_DIR)/Drivers/BSP/STM32H7xx_Nucleo_144/stm32h7xx_nucleo_144.c \
$(STM_HAL_DRIVERS) \
src/system_stm32h7xx.c \
$(STM_NET)

# ASM sources
ASM_SOURCES =  \
startup_stm32h743xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m7

# fpu
FPU = -mfpu=fpv5-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32H743xx


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-Iinc \
-I$(STM_NET_DIR)/include \
-I$(STMCUBE_DIR)/Middlewares/Third_Party/LwIP/system \
-I$(STMCUBE_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc \
-I$(STMCUBE_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy \
-I$(STMCUBE_DIR)/Drivers/BSP/Components/lan8742 \
-I$(STMCUBE_DIR)/Drivers/BSP/STM32H7xx_Nucleo_144 \
-I$(STM_NET_DIR)/include/netif/ppp \
-I$(STMCUBE_DIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
-I$(STM_NET_DIR)/include/lwip \
-I$(STM_NET_DIR)/include/lwip/apps \
-I$(STM_NET_DIR)/include/lwip/priv \
-I$(STM_NET_DIR)/include/lwip/prot \
-I$(STM_NET_DIR)/include/netif \
-I$(STM_NET_DIR)/include/posix \
-I$(STM_NET_DIR)/include/posix/sys \
-I$(STMCUBE_DIR)/Middlewares/Third_Party/LwIP/system/arch \
-I$(STMCUBE_DIR)/Drivers/CMSIS/Include \
-I$(STMCUBE_DIR)/Drivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32H743ZITx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***