# target
TARGET = temp

# building variables
# debug build?
# it will effect on cflags
DEBUG = 0
# optimization
OPT = -Og



# paths
# Build path
BUILD_DIR = GCC/build

# ASM sources
ASM_SOURCES = GCC/startup_stm32f407xx.s

# source
# C sources

C_SOURCES = $(wildcard USER/*.c)
C_SOURCES += $(wildcard CORE/*.c)
C_SOURCES += $(wildcard SYSTEM/delay/*.c)
C_SOURCES += $(wildcard SYSTEM/sys/*.c)
C_SOURCES += $(wildcard SYSTEM/usart/*.c)
C_SOURCES += $(wildcard SYSTEM/shell/*.c)
C_SOURCES += $(wildcard STM32F4xx_FWLib/src/*.c)
C_SOURCES += $(wildcard HARDWARE/CONTROL/*.c)
#C_SOURCES += $(wildcard HARDWARE/OLED_IIC/*.c)
C_SOURCES += $(wildcard HARDWARE/OLED_SPI/*.c)
C_SOURCES += $(wildcard HARDWARE/BSP/*.c)
C_SOURCES += $(wildcard HARDWARE/USART/*.c)
C_SOURCES += $(wildcard HARDWARE/IIC/*.c)
C_SOURCES += $(wildcard HARDWARE/MPU6050/*.c)
C_SOURCES += $(wildcard HARDWARE/MPU6050/eMPL/*.c)

RTOS_SOURCES = $(wildcard RTOS/*.c)
RTOS_SOURCES += RTOS/portable/GCC/ARM_CM4F/port.c
RTOS_SOURCES += RTOS/portable/MemMang/heap_4.c

C_SOURCES +=$(RTOS_SOURCES)

#c++ sources

#CXX_SOURCES = $(wildcard User/*.cpp)
#CXX_SOURCES += $(wildcard USER_LIB/src/*.cpp)



# C includes
C_INCLUDES =  -ICORE
C_INCLUDES += -IUSER
C_INCLUDES += -ISTM32F4xx_FWLib/inc
C_INCLUDES += -ISYSTEM/delay 
C_INCLUDES += -ISYSTEM/sys
C_INCLUDES += -ISYSTEM/usart
C_INCLUDES += -ISYSTEM/shell
C_INCLUDES += -IHARDWARE/CONTROL
#C_INCLUDES += -IHARDWARE/OLED_IIC
C_INCLUDES += -IHARDWARE/OLED_SPI
C_INCLUDES += -IHARDWARE/BSP
C_INCLUDES += -IHARDWARE/IIC
C_INCLUDES += -IHARDWARE/MPU6050
C_INCLUDES += -IHARDWARE/MPU6050/eMPL

C_INCLUDES += -IRTOS/include 
C_INCLUDES += -IRTOS/portable/GCC/ARM_CM4F



# C++ includes
CXX_INCLUDES =  \
-IUSER_LIB/inc


#-IHead/ADC

# macros for gcc
# AS defines
AS_DEFS = 

# mybe it contens the device 
# C defines
C_DEFS =  




# AS includes
AS_INCLUDES = 




#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX= $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CXX= $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 

# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CXXFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CXXFLAGS += $(CFLAGS) $(CXX_INCLUDES)

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
CXXFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = GCC/STM32F407ZGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -u_printf_float -u_sprintf_float -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# build the application
# list of C objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of C++ objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
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

OpenOCD_PATH = C:/Program Files (x86)/openocd-20240813/OpenOCD-20240813-0.12.0/share/openocd/scripts

clean:
	-rm -fR $(BUILD_DIR)/*.o
	-rm -fR $(BUILD_DIR)/*.d
	-rm -fR $(BUILD_DIR)/*.lst
dol:
	echo "this is a test function"  
	openocd -f ./GCC/config/cmsis-dap.cfg  -f ./GCC/config/stm32f4x.cfg -c init -c "reset halt; wait_halt; flash write_image erase ./${BUILD_DIR}/$(TARGET).bin 0x08000000" -c reset -c shutdown

#-c "program ./build/make_test.elf verify reset exit 0x08000000"

rebuild_run:
	make clean
	make -j20 
	make dol
	echo "program is running......"

build_run:
	make -j20 
	make dol
	echo "program is running......"

con:
	openocd -f $(OpenOCD_PATH)/interface/cmsis-dap.cfg -f $(OpenOCD_PATH)/target/stm32f4x.cfg
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
