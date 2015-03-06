################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := c:/ti/ccsv6/tools/compiler/arm_5.1.6

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./tm4c123gh6pm_startup_ccs.obj" \
"./code/tivaware/src/cpu.obj" \
"./code/tivaware/src/gpio.obj" \
"./code/tivaware/src/interrupt.obj" \
"./code/tivaware/src/ssi.obj" \
"./code/tivaware/src/sysctl.obj" \
"./code/tivaware/src/timer.obj" \
"./code/tivaware/src/uart.obj" \
"./code/templates/src/C_SOURCE_TEMPLATE.obj" \
"./code/main.obj" \
"./code/author/src/ektm4c123gxl.obj" \
"./code/author/src/functions_gpio.obj" \
"./code/author/src/functions_spi.obj" \
"./code/author/src/functions_tim.obj" \
"./code/author/src/functions_uart.obj" \
"./code/author/src/mfrc522.obj" \
"../tm4c123gh6pm.cmd" \
-l"libc.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include code/tivaware/src/subdir_vars.mk
-include code/templates/src/subdir_vars.mk
-include code/subdir_vars.mk
-include code/author/src/subdir_vars.mk
-include subdir_rules.mk
-include code/tivaware/src/subdir_rules.mk
-include code/templates/src/subdir_rules.mk
-include code/subdir_rules.mk
-include code/author/src/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
RFID\ Project.out \

EXE_OUTPUTS__QUOTED += \
"RFID Project.out" \

BIN_OUTPUTS += \
RFID\ Project.hex \

BIN_OUTPUTS__QUOTED += \
"RFID Project.hex" \


# All Target
all: RFID\ Project.out

# Tool invocations
RFID\ Project.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.6/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --display_error_number --diag_warning=225 --diag_wrap=off -z -m"RFID Project.map" --heap_size=1024 --stack_size=1024 -i"c:/ti/ccsv6/tools/compiler/arm_5.1.6/lib" -i"c:/ti/ccsv6/tools/compiler/arm_5.1.6/include" --reread_libs --warn_sections --display_error_number --diag_wrap=off --xml_link_info="RFID Project_linkInfo.xml" --rom_model -o "RFID Project.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

RFID\ Project.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"c:/ti/ccsv6/tools/compiler/arm_5.1.6/bin/armhex"  -o "RFID Project.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "tm4c123gh6pm_startup_ccs.pp" "code\tivaware\src\cpu.pp" "code\tivaware\src\gpio.pp" "code\tivaware\src\interrupt.pp" "code\tivaware\src\ssi.pp" "code\tivaware\src\sysctl.pp" "code\tivaware\src\timer.pp" "code\tivaware\src\uart.pp" "code\templates\src\C_SOURCE_TEMPLATE.pp" "code\main.pp" "code\author\src\ektm4c123gxl.pp" "code\author\src\functions_gpio.pp" "code\author\src\functions_spi.pp" "code\author\src\functions_tim.pp" "code\author\src\functions_uart.pp" "code\author\src\mfrc522.pp" 
	-$(RM) "tm4c123gh6pm_startup_ccs.obj" "code\tivaware\src\cpu.obj" "code\tivaware\src\gpio.obj" "code\tivaware\src\interrupt.obj" "code\tivaware\src\ssi.obj" "code\tivaware\src\sysctl.obj" "code\tivaware\src\timer.obj" "code\tivaware\src\uart.obj" "code\templates\src\C_SOURCE_TEMPLATE.obj" "code\main.obj" "code\author\src\ektm4c123gxl.obj" "code\author\src\functions_gpio.obj" "code\author\src\functions_spi.obj" "code\author\src\functions_tim.obj" "code\author\src\functions_uart.obj" "code\author\src\mfrc522.obj" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

