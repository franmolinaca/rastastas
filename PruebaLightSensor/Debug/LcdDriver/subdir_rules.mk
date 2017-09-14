################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
LcdDriver/Crystalfontz128x128_ST7735.obj: ../LcdDriver/Crystalfontz128x128_ST7735.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP432 Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Users/franmolinaca/Git/rastastas/PruebaLightSensor" --include_path="/Users/franmolinaca/ti/simplelink_msp432p4_sdk_1_50_00_12/source" --include_path="/Users/franmolinaca/ti/simplelink_msp432p4_sdk_1_50_00_12/source/third_party/CMSIS/Include" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --advice:power=none --define=__MSP432P401R__ --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="LcdDriver/Crystalfontz128x128_ST7735.d" --obj_directory="LcdDriver" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.obj: ../LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP432 Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/Users/franmolinaca/Git/rastastas/PruebaLightSensor" --include_path="/Users/franmolinaca/ti/simplelink_msp432p4_sdk_1_50_00_12/source" --include_path="/Users/franmolinaca/ti/simplelink_msp432p4_sdk_1_50_00_12/source/third_party/CMSIS/Include" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.3.LTS/include" --advice:power=none --define=__MSP432P401R__ --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.d" --obj_directory="LcdDriver" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


