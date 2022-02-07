################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
libinverter/%.obj: ../libinverter/%.cpp $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="/Users/bo/prg/workspace_v11/tm3n" --include_path="/Users/bo/prg/workspace_v11/tm3n/device_support" --include_path="/Users/bo/prg/workspace_v11/tm3n/libinverter" --include_path="/Users/bo/ti/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/driverlib/f2837xd/driverlib" --include_path="/Applications/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --advice:performance=all --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="libinverter/$(basename $(<F)).d_raw" --obj_directory="libinverter" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


