## This cmakelist.txt file was generated from
## the UAV Toolbox Support Package for PX4 Autopilots

set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")
set(MAVLINK_DIALECT_UAVIONIX "uAvionix")

add_definitions(
-DMODEL=offline_snac_SITL -DNUMST=1 -DNCSTATES=0 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0 -DXCP_DAQ_SUPPORT -DXCP_CALIBRATION_SUPPORT -DXCP_TIMESTAMP_SUPPORT -DXCP_TIMESTAMP_BASED_ON_SIMULATION_TIME -DXCP_SET_MTA_SUPPORT -DEXTMODE_XCP_TRIGGER_SUPPORT -DINTERNAL_XCP_MEM_BLOCK_1_SIZE=128 -DINTERNAL_XCP_MEM_BLOCK_1_NUMBER=1 -DINTERNAL_XCP_MEM_BLOCK_2_SIZE=168 -DINTERNAL_XCP_MEM_BLOCK_2_NUMBER=4 -DINTERNAL_XCP_MEM_BLOCK_3_SIZE=304 -DINTERNAL_XCP_MEM_BLOCK_3_NUMBER=4 -DINTERNAL_XCP_MEM_RESERVED_POOLS_TOTAL_SIZE=6070 -DINTERNAL_XCP_MEM_RESERVED_POOLS_NUMBER=5 -DXCP_MEM_DAQ_RESERVED_POOL_BLOCKS_NUMBER=3 -DXCP_MEM_DAQ_RESERVED_POOLS_NUMBER=1 -DXCP_MIN_EVENT_NO_RESERVED_POOL=1 -DXCP_MAX_CTO_SIZE=255 -DXCP_MAX_DTO_SIZE=65532 -DXCP_MAX_ODT_ENTRY_SIZE=255 -DEXTMODE_STATIC -DEXTMODE_STATIC_SIZE=1000000 -DON_TARGET_WAIT_FOR_START=1 -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTID01EQ=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DEXT_MODE=1 -DINTEGER_CODE=0 -DMT=0 -DPX4 -DHOST_TARGET -DNULL=0 -DMW_PX4_POSIX_BUILD -DEXTMODE_DISABLETESTING -DEXTMODE_DISABLEPRINTF -DEXTMODE_DISABLE_ARGS_PROCESSING=1 -D__linux__ -DXCP_CUSTOM_PLATFORM -DSTACK_SIZE=64 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DRT -D_GNU_LINUX -DPORTABLE_WORDSIZES )

px4_add_module(
    MODULE modules__px4_simulink_app
    MAIN px4_simulink_app
    STACK_MAIN 2000
SRCS
   MW_uORB_Read.cpp
   MW_uORB_Write.cpp
   MW_PX4_PWM.cpp
   xcp_ext_mode.c
   ert_main.cpp
   offline_snac_SITL.cpp
   offline_snac_SITL_data.cpp
   rtGetInf.cpp
   rtGetNaN.cpp
   rt_nonfinite.cpp
   xcp_ext_common.c
   xcp_ext_classic_trigger.c
   xcp.c
   xcp_standard.c
   xcp_daq.c
   xcp_calibration.c
   xcp_fifo.c
   xcp_transport.c
   xcp_mem_default.c
   xcp_drv_rtiostream.c
   xcp_utils.c
   xcp_frame_tcp.c
   xcp_ext_param_default_tcp.c
   MW_PX4_TaskControl.cpp
   nuttxinitialize.cpp
   MW_PX4_rtiostream_tcpip.cpp
   xcp_platform_custom.cpp
    COMPILE_FLAGS
       -fpermissive
       -Wno-narrowing
       -Wno-address-of-packed-member 
       -Wno-cast-align
    INCLUDES
       ${PX4_SOURCE_DIR}/src/modules/mavlink
       ${MAVLINK_LIBRARY_DIR}
       ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
       ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}
       /home/engelhardt/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023
       /home/engelhardt/Documents/MATLAB/SupportPackages/R2024a/toolbox/target/supportpackages/px4/include
       /home/engelhardt/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023/offline_snac_SITL_ert_rtw
       /usr/local/MATLAB/R2024a/extern/include
       /usr/local/MATLAB/R2024a/simulink/include
       /usr/local/MATLAB/R2024a/rtw/c/src
       /usr/local/MATLAB/R2024a/rtw/c/src/ext_mode/common
       /usr/local/MATLAB/R2024a/rtw/c/ert
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/server/include
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/server/common
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/server/protocol/src
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/server/protocol/include
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/server/transport/include
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/server/transport/src
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/server/platform/include
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/server/platform/default
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/ext_mode/include
       /usr/local/MATLAB/R2024a/toolbox/coder/xcp/src/target/ext_mode/src
       /usr/local/MATLAB/R2024a/toolbox/coder/rtiostream/src
)

get_target_property(PX4_SL_APP_COMPILE_FLAGS modules__px4_simulink_app COMPILE_OPTIONS)
list(REMOVE_ITEM PX4_SL_APP_COMPILE_FLAGS -Werror)
set_target_properties(modules__px4_simulink_app PROPERTIES COMPILE_OPTIONS "${PX4_SL_APP_COMPILE_FLAGS}")

