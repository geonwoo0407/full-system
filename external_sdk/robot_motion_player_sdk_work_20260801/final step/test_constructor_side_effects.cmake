if(NOT DEFINED SDK_SOURCE_DIR)
    message(FATAL_ERROR "SDK_SOURCE_DIR is required")
endif()

file(READ "${SDK_SOURCE_DIR}/step_dynamixel.cpp" dxl_source)
file(READ "${SDK_SOURCE_DIR}/dynamixel_motion_hardware.cpp" hardware_source)

string(FIND "${dxl_source}" "Dxl::Dxl()" constructor_start)
string(FIND "${dxl_source}" "bool Dxl::Preflight() noexcept" preflight_start)
string(FIND "${dxl_source}" "bool Dxl::Initialize() noexcept" initialize_start)
if(constructor_start EQUAL -1 OR preflight_start EQUAL -1
   OR initialize_start EQUAL -1 OR NOT constructor_start LESS preflight_start
   OR NOT preflight_start LESS initialize_start)
    message(FATAL_ERROR "Could not locate Dxl constructor/Preflight/Initialize definitions")
endif()
math(EXPR constructor_length "${preflight_start} - ${constructor_start}")
string(SUBSTRING "${dxl_source}" ${constructor_start}
       ${constructor_length} constructor_source)

foreach(forbidden_call openPort setBaudRate SetTorqueEnabled write1ByteTxRx
                       write2ByteTxRx read1ByteTxRx read2ByteTxRx read4ByteTxRx)
    string(FIND "${constructor_source}" "${forbidden_call}(" forbidden_position)
    if(NOT forbidden_position EQUAL -1)
        message(FATAL_ERROR
            "Dxl constructor contains forbidden hardware call: ${forbidden_call}")
    endif()
endforeach()
string(FIND "${constructor_source}" "dxl_id.push_back" constructor_id_filter)
if(NOT constructor_id_filter EQUAL -1)
    message(FATAL_ERROR "Dxl constructor must not filter or populate motor IDs")
endif()

string(FIND "${dxl_source}" "Dxl::~Dxl()" destructor_start)
if(destructor_start EQUAL -1 OR NOT initialize_start LESS destructor_start)
    message(FATAL_ERROR "Could not locate Dxl destructor after Initialize")
endif()
math(EXPR initialize_length "${destructor_start} - ${initialize_start}")
string(SUBSTRING "${dxl_source}" ${initialize_start}
       ${initialize_length} initialize_source)
math(EXPR preflight_length "${initialize_start} - ${preflight_start}")
string(SUBSTRING "${dxl_source}" ${preflight_start}
       ${preflight_length} dxl_preflight_source)

foreach(required_call ValidateDynamixelMotionHardwareConfig dxl_id.push_back
                      openPort setBaudRate write1ByteTxOnly clearPort
                      read1ByteTxRx DxlReg_TorqueEnable)
    string(FIND "${dxl_preflight_source}" "${required_call}" required_position)
    if(required_position EQUAL -1)
        message(FATAL_ERROR
            "Dxl::Preflight does not contain required operation: ${required_call}")
    endif()
endforeach()
foreach(forbidden_register DxlReg_OperatingMode DxlReg_PositionPGain
                           DxlReg_PositionIGain DxlReg_PositionDGain
                           DxlReg_DriveMode DxlReg_ProfileAcceleration
                           DxlReg_ProfileVelocity)
    string(FIND "${dxl_preflight_source}" "${forbidden_register}" forbidden_position)
    if(NOT forbidden_position EQUAL -1)
        message(FATAL_ERROR
            "Dxl::Preflight writes or references forbidden setting: ${forbidden_register}")
    endif()
endforeach()
string(FIND "${dxl_preflight_source}" "bool torque_off_verified = true" aggregate_start)
string(FIND "${dxl_preflight_source}" "for (const uint8_t id : dxl_id)" tx_loop)
string(FIND "${dxl_preflight_source}" "uint8_t torque_enabled = 1" read_loop)
string(FIND "${dxl_preflight_source}" "if (!torque_off_verified)" aggregate_failure)
string(FIND "${dxl_preflight_source}" "torque_enabled != 0" torque_readback_check)
string(FIND "${dxl_preflight_source}" "result != COMM_SUCCESS" tx_failure_check)
string(FIND "${dxl_preflight_source}" "dxl_error != 0" read_failure_check)
if(aggregate_start EQUAL -1 OR tx_loop EQUAL -1 OR read_loop EQUAL -1
   OR aggregate_failure EQUAL -1 OR torque_readback_check EQUAL -1
   OR tx_failure_check EQUAL -1 OR read_failure_check EQUAL -1
   OR NOT tx_loop LESS read_loop OR NOT read_loop LESS aggregate_failure)
    message(FATAL_ERROR "Dxl::Preflight must reject transmit, read, and nonzero readback failures")
endif()

foreach(required_call Preflight DxlReg_OperatingMode SetPIDGain)
    string(FIND "${initialize_source}" "${required_call}" required_position)
    if(required_position EQUAL -1)
        message(FATAL_ERROR
            "Dxl::Initialize does not contain required operation: ${required_call}")
    endif()
endforeach()
string(FIND "${initialize_source}" "write1ByteTxOnly(" mode_tx_only)
string(FIND "${initialize_source}" "write1ByteTxRx(" mode_tx_rx)
string(FIND "${initialize_source}" "read1ByteTxRx(" mode_readback)
string(FIND "${initialize_source}" "verified_mode !=" mode_value_check)
string(FIND "${initialize_source}" "if (!SetPIDGain(PID_Gain))" pid_result_check)
string(FIND "${initialize_source}" "motion_configured_ = true" configured_assignment)
if(mode_tx_only EQUAL -1 OR mode_readback EQUAL -1
   OR mode_value_check EQUAL -1 OR pid_result_check EQUAL -1
   OR configured_assignment EQUAL -1 OR NOT mode_tx_rx EQUAL -1
   OR NOT mode_tx_only LESS mode_readback
   OR NOT pid_result_check LESS configured_assignment)
    message(FATAL_ERROR
        "Dxl::Initialize must verify TxOnly mode/PID configuration before becoming ready")
endif()

string(FIND "${dxl_source}" "bool Dxl::SetPIDGain" pid_start)
string(FIND "${dxl_source}" "int16_t Dxl::SetPresentMode" pid_end)
if(pid_start EQUAL -1 OR pid_end EQUAL -1 OR NOT pid_start LESS pid_end)
    message(FATAL_ERROR "Could not locate Dxl::SetPIDGain")
endif()
math(EXPR pid_length "${pid_end} - ${pid_start}")
string(SUBSTRING "${dxl_source}" ${pid_start} ${pid_length} pid_source)
foreach(required_pid_operation write2ByteTxOnly read2ByteTxRx clearPort
                               DxlReg_PositionPGain DxlReg_PositionIGain
                               DxlReg_PositionDGain "verified_gain != gain.value")
    string(FIND "${pid_source}" "${required_pid_operation}" pid_operation)
    if(pid_operation EQUAL -1)
        message(FATAL_ERROR
            "Dxl::SetPIDGain lacks verified TxOnly operation: ${required_pid_operation}")
    endif()
endforeach()
string(FIND "${pid_source}" "write2ByteTxRx(" pid_tx_rx)
if(NOT pid_tx_rx EQUAL -1)
    message(FATAL_ERROR "Dxl::SetPIDGain must not depend on write status replies")
endif()

string(FIND "${dxl_source}" "bool Dxl::ConfigureTimeBasedProfile()" profile_start)
string(FIND "${dxl_source}" "bool Dxl::syncWriteTimeBasedTheta" profile_end)
if(profile_start EQUAL -1 OR profile_end EQUAL -1
   OR NOT profile_start LESS profile_end)
    message(FATAL_ERROR "Could not locate Dxl::ConfigureTimeBasedProfile")
endif()
math(EXPR profile_length "${profile_end} - ${profile_start}")
string(SUBSTRING "${dxl_source}" ${profile_start}
       ${profile_length} profile_source)
foreach(required_profile_operation write1ByteTxOnly clearPort read1ByteTxRx
                                   "verified_mode & DRIVE_MODE_TIME_BASED_BIT")
    string(FIND "${profile_source}" "${required_profile_operation}" profile_operation)
    if(profile_operation EQUAL -1)
        message(FATAL_ERROR
            "Time-based Drive Mode lacks TxOnly/readback verification: ${required_profile_operation}")
    endif()
endforeach()
string(FIND "${profile_source}" "write1ByteTxRx(" profile_tx_rx)
if(NOT profile_tx_rx EQUAL -1)
    message(FATAL_ERROR "Drive Mode write must not depend on a write status reply")
endif()

string(FIND "${dxl_source}" "bool Dxl::SetTorqueEnabled(bool enabled)" torque_start)
string(FIND "${dxl_source}" "bool Dxl::IsReady() const" torque_end)
if(torque_start EQUAL -1 OR torque_end EQUAL -1
   OR NOT torque_start LESS torque_end)
    message(FATAL_ERROR "Could not locate Dxl torque enable implementation")
endif()
math(EXPR torque_length "${torque_end} - ${torque_start}")
string(SUBSTRING "${dxl_source}" ${torque_start}
       ${torque_length} torque_source)
foreach(required_torque_operation write1ByteTxOnly read1ByteTxRx clearPort
                                  DxlReg_TorqueEnable
                                  "actual_value == requested_value"
                                  DisableTorqueBestEffort)
    string(FIND "${torque_source}" "${required_torque_operation}" torque_operation)
    if(torque_operation EQUAL -1)
        message(FATAL_ERROR
            "Torque control lacks TxOnly/readback/rollback operation: ${required_torque_operation}")
    endif()
endforeach()
string(FIND "${torque_source}" "write1ByteTxRx(" torque_tx_rx)
string(FIND "${torque_source}" "if (!enabled) return DisableTorqueBestEffort()"
       torque_off_best_effort)
string(FIND "${torque_source}" "DisableTorqueBestEffort();" torque_rollback)
string(FIND "${torque_source}" "for (const uint8_t id : dxl_id)"
       torque_all_ids_loop)
if(NOT torque_tx_rx EQUAL -1 OR torque_off_best_effort EQUAL -1
   OR torque_rollback EQUAL -1 OR torque_all_ids_loop EQUAL -1)
    message(FATAL_ERROR
        "Torque OFF must cover all IDs and Torque ON failure must roll back without TxRx writes")
endif()
foreach(retry_policy "if (motion_configured_ && port_opened_)"
                     "motion_configured_ = false"
                     "motion_configured_ = true")
    string(FIND "${initialize_source}" "${retry_policy}" policy_position)
    if(policy_position EQUAL -1)
        message(FATAL_ERROR
            "Dxl::Initialize retry policy is missing: ${retry_policy}")
    endif()
endforeach()

string(LENGTH "${dxl_source}" source_length)
math(EXPR destructor_length "${source_length} - ${destructor_start}")
string(SUBSTRING "${dxl_source}" ${destructor_start}
       ${destructor_length} destructor_source)
string(FIND "${destructor_source}" "if (!port_opened_) return;"
       destructor_guard)
string(FIND "${destructor_source}" "portHandler->closePort()"
       destructor_close)
if(destructor_guard EQUAL -1 OR destructor_close EQUAL -1
   OR NOT destructor_guard LESS destructor_close)
    message(FATAL_ERROR "Dxl destructor does not guard closePort by open state")
endif()

string(FIND "${hardware_source}"
       "bool DynamixelMotionHardware::initialize() noexcept" hardware_initialize_start)
string(FIND "${hardware_source}"
       "bool DynamixelMotionHardware::preflight() noexcept" hardware_preflight_start)
string(FIND "${hardware_source}"
       "bool DynamixelMotionHardware::preflightReady() const noexcept"
       hardware_preflight_end)
if(hardware_preflight_start EQUAL -1 OR hardware_preflight_end EQUAL -1
   OR NOT hardware_preflight_start LESS hardware_preflight_end)
    message(FATAL_ERROR "Could not locate DynamixelMotionHardware preflight")
endif()
math(EXPR hardware_preflight_length
     "${hardware_preflight_end} - ${hardware_preflight_start}")
string(SUBSTRING "${hardware_source}" ${hardware_preflight_start}
       ${hardware_preflight_length} hardware_preflight_source)
string(FIND "${hardware_preflight_source}" "active_dxl_->Initialize()"
       dxl_initialize_call)
string(FIND "${hardware_preflight_source}" "active_dxl_->Preflight()"
       dxl_preflight_call)
string(FIND "${hardware_preflight_source}" "GetJointTheta()"
       preflight_position_read_call)
string(FIND "${hardware_preflight_source}" "SetTorqueEnabled(true)"
       preflight_torque_enable_call)
if(dxl_preflight_call EQUAL -1 OR preflight_position_read_call EQUAL -1)
    message(FATAL_ERROR
        "DynamixelMotionHardware preflight must run safe Dxl preflight and read positions")
endif()
if(NOT dxl_initialize_call EQUAL -1 OR NOT preflight_torque_enable_call EQUAL -1)
    message(FATAL_ERROR
        "DynamixelMotionHardware preflight must never run motion initialization or enable torque")
endif()

string(FIND "${hardware_source}" "ConfigureTimeBasedProfile()" profile_call)
string(FIND "${hardware_source}" "SetTorqueEnabled(true)" torque_enable_call)
string(FIND "${hardware_source}" "if (initialized_ && active_dxl_->IsReady())"
       initialized_guard)
if(hardware_initialize_start EQUAL -1 OR profile_call EQUAL -1
   OR torque_enable_call EQUAL -1 OR initialized_guard EQUAL -1
   OR NOT hardware_initialize_start LESS profile_call
   OR NOT profile_call LESS torque_enable_call)
    message(FATAL_ERROR
        "DynamixelMotionHardware initialization order is incorrect")
endif()

string(FIND "${hardware_source}" "bool DynamixelMotionHardware::ready() const noexcept"
       hardware_initialize_end)
math(EXPR hardware_initialize_length
     "${hardware_initialize_end} - ${hardware_initialize_start}")
string(SUBSTRING "${hardware_source}" ${hardware_initialize_start}
       ${hardware_initialize_length} hardware_initialize_source)
string(FIND "${hardware_initialize_source}" "preflight()" preflight_call)
string(FIND "${hardware_initialize_source}" "active_dxl_->Initialize()"
       motion_initialize_call)
string(FIND "${hardware_initialize_source}" "ConfigureTimeBasedProfile()"
       initialize_profile_call)
string(FIND "${hardware_initialize_source}" "GetJointTheta()" position_read_call)
string(FIND "${hardware_initialize_source}" "initialized_ = true"
       initialized_assignment)
string(FIND "${hardware_initialize_source}" "SetTorqueEnabled(true)"
       initialize_torque_enable_call)
if(preflight_call EQUAL -1 OR motion_initialize_call EQUAL -1
   OR position_read_call EQUAL -1
   OR initialize_profile_call EQUAL -1 OR initialize_torque_enable_call EQUAL -1
   OR initialized_assignment EQUAL -1
   OR NOT preflight_call LESS initialize_profile_call
   OR NOT motion_initialize_call LESS initialize_profile_call
   OR NOT initialize_profile_call LESS position_read_call
   OR NOT position_read_call LESS initialize_torque_enable_call
   OR NOT initialize_torque_enable_call LESS initialized_assignment)
    message(FATAL_ERROR
        "Torque must be the final motion-ready operation in initialize")
endif()
