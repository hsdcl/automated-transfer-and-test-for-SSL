# function status return values
NT_OK = 0
NT_INITIALIZATION_ERROR = 1
NT_NOT_INITIALIZED_ERROR = 2
NT_NO_SYSTEMS_FOUND_ERROR = 3
NT_TOO_MANY_SYSTEMS_ERROR = 4
NT_INVALID_SYSTEM_INDEX_ERROR = 5
NT_INVALID_CHANNEL_INDEX_ERROR = 6
NT_TRANSMIT_ERROR = 7
NT_WRITE_ERROR = 8
NT_INVALID_PARAMETER_ERROR = 9
NT_READ_ERROR = 10
NT_INTERNAL_ERROR = 12
NT_WRONG_MODE_ERROR = 13
NT_PROTOCOL_ERROR = 14
NT_TIMEOUT_ERROR = 15
NT_ID_LIST_TOO_SMALL_ERROR = 17
NT_SYSTEM_ALREADY_ADDED_ERROR = 18
NT_WRONG_CHANNEL_TYPE_ERROR = 19
NT_CANCELED_ERROR = 20
NT_INVALID_SYSTEM_LOCATOR_ERROR = 21
NT_INPUT_BUFFER_OVERFLOW_ERROR = 22
NT_QUERYBUFFER_SIZE_ERROR = 23
NT_DRIVER_ERROR = 24
NT_NO_SENSOR_PRESENT_ERROR = 129
NT_AMPLITUDE_TOO_LOW_ERROR = 130
NT_AMPLITUDE_TOO_HIGH_ERROR = 131
NT_FREQUENCY_TOO_LOW_ERROR = 132
NT_FREQUENCY_TOO_HIGH_ERROR = 133
NT_SCAN_TARGET_TOO_HIGH_ERROR = 135
NT_SCAN_SPEED_TOO_LOW_ERROR = 136
NT_SCAN_SPEED_TOO_HIGH_ERROR = 137
NT_SENSOR_DISABLED_ERROR = 140
NT_COMMAND_OVERRIDDEN_ERROR = 141
NT_END_STOP_REACHED_ERROR = 142
NT_WRONG_SENSOR_TYPE_ERROR = 143
NT_COULD_NOT_FIND_REF_ERROR = 144
NT_WRONG_END_EFFECTOR_TYPE_ERROR = 145
NT_MOVEMENT_LOCKED_ERROR = 146
NT_RANGE_LIMIT_REACHED_ERROR = 147
NT_PHYSICAL_POSITION_UNKNOWN_ERROR = 148
NT_OUTPUT_BUFFER_OVERFLOW_ERROR = 149
NT_COMMAND_NOT_PROCESSABLE_ERROR = 150
NT_WAITING_FOR_TRIGGER_ERROR = 151
NT_COMMAND_NOT_TRIGGERABLE_ERROR = 152
NT_COMMAND_QUEUE_FULL_ERROR = 153
NT_INVALID_COMPONENT_ERROR = 154
NT_INVALID_SUB_COMPONENT_ERROR = 155
NT_INVALID_PROPERTY_ERROR = 156
NT_PERMISSION_DENIED_ERROR = 157
NT_CALIBRATION_FAILED_ERROR = 160
NT_UNKNOWN_COMMAND_ERROR = 240
NT_OTHER_ERROR = 255

# general definitions
NT_UNDEFINED = 0
NT_FALSE = 0
NT_TRUE = 1
NT_DISABLED = 0
NT_ENABLED = 1
NT_FALLING_EDGE = 0
NT_RISING_EDGE = 1
NT_FORWARD = 0
NT_BACKWARD = 1


# configuration flags for NT_InitDevices
NT_SYNCHRONOUS_COMMUNICATION = 0
NT_ASYNCHRONOUS_COMMUNICATION = 1
NT_HARDWARE_RESET = 2

# return values from NT_GetInitState
NT_INIT_STATE_NONE = 0
NT_INIT_STATE_SYNC = 1
NT_INIT_STATE_ASYNC = 2

# Hand Control Module modes for NT_SetHCMEnabled
NT_HCM_DISABLED = 0
NT_HCM_ENABLED = 1

# configuration values for NT_SetAccumulateRelativePositions_X
NT_NO_ACCUMULATE_RELATIVE_POSITIONS = 0
NT_ACCUMULATE_RELATIVE_POSITIONS = 1

# configuration values for NT_SetSensorEnabled_X
NT_SENSOR_DISABLED = 0
NT_SENSOR_ENABLED = 1

# infinite timeout for functions that wait
NT_TIMEOUT_INFINITE = 0xFFFFFFFF

# packet types for asynchronous mode
NT_NO_PACKET_TYPE = 0
NT_ERROR_PACKET_TYPE = 1
NT_POSITION_PACKET_TYPE = 2
NT_COMPLETED_PACKET_TYPE = 3
NT_STATUS_PACKET_TYPE = 4
NT_ANGLE_PACKET_TYPE = 5
NT_VOLTAGE_LEVEL_PACKET_TYPE = 6
NT_SENSOR_TYPE_PACKET_TYPE = 7
NT_SENSOR_ENABLED_PACKET_TYPE = 8
NT_END_EFFECTOR_TYPE_PACKET_TYPE = 9
NT_GRIPPER_OPENING_PACKET_TYPE = 10
NT_FORCE_PACKET_TYPE = 11
NT_MOVE_SPEED_PACKET_TYPE = 12
NT_PHYSICAL_POSITION_KNOWN_PACKET_TYPE = 13
NT_POSITION_LIMIT_PACKET_TYPE = 14
NT_ANGLE_LIMIT_PACKET_TYPE = 15
NT_SAFE_DIRECTION_PACKET_TYPE = 16
NT_SCALE_PACKET_TYPE = 17
NT_MOVE_ACCELERATION_PACKET_TYPE = 18
NT_CHANNEL_PROPERTY_PACKET_TYPE = 19
NT_CAPTURE_BUFFER_PACKET_TYPE = 20
NT_TRIGGERED_PACKET_TYPE = 21
NT_INVALID_PACKET_TYPE = 255

# channel status codes
NT_STOPPED_STATUS = 0
NT_STEPPING_STATUS = 1
NT_SCANNING_STATUS = 2
NT_HOLDING_STATUS = 3
NT_TARGET_STATUS = 4
NT_SENSOR_CLOSED_STATUS = 5
NT_LIMIT_POSITION_STATUS = 10
NT_SHORT_CIRCUIT_STATUS = 11