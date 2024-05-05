/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5 */

#ifndef PB_MSGS_CAN_MSGS_CAN_PB_H_INCLUDED
#define PB_MSGS_CAN_MSGS_CAN_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _msgs_can_ActActions {
    msgs_can_ActActions_START_GRAB_PLANTS = 0,
    msgs_can_ActActions_STOP_GRAB_PLANTS = 1,
    msgs_can_ActActions_RELEASE_PLANT = 2,
    msgs_can_ActActions_TURN_SOLAR_PANEL = 3,
    msgs_can_ActActions_INITIALIZING = 4,
    msgs_can_ActActions_FREE = 5
} msgs_can_ActActions;

typedef enum _msgs_can_Status_StatusType {
    msgs_can_Status_StatusType_OK = 0,
    msgs_can_Status_StatusType_INIT = 1,
    msgs_can_Status_StatusType_WARN = 2,
    msgs_can_Status_StatusType_ERROR = 3
} msgs_can_Status_StatusType;

typedef enum _msgs_can_Status_ErrorType {
    msgs_can_Status_ErrorType_NONE = 0,
    msgs_can_Status_ErrorType_INIT_PERIPHERALS = 1,
    msgs_can_Status_ErrorType_INIT_CAN = 2,
    msgs_can_Status_ErrorType_PROTO_ENCODE = 3,
    msgs_can_Status_ErrorType_PROTO_DECODE = 4,
    msgs_can_Status_ErrorType_CMD_VEL_TIMEOUT = 5,
    msgs_can_Status_ErrorType_CAN_TX = 6,
    msgs_can_Status_ErrorType_CAN_RX = 7,
    msgs_can_Status_ErrorType_INVALID_CONFIG = 8
} msgs_can_Status_ErrorType;

/* Struct definitions */
typedef struct _msgs_can_ActCmd {
    bool has_action;
    msgs_can_ActActions action;
    bool has_value;
    float value;
} msgs_can_ActCmd;

typedef struct _msgs_can_BaseConfig {
    bool has_max_accel;
    float max_accel; /* rad/s^2 */
    bool has_cmd_vel_timeout;
    float cmd_vel_timeout; /* seconds */
    bool has_wheel_radius;
    float wheel_radius; /* meters */
    bool has_base_radius;
    float base_radius; /* meters */
} msgs_can_BaseConfig;

typedef struct _msgs_can_BaseVel {
    bool has_x;
    float x;
    bool has_y;
    float y;
    bool has_theta;
    float theta;
} msgs_can_BaseVel;

typedef struct _msgs_can_ImuData {
    bool has_acc_x;
    float acc_x;
    bool has_acc_y;
    float acc_y;
    bool has_acc_z;
    float acc_z;
    bool has_gyro_x;
    float gyro_x;
    bool has_gyro_y;
    float gyro_y;
    bool has_gyro_z;
    float gyro_z;
} msgs_can_ImuData;

typedef struct _msgs_can_Status {
    bool has_timestamp;
    float timestamp; /* seconds since epoch */
    bool has_status;
    msgs_can_Status_StatusType status;
    bool has_error;
    msgs_can_Status_ErrorType error;
    pb_callback_t message;
} msgs_can_Status;

typedef struct _msgs_can_ActStatus {
    bool has_status;
    msgs_can_Status status;
    bool has_action;
    msgs_can_ActActions action;
    bool has_plant_count;
    int32_t plant_count;
} msgs_can_ActStatus;

typedef struct _msgs_can_Log {
    bool has_config;
    msgs_can_BaseConfig config;
    pb_callback_t status;
} msgs_can_Log;

/* STM Uses this to return the current configuration after a change
 If missing or wrong values in the request, set status to ERROR etc */
typedef struct _msgs_can_RetBaseConfig {
    bool has_config;
    msgs_can_BaseConfig config;
    bool has_status;
    msgs_can_Status status;
} msgs_can_RetBaseConfig;

/* Use this as an heartbeat sent at a regular interval. Also, send one right away after a status change */
typedef struct _msgs_can_StatusReport {
    /* Keep short : use only status if everything is OK */
    bool has_status;
    msgs_can_Status status;
} msgs_can_StatusReport;


/* Helper constants for enums */
#define _msgs_can_ActActions_MIN msgs_can_ActActions_START_GRAB_PLANTS
#define _msgs_can_ActActions_MAX msgs_can_ActActions_FREE
#define _msgs_can_ActActions_ARRAYSIZE ((msgs_can_ActActions)(msgs_can_ActActions_FREE+1))

#define _msgs_can_Status_StatusType_MIN msgs_can_Status_StatusType_OK
#define _msgs_can_Status_StatusType_MAX msgs_can_Status_StatusType_ERROR
#define _msgs_can_Status_StatusType_ARRAYSIZE ((msgs_can_Status_StatusType)(msgs_can_Status_StatusType_ERROR+1))

#define _msgs_can_Status_ErrorType_MIN msgs_can_Status_ErrorType_NONE
#define _msgs_can_Status_ErrorType_MAX msgs_can_Status_ErrorType_INVALID_CONFIG
#define _msgs_can_Status_ErrorType_ARRAYSIZE ((msgs_can_Status_ErrorType)(msgs_can_Status_ErrorType_INVALID_CONFIG+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define msgs_can_BaseVel_init_default            {false, 0, false, 0, false, 0}
#define msgs_can_Status_init_default             {false, 0, false, _msgs_can_Status_StatusType_MIN, false, _msgs_can_Status_ErrorType_MIN, {{NULL}, NULL}}
#define msgs_can_StatusReport_init_default       {false, msgs_can_Status_init_default}
#define msgs_can_Log_init_default                {false, msgs_can_BaseConfig_init_default, {{NULL}, NULL}}
#define msgs_can_BaseConfig_init_default         {false, 0, false, 0, false, 0, false, 0}
#define msgs_can_RetBaseConfig_init_default      {false, msgs_can_BaseConfig_init_default, false, msgs_can_Status_init_default}
#define msgs_can_ImuData_init_default            {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define msgs_can_ActCmd_init_default             {false, _msgs_can_ActActions_MIN, false, 0}
#define msgs_can_ActStatus_init_default          {false, msgs_can_Status_init_default, false, _msgs_can_ActActions_MIN, false, 0}
#define msgs_can_BaseVel_init_zero               {false, 0, false, 0, false, 0}
#define msgs_can_Status_init_zero                {false, 0, false, _msgs_can_Status_StatusType_MIN, false, _msgs_can_Status_ErrorType_MIN, {{NULL}, NULL}}
#define msgs_can_StatusReport_init_zero          {false, msgs_can_Status_init_zero}
#define msgs_can_Log_init_zero                   {false, msgs_can_BaseConfig_init_zero, {{NULL}, NULL}}
#define msgs_can_BaseConfig_init_zero            {false, 0, false, 0, false, 0, false, 0}
#define msgs_can_RetBaseConfig_init_zero         {false, msgs_can_BaseConfig_init_zero, false, msgs_can_Status_init_zero}
#define msgs_can_ImuData_init_zero               {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define msgs_can_ActCmd_init_zero                {false, _msgs_can_ActActions_MIN, false, 0}
#define msgs_can_ActStatus_init_zero             {false, msgs_can_Status_init_zero, false, _msgs_can_ActActions_MIN, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define msgs_can_ActCmd_action_tag               1
#define msgs_can_ActCmd_value_tag                2
#define msgs_can_BaseConfig_max_accel_tag        1
#define msgs_can_BaseConfig_cmd_vel_timeout_tag  2
#define msgs_can_BaseConfig_wheel_radius_tag     3
#define msgs_can_BaseConfig_base_radius_tag      4
#define msgs_can_BaseVel_x_tag                   1
#define msgs_can_BaseVel_y_tag                   2
#define msgs_can_BaseVel_theta_tag               3
#define msgs_can_ImuData_acc_x_tag               1
#define msgs_can_ImuData_acc_y_tag               2
#define msgs_can_ImuData_acc_z_tag               3
#define msgs_can_ImuData_gyro_x_tag              4
#define msgs_can_ImuData_gyro_y_tag              5
#define msgs_can_ImuData_gyro_z_tag              6
#define msgs_can_Status_timestamp_tag            1
#define msgs_can_Status_status_tag               2
#define msgs_can_Status_error_tag                3
#define msgs_can_Status_message_tag              4
#define msgs_can_ActStatus_status_tag            1
#define msgs_can_ActStatus_action_tag            2
#define msgs_can_ActStatus_plant_count_tag       3
#define msgs_can_Log_config_tag                  1
#define msgs_can_Log_status_tag                  2
#define msgs_can_RetBaseConfig_config_tag        1
#define msgs_can_RetBaseConfig_status_tag        2
#define msgs_can_StatusReport_status_tag         1

/* Struct field encoding specification for nanopb */
#define msgs_can_BaseVel_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, FLOAT,    x,                 1) \
X(a, STATIC,   OPTIONAL, FLOAT,    y,                 2) \
X(a, STATIC,   OPTIONAL, FLOAT,    theta,             3)
#define msgs_can_BaseVel_CALLBACK NULL
#define msgs_can_BaseVel_DEFAULT NULL

#define msgs_can_Status_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, FLOAT,    timestamp,         1) \
X(a, STATIC,   OPTIONAL, UENUM,    status,            2) \
X(a, STATIC,   OPTIONAL, UENUM,    error,             3) \
X(a, CALLBACK, OPTIONAL, STRING,   message,           4)
#define msgs_can_Status_CALLBACK pb_default_field_callback
#define msgs_can_Status_DEFAULT NULL

#define msgs_can_StatusReport_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  status,            1)
#define msgs_can_StatusReport_CALLBACK NULL
#define msgs_can_StatusReport_DEFAULT NULL
#define msgs_can_StatusReport_status_MSGTYPE msgs_can_Status

#define msgs_can_Log_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  config,            1) \
X(a, CALLBACK, REPEATED, MESSAGE,  status,            2)
#define msgs_can_Log_CALLBACK pb_default_field_callback
#define msgs_can_Log_DEFAULT NULL
#define msgs_can_Log_config_MSGTYPE msgs_can_BaseConfig
#define msgs_can_Log_status_MSGTYPE msgs_can_Status

#define msgs_can_BaseConfig_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, FLOAT,    max_accel,         1) \
X(a, STATIC,   OPTIONAL, FLOAT,    cmd_vel_timeout,   2) \
X(a, STATIC,   OPTIONAL, FLOAT,    wheel_radius,      3) \
X(a, STATIC,   OPTIONAL, FLOAT,    base_radius,       4)
#define msgs_can_BaseConfig_CALLBACK NULL
#define msgs_can_BaseConfig_DEFAULT NULL

#define msgs_can_RetBaseConfig_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  config,            1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  status,            2)
#define msgs_can_RetBaseConfig_CALLBACK NULL
#define msgs_can_RetBaseConfig_DEFAULT NULL
#define msgs_can_RetBaseConfig_config_MSGTYPE msgs_can_BaseConfig
#define msgs_can_RetBaseConfig_status_MSGTYPE msgs_can_Status

#define msgs_can_ImuData_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, FLOAT,    acc_x,             1) \
X(a, STATIC,   OPTIONAL, FLOAT,    acc_y,             2) \
X(a, STATIC,   OPTIONAL, FLOAT,    acc_z,             3) \
X(a, STATIC,   OPTIONAL, FLOAT,    gyro_x,            4) \
X(a, STATIC,   OPTIONAL, FLOAT,    gyro_y,            5) \
X(a, STATIC,   OPTIONAL, FLOAT,    gyro_z,            6)
#define msgs_can_ImuData_CALLBACK NULL
#define msgs_can_ImuData_DEFAULT NULL

#define msgs_can_ActCmd_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, UENUM,    action,            1) \
X(a, STATIC,   OPTIONAL, FLOAT,    value,             2)
#define msgs_can_ActCmd_CALLBACK NULL
#define msgs_can_ActCmd_DEFAULT NULL

#define msgs_can_ActStatus_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  status,            1) \
X(a, STATIC,   OPTIONAL, UENUM,    action,            2) \
X(a, STATIC,   OPTIONAL, INT32,    plant_count,       3)
#define msgs_can_ActStatus_CALLBACK NULL
#define msgs_can_ActStatus_DEFAULT NULL
#define msgs_can_ActStatus_status_MSGTYPE msgs_can_Status

extern const pb_msgdesc_t msgs_can_BaseVel_msg;
extern const pb_msgdesc_t msgs_can_Status_msg;
extern const pb_msgdesc_t msgs_can_StatusReport_msg;
extern const pb_msgdesc_t msgs_can_Log_msg;
extern const pb_msgdesc_t msgs_can_BaseConfig_msg;
extern const pb_msgdesc_t msgs_can_RetBaseConfig_msg;
extern const pb_msgdesc_t msgs_can_ImuData_msg;
extern const pb_msgdesc_t msgs_can_ActCmd_msg;
extern const pb_msgdesc_t msgs_can_ActStatus_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define msgs_can_BaseVel_fields &msgs_can_BaseVel_msg
#define msgs_can_Status_fields &msgs_can_Status_msg
#define msgs_can_StatusReport_fields &msgs_can_StatusReport_msg
#define msgs_can_Log_fields &msgs_can_Log_msg
#define msgs_can_BaseConfig_fields &msgs_can_BaseConfig_msg
#define msgs_can_RetBaseConfig_fields &msgs_can_RetBaseConfig_msg
#define msgs_can_ImuData_fields &msgs_can_ImuData_msg
#define msgs_can_ActCmd_fields &msgs_can_ActCmd_msg
#define msgs_can_ActStatus_fields &msgs_can_ActStatus_msg

/* Maximum encoded size of messages (where known) */
/* msgs_can_Status_size depends on runtime parameters */
/* msgs_can_StatusReport_size depends on runtime parameters */
/* msgs_can_Log_size depends on runtime parameters */
/* msgs_can_RetBaseConfig_size depends on runtime parameters */
/* msgs_can_ActStatus_size depends on runtime parameters */
#define msgs_can_ActCmd_size                     7
#define msgs_can_BaseConfig_size                 20
#define msgs_can_BaseVel_size                    15
#define msgs_can_ImuData_size                    30

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
