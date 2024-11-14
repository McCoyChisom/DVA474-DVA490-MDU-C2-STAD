// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to templates/grpc_server.cpp.j2
#include "grpc_server.h"

#include <grpc++/server_builder.h>
#include <grpc++/security/server_credentials.h>

#include "log.h"

namespace mavsdk {
namespace mavsdk_server {

void GrpcServer::set_port(const int port)
{
    _port = port;
}

int GrpcServer::run()
{
    grpc::ServerBuilder builder;
    setup_port(builder);

    builder.RegisterService(&_core);

#ifdef ACTION_ENABLED
    builder.RegisterService(&_action_service);
#endif

#ifdef ACTION_SERVER_ENABLED
    builder.RegisterService(&_action_server_service);
#endif

#ifdef ARM_AUTHORIZER_SERVER_ENABLED
    builder.RegisterService(&_arm_authorizer_server_service);
#endif

#ifdef CALIBRATION_ENABLED
    builder.RegisterService(&_calibration_service);
#endif

#ifdef CAMERA_ENABLED
    builder.RegisterService(&_camera_service);
#endif

#ifdef CAMERA_SERVER_ENABLED
    builder.RegisterService(&_camera_server_service);
#endif

#ifdef COMPONENT_METADATA_ENABLED
    builder.RegisterService(&_component_metadata_service);
#endif

#ifdef COMPONENT_METADATA_SERVER_ENABLED
    builder.RegisterService(&_component_metadata_server_service);
#endif

#ifdef EVENTS_ENABLED
    builder.RegisterService(&_events_service);
#endif

#ifdef FAILURE_ENABLED
    builder.RegisterService(&_failure_service);
#endif

#ifdef FOLLOW_ME_ENABLED
    builder.RegisterService(&_follow_me_service);
#endif

#ifdef FTP_ENABLED
    builder.RegisterService(&_ftp_service);
#endif

#ifdef FTP_SERVER_ENABLED
    builder.RegisterService(&_ftp_server_service);
#endif

#ifdef GEOFENCE_ENABLED
    builder.RegisterService(&_geofence_service);
#endif

#ifdef GIMBAL_ENABLED
    builder.RegisterService(&_gimbal_service);
#endif

#ifdef GRIPPER_ENABLED
    builder.RegisterService(&_gripper_service);
#endif

#ifdef INFO_ENABLED
    builder.RegisterService(&_info_service);
#endif

#ifdef LOG_FILES_ENABLED
    builder.RegisterService(&_log_files_service);
#endif

#ifdef LOG_STREAMING_ENABLED
    builder.RegisterService(&_log_streaming_service);
#endif

#ifdef MANUAL_CONTROL_ENABLED
    builder.RegisterService(&_manual_control_service);
#endif

#ifdef MISSION_ENABLED
    builder.RegisterService(&_mission_service);
#endif

#ifdef MISSION_RAW_ENABLED
    builder.RegisterService(&_mission_raw_service);
#endif

#ifdef MISSION_RAW_SERVER_ENABLED
    builder.RegisterService(&_mission_raw_server_service);
#endif

#ifdef MOCAP_ENABLED
    builder.RegisterService(&_mocap_service);
#endif

#ifdef OFFBOARD_ENABLED
    builder.RegisterService(&_offboard_service);
#endif

#ifdef PARAM_ENABLED
    builder.RegisterService(&_param_service);
#endif

#ifdef PARAM_SERVER_ENABLED
    builder.RegisterService(&_param_server_service);
#endif

#ifdef RTK_ENABLED
    builder.RegisterService(&_rtk_service);
#endif

#ifdef SERVER_UTILITY_ENABLED
    builder.RegisterService(&_server_utility_service);
#endif

#ifdef SHELL_ENABLED
    builder.RegisterService(&_shell_service);
#endif

#ifdef TELEMETRY_ENABLED
    builder.RegisterService(&_telemetry_service);
#endif

#ifdef TELEMETRY_SERVER_ENABLED
    builder.RegisterService(&_telemetry_server_service);
#endif

#ifdef TRANSPONDER_ENABLED
    builder.RegisterService(&_transponder_service);
#endif

#ifdef TUNE_ENABLED
    builder.RegisterService(&_tune_service);
#endif

#ifdef WINCH_ENABLED
    builder.RegisterService(&_winch_service);
#endif

#ifdef ENABLE_PROTO_REFLECTION
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
#endif

    grpc::EnableDefaultHealthCheckService(true);
    _server = builder.BuildAndStart();

    if (_bound_port != 0) {
        LogInfo() << "Server started";
        LogInfo() << "Server set to listen on 0.0.0.0:" << _bound_port;
    } else {
        LogErr() << "Failed to bind server to port " << _port;
    }

    return _bound_port;
}

void GrpcServer::wait()
{
    if (_server != nullptr) {
        _server->Wait();
    } else {
        LogWarn() << "Calling 'wait()' on a non-existing server. Did you call 'run()' before?";
    }
}

void GrpcServer::stop()
{
    if (_server != nullptr) {
        _core.stop();

#ifdef ACTION_ENABLED
        _action_service.stop();
#endif

#ifdef ACTION_SERVER_ENABLED
        _action_server_service.stop();
#endif

#ifdef ARM_AUTHORIZER_SERVER_ENABLED
        _arm_authorizer_server_service.stop();
#endif

#ifdef CALIBRATION_ENABLED
        _calibration_service.stop();
#endif

#ifdef CAMERA_ENABLED
        _camera_service.stop();
#endif

#ifdef CAMERA_SERVER_ENABLED
        _camera_server_service.stop();
#endif

#ifdef COMPONENT_METADATA_ENABLED
        _component_metadata_service.stop();
#endif

#ifdef COMPONENT_METADATA_SERVER_ENABLED
        _component_metadata_server_service.stop();
#endif

#ifdef EVENTS_ENABLED
        _events_service.stop();
#endif

#ifdef FAILURE_ENABLED
        _failure_service.stop();
#endif

#ifdef FOLLOW_ME_ENABLED
        _follow_me_service.stop();
#endif

#ifdef FTP_ENABLED
        _ftp_service.stop();
#endif

#ifdef FTP_SERVER_ENABLED
        _ftp_server_service.stop();
#endif

#ifdef GEOFENCE_ENABLED
        _geofence_service.stop();
#endif

#ifdef GIMBAL_ENABLED
        _gimbal_service.stop();
#endif

#ifdef GRIPPER_ENABLED
        _gripper_service.stop();
#endif

#ifdef INFO_ENABLED
        _info_service.stop();
#endif

#ifdef LOG_FILES_ENABLED
        _log_files_service.stop();
#endif

#ifdef LOG_STREAMING_ENABLED
        _log_streaming_service.stop();
#endif

#ifdef MANUAL_CONTROL_ENABLED
        _manual_control_service.stop();
#endif

#ifdef MISSION_ENABLED
        _mission_service.stop();
#endif

#ifdef MISSION_RAW_ENABLED
        _mission_raw_service.stop();
#endif

#ifdef MISSION_RAW_SERVER_ENABLED
        _mission_raw_server_service.stop();
#endif

#ifdef MOCAP_ENABLED
        _mocap_service.stop();
#endif

#ifdef OFFBOARD_ENABLED
        _offboard_service.stop();
#endif

#ifdef PARAM_ENABLED
        _param_service.stop();
#endif

#ifdef PARAM_SERVER_ENABLED
        _param_server_service.stop();
#endif

#ifdef RTK_ENABLED
        _rtk_service.stop();
#endif

#ifdef SERVER_UTILITY_ENABLED
        _server_utility_service.stop();
#endif

#ifdef SHELL_ENABLED
        _shell_service.stop();
#endif

#ifdef TELEMETRY_ENABLED
        _telemetry_service.stop();
#endif

#ifdef TELEMETRY_SERVER_ENABLED
        _telemetry_server_service.stop();
#endif

#ifdef TRANSPONDER_ENABLED
        _transponder_service.stop();
#endif

#ifdef TUNE_ENABLED
        _tune_service.stop();
#endif

#ifdef WINCH_ENABLED
        _winch_service.stop();
#endif

        _server->Shutdown();
    } else {
        LogWarn() << "Calling 'stop()' on a non-existing server. Did you call 'run()' before?";
    }
}

void GrpcServer::setup_port(grpc::ServerBuilder& builder)
{
    const std::string server_address("0.0.0.0:" + std::to_string(_port));
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials(), &_bound_port);
}

} // namespace mavsdk_server
} // namespace mavsdk