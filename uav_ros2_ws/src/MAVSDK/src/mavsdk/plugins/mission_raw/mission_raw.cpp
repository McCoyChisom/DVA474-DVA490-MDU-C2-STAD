// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/mission_raw/mission_raw.proto)

#include <iomanip>

#include "mission_raw_impl.h"
#include "plugins/mission_raw/mission_raw.h"

namespace mavsdk {

using MissionProgress = MissionRaw::MissionProgress;
using MissionItem = MissionRaw::MissionItem;
using MissionImportData = MissionRaw::MissionImportData;

MissionRaw::MissionRaw(System& system) :
    PluginBase(),
    _impl{std::make_unique<MissionRawImpl>(system)}
{}

MissionRaw::MissionRaw(std::shared_ptr<System> system) :
    PluginBase(),
    _impl{std::make_unique<MissionRawImpl>(system)}
{}

MissionRaw::~MissionRaw() {}

void MissionRaw::upload_mission_async(
    std::vector<MissionItem> mission_items, const ResultCallback callback)
{
    _impl->upload_mission_async(mission_items, callback);
}

MissionRaw::Result MissionRaw::upload_mission(std::vector<MissionItem> mission_items) const
{
    return _impl->upload_mission(mission_items);
}

void MissionRaw::upload_geofence_async(
    std::vector<MissionItem> mission_items, const ResultCallback callback)
{
    _impl->upload_geofence_async(mission_items, callback);
}

MissionRaw::Result MissionRaw::upload_geofence(std::vector<MissionItem> mission_items) const
{
    return _impl->upload_geofence(mission_items);
}

void MissionRaw::upload_rally_points_async(
    std::vector<MissionItem> mission_items, const ResultCallback callback)
{
    _impl->upload_rally_points_async(mission_items, callback);
}

MissionRaw::Result MissionRaw::upload_rally_points(std::vector<MissionItem> mission_items) const
{
    return _impl->upload_rally_points(mission_items);
}

MissionRaw::Result MissionRaw::cancel_mission_upload() const
{
    return _impl->cancel_mission_upload();
}

void MissionRaw::download_mission_async(const DownloadMissionCallback callback)
{
    _impl->download_mission_async(callback);
}

std::pair<MissionRaw::Result, std::vector<MissionRaw::MissionItem>>
MissionRaw::download_mission() const
{
    return _impl->download_mission();
}

void MissionRaw::download_geofence_async(const DownloadGeofenceCallback callback)
{
    _impl->download_geofence_async(callback);
}

std::pair<MissionRaw::Result, std::vector<MissionRaw::MissionItem>>
MissionRaw::download_geofence() const
{
    return _impl->download_geofence();
}

void MissionRaw::download_rallypoints_async(const DownloadRallypointsCallback callback)
{
    _impl->download_rallypoints_async(callback);
}

std::pair<MissionRaw::Result, std::vector<MissionRaw::MissionItem>>
MissionRaw::download_rallypoints() const
{
    return _impl->download_rallypoints();
}

MissionRaw::Result MissionRaw::cancel_mission_download() const
{
    return _impl->cancel_mission_download();
}

void MissionRaw::start_mission_async(const ResultCallback callback)
{
    _impl->start_mission_async(callback);
}

MissionRaw::Result MissionRaw::start_mission() const
{
    return _impl->start_mission();
}

void MissionRaw::pause_mission_async(const ResultCallback callback)
{
    _impl->pause_mission_async(callback);
}

MissionRaw::Result MissionRaw::pause_mission() const
{
    return _impl->pause_mission();
}

void MissionRaw::clear_mission_async(const ResultCallback callback)
{
    _impl->clear_mission_async(callback);
}

MissionRaw::Result MissionRaw::clear_mission() const
{
    return _impl->clear_mission();
}

void MissionRaw::set_current_mission_item_async(int32_t index, const ResultCallback callback)
{
    _impl->set_current_mission_item_async(index, callback);
}

MissionRaw::Result MissionRaw::set_current_mission_item(int32_t index) const
{
    return _impl->set_current_mission_item(index);
}

MissionRaw::MissionProgressHandle
MissionRaw::subscribe_mission_progress(const MissionProgressCallback& callback)
{
    return _impl->subscribe_mission_progress(callback);
}

void MissionRaw::unsubscribe_mission_progress(MissionProgressHandle handle)
{
    _impl->unsubscribe_mission_progress(handle);
}

MissionRaw::MissionProgress MissionRaw::mission_progress() const
{
    return _impl->mission_progress();
}

MissionRaw::MissionChangedHandle
MissionRaw::subscribe_mission_changed(const MissionChangedCallback& callback)
{
    return _impl->subscribe_mission_changed(callback);
}

void MissionRaw::unsubscribe_mission_changed(MissionChangedHandle handle)
{
    _impl->unsubscribe_mission_changed(handle);
}

std::pair<MissionRaw::Result, MissionRaw::MissionImportData>
MissionRaw::import_qgroundcontrol_mission(std::string qgc_plan_path) const
{
    return _impl->import_qgroundcontrol_mission(qgc_plan_path);
}

std::pair<MissionRaw::Result, MissionRaw::MissionImportData>
MissionRaw::import_qgroundcontrol_mission_from_string(std::string qgc_plan) const
{
    return _impl->import_qgroundcontrol_mission_from_string(qgc_plan);
}

bool operator==(const MissionRaw::MissionProgress& lhs, const MissionRaw::MissionProgress& rhs)
{
    return (rhs.current == lhs.current) && (rhs.total == lhs.total);
}

std::ostream& operator<<(std::ostream& str, MissionRaw::MissionProgress const& mission_progress)
{
    str << std::setprecision(15);
    str << "mission_progress:" << '\n' << "{\n";
    str << "    current: " << mission_progress.current << '\n';
    str << "    total: " << mission_progress.total << '\n';
    str << '}';
    return str;
}

bool operator==(const MissionRaw::MissionItem& lhs, const MissionRaw::MissionItem& rhs)
{
    return (rhs.seq == lhs.seq) && (rhs.frame == lhs.frame) && (rhs.command == lhs.command) &&
           (rhs.current == lhs.current) && (rhs.autocontinue == lhs.autocontinue) &&
           ((std::isnan(rhs.param1) && std::isnan(lhs.param1)) || rhs.param1 == lhs.param1) &&
           ((std::isnan(rhs.param2) && std::isnan(lhs.param2)) || rhs.param2 == lhs.param2) &&
           ((std::isnan(rhs.param3) && std::isnan(lhs.param3)) || rhs.param3 == lhs.param3) &&
           ((std::isnan(rhs.param4) && std::isnan(lhs.param4)) || rhs.param4 == lhs.param4) &&
           (rhs.x == lhs.x) && (rhs.y == lhs.y) &&
           ((std::isnan(rhs.z) && std::isnan(lhs.z)) || rhs.z == lhs.z) &&
           (rhs.mission_type == lhs.mission_type);
}

std::ostream& operator<<(std::ostream& str, MissionRaw::MissionItem const& mission_item)
{
    str << std::setprecision(15);
    str << "mission_item:" << '\n' << "{\n";
    str << "    seq: " << mission_item.seq << '\n';
    str << "    frame: " << mission_item.frame << '\n';
    str << "    command: " << mission_item.command << '\n';
    str << "    current: " << mission_item.current << '\n';
    str << "    autocontinue: " << mission_item.autocontinue << '\n';
    str << "    param1: " << mission_item.param1 << '\n';
    str << "    param2: " << mission_item.param2 << '\n';
    str << "    param3: " << mission_item.param3 << '\n';
    str << "    param4: " << mission_item.param4 << '\n';
    str << "    x: " << mission_item.x << '\n';
    str << "    y: " << mission_item.y << '\n';
    str << "    z: " << mission_item.z << '\n';
    str << "    mission_type: " << mission_item.mission_type << '\n';
    str << '}';
    return str;
}

bool operator==(const MissionRaw::MissionImportData& lhs, const MissionRaw::MissionImportData& rhs)
{
    return (rhs.mission_items == lhs.mission_items) && (rhs.geofence_items == lhs.geofence_items) &&
           (rhs.rally_items == lhs.rally_items);
}

std::ostream&
operator<<(std::ostream& str, MissionRaw::MissionImportData const& mission_import_data)
{
    str << std::setprecision(15);
    str << "mission_import_data:" << '\n' << "{\n";
    str << "    mission_items: [";
    for (auto it = mission_import_data.mission_items.begin();
         it != mission_import_data.mission_items.end();
         ++it) {
        str << *it;
        str << (it + 1 != mission_import_data.mission_items.end() ? ", " : "]\n");
    }
    str << "    geofence_items: [";
    for (auto it = mission_import_data.geofence_items.begin();
         it != mission_import_data.geofence_items.end();
         ++it) {
        str << *it;
        str << (it + 1 != mission_import_data.geofence_items.end() ? ", " : "]\n");
    }
    str << "    rally_items: [";
    for (auto it = mission_import_data.rally_items.begin();
         it != mission_import_data.rally_items.end();
         ++it) {
        str << *it;
        str << (it + 1 != mission_import_data.rally_items.end() ? ", " : "]\n");
    }
    str << '}';
    return str;
}

std::ostream& operator<<(std::ostream& str, MissionRaw::Result const& result)
{
    switch (result) {
        case MissionRaw::Result::Unknown:
            return str << "Unknown";
        case MissionRaw::Result::Success:
            return str << "Success";
        case MissionRaw::Result::Error:
            return str << "Error";
        case MissionRaw::Result::TooManyMissionItems:
            return str << "Too Many Mission Items";
        case MissionRaw::Result::Busy:
            return str << "Busy";
        case MissionRaw::Result::Timeout:
            return str << "Timeout";
        case MissionRaw::Result::InvalidArgument:
            return str << "Invalid Argument";
        case MissionRaw::Result::Unsupported:
            return str << "Unsupported";
        case MissionRaw::Result::NoMissionAvailable:
            return str << "No Mission Available";
        case MissionRaw::Result::TransferCancelled:
            return str << "Transfer Cancelled";
        case MissionRaw::Result::FailedToOpenQgcPlan:
            return str << "Failed To Open Qgc Plan";
        case MissionRaw::Result::FailedToParseQgcPlan:
            return str << "Failed To Parse Qgc Plan";
        case MissionRaw::Result::NoSystem:
            return str << "No System";
        case MissionRaw::Result::Denied:
            return str << "Denied";
        case MissionRaw::Result::MissionTypeNotConsistent:
            return str << "Mission Type Not Consistent";
        case MissionRaw::Result::InvalidSequence:
            return str << "Invalid Sequence";
        case MissionRaw::Result::CurrentInvalid:
            return str << "Current Invalid";
        case MissionRaw::Result::ProtocolError:
            return str << "Protocol Error";
        case MissionRaw::Result::IntMessagesNotSupported:
            return str << "Int Messages Not Supported";
        default:
            return str << "Unknown";
    }
}

} // namespace mavsdk