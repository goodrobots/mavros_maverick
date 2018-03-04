/**
 * @brief Vehicule Info plugin
 * @file vehicle_info.cpp
 * @author Gregoire Linard <gregoire.linard@azurdrones.com>
 *
 * @addtogroup plugin
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include "mavros_maverick/VehicleInfo.h"

namespace mavros {
namespace maverick_plugins {
using mavlink::common::MAV_TYPE;
using mavlink::common::MAV_AUTOPILOT;
using mavlink::common::MAV_STATE;
using mavlink::common::MAV_PROTOCOL_CAPABILITY;
using utils::enum_value;

/**
 * @brief  Vehicle Info monitor plugin.
 *
 * This plugin allows requesting standard vehicle information.
 *
 */
class VehicleInfoPlugin : public plugin::PluginBase {
public:
	using MT = mavlink::common::MAV_PARAM_TYPE;
    
    VehicleInfoPlugin() : PluginBase(),
		vehicle_info_nh("~")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		vehicle_info_srv =  vehicle_info_nh.advertiseService("get_vehicle_info", &VehicleInfoPlugin::vehicle_info_cb,this);
    
    }

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&VehicleInfoPlugin::handle_heartbeat),
                   make_handler(&VehicleInfoPlugin::handle_autopilot_version)
		};
	}

private:
	ros::NodeHandle vehicle_info_nh;

	ros::ServiceServer vehicle_info_srv;

	MAV_AUTOPILOT autopilot;
	MAV_TYPE type;
	std::string mode;
    uint32_t mode_id;
	MAV_STATE system_status;
    MAV_PROTOCOL_CAPABILITY capabilities;
    uint32_t flight_sw_version;
    uint32_t middleware_sw_version;
    uint32_t os_sw_version;
    uint32_t board_version;
    uint16_t vendor_id;
    uint16_t product_id;
    uint64_t uid;


	/* -*- message handlers -*- */
	void handle_heartbeat(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HEARTBEAT &hb)
	{
		using mavlink::common::MAV_MODE_FLAG;

		if (!m_uas->is_my_target(msg->sysid)) {
			return;
		}
		      
        type = static_cast<MAV_TYPE>(hb.type);
		autopilot = static_cast<MAV_AUTOPILOT>(hb.autopilot);
		mode = m_uas->str_mode_v10(hb.base_mode, hb.custom_mode);
		system_status = static_cast<MAV_STATE>(hb.system_status);

        if (!(hb.base_mode & enum_value(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED))){
            mode_id = hb.base_mode;
        }else{
            mode_id = hb.custom_mode;
        }
		
	}

	void handle_autopilot_version(const mavlink::mavlink_message_t *msg, mavlink::common::msg::AUTOPILOT_VERSION &apv)
	{
		
        if (m_uas->is_my_target(msg->sysid, msg->compid)) {

            process_autopilot_version_normal(apv, msg->sysid, msg->compid);
            
        }
	}

	/* -*-ROS callbacks -*- */

	 bool vehicle_info_cb(mavros_maverick::VehicleInfo::Request &req,
			mavros_maverick::VehicleInfo::Response &res)
	{
        res.type =  enum_value(type);
        res.autopilot = enum_value(autopilot); 
        res.mode = mode;
        res.mode_id = mode_id;
        res.system_status = enum_value(system_status);
        res.sysid = m_uas->get_tgt_system();
		res.compid = m_uas->get_tgt_component();

        res.capabilities = enum_value(capabilities);
        res.flight_sw_version = flight_sw_version;
        res.middleware_sw_version = middleware_sw_version;
        res.os_sw_version = os_sw_version;
        res.board_version = board_version;
        res.vendor_id = vendor_id;
        res.product_id = product_id;
        res.uid = uid;

        res.success = true;    
    }

	/* -*- Utilities -*- */

    void process_autopilot_version_normal(mavlink::common::msg::AUTOPILOT_VERSION &apv, uint8_t sysid, uint8_t compid)
	{


		capabilities = static_cast<MAV_PROTOCOL_CAPABILITY>(apv.capabilities);
		flight_sw_version = apv.flight_sw_version;
		// custom_version_to_hex_string(apv.flight_custom_version).c_str());
		middleware_sw_version = apv.middleware_sw_version;
		//custom_version_to_hex_string(apv.middleware_custom_version).c_str());
		os_sw_version = apv.os_sw_version;
		//custom_version_to_hex_string(apv.os_custom_version).c_str());
		board_version = apv.board_version;
		vendor_id = apv.vendor_id;
        product_id = apv.product_id;
		uid = apv.uid;
	}

	
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::maverick_plugins::VehicleInfoPlugin, mavros::plugin::PluginBase)
