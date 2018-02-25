/**
 * @brief Param value monitor plugin
 * @file param_value.cpp
 * @author Gregoire Linard <gregoire.linard@azurdrones.com>
 *
 * @addtogroup plugin
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include "mavros_maverick/Param.h"

namespace mavros {
namespace maverick_plugins {
/**
 * @brief  Param value monitor plugin.
 *
 * This plugin allows publishing a topic to catch parameter that changed value.
 *
 */
class ParamValuePlugin : public plugin::PluginBase {
public:
	ParamValuePlugin() : PluginBase(),
		param_value_nh("~")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		param_value_pub = param_value_nh.advertise<mavros_maverick::Param>("param_value", 100);
    }

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&ParamValuePlugin::handle_param_value)
		};
	}

private:
	ros::NodeHandle param_value_nh;

	ros::Publisher param_value_pub;

	/* -*- message handlers -*- */

	void handle_param_value(const mavlink::mavlink_message_t *msg, mavlink::common::msg::PARAM_VALUE &pmsg)
	{
        mavros_maverick::Param param_msg;
        
        param_msg.param_id =  mavlink::to_string(pmsg.param_id);
        param_msg.param_value = pmsg.param_value;
        param_msg.param_type = pmsg.param_type;
        param_msg.param_count = pmsg.param_count;
        param_msg.param_index = pmsg.param_index;
        
        param_value_pub.publish(param_msg);
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::maverick_plugins::ParamValuePlugin, mavros::plugin::PluginBase)
