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
using utils::enum_value;

/**
 * @brief  Param value monitor plugin.
 *
 * This plugin allows publishing a topic to catch parameter that changed value.
 *
 */
class ParamValuePlugin : public plugin::PluginBase {
public:
	using MT = mavlink::common::MAV_PARAM_TYPE;
    
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
        param_msg.param_type = pmsg.param_type;
        param_msg.param_count = pmsg.param_count;
        param_msg.param_index = pmsg.param_index;

        //Fill param_value field:
        if (m_uas->is_ardupilotmega()){
				set_value_apm(param_msg,pmsg);
		}else{
				set_value_px4(param_msg,pmsg);
        }

		//Publish       
        param_value_pub.publish(param_msg);
	}

	void set_value_apm(mavros_maverick::Param &param_msg,mavlink::common::msg::PARAM_VALUE &pmsg)
	{
		int32_t int_tmp;
		float float_tmp;

        if ((pmsg.param_type == enum_value(MT::REAL32)) || (pmsg.param_type == enum_value(MT::REAL64)))
        {

            float_tmp = static_cast<double>(pmsg.param_value);
            //float_tmp = floor(pow(10,7)*float_tmp)/pow(10,7); //rounding
            param_msg.value.real = float_tmp;
            param_msg.value.integer = 0;
        }
        else if((pmsg.param_type == enum_value(MT::INT8))||(pmsg.param_type == enum_value(MT::UINT8))
                ||(pmsg.param_type == enum_value(MT::INT16))||(pmsg.param_type == enum_value(MT::UINT16))
                ||(pmsg.param_type == enum_value(MT::INT32))||(pmsg.param_type == enum_value(MT::UINT32)))
        {
            int_tmp = static_cast<int32_t>(pmsg.param_value);
            param_msg.value.integer = int_tmp;
            param_msg.value.real = 0.0f;
        }
        else
        {
            ROS_WARN_NAMED("param", "PM: Unsupported param %.16s (%u/%u) type: %u",
                           pmsg.param_id.data(), pmsg.param_index, pmsg.param_count, pmsg.param_type);
            param_msg.value.integer = 0;
            param_msg.value.real = 0.0f;
        }
    }

    void set_value_px4(mavros_maverick::Param &param_msg,mavlink::common::msg::PARAM_VALUE &pmsg)
	{
		mavlink::mavlink_param_union_t uv;
		uv.param_float = pmsg.param_value;

		// #170 - copy union value to itermediate var
		int int_tmp;
		float float_tmp;

		switch (pmsg.param_type) {
		case enum_value(MT::INT8):
			int_tmp = uv.param_int8;
			param_msg.value.integer = int_tmp;
            param_msg.value.real = 0.0f;
			break;
		case enum_value(MT::UINT8):
			int_tmp = uv.param_uint8;
			param_msg.value.integer = int_tmp;
            param_msg.value.real = 0.0f;
			break;
		case enum_value(MT::INT16):
			int_tmp = uv.param_int16;
			param_msg.value.integer = int_tmp;
            param_msg.value.real = 0.0f;
			break;
		case enum_value(MT::UINT16):
			int_tmp = uv.param_uint16;
			param_msg.value.integer = int_tmp;
            param_msg.value.real = 0.0f;
			break;
		case enum_value(MT::INT32):
			int_tmp = uv.param_int32;
			param_msg.value.integer = int_tmp;
            param_msg.value.real = 0.0f;
			break;
		case enum_value(MT::UINT32):
			int_tmp = uv.param_uint32;
			param_msg.value.integer = int_tmp;
            param_msg.value.real = 0.0f;
			break;
		case enum_value(MT::REAL32):
			float_tmp = uv.param_float;
			param_msg.value.real = float_tmp;
            param_msg.value.integer = 0;
			break;

		default:
			ROS_WARN_NAMED("param", "PM: Unsupported param %.16s (%u/%u) type: %u",
					pmsg.param_id.data(), pmsg.param_index, pmsg.param_count, pmsg.param_type);
			param_msg.value.integer = 0;
            param_msg.value.real = 0.0f;
		};
	}	
	
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::maverick_plugins::ParamValuePlugin, mavros::plugin::PluginBase)
