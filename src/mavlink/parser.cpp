#include "../mavlink/parser.hpp"
#include "../mavlink/receiver.hpp"

enum ENUM_MAV_CMDS {
	ENUM_HANDLER_FUNC(mavlink_attitude_quaternion_handler),
	ENUM_HANDLER_FUNC(mavlink_local_position_ned_handler)
};

struct mavlink_parser_item cmd_list[] = {
	MAV_CMD_DEF(mavlink_attitude_quaternion_handler, 31),
	MAV_CMD_DEF(mavlink_local_position_ned_handler, 32),
};

void parse_mavlink_received_msg(mavlink_message_t *msg)
{
	int i;
	for(i = 0; i < (signed int)CMD_LEN(cmd_list); i++) {
		if(msg->msgid == cmd_list[i].msg_id) {
			cmd_list[i].handler(msg);
			break;
		}
	}
}
