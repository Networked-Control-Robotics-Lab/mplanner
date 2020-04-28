#include "../mavlink/parser.hpp"
#include "../mavlink/receiver.hpp"

enum ENUM_MAV_CMDS {
	/* mavlink common messages */
	ENUM_HANDLER_FUNC(mavlink_attitude_quaternion_handler),
	ENUM_HANDLER_FUNC(mavlink_local_position_ned_handler),
	
	/* mavlink extended messages */
	ENUM_HANDLER_FUNC(mavlink_polynomial_trajectory_ack_handler)
};

struct mavlink_parser_item cmd_list[] = {
	/* mavlink common messages */
	MAV_CMD_DEF(mavlink_attitude_quaternion_handler, 31),
	MAV_CMD_DEF(mavlink_local_position_ned_handler, 32),

	/* mavlink extended messages */
	MAV_CMD_DEF(mavlink_polynomial_trajectory_ack_handler, 11002)
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
