#ifndef CAN_H
#define CAN_H

#include <FlexCAN.h>
#include "WOLF_COMS.h"
#include "IMU.h"

class CAN {
public:
	CAN(uint k_baud);
	void begin();
	bool RFCANMsg(WOLF_COMS::RFMsg_union msg);
	void imuCANMsg(IMU::imuUnion data);
	bool vehicleCANMSg();
	~CAN();
private:
	FlexCAN CANbus;
	CAN_message_t canTXmsg;
};

#endif //CAN.h
