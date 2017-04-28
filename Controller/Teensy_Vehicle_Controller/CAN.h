#ifndef CAN_H
#define CAN_H

#include <FlexCAN.h>
#include "WOLF_COMS.h"

class CAN {
public:

	CAN(uint k_baud);
	void begin();
	bool nRFCANMsg(WOLF_COMS::nRF24Msg_union msg);
	~CAN();
private:
	byte headerOffset = 10;	//offset by header frame size
	FlexCAN CANbus;
	CAN_message_t canTXmsg;
};

#endif //CAN.h
