#include "canprotocol.h"

namespace edu
{

void makeCanStdID(std::int32_t sysID, std::int32_t nodeID, std::int32_t* inputAddress, std::int32_t* outputAddress, std::int32_t* broadcastAddress)
{
	std::int32_t sID  =    sysID << 8;
	std::int32_t iBit =    INPUTBIT  << 7;
	std::int32_t oBit =    OUTPUTBIT << 7;
	std::int32_t nID  =    nodeID;

	*inputAddress     = sID | iBit  | nID;
	*outputAddress    = sID | oBit  | nID;
	*broadcastAddress = sID | iBit  | 0b0000000;
}

}
