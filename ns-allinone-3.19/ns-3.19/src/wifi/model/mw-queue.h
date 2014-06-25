#ifndef MW_QUEUE_H
#define MW_QUEUE_H

#include <iostream>
#include <stdio.h>
#include <map>
#include "ns3/object.h"
#include "wifi-net-device.h"
#include "ns3/traced-value.h" //JJH 0530

using namespace std;
#define _DEBUG_MW_QUEUE 0


//using namespace std;


namespace ns3 {

class MWItem{
public:
	MWItem (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
	:dest(dest)
	{
		this->packet=packet;
		this->protocolNumber=protocolNumber;
	}
	Ptr<Packet> GetPacket() {
		return packet;
	}
	const Address& GetAdderss()
	{
		return dest;
	}
	uint16_t GetprotocolNumber(){
		return protocolNumber;
	}

private:
	Ptr<Packet> packet;
	const Address dest;
	uint16_t protocolNumber;
};


class MWQueue : public Object
{
public:
	static TypeId GetTypeId (void);
	MWQueue ();
	~MWQueue ();

	bool Enqueue (Ptr<Packet> packet,const Address& dest, uint16_t protocolNumber);
	void PushFront (MWItem* mwItem);
	MWItem* Dequeue ();
	MWItem* GetFront ();
	void Cleanup (void);
	uint32_t GetSize();

private:

	typedef std::list<MWItem*> PacketQueue;
	typedef std::list<MWItem*>::iterator PacketQueueI;

	PacketQueue m_queue; //!< Packet (struct Item) queue

	TracedValue<uint32_t> m_size;
	uint32_t m_maxSize; //!< Queue capacity
};



}

#endif /* ADDRESS_TABLE_H */
