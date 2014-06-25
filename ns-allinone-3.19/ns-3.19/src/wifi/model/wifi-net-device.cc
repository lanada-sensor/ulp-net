/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include "wifi-net-device.h"
#include "wifi-mac.h"
#include "wifi-phy.h"
#include "wifi-remote-station-manager.h"
#include "wifi-channel.h"
#include "ns3/llc-snap-header.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/pointer.h"
#include "ns3/node.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/log.h"
#include <iostream>

NS_LOG_COMPONENT_DEFINE ("WifiNetDevice");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (WifiNetDevice)
  ;

TypeId
WifiNetDevice::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::WifiNetDevice")
    .SetParent<NetDevice> ()
    .AddConstructor<WifiNetDevice> ()
    .AddAttribute ("Mtu", "The MAC-level Maximum Transmission Unit",
                   UintegerValue (MAX_MSDU_SIZE - LLC_SNAP_HEADER_LENGTH),
                   MakeUintegerAccessor (&WifiNetDevice::SetMtu,
                                         &WifiNetDevice::GetMtu),
                   MakeUintegerChecker<uint16_t> (1,MAX_MSDU_SIZE - LLC_SNAP_HEADER_LENGTH))
    .AddAttribute ("Channel", "The channel attached to this device",
                   PointerValue (),
                   MakePointerAccessor (&WifiNetDevice::DoGetChannel),
                   MakePointerChecker<WifiChannel> ())
    .AddAttribute ("Phy", "The PHY layer attached to this device.",
                   PointerValue (),
                   MakePointerAccessor (&WifiNetDevice::GetPhy,
                                        &WifiNetDevice::SetPhy),
                   MakePointerChecker<WifiPhy> ())
    .AddAttribute ("Mac", "The MAC layer attached to this device.",
                   PointerValue (),
                   MakePointerAccessor (&WifiNetDevice::GetMac,
                                        &WifiNetDevice::SetMac),
                   MakePointerChecker<WifiMac> ())
    .AddAttribute ("RemoteStationManager", "The station manager attached to this device.",
                   PointerValue (),
                   MakePointerAccessor (&WifiNetDevice::SetRemoteStationManager,
                                        &WifiNetDevice::GetRemoteStationManager),
                   MakePointerChecker<WifiRemoteStationManager> ())
  ;
  return tid;
}

WifiNetDevice::WifiNetDevice ()
  : m_configComplete (false)
{
  NS_LOG_FUNCTION_NOARGS ();
  test2 =0;
  test1 =0;
  dataRate = 1;
}
WifiNetDevice::~WifiNetDevice ()
{
  NS_LOG_FUNCTION_NOARGS ();
}



///////////////////////////////////////
// MW-MAC  add by LANADA
///////////////////////////////////////
// constructor //
WifiNetDevice::WifiNetDevice (uint32_t isMain)
  : m_configComplete (false)
{
  NS_LOG_FUNCTION_NOARGS ();
  m_main = 0;
  m_ulp = 0;
  isMwmac = true;
  this->isMain = isMain;
  isWorking = false;
  isAcked = 1;
  isTXed = 1;
  dataRate = 1;
//  isWakeupReq = false;
//  tempProtocolNumber=0;
//  channelInfo = 0;
//  timeOutValue = Seconds(0.0);

  //mwQueue
//  mwQueue = new MWQueue();
  mwQueue = CreateObject<MWQueue> (); //JJH 0530 for tracing
//  if(isMain) //JJH 0507
//  {
//      isSleep = true;
//  }
//  else
//  {
//      isSleep = false;
//  }
}
// install //
bool
WifiNetDevice::SetMainDevice(Ptr<WifiNetDevice> m_main)
{
       this->m_main = m_main;
       Ptr<WifiNetDevice> temp_ptr = DynamicCast<WifiNetDevice>(this->m_main);
       temp_ptr->Sleep(false);
       return true;
}
bool
WifiNetDevice::SetUlpDevice(Ptr<WifiNetDevice> m_ulp){
       this->m_ulp = m_ulp;
//       this->m_ulp->SetReceiveCallback (MakeCallback (&WifiNetDevice::RecvFromUlp, this));
       return true;
}

Ptr<WifiNetDevice>
WifiNetDevice::GetMainDevice (void){
    return this->m_main;
}

Ptr<WifiNetDevice>
WifiNetDevice::GetUlpDevice (void){
    return this->m_ulp;
}


bool
WifiNetDevice::SetValue(uint32_t index, uint32_t value)
{
	switch(index)
	{
	case 1 :
		isMain = value;
		this->m_MacLow->SetValue(index,value);
		break;
	case 2 :
		mainType = value;
		this->m_MacLow->SetValue(index,value);
		break;
	case 3 :
		channelType = value;
		this->m_MacLow->SetValue(index,value);
		break;
	}
	return true;
}
uint32_t
WifiNetDevice::GetValue(uint32_t index){

	switch(index)
	{
	case 1 :
		return isMain;
		break;
	case 2 :
		return mainType;
		break;
	case 3 :
		return channelType;
		break;
	}
	return -1;
}


bool
WifiNetDevice::SetAddressTable(AddressTable* table){
    this->m_addressTable = table;
    return true;
}

AddressTable*
WifiNetDevice::GetAddressTable()
{
	return this->m_addressTable;
}


bool
WifiNetDevice::SetDataRate(uint32_t dataRate)
{
	this->dataRate = dataRate;
	return true;
}

bool
WifiNetDevice::SetMacLow()
{
	Ptr<RegularWifiMac> temp_regwifimac= this->GetMac()->GetObject<RegularWifiMac>();
	Ptr<DcaTxop> temp_DcaTxop = temp_regwifimac->GetDcaTxop();
	m_MacLow = temp_DcaTxop->m_low;
	m_MacLow->SetDevice(this);
	return true;
}

Ptr<MacLow>
WifiNetDevice::GetMacLow()
{
	return m_MacLow;
}



bool
WifiNetDevice::SetDeviceContainer(NetDeviceContainer* netDevice)
{
	this->netDevice = netDevice;
	return true;
}

NetDeviceContainer*
WifiNetDevice::GetDeviceContainer()
{
	return this->netDevice;
}



// sleep
bool
WifiNetDevice::Sleep(bool isSender)
{
//#if _DEBUG_NET_DEVICE
//    std::cout<<"[Device] Main Device Sleeps! [node id :"<<this->GetNode()->GetId()<<"]\n";
//#endif
//	this->isSleep=1;
//	this->m_mac->SetMacSleep(this->isSleep,isSender,0);
	return true;
}


// send
uint32_t
WifiNetDevice::SendAfterWakeupSignal(bool retransmission)
{
//	if(Simulator::Now() > Seconds(9.7) && this->GetNode()->GetId()==2 && Simulator::Now() < Seconds(10.0))
//	{
//		std::cout<<"SendAfterWakeupSignal "<<retransmission<<"\ttime:"<<Simulator::Now().GetSeconds()<<std::endl;
//	}
//	if(retransmission == true)
//		std::cout<<"SendAfterWakeupSignal retransmission "<<this->GetNode()->GetId()<<std::endl;
	if(this->isMain == 2) //ULP radio
	{
		this->GetMainDevice()->SendAfterWakeupSignal(false);
	}
	else if (this->isMain == 1) // Main radio
	{
		isAcked = 0;
		isTXed = 0;
		// real send
		if (retransmission == false) // retransmission case/ not wake up
		{
//			this->m_MacLow->SetLowSleep(2,1,0); // wake up req/ sender/ 0 //0625

		}
		MWItem* mwItem = this->mwQueue->GetFront();
		if(mwItem == 0){
			std::cout<<"mwItem = 0  "<<this->GetNode()->GetId()<<" "<<isWorking<<" "<<this->GetMacLow()->GetSleepCnt()<<std::endl;
			this->m_MacLow->SetLowSleep(-1,-1,1);
			return -1;
		}


		Ptr<Packet> packet = mwItem->GetPacket()->Copy();
 		LlcSnapHeader llc;
 		llc.SetType(mwItem->GetprotocolNumber());
 		packet->AddHeader(llc);

 		packet->RemoveAllPacketTags();
		MWTag mainTag;
		if(this->channelType == 1) // shared channel
		{
			mainTag.SetSimpleValue(0xff);//shared Main device = 0xff, separate Main device or ULP = 0x0f
		}
		else
		{
			mainTag.SetSimpleValue(0x0f);
		}
		packet->AddPacketTag(mainTag);

		m_mac->NotifyTx(packet);

		Mac48Address realTo = Mac48Address::ConvertFrom(mwItem->GetAdderss());//Main address


		m_mac->Enqueue(packet, realTo);
//		Sleep(true); //Sleep after finishing the sending a data
		//here
	}
	return 0;
}

uint32_t
WifiNetDevice::SendDoneAck()
{

	this->isAcked = 1;
	this->isTXed = 1;
//	this->m_MacLow->SetLowSleep(1,1,0); // sleep request/ sender/ 0

	MWItem* mwItem = this->mwQueue->Dequeue();
	delete mwItem;
	this->isWorking = false;
	this->SendFromMWQueue();
	return 0;
}

uint32_t
WifiNetDevice::SendFromMWQueue()
{
	if(mwQueue->GetSize()==0)
	{
		return -1;
	}
	if(isWorking)
	{

		return -2; //kdw
	}


	isWorking = true;
	MWItem* mwItem = this->mwQueue->GetFront();
	Ptr<Packet> packet = mwItem->GetPacket();
//	Address* tempAddress = new Address(mwItem->GetAdderss());

	//	MWtempAddress = tempAddress;



#if _DEBUG_NET_DEVICE
	std::cout<<"isMain [node id :"<<this->GetNode()->GetId()<<"]\n";
#endif

	////
	this->m_MacLow->SetPacketId(1,(uint32_t)packet->GetUid());

	////////
	this->GetUlpDevice()->Send(packet,m_addressTable->GetUlpAddress(mwItem->GetAdderss()),mwItem->GetprotocolNumber());
	return 0;
}

bool
WifiNetDevice::Send (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{

	if(this->isMwmac == true)
	{
		if(isMain == 1) // main device
		{
			packet->RemoveAllPacketTags();
			MWTag mainTag;
			if(channelType == 1)
			{
				mainTag.SetSimpleValue(0xff);//shared Main device = 0xff, separate Main device or ULP = 0x0f
			}
			else
			{
				mainTag.SetSimpleValue(0x0f);
			}
			packet->AddPacketTag(mainTag);
			if (!(this->mwQueue->Enqueue(packet,dest,protocolNumber)))
			{
				std::cout<<"queue overflow "<<this->GetNode()->GetId()<<std::endl;
			}
			this->SendFromMWQueue();
		}
		else if(isMain == 2)//ULP
		{
//			uint32_t ulpPacketSize = packet->GetSize()*2/5;
			uint32_t ulpPacketSize = packet->GetSize()*(dataRate)/250+1;
			uint8_t* temp = new uint8_t[ulpPacketSize];
			temp[0] = 127;

			if (this->channelType == 2) //separate channel
			{
				ulpPacketSize=1;
			}

			Ptr<Packet> newPacket = Create<Packet> (temp,ulpPacketSize);
			NS_ASSERT(Mac48Address::IsMatchingType(dest));

			Mac48Address realTo = Mac48Address::ConvertFrom(dest);

			LlcSnapHeader llc;
			llc.SetType(protocolNumber);
			newPacket->AddHeader(llc);

			MWTag ulpTag;
			ulpTag.SetSimpleValue(0x0f);//shared Main device = 0xff, separate Main device or ULP = 0x0f
			newPacket->AddPacketTag(ulpTag);

			this->m_MacLow->SetPacketId(1,(uint32_t)packet->GetUid());

			m_mac->NotifyTx(newPacket);
			m_mac->Enqueue(newPacket, realTo);

			free(temp);
			return true;
		}
	}
	else
	{
		NS_ASSERT (Mac48Address::IsMatchingType (dest));

		Mac48Address realTo = Mac48Address::ConvertFrom (dest);

		LlcSnapHeader llc;
		llc.SetType (protocolNumber);
		packet->AddHeader (llc);

		m_mac->NotifyTx (packet);
		m_mac->Enqueue (packet, realTo);
	}
	return true;
}
void
WifiNetDevice::SetBuffer(Ptr<Packet> packet)
{
    this->m_bufferlen=packet->GetSize();
    this->m_buffer=packet->Copy();
}

Ptr<Packet>
WifiNetDevice::GetBuffer(void) const
{
    return this->m_buffer;
}

uint32_t
WifiNetDevice::GetBufferlen (void) const
{
    return this->m_bufferlen;
}

Ptr<MWQueue>
WifiNetDevice::GetMWQueue (void) const
{
    return this->mwQueue;
}

// retransmission //
void
WifiNetDevice::RetransmissionReq(uint32_t option)
{
	if(option == 1)
	{
		std::cout<<"hello!!! txDone"<<std::endl;
		this->m_MacLow->SetLowSleep(-1,-1,2);
		isWorking = false;
		this->SendFromMWQueue();
		return;
	}
	if (isMain == 1) // Main Radio
	{
//		Time timeout = Simulator::Now() + Seconds(0.1);
//		Time timeout = Simulator::Now() + timeOutValue;
//		m_MWTimeoutEvent = Simulator::Schedule(timeout,&WifiNetDevice::RetransmissionMain,this,1);

//		if (isAcked == 1)
//			return;

		RetransmissionMain(1);

	}
	else if (isMain == 2) // ULP Radio
	{
		this->GetMainDevice()->RetransmissionMain(2);
	}
}
void
WifiNetDevice::RetransmissionMain(uint32_t option)
{
	if (isMain == 2) // ULP Radio
			return;
//	isWorking = false;
	if (option == 1) // retransmission: dst wakeup/ send data again
	{
		this->SendAfterWakeupSignal(true);
	}
	else if (option == 2) // retransmission: dst sleep/ send wake up signa again
	{
		isWorking = false;
		this->SendFromMWQueue();
	}
}


void
WifiNetDevice::SetIsTXed(uint32_t i)
{
	this->isTXed = i;
}
uint32_t
WifiNetDevice::GetIsTXed()
{
	return this->isTXed;
}
///////////////////////////////////////
// MW-MAC  END!
///////////////////////////////////////


void
WifiNetDevice::DoDispose (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  m_node = 0;
  m_mac->Dispose ();
  m_phy->Dispose ();
  m_stationManager->Dispose ();
  m_mac = 0;
  m_phy = 0;
  m_stationManager = 0;
  // chain up.
  NetDevice::DoDispose ();
}

void
WifiNetDevice::DoInitialize (void)
{
  m_phy->Initialize ();
  m_mac->Initialize ();
  m_stationManager->Initialize ();
  NetDevice::DoInitialize ();
}

void
WifiNetDevice::CompleteConfig (void)
{
  if (m_mac == 0
      || m_phy == 0
      || m_stationManager == 0
      || m_node == 0
      || m_configComplete)
    {
      return;
    }
  m_mac->SetWifiRemoteStationManager (m_stationManager);
  m_mac->SetWifiPhy (m_phy);
  m_mac->SetForwardUpCallback (MakeCallback (&WifiNetDevice::ForwardUp, this));
  m_mac->SetLinkUpCallback (MakeCallback (&WifiNetDevice::LinkUp, this));
  m_mac->SetLinkDownCallback (MakeCallback (&WifiNetDevice::LinkDown, this));
  m_stationManager->SetupPhy (m_phy);
  m_configComplete = true;
}

void
WifiNetDevice::SetMac (Ptr<WifiMac> mac)
{
  m_mac = mac;
  CompleteConfig ();
}
void
WifiNetDevice::SetPhy (Ptr<WifiPhy> phy)
{
  m_phy = phy;
  CompleteConfig ();
}
void
WifiNetDevice::SetRemoteStationManager (Ptr<WifiRemoteStationManager> manager)
{
  m_stationManager = manager;
  CompleteConfig ();
}
Ptr<WifiMac>
WifiNetDevice::GetMac (void) const
{
  return m_mac;
}
Ptr<WifiPhy>
WifiNetDevice::GetPhy (void) const
{
  return m_phy;
}
Ptr<WifiRemoteStationManager>
WifiNetDevice::GetRemoteStationManager (void) const
{
  return m_stationManager;
}

void
WifiNetDevice::SetIfIndex (const uint32_t index)
{
  m_ifIndex = index;
}
uint32_t
WifiNetDevice::GetIfIndex (void) const
{
  return m_ifIndex;
}
Ptr<Channel>
WifiNetDevice::GetChannel (void) const
{
  return m_phy->GetChannel ();
}
Ptr<WifiChannel>
WifiNetDevice::DoGetChannel (void) const
{
  return m_phy->GetChannel ();
}
void
WifiNetDevice::SetAddress (Address address)
{
  m_mac->SetAddress (Mac48Address::ConvertFrom (address));
}
Address
WifiNetDevice::GetAddress (void) const
{
  return m_mac->GetAddress ();
}
bool
WifiNetDevice::SetMtu (const uint16_t mtu)
{
  if (mtu > MAX_MSDU_SIZE - LLC_SNAP_HEADER_LENGTH)
    {
      return false;
    }
  m_mtu = mtu;
  return true;
}
uint16_t
WifiNetDevice::GetMtu (void) const
{
  return m_mtu;
}
bool
WifiNetDevice::IsLinkUp (void) const
{
  return m_phy != 0 && m_linkUp;
}
void
WifiNetDevice::AddLinkChangeCallback (Callback<void> callback)
{
  m_linkChanges.ConnectWithoutContext (callback);
}
bool
WifiNetDevice::IsBroadcast (void) const
{
  return true;
}
Address
WifiNetDevice::GetBroadcast (void) const
{
  return Mac48Address::GetBroadcast ();
}
bool
WifiNetDevice::IsMulticast (void) const
{
  return true;
}
Address
WifiNetDevice::GetMulticast (Ipv4Address multicastGroup) const
{
  return Mac48Address::GetMulticast (multicastGroup);
}
Address WifiNetDevice::GetMulticast (Ipv6Address addr) const
{
  return Mac48Address::GetMulticast (addr);
}
bool
WifiNetDevice::IsPointToPoint (void) const
{
  return false;
}
bool
WifiNetDevice::IsBridge (void) const
{
  return false;
}

Ptr<Node>
WifiNetDevice::GetNode (void) const
{
  return m_node;
}
void
WifiNetDevice::SetNode (Ptr<Node> node)
{
  m_node = node;
  CompleteConfig ();
}
bool
WifiNetDevice::NeedsArp (void) const
{
  return true;
}
void
WifiNetDevice::SetReceiveCallback (NetDevice::ReceiveCallback cb)
{
  m_forwardUp = cb;
}

void
WifiNetDevice::ForwardUp (Ptr<Packet> packet, Mac48Address from, Mac48Address to)
{
  LlcSnapHeader llc;
  packet->RemoveHeader (llc);
  enum NetDevice::PacketType type;
  if (to.IsBroadcast ())
    {
      type = NetDevice::PACKET_BROADCAST;
    }
  else if (to.IsGroup ())
    {
      type = NetDevice::PACKET_MULTICAST;
    }
  else if (to == m_mac->GetAddress ())
    {
      type = NetDevice::PACKET_HOST;
    }
  else
    {
      type = NetDevice::PACKET_OTHERHOST;
    }

//  if (this->isMwmac)
//  {
//	  if (this->isMain == 1) // Main radio
//	  {
//		  this->m_MacLow->SetLowSleep(1,2,0); // sleep request/ receiver(got data) /0
//	  }
//  }

  if (type != NetDevice::PACKET_OTHERHOST)
    {
      m_mac->NotifyRx (packet);
      m_forwardUp (this, packet, llc.GetType (), from);
    }

  if (!m_promiscRx.IsNull ())
    {
      m_mac->NotifyPromiscRx (packet);
      m_promiscRx (this, packet, llc.GetType (), from, to, type);
    }
}

void
WifiNetDevice::LinkUp (void)
{
  m_linkUp = true;
  m_linkChanges ();
}
void
WifiNetDevice::LinkDown (void)
{
  m_linkUp = false;
  m_linkChanges ();
}

bool
WifiNetDevice::SendFrom (Ptr<Packet> packet, const Address& source, const Address& dest, uint16_t protocolNumber)
{
  NS_ASSERT (Mac48Address::IsMatchingType (dest));
  NS_ASSERT (Mac48Address::IsMatchingType (source));

  Mac48Address realTo = Mac48Address::ConvertFrom (dest);
  Mac48Address realFrom = Mac48Address::ConvertFrom (source);

  LlcSnapHeader llc;
  llc.SetType (protocolNumber);
  packet->AddHeader (llc);

  m_mac->NotifyTx (packet);
  m_mac->Enqueue (packet, realTo, realFrom);

  return true;
}

void
WifiNetDevice::SetPromiscReceiveCallback (PromiscReceiveCallback cb)
{
  m_promiscRx = cb;
  m_mac->SetPromisc();
}

bool
WifiNetDevice::SupportsSendFrom (void) const
{
  return m_mac->SupportsSendFrom ();
}

TypeId
MWTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MyTag")
    .SetParent<Tag> ()
    .AddConstructor<MWTag> ()
    .AddAttribute ("SimpleValue",
                   "A simple value",
                   EmptyAttributeValue (),
                   MakeUintegerAccessor (&MWTag::GetSimpleValue),
                   MakeUintegerChecker<uint8_t> ())
  ;
  return tid;
}
TypeId
MWTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
MWTag::GetSerializedSize (void) const
{
  return 1;
}
void
MWTag::Serialize (TagBuffer i) const
{
  i.WriteU8 (m_simpleValue);
}
void
MWTag::Deserialize (TagBuffer i)
{
  m_simpleValue = i.ReadU8 ();
}
void
MWTag::Print (std::ostream &os) const
{
  os << "v=" << (uint32_t)m_simpleValue;
}
void
MWTag::SetSimpleValue (uint8_t value)
{
  m_simpleValue = value;
}
uint8_t
MWTag::GetSimpleValue (void) const
{
  return m_simpleValue;
}

void
MWTag::SetAddressValue (Mac48Address address){
	this->address = address;
}
Mac48Address
MWTag::GetAddressValue (void) const
{
	return this->address;
}

} // namespace ns3

