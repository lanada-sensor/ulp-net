/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
 * Copyright (c) 2009 MIRKO BANCHI
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
 * Author: Mirko Banchi <mk.banchi@gmail.com>
 */

#include "ns3/assert.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/tag.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/double.h"

#include "mac-low.h"
#include "wifi-phy.h"
#include "wifi-mac-trailer.h"
#include "qos-utils.h"
#include "edca-txop-n.h"
#include "snr-tag.h"

#include <iostream>

NS_LOG_COMPONENT_DEFINE ("MacLow");

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT std::clog << "[mac=" << m_self << "] "
#define _DEBUG_MAC_LOW 0

namespace ns3 {

MacLowTransmissionListener::MacLowTransmissionListener ()
{
}
MacLowTransmissionListener::~MacLowTransmissionListener ()
{
}
void
MacLowTransmissionListener::GotBlockAck (const CtrlBAckResponseHeader *blockAck,
                                         Mac48Address source)
{
}
void
MacLowTransmissionListener::MissedBlockAck (void)
{
}
MacLowDcfListener::MacLowDcfListener ()
{
}
MacLowDcfListener::~MacLowDcfListener ()
{
}

MacLowBlockAckEventListener::MacLowBlockAckEventListener ()
{
}
MacLowBlockAckEventListener::~MacLowBlockAckEventListener ()
{
}

MacLowTransmissionParameters::MacLowTransmissionParameters ()
  : m_nextSize (0),
    m_waitAck (ACK_NONE),
    m_sendRts (false),
    m_overrideDurationId (Seconds (0))
{
}
void
MacLowTransmissionParameters::EnableNextData (uint32_t size)
{
  m_nextSize = size;
}
void
MacLowTransmissionParameters::DisableNextData (void)
{
  m_nextSize = 0;
}
void
MacLowTransmissionParameters::EnableOverrideDurationId (Time durationId)
{
  m_overrideDurationId = durationId;
}
void
MacLowTransmissionParameters::DisableOverrideDurationId (void)
{
  m_overrideDurationId = Seconds (0);
}
void
MacLowTransmissionParameters::EnableSuperFastAck (void)
{
  m_waitAck = ACK_SUPER_FAST;
}
void
MacLowTransmissionParameters::EnableBasicBlockAck (void)
{
  m_waitAck = BLOCK_ACK_BASIC;
}
void
MacLowTransmissionParameters::EnableCompressedBlockAck (void)
{
  m_waitAck = BLOCK_ACK_COMPRESSED;
}
void
MacLowTransmissionParameters::EnableMultiTidBlockAck (void)
{
  m_waitAck = BLOCK_ACK_MULTI_TID;
}
void
MacLowTransmissionParameters::EnableFastAck (void)
{
  m_waitAck = ACK_FAST;
}
void
MacLowTransmissionParameters::EnableAck (void)
{
  m_waitAck = ACK_NORMAL;
}
void
MacLowTransmissionParameters::DisableAck (void)
{
  m_waitAck = ACK_NONE;
}
void
MacLowTransmissionParameters::EnableRts (void)
{
  m_sendRts = true;
}
void
MacLowTransmissionParameters::DisableRts (void)
{
  m_sendRts = false;
}
bool
MacLowTransmissionParameters::MustWaitAck (void) const
{
  return (m_waitAck != ACK_NONE);
}
bool
MacLowTransmissionParameters::MustWaitNormalAck (void) const
{
  return (m_waitAck == ACK_NORMAL);
}
bool
MacLowTransmissionParameters::MustWaitFastAck (void) const
{
  return (m_waitAck == ACK_FAST);
}
bool
MacLowTransmissionParameters::MustWaitSuperFastAck (void) const
{
  return (m_waitAck == ACK_SUPER_FAST);
}
bool
MacLowTransmissionParameters::MustWaitBasicBlockAck (void) const
{
  return (m_waitAck == BLOCK_ACK_BASIC) ? true : false;
}
bool
MacLowTransmissionParameters::MustWaitCompressedBlockAck (void) const
{
  return (m_waitAck == BLOCK_ACK_COMPRESSED) ? true : false;
}
bool
MacLowTransmissionParameters::MustWaitMultiTidBlockAck (void) const
{
  return (m_waitAck == BLOCK_ACK_MULTI_TID) ? true : false;
}
bool
MacLowTransmissionParameters::MustSendRts (void) const
{
  return m_sendRts;
}
bool
MacLowTransmissionParameters::HasDurationId (void) const
{
  return (m_overrideDurationId != Seconds (0));
}
Time
MacLowTransmissionParameters::GetDurationId (void) const
{
  NS_ASSERT (m_overrideDurationId != Seconds (0));
  return m_overrideDurationId;
}
bool
MacLowTransmissionParameters::HasNextPacket (void) const
{
  return (m_nextSize != 0);
}
uint32_t
MacLowTransmissionParameters::GetNextPacketSize (void) const
{
  NS_ASSERT (HasNextPacket ());
  return m_nextSize;
}

std::ostream &operator << (std::ostream &os, const MacLowTransmissionParameters &params)
{
  os << "["
  << "send rts=" << params.m_sendRts << ", "
  << "next size=" << params.m_nextSize << ", "
  << "dur=" << params.m_overrideDurationId << ", "
  << "ack=";
  switch (params.m_waitAck)
    {
    case MacLowTransmissionParameters::ACK_NONE:
      os << "none";
      break;
    case MacLowTransmissionParameters::ACK_NORMAL:
      os << "normal";
      break;
    case MacLowTransmissionParameters::ACK_FAST:
      os << "fast";
      break;
    case MacLowTransmissionParameters::ACK_SUPER_FAST:
      os << "super-fast";
      break;
    case MacLowTransmissionParameters::BLOCK_ACK_BASIC:
      os << "basic-block-ack";
      break;
    case MacLowTransmissionParameters::BLOCK_ACK_COMPRESSED:
      os << "compressed-block-ack";
      break;
    case MacLowTransmissionParameters::BLOCK_ACK_MULTI_TID:
      os << "multi-tid-block-ack";
      break;
    }
  os << "]";
  return os;
}


/**
 * Listener for PHY events. Forwards to MacLow
 */
class PhyMacLowListener : public ns3::WifiPhyListener
{
public:
  /**
   * Create a PhyMacLowListener for the given MacLow.
   *
   * \param macLow
   */
  PhyMacLowListener (ns3::MacLow *macLow)
    : m_macLow (macLow)
  {
  }
  virtual ~PhyMacLowListener ()
  {
  }
  virtual void NotifyRxStart (Time duration)
  {
  }
  virtual void NotifyRxEndOk (void)
  {
  }
  virtual void NotifyRxEndError (void)
  {
  }
  virtual void NotifyTxStart (Time duration)
  {
  }
  virtual void NotifyMaybeCcaBusyStart (Time duration)
  {
  }
  virtual void NotifySwitchingStart (Time duration)
  {
    m_macLow->NotifySwitchingStartNow (duration);
  }
private:
  ns3::MacLow *m_macLow;
};


MacLow::MacLow ()
  : m_normalAckTimeoutEvent (),
    m_fastAckTimeoutEvent (),
    m_superFastAckTimeoutEvent (),
    m_fastAckFailedTimeoutEvent (),
    m_blockAckTimeoutEvent (),
    m_ctsTimeoutEvent (),
    m_sendCtsEvent (),
    m_sendAckEvent (),
    m_sendDataEvent (),
    m_waitSifsEvent (),
    m_endTxNoAckEvent (),
    m_currentPacket (0),
    m_listener (0),
    m_phyMacLowListener (0),
    m_ctsToSelfSupported (false)
{
  NS_LOG_FUNCTION (this);
  m_lastNavDuration = Seconds (0);
  m_lastNavStart = Seconds (0);
  m_promisc = false;
  isMain=0;
  mainType=0;
  channelType=0;
  m_sleep_s_cnt=0;
  m_sleep_r_cnt=0;
  this->sendPacketId = -1;
  this->recvPacketId = -1;
  checkPacketId = -1;
}

MacLow::~MacLow ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
MacLow::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MacLow")
    .SetParent<Object> ()
    .AddConstructor<MacLow> ()
    .AddTraceSource("SleepCount","Main radio's ack count value",
      	     		   MakeTraceSourceAccessor (&MacLow::m_sleep_cnt))
  ;
  return tid;
}

void
MacLow::SetupPhyMacLowListener (Ptr<WifiPhy> phy)
{
  m_phyMacLowListener = new PhyMacLowListener (this);
  phy->RegisterListener (m_phyMacLowListener);
}


void
MacLow::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_normalAckTimeoutEvent.Cancel ();
  m_fastAckTimeoutEvent.Cancel ();
  m_superFastAckTimeoutEvent.Cancel ();
  m_fastAckFailedTimeoutEvent.Cancel ();
  m_blockAckTimeoutEvent.Cancel ();
  m_ctsTimeoutEvent.Cancel ();
  m_sendCtsEvent.Cancel ();
  m_sendAckEvent.Cancel ();
  m_sendDataEvent.Cancel ();
  m_waitSifsEvent.Cancel ();
  m_endTxNoAckEvent.Cancel ();
   m_waitRifsEvent.Cancel();
  m_phy = 0;
  m_stationManager = 0;
  if (m_phyMacLowListener != 0)
    {
	  delete m_phyMacLowListener;
	  m_phyMacLowListener = 0;
    }
}

void
MacLow::CancelAllEvents (void)
{
  NS_LOG_FUNCTION (this);
  bool oneRunning = false;
  if (m_normalAckTimeoutEvent.IsRunning ())
    {
      m_normalAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
  if (m_fastAckTimeoutEvent.IsRunning ())
    {
      m_fastAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
  if (m_superFastAckTimeoutEvent.IsRunning ())
    {
      m_superFastAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
  if (m_fastAckFailedTimeoutEvent.IsRunning ())
    {
      m_fastAckFailedTimeoutEvent.Cancel ();
      oneRunning = true;
    }
  if (m_blockAckTimeoutEvent.IsRunning ())
    {
      m_blockAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
  if (m_ctsTimeoutEvent.IsRunning ())
    {
      m_ctsTimeoutEvent.Cancel ();
      oneRunning = true;
    }
  if (m_sendCtsEvent.IsRunning ())
    {
      m_sendCtsEvent.Cancel ();
      oneRunning = true;
    }
  if (m_sendAckEvent.IsRunning ())
    {
      m_sendAckEvent.Cancel ();
      oneRunning = true;
    }
  if (m_sendDataEvent.IsRunning ())
    {
      m_sendDataEvent.Cancel ();
      oneRunning = true;
    }
  if (m_waitSifsEvent.IsRunning ())
    {
      m_waitSifsEvent.Cancel ();
      oneRunning = true;
    }
  if (m_waitRifsEvent.IsRunning ())
    {
      m_waitRifsEvent.Cancel ();
      oneRunning = true;
    }
  if (m_endTxNoAckEvent.IsRunning ()) 
    {
      m_endTxNoAckEvent.Cancel ();
      oneRunning = true;
    }
  if (oneRunning && m_listener != 0)
    {
      m_listener->Cancel ();
      m_listener = 0;
    }
}

///////////////////////////////////////
// MW-MAC  add by LANADA
///////////////////////////////////////

bool
MacLow::SetValue(uint32_t index, uint32_t value)
{
	switch(index)
	{
	case 1 :
		this->isMain = value;
		break;
	case 2 :
		mainType = value;
		break;
	case 3 :
		channelType = value;
		break;
	}
	return true;
}
uint32_t
MacLow::GetValue(uint32_t index){

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
MacLow::SetDevice(Ptr<WifiNetDevice> device)
{
	this->m_device = device;
	return true;
}
Ptr<WifiNetDevice>
MacLow::GetDevice()
{
	return this->m_device;
}

bool
MacLow::IsSleep()
{
	if (m_sleep_cnt == 0) // sleep
		return true;
	else
		return false;
}

int32_t
MacLow::GetSleepCnt()
{
	std::cout<<m_sleep_s_cnt<<" "<<sleepRequest.size()<<std::endl;
	return m_sleep_s_cnt + sleepRequest.size() ;

}

int32_t
MacLow::GetLowSleep(uint32_t option)
{
	if (option == 1)
		return m_sleep_s_cnt;
	else
		return m_sleep_r_cnt;
}

void
MacLow::SetLowSleep(uint32_t isSleepReq, uint32_t isSender, uint32_t option)
{
	if (this->isMain == 2) // ULP return;
		return;



//	if (option == 1) // empty queue -> send cnt = 0;
//	{
//		if (m_sleep_s_cnt>0)
//			m_sleep_s_cnt = 0;
//	}
//	if (option == 2) //problem!! txdone
//	{
//		if (m_sleep_s_cnt>0)
//			m_sleep_s_cnt -- ;
//	}
//	if (option == 3) //wake up dst ok but wakeup src fail!!
//	{
//		if (m_sleep_r_cnt>0)
//			m_sleep_r_cnt -- ;
//	}


	if (isSleepReq == 2) // wake up request
	{
		if (isSender == 1) // Sender
		{
			if (m_sleep_s_cnt == 0)
			{
				this->m_sleep_s_cnt = 1;
			}
			else
			{
				std::cout<<"aleady wake up  "<<this->GetDevice()->GetNode()->GetId()<<std::endl;
			}
			#if _DEBUG_MAC_LOW
			if(m_sleep_s_cnt+m_sleep_r_cnt == 1)
				std::cout<<"[mac=" << m_self << "]^"<<"[REQ Wakeup] (Sender) at "<<Simulator::Now().GetSeconds()<<"\t("<<isSleepReq<<" "<<isSender<<" "<<option<<")"<<std::endl;
			else
				std::cout<<"[mac=" << m_self << "] "<<"[REQ Wakeup] (Sender) at "<<Simulator::Now().GetSeconds()<<"\t("<<isSleepReq<<" "<<isSender<<" "<<option<<")"<<std::endl;
			#endif
		}
		else if (isSender == 2) // Recv
		{
//			this->m_sleep_r_cnt ++;

//			this->sleepRequest

			for (std::list<uint32_t>::iterator it = this->sleepRequest.begin(); it != this->sleepRequest.end(); it++)
			{
				if (*it == option)
				{
					return;
				}
			}
			sleepRequest.push_front(option);


			#if _DEBUG_MAC_LOW
			if(m_sleep_s_cnt+m_sleep_r_cnt == 1)
				std::cout<<"[mac=" << m_self << "]^"<<"[REQ Wakeup] (Recv  ) at "<<Simulator::Now().GetSeconds()<<"\t("<<isSleepReq<<" "<<isSender<<" "<<option<<")"<<std::endl;
			else
				std::cout<<"[mac=" << m_self << "] "<<"[REQ Wakeup] (Recv  ) at "<<Simulator::Now().GetSeconds()<<"\t("<<isSleepReq<<" "<<isSender<<" "<<option<<")"<<std::endl;
			#endif
		}
	}
	else if (isSleepReq == 1) // sleep request
	{
		if(IsSleep())
			return;

		if (isSender == 1) // Sender // sleep after got ack
		{
			if (m_sleep_s_cnt > 0)
				this->m_sleep_s_cnt = 0;

			#if _DEBUG_MAC_LOW
			if (IsSleep())
				std::cout<<"[mac=" << m_self << "]-"<<"[REQ Sleep ] (Sender) at "<<Simulator::Now().GetSeconds()<<"\t("<<isSleepReq<<" "<<isSender<<" "<<option<<")"<<std::endl;
			else
				std::cout<<"[mac=" << m_self << "] "<<"[REQ Sleep ] (Sender) at "<<Simulator::Now().GetSeconds()<<"\t("<<isSleepReq<<" "<<isSender<<" "<<option<<")"<<std::endl;
			#endif
		}
		else if (isSender == 2) // Recv
		{
			bool flag = false;
			for (std::list<uint32_t>::iterator it = this->sleepRequest.begin(); it != this->sleepRequest.end(); it++)
			{
				if (*it == option)
				{
					it = sleepRequest.erase(it);
					flag = true;
				}
			}
			if(!flag)
				std::cout<<"sleep request but no!!!\n";


			#if _DEBUG_MAC_LOW
			if (IsSleep())
				std::cout<<"[mac=" << m_self << "]-"<<"[REQ Sleep ] (Recv  ) at "<<Simulator::Now().GetSeconds()<<"\t("<<isSleepReq<<" "<<isSender<<" "<<option<<")"<<std::endl;
			else
				std::cout<<"[mac=" << m_self << "] "<<"[REQ Sleep ] (Recv  ) at "<<Simulator::Now().GetSeconds()<<"\t("<<isSleepReq<<" "<<isSender<<" "<<option<<")"<<std::endl;
			#endif
		}
	}

	m_sleep_cnt = m_sleep_s_cnt + sleepRequest.size();

//	if(Simulator::Now() > Seconds(50.0) && this->GetDevice()->GetNode()->GetId()==16 && Simulator::Now() < Seconds(70.0))
//	if(this->GetDevice()->GetNode()->GetId()==8 )
//	{
//		std::cout<<"MacLow::SetLowSleep "<<isSleepReq<<" "<<isSender<<" \t"<<m_sleep_s_cnt<<" "<<sleepRequest.size()<<"\t"<<this->GetDevice()->GetMWQueue()->GetSize()<<" "<<Simulator::Now().GetSeconds()<<std::endl;
//	}
}

void
MacLow::UlpData()
{
    if (m_txParams.MustWaitNormalAck ()
        && m_normalAckTimeoutEvent.IsRunning ())
      {
        m_normalAckTimeoutEvent.Cancel ();
        NotifyAckTimeoutResetNow ();
      }
    if (m_txParams.MustWaitFastAck ()
        && m_fastAckTimeoutEvent.IsRunning ())
      {
        m_fastAckTimeoutEvent.Cancel ();
        NotifyAckTimeoutResetNow ();
      }
}

void
MacLow::SetPacketId(uint32_t option, uint32_t packetId)
{
	if (option == 1)
		this->sendPacketId = packetId;
	else if (option == 2)
		this->recvPacketId = packetId;
	else
		this->checkPacketId = packetId;

}

uint32_t
MacLow::GetPacketId(uint32_t option)
{
	if (option == 1)
		return this->sendPacketId;
	else if (option == 2)
		return this->recvPacketId;
	else
		return this->checkPacketId;

}





void
MacLow::ReceiveOk (Ptr<Packet> packet, double rxSnr, WifiMode txMode, WifiPreamble preamble)
{
  NS_LOG_FUNCTION (this << packet << rxSnr << txMode << preamble);
  /* A packet is received from the PHY.
   * When we have handled this packet,
   * we handle any packet present in the
   * packet queue.
   */
  if(this->isMain==1) // Main radio
  {
	  if (this->IsSleep()) //sleep state
	  {
//		  std::cout<<"reject sleep"<<std::endl;
		  return;
	  }
  }

  WifiMacHeader hdr;
  packet->RemoveHeader (hdr);

  bool isPrevNavZero = IsNavZero ();
  NS_LOG_DEBUG ("duration/id=" << hdr.GetDuration ());
  NotifyNav (packet,hdr, txMode, preamble);
  if (hdr.IsRts ())
    {

      /* see section 9.2.5.7 802.11-1999
       * A STA that is addressed by an RTS frame shall transmit a CTS frame after a SIFS
       * period if the NAV at the STA receiving the RTS frame indicates that the medium is
       * idle. If the NAV at the STA receiving the RTS indicates the medium is not idle,
       * that STA shall not respond to the RTS frame.
       */
	  if (isPrevNavZero
			  && hdr.GetAddr1 () == m_self)
	  {

		  if (isMain == 2) //ulp
		  {
			  if (channelType == 1) //shared channel
			  {
				  if (this->GetDevice()->GetMainDevice()->GetMacLow()->GetLowSleep(2)>0)
				  {
					  std::cout<<"****** already wake up!! r"<<this->GetDevice()->GetMainDevice()->GetNode()->GetId()<<std::endl;
//					  return;
				  }
			  }
		  }

		  NS_LOG_DEBUG ("rx RTS from=" << hdr.GetAddr2 () << ", schedule CTS");
		  NS_ASSERT (m_sendCtsEvent.IsExpired ());
		  m_stationManager->ReportRxOk (hdr.GetAddr2 (), &hdr,
				  rxSnr, txMode);
		  m_sendCtsEvent = Simulator::Schedule (GetSifs (),
				  &MacLow::SendCtsAfterRts, this,
				  hdr.GetAddr2 (),
				  hdr.GetDuration (),
				  txMode,
				  rxSnr);


		  if(this->isMain == 1 ) // Main
		  {
			  MWLowTag tagCopy;
			  packet->PeekPacketTag (tagCopy);
			  //    		  std::cout<<"recv : "<<tagCopy.GetSimpleValue()<<" "<<this->m_self<<std::endl;
			  if (this->GetPacketId(2) == tagCopy.GetSimpleValue())
			  {
//				  std::cout<<"!!!!!!!!!!!!!!! main "<<tagCopy.GetSimpleValue()<<" "<<this->GetDevice()->GetNode()->GetId()<<std::endl;
//				  return;
			  }
			  this->SetPacketId(2,tagCopy.GetSimpleValue());

			  //        	if (this->GetDevice()->GetNode()->GetId()==0)
			  //        	{
			  //        		std::cout<<"get rts from "<<hdr.GetAddr2 ()<<std::endl;
			  //        	}
			  if(isMain == 1)
			  {

//				  if (this->GetDevice()->GetNode()->GetId()==16)
//					  if (Simulator::Now() > Seconds(18.0))
//					  {
//						  std::cout<<"* got rts main"<<tagCopy.GetSimpleValue()<<" "<<hdr.GetAddr2 ()<<std::endl;
//						  std::cout << "request list from main: ";
//						  for (std::list<uint32_t>::iterator it = this->GetDevice()->GetUlpDevice()->GetMacLow()->request.begin(); it != this->GetDevice()->GetUlpDevice()->GetMacLow()->request.end(); it++)
//						  {
//							  if (*it ==tagCopy.GetSimpleValue())
//							  {
//								  it=this->GetDevice()->GetUlpDevice()->GetMacLow()->request.erase(it);
//							  }
//
//						  }
//						    std::cout<<std::endl;
//					  }
				  if(channelType == 2){
					  for (std::list<uint32_t>::iterator it = this->GetDevice()->GetUlpDevice()->GetMacLow()->request.begin(); it != this->GetDevice()->GetUlpDevice()->GetMacLow()->request.end(); it++)
					  {
						  if (*it ==tagCopy.GetSimpleValue())
						  {
							  it=this->GetDevice()->GetUlpDevice()->GetMacLow()->request.erase(it);
						  }

					  }
				  }
			  }
		  }
		  else if(this->isMain == 2 ) // ULP
		  {

			  WifiMacTrailer fcs;
			  packet->RemoveTrailer (fcs);
			  m_rxCallback (packet, &hdr);

			  //////////////////
			  MWLowTag tagCopy;
			  packet->PeekPacketTag (tagCopy);
			  //    		  std::cout<<"recv : "<<tagCopy.GetSimpleValue()<<" "<<this->m_self<<std::endl;



			  if (this->GetPacketId(2) == tagCopy.GetSimpleValue())
			  {
//				  std::cout<<"!!!!!!!!!!!!!!! ulp "<<tagCopy.GetSimpleValue()<<" "<<this->GetDevice()->GetMainDevice()->GetNode()->GetId()<<std::endl;
//				  return;
			  }
			  this->SetPacketId(2,tagCopy.GetSimpleValue());
			  ///////////////////

/*
			  if(channelType == 2){
				  for (std::list<uint32_t>::iterator it = request.begin(); it != request.end(); it++)
				  {
					  if (*it == tagCopy.GetSimpleValue()){
						  std::cout<<"NO!!!!\n";
						  this->GetDevice()->GetMainDevice()->GetMacLow()->SetLowSleep(-1,-1,3);
						  it=request.erase(it);

					  }
				  }
				  request.push_front(tagCopy.GetSimpleValue());
			  }*/

//			  this->m_device->GetMainDevice()->GetMacLow()->SetLowSleep(2,2,0); //wake up req/ receiver //0625
		  }
	  }
	  else
	  {
		  NS_LOG_DEBUG ("rx RTS from=" << hdr.GetAddr2 () << ", cannot schedule CTS");
	  }
    }
  else if (hdr.IsCts ()
           && hdr.GetAddr1 () == m_self
           && m_ctsTimeoutEvent.IsRunning ()
           && m_currentPacket != 0)
    {
      NS_LOG_DEBUG ("receive cts from=" << m_currentHdr.GetAddr1 ());

      SnrTag tag;
      packet->RemovePacketTag (tag);
      m_stationManager->ReportRxOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                    rxSnr, txMode);
      m_stationManager->ReportRtsOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                     rxSnr, txMode, tag.Get ());

      m_ctsTimeoutEvent.Cancel ();
      NotifyCtsTimeoutResetNow ();
      m_listener->GotCts (rxSnr, txMode);


      if (this->isMain == 1) // Main Radio
      {
    	  NS_ASSERT (m_sendDataEvent.IsExpired ());
    	  m_sendDataEvent = Simulator::Schedule (GetSifs (),
    			  &MacLow::SendDataAfterCts, this,
    			  hdr.GetAddr1 (),
    			  hdr.GetDuration (),
    			  txMode);
//    	  bool properCts = true;
//    	  MWLowTag tagCopy;
//    	  packet->PeekPacketTag (tagCopy);
//    	  std::cout<<"cts recv :"<<tagCopy.GetSimpleValue()<<" "<<this->GetPacketId(1)<<" "<<this->m_self<<std::endl;
//    	  if (this->GetPacketId(1) != tagCopy.GetSimpleValue())
//    	  {
//    		  std::cout<<"####### main ######### "<<tagCopy.GetSimpleValue()<<" "<<this->GetDevice()->GetMainDevice()->GetNode()->GetId()<<std::endl;
////    		  properCts = false;
//    	  }

//    	  if(this->GetDevice()->GetNode()->GetId() == 12)
//    		  std::cout<<"[4] get Main CTS to "<<m_currentHdr.GetAddr1 ()<<" "<<Simulator::Now().GetSeconds()<<std::endl;
      }
      else // ULP Radio
      {
    	  m_stationManager->ReportDataOk(m_currentHdr.GetAddr1 (), &m_currentHdr,
    			  rxSnr, txMode, tag.Get ());

//    	  if(this->GetDevice()->GetMainDevice()->GetNode()->GetId()==12)
//    		  std::cout<<"[2] recv ULP CTS to "<<m_currentHdr.GetAddr1 ()<<" "<<Simulator::Now().GetSeconds()<<std::endl;

    	  UlpData();
    	  WifiMacTrailer fcs;
    	  packet->RemoveTrailer (fcs);

    	  MWLowTag tagCopy;
    	  packet->PeekPacketTag (tagCopy);
//    	  std::cout<<"cts recv :"<<tagCopy.GetSimpleValue()<<" "<<this->m_self<<std::endl;

    	  Ptr<WifiNetDevice> temp_dev=
    			  DynamicCast<WifiNetDevice>(this->GetDevice()->GetMainDevice()->GetDeviceContainer()->Get(tagCopy.GetSimpleValue()));
    	  temp_dev->GetMacLow()->SetLowSleep(2,2,this->GetDevice()->GetMainDevice()->GetNode()->GetId());
    	  this->GetDevice()->GetMainDevice()->GetMacLow()->SetLowSleep(2,1,-1);

    	  this->m_device->SendAfterWakeupSignal(false);
      }
    }
  else if (hdr.IsAck ()
           && hdr.GetAddr1 () == m_self
           && (m_normalAckTimeoutEvent.IsRunning ()
               || m_fastAckTimeoutEvent.IsRunning ()
               || m_superFastAckTimeoutEvent.IsRunning ())
           && m_txParams.MustWaitAck ())
    {
      NS_LOG_DEBUG ("receive ack from=" << m_currentHdr.GetAddr1 ());
      SnrTag tag;
      packet->RemovePacketTag (tag);
      m_stationManager->ReportRxOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                    rxSnr, txMode);
      m_stationManager->ReportDataOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
                                      rxSnr, txMode, tag.Get ());
      bool gotAck = false;
      if (m_txParams.MustWaitNormalAck ()
          && m_normalAckTimeoutEvent.IsRunning ())
        {
          m_normalAckTimeoutEvent.Cancel ();
          NotifyAckTimeoutResetNow ();
          gotAck = true;
        }
      if (m_txParams.MustWaitFastAck ()
          && m_fastAckTimeoutEvent.IsRunning ())
        {
          m_fastAckTimeoutEvent.Cancel ();
          NotifyAckTimeoutResetNow ();
          gotAck = true;
        }
      if (gotAck)
        {
          m_listener->GotAck (rxSnr, txMode);
          if(this->isMain == 1) // Main radio //
          {

//        	  if(this->GetDevice()->GetNode()->GetId() == 12)
//        		  std::cout<<"[5] get Main ACK to "<<m_currentHdr.GetAddr1 ()<<" \t"<<Simulator::Now().GetSeconds()<<std::endl;
        	  MWLowTag tagCopy;
        	  packet->PeekPacketTag (tagCopy);
//        	  std::cout<<"ack recv :"<<tagCopy.GetSimpleValue()<<" "<<this->m_self<<std::endl;

        	  Ptr<WifiNetDevice> temp_dev=
        			  DynamicCast<WifiNetDevice>(this->GetDevice()->GetDeviceContainer()->Get(tagCopy.GetSimpleValue()));
        	  temp_dev->GetMacLow()->SetLowSleep(1,2,this->GetDevice()->GetNode()->GetId());

        	  this->SetLowSleep(1,1,-1); // sleep request/ sender/ 0
//

        	  this->m_device->SendDoneAck();
          }
        }
      if (m_txParams.HasNextPacket ())
        {
          m_waitSifsEvent = Simulator::Schedule (GetSifs (),
                                                 &MacLow::WaitSifsAfterEndTx, this);
        }
    }
  else if (hdr.IsBlockAck () && hdr.GetAddr1 () == m_self
           && (m_txParams.MustWaitBasicBlockAck () || m_txParams.MustWaitCompressedBlockAck ())
           && m_blockAckTimeoutEvent.IsRunning ())
    {
      NS_LOG_DEBUG ("got block ack from " << hdr.GetAddr2 ());
      CtrlBAckResponseHeader blockAck;
      packet->RemoveHeader (blockAck);
      m_blockAckTimeoutEvent.Cancel ();
      m_listener->GotBlockAck (&blockAck, hdr.GetAddr2 ());
    }
  else if (hdr.IsBlockAckReq () && hdr.GetAddr1 () == m_self)
    {
      CtrlBAckRequestHeader blockAckReq;
      packet->RemoveHeader (blockAckReq);
      if (!blockAckReq.IsMultiTid ())
        {
          uint8_t tid = blockAckReq.GetTidInfo ();
          AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), tid));
          if (it != m_bAckAgreements.end ())
            {
              //Update block ack cache
              BlockAckCachesI i = m_bAckCaches.find (std::make_pair (hdr.GetAddr2 (), tid));
              NS_ASSERT (i != m_bAckCaches.end ());
              (*i).second.UpdateWithBlockAckReq (blockAckReq.GetStartingSequence ());

              NS_ASSERT (m_sendAckEvent.IsExpired ());
              /* See section 11.5.3 in IEEE802.11 for mean of this timer */
              ResetBlockAckInactivityTimerIfNeeded (it->second.first);
              if ((*it).second.first.IsImmediateBlockAck ())
                {
                  NS_LOG_DEBUG ("rx blockAckRequest/sendImmediateBlockAck from=" << hdr.GetAddr2 ());
                  m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                                        &MacLow::SendBlockAckAfterBlockAckRequest, this,
                                                        blockAckReq,
                                                        hdr.GetAddr2 (),
                                                        hdr.GetDuration (),
                                                        txMode);
                }
              else
                {
                  NS_FATAL_ERROR ("Delayed block ack not supported.");
                }
            }
          else
            {
              NS_LOG_DEBUG ("There's not a valid agreement for this block ack request.");
            }
        }
      else
        {
          NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
        }
    }
  else if (hdr.IsCtl ())
    {
      NS_LOG_DEBUG ("rx drop " << hdr.GetTypeString ());
    }
  else if (hdr.GetAddr1 () == m_self)
    {
      m_stationManager->ReportRxOk (hdr.GetAddr2 (), &hdr,
                                    rxSnr, txMode);

      if (hdr.IsQosData () && StoreMpduIfNeeded (packet, hdr))
        {
          /* From section 9.10.4 in IEEE802.11:
             Upon the receipt of a QoS data frame from the originator for which
             the Block Ack agreement exists, the recipient shall buffer the MSDU
             regardless of the value of the Ack Policy subfield within the
             QoS Control field of the QoS data frame. */
          if (hdr.IsQosAck ())
            {
              AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
              RxCompleteBufferedPacketsWithSmallerSequence (it->second.first.GetStartingSequence (),
                                                            hdr.GetAddr2 (), hdr.GetQosTid ());
              RxCompleteBufferedPacketsUntilFirstLost (hdr.GetAddr2 (), hdr.GetQosTid ());
              NS_ASSERT (m_sendAckEvent.IsExpired ());
              m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                                    &MacLow::SendAckAfterData, this,
                                                    hdr.GetAddr2 (),
                                                    hdr.GetDuration (),
                                                    txMode,
                                                    rxSnr);
            }
          else if (hdr.IsQosBlockAck ())
            {
              AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
              /* See section 11.5.3 in IEEE802.11 for mean of this timer */
              ResetBlockAckInactivityTimerIfNeeded (it->second.first);
            }
          return;
        }
      else if (hdr.IsQosData () && hdr.IsQosBlockAck ())
        {
          /* This happens if a packet with ack policy Block Ack is received and a block ack
             agreement for that packet doesn't exist.

             From section 11.5.3 in IEEE802.11e:
             When a recipient does not have an active Block ack for a TID, but receives
             data MPDUs with the Ack Policy subfield set to Block Ack, it shall discard
             them and shall send a DELBA frame using the normal access
             mechanisms. */
          AcIndex ac = QosUtilsMapTidToAc (hdr.GetQosTid ());
          m_edcaListeners[ac]->BlockAckInactivityTimeout (hdr.GetAddr2 (), hdr.GetQosTid ());
          return;
        }
      else if (hdr.IsQosData () && hdr.IsQosNoAck ())
        {
          NS_LOG_DEBUG ("rx unicast/noAck from=" << hdr.GetAddr2 ());
        }
      else if (hdr.IsData () || hdr.IsMgt ())
        {
          NS_LOG_DEBUG ("rx unicast/sendAck from=" << hdr.GetAddr2 ());
          NS_ASSERT (m_sendAckEvent.IsExpired ());
          m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                                &MacLow::SendAckAfterData, this,
                                                hdr.GetAddr2 (),
                                                hdr.GetDuration (),
                                                txMode,
                                                rxSnr);
        }
      goto rxPacket;
    }
  else if (hdr.GetAddr1 ().IsGroup ())
    {
      if (hdr.IsData () || hdr.IsMgt ())
        {
          NS_LOG_DEBUG ("rx group from=" << hdr.GetAddr2 ());
          goto rxPacket;
        }
      else
        {
          // DROP
        }
    }
  else if (m_promisc)
    {
      NS_ASSERT (hdr.GetAddr1 () != m_self);
      if (hdr.IsData ())
        {
          goto rxPacket;
        }
    }
  else
    {
      //NS_LOG_DEBUG_VERBOSE ("rx not-for-me from %d", GetSource (packet));
    }
  return;
rxPacket:
  WifiMacTrailer fcs;
  packet->RemoveTrailer (fcs);
  m_rxCallback (packet, &hdr);
  return;
}
///////////////////////////////////////
// MW-MAC  END
///////////////////////////////////////
void
MacLow::SetPhy (Ptr<WifiPhy> phy)
{
  m_phy = phy;
  m_phy->SetReceiveOkCallback (MakeCallback (&MacLow::ReceiveOk, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&MacLow::ReceiveError, this));
  SetupPhyMacLowListener (phy);
}
void
MacLow::SetWifiRemoteStationManager (Ptr<WifiRemoteStationManager> manager)
{
  m_stationManager = manager;
}

void
MacLow::SetAddress (Mac48Address ad)
{
  m_self = ad;
}
void
MacLow::SetAckTimeout (Time ackTimeout)
{
  m_ackTimeout = ackTimeout;
}
void
MacLow::SetBasicBlockAckTimeout (Time blockAckTimeout)
{
  m_basicBlockAckTimeout = blockAckTimeout;
}
void
MacLow::SetCompressedBlockAckTimeout (Time blockAckTimeout)
{
  m_compressedBlockAckTimeout = blockAckTimeout;
}
void
MacLow::SetCtsToSelfSupported (bool enable)
{
  m_ctsToSelfSupported = enable;
}
bool
MacLow::GetCtsToSelfSupported () const
{
  return m_ctsToSelfSupported;
}
void
MacLow::SetCtsTimeout (Time ctsTimeout)
{
  m_ctsTimeout = ctsTimeout;
}
void
MacLow::SetSifs (Time sifs)
{
  m_sifs = sifs;
}
void
MacLow::SetSlotTime (Time slotTime)
{
  m_slotTime = slotTime;
}
void
MacLow::SetPifs (Time pifs)
{
  m_pifs = pifs;
}
void
MacLow::SetRifs (Time rifs)
{
  m_rifs = rifs;
}
void
MacLow::SetBssid (Mac48Address bssid)
{
  m_bssid = bssid;
}
void
MacLow::SetPromisc (void)
{
  m_promisc = true;
}
Mac48Address
MacLow::GetAddress (void) const
{
  return m_self;
}
Time
MacLow::GetAckTimeout (void) const
{
  return m_ackTimeout;
}
Time
MacLow::GetBasicBlockAckTimeout () const
{
  return m_basicBlockAckTimeout;
}
Time
MacLow::GetCompressedBlockAckTimeout () const
{
  return m_compressedBlockAckTimeout;
}
Time
MacLow::GetCtsTimeout (void) const
{
  return m_ctsTimeout;
}
Time
MacLow::GetSifs (void) const
{
  return m_sifs;
}
Time
MacLow::GetRifs (void) const
{
  return m_rifs;
}
Time
MacLow::GetSlotTime (void) const
{
  return m_slotTime;
}
Time
MacLow::GetPifs (void) const
{
  return m_pifs;
}
Mac48Address
MacLow::GetBssid (void) const
{
  return m_bssid;
}
bool
MacLow::IsPromisc (void) const
{
  return m_promisc;
}

void
MacLow::SetRxCallback (Callback<void,Ptr<Packet>,const WifiMacHeader *> callback)
{
  m_rxCallback = callback;
}
void
MacLow::RegisterDcfListener (MacLowDcfListener *listener)
{
  m_dcfListeners.push_back (listener);
}


void
MacLow::StartTransmission (Ptr<const Packet> packet,
                           const WifiMacHeader* hdr,
                           MacLowTransmissionParameters params,
                           MacLowTransmissionListener *listener)
{
  NS_LOG_FUNCTION (this << packet << hdr << params << listener);
  /* m_currentPacket is not NULL because someone started
   * a transmission and was interrupted before one of:
   *   - ctsTimeout
   *   - sendDataAfterCTS
   * expired. This means that one of these timers is still
   * running. They are all cancelled below anyway by the
   * call to CancelAllEvents (because of at least one
   * of these two timer) which will trigger a call to the
   * previous listener's cancel method.
   *
   * This typically happens because the high-priority
   * QapScheduler has taken access to the channel from
   * one of the Edca of the QAP.
   */
  m_currentPacket = packet->Copy ();
  m_currentHdr = *hdr;
  CancelAllEvents ();
  m_listener = listener;
  m_txParams = params;

  //kdw
//  this->sendPacketId = packet->GetUid();

  //NS_ASSERT (m_phy->IsStateIdle ());

  NS_LOG_DEBUG ("startTx size=" << GetSize (m_currentPacket, &m_currentHdr) <<
                ", to=" << m_currentHdr.GetAddr1 () << ", listener=" << m_listener);

  if (m_txParams.MustSendRts ())
    {
      SendRtsForPacket ();
    }
  else
    {
     if (NeedCtsToSelf() && m_ctsToSelfSupported)
        {
          SendCtsToSelf();
        }
      else
        {
          SendDataPacket ();
       }
    }

  /* When this method completes, we have taken ownership of the medium. */
  NS_ASSERT (m_phy->IsStateTx ());
}
bool
MacLow::NeedCtsToSelf (void)
{
  WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
  return m_stationManager->NeedCtsToSelf (dataTxVector);
}
void
MacLow::ReceiveError (Ptr<const Packet> packet, double rxSnr)
{
  NS_LOG_FUNCTION (this << packet << rxSnr);
  NS_LOG_DEBUG ("rx failed ");
  if (m_txParams.MustWaitFastAck ())
    {
      NS_ASSERT (m_fastAckFailedTimeoutEvent.IsExpired ());
      m_fastAckFailedTimeoutEvent = Simulator::Schedule (GetSifs (),
                                                         &MacLow::FastAckFailedTimeout, this);
    }
  return;
}

void
MacLow::NotifySwitchingStartNow (Time duration)
{
  NS_LOG_DEBUG ("switching channel. Cancelling MAC pending events");
  m_stationManager->Reset ();
  CancelAllEvents ();
  if (m_navCounterResetCtsMissed.IsRunning ())
    {
      m_navCounterResetCtsMissed.Cancel ();
    }
  m_lastNavStart = Simulator::Now ();
  m_lastNavDuration = Seconds (0);
  m_currentPacket = 0;
  m_listener = 0;
}



uint32_t
MacLow::GetAckSize (void) const
{
  WifiMacHeader ack;
  ack.SetType (WIFI_MAC_CTL_ACK);
  return ack.GetSize () + 4;
}
uint32_t
MacLow::GetBlockAckSize (enum BlockAckType type) const
{
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_CTL_BACKRESP);
  CtrlBAckResponseHeader blockAck;
  if (type == BASIC_BLOCK_ACK)
    {
      blockAck.SetType (BASIC_BLOCK_ACK);
    }
  else if (type == COMPRESSED_BLOCK_ACK)
    {
      blockAck.SetType (COMPRESSED_BLOCK_ACK);
    }
  else if (type == MULTI_TID_BLOCK_ACK)
    {
      //Not implemented
      NS_ASSERT (false);
    }
  return hdr.GetSize () + blockAck.GetSerializedSize () + 4;
}
uint32_t
MacLow::GetRtsSize (void) const
{
  WifiMacHeader rts;
  rts.SetType (WIFI_MAC_CTL_RTS);
  return rts.GetSize () + 4;
}
Time
MacLow::GetAckDuration (Mac48Address to, WifiTxVector dataTxVector) const
{
  WifiTxVector ackTxVector = GetAckTxVectorForData (to, dataTxVector.GetMode());
  return GetAckDuration (ackTxVector);
}
Time
MacLow::GetAckDuration (WifiTxVector ackTxVector) const
{
  WifiPreamble preamble;
  if (ackTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;
  return m_phy->CalculateTxDuration (GetAckSize (), ackTxVector, preamble);
}
Time
MacLow::GetBlockAckDuration (Mac48Address to, WifiTxVector blockAckReqTxVector, enum BlockAckType type) const
{
  /*
   * For immediate BlockAck we should transmit the frame with the same WifiMode
   * as the BlockAckReq.
   *
   * from section 9.6 in IEEE802.11e:
   * The BlockAck control frame shall be sent at the same rate and modulation class as
   * the BlockAckReq frame if it is sent in response to a BlockAckReq frame.
   */
  WifiPreamble preamble;
  if (blockAckReqTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;
  return m_phy->CalculateTxDuration (GetBlockAckSize (type), blockAckReqTxVector, preamble);
}
Time
MacLow::GetCtsDuration (Mac48Address to, WifiTxVector rtsTxVector) const
{
  WifiTxVector ctsTxVector = GetCtsTxVectorForRts (to, rtsTxVector.GetMode());
  return GetCtsDuration (ctsTxVector);
}

Time
MacLow::GetCtsDuration (WifiTxVector ctsTxVector) const
{
  WifiPreamble preamble;
  if (ctsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;
  return m_phy->CalculateTxDuration (GetCtsSize (), ctsTxVector, preamble);
}
uint32_t
MacLow::GetCtsSize (void) const
{
  WifiMacHeader cts;
  cts.SetType (WIFI_MAC_CTL_CTS);
  return cts.GetSize () + 4;
}
uint32_t
MacLow::GetSize (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
{
  WifiMacTrailer fcs;
  return packet->GetSize () + hdr->GetSize () + fcs.GetSerializedSize ();
}

WifiTxVector
MacLow::GetCtsToSelfTxVector (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
{
  return m_stationManager->GetCtsToSelfTxVector (hdr, packet);
}

WifiTxVector
MacLow::GetRtsTxVector (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
{
  Mac48Address to = hdr->GetAddr1 ();
  return m_stationManager->GetRtsTxVector (to, hdr, packet);
}
WifiTxVector
MacLow::GetDataTxVector (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
{
  Mac48Address to = hdr->GetAddr1 ();
  WifiMacTrailer fcs;
  uint32_t size =  packet->GetSize ()+ hdr->GetSize () + fcs.GetSerializedSize ();
  //size is not used in anything!! will not worry about aggregation
  return m_stationManager->GetDataTxVector (to, hdr, packet, size);
}
WifiTxVector
MacLow::GetCtsTxVector (Mac48Address to, WifiMode rtsTxMode) const
{
  return m_stationManager->GetCtsTxVector (to, rtsTxMode);
}
WifiTxVector
MacLow::GetAckTxVector (Mac48Address to, WifiMode dataTxMode) const
{
  return m_stationManager->GetAckTxVector (to, dataTxMode);
}
WifiTxVector
MacLow::GetBlockAckTxVector (Mac48Address to, WifiMode dataTxMode) const
{
  return m_stationManager->GetBlockAckTxVector (to, dataTxMode);
}

WifiTxVector
MacLow::GetCtsTxVectorForRts (Mac48Address to, WifiMode rtsTxMode) const
{
  return GetCtsTxVector (to, rtsTxMode);
}
WifiTxVector
MacLow::GetAckTxVectorForData (Mac48Address to, WifiMode dataTxMode) const
{
  return GetAckTxVector (to, dataTxMode);
}


Time
MacLow::CalculateOverallTxTime (Ptr<const Packet> packet,
                                const WifiMacHeader* hdr,
                                const MacLowTransmissionParameters& params) const
{
  WifiPreamble preamble;
  Time txTime = Seconds (0);
  if (params.MustSendRts ())
    {
      WifiTxVector rtsTxVector = GetRtsTxVector (packet, hdr);
      //standard says RTS packets can have GF format sec 9.6.0e.1 page 110 bullet b 2
      if (m_phy->GetGreenfield () && m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
        {
          preamble = WIFI_PREAMBLE_HT_GF;
        }
      else if (rtsTxVector.GetMode ().GetModulationClass () == WIFI_MOD_CLASS_HT)
        {
          preamble = WIFI_PREAMBLE_HT_MF;
        }
      else
        {
          preamble = WIFI_PREAMBLE_LONG;
        }
      txTime += m_phy->CalculateTxDuration (GetRtsSize (), rtsTxVector, preamble);
      txTime += GetCtsDuration (hdr->GetAddr1 (), rtsTxVector);
      txTime += Time (GetSifs () * 2);
    }
  WifiTxVector dataTxVector = GetDataTxVector (packet, hdr);
  //standard says RTS packets can have GF format sec 9.6.0e.1 page 110 bullet b 2
  if ( m_phy->GetGreenfield()&& m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
    preamble= WIFI_PREAMBLE_HT_GF;
  else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;
  uint32_t dataSize = GetSize (packet, hdr);
  txTime += m_phy->CalculateTxDuration (dataSize, dataTxVector, preamble);
  if (params.MustWaitAck ())
    {
      txTime += GetSifs ();
      txTime += GetAckDuration (hdr->GetAddr1 (), dataTxVector);
    }
  return txTime;
}

Time
MacLow::CalculateTransmissionTime (Ptr<const Packet> packet,
                                   const WifiMacHeader* hdr,
                                   const MacLowTransmissionParameters& params) const
{
  Time txTime = CalculateOverallTxTime (packet, hdr, params);
  if (params.HasNextPacket ())
    {
      WifiTxVector dataTxVector = GetDataTxVector (packet, hdr);
      WifiPreamble preamble;
        //standard says RTS packets can have GF format sec 9.6.0e.1 page 110 bullet b 2
      if ( m_phy->GetGreenfield()&& m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
         preamble= WIFI_PREAMBLE_HT_GF;
      else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
        preamble= WIFI_PREAMBLE_HT_MF;
      else
        preamble=WIFI_PREAMBLE_LONG;
      txTime += GetSifs ();
      txTime += m_phy->CalculateTxDuration (params.GetNextPacketSize (), dataTxVector, preamble);
    }
  return txTime;
}

void
MacLow::NotifyNav (Ptr<const Packet> packet,const WifiMacHeader &hdr, WifiMode txMode, WifiPreamble preamble)
{
  NS_ASSERT (m_lastNavStart <= Simulator::Now ());

  Time duration;
  if(this->channelType == 1)
  {
	  if(this->isMain == 2)
	  {
		  duration = hdr.GetDuration () + hdr.GetDuration ();
	  }
	  else
	  {
		  duration = hdr.GetDuration ();
	  }
  }
  else
  {
	  duration = hdr.GetDuration ();
  }
  if (hdr.IsCfpoll ()
      && hdr.GetAddr2 () == m_bssid)
    {
      // see section 9.3.2.2 802.11-1999
      DoNavResetNow (duration);
      return;
    }
  /// \todo We should also handle CF_END specially here
  /// but we don't for now because we do not generate them.
  else if (hdr.GetAddr1 () != m_self)
    {
      // see section 9.2.5.4 802.11-1999
      bool navUpdated = DoNavStartNow (duration);
      if (hdr.IsRts () && navUpdated)
        {
          /**
           * A STA that used information from an RTS frame as the most recent basis to update its NAV setting
           * is permitted to reset its NAV if no PHY-RXSTART.indication is detected from the PHY during a
           * period with a duration of (2 * aSIFSTime) + (CTS_Time) + (2 * aSlotTime) starting at the
           * PHY-RXEND.indication corresponding to the detection of the RTS frame. The “CTS_Time” shall
           * be calculated using the length of the CTS frame and the data rate at which the RTS frame
           * used for the most recent NAV update was received.
           */
          WifiMacHeader cts;
          cts.SetType (WIFI_MAC_CTL_CTS);
          WifiTxVector txVector=GetRtsTxVector (packet, &hdr);
          Time navCounterResetCtsMissedDelay =
            m_phy->CalculateTxDuration (cts.GetSerializedSize (), txVector, preamble) +
            Time (2 * GetSifs ()) + Time (2 * GetSlotTime ());
          m_navCounterResetCtsMissed = Simulator::Schedule (navCounterResetCtsMissedDelay,
                                                            &MacLow::NavCounterResetCtsMissed, this,
                                                            Simulator::Now ());
        }
    }
}

void
MacLow::NavCounterResetCtsMissed (Time rtsEndRxTime)
{
  if (m_phy->GetLastRxStartTime () < rtsEndRxTime)
    {
      DoNavResetNow (Seconds (0.0));
    }
}

void
MacLow::DoNavResetNow (Time duration)
{
  for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->NavReset (duration);
    }
  m_lastNavStart = Simulator::Now ();
  m_lastNavStart = duration;
}
bool
MacLow::DoNavStartNow (Time duration)
{
  for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->NavStart (duration);
    }
  Time newNavEnd = Simulator::Now () + duration;
  Time oldNavEnd = m_lastNavStart + m_lastNavDuration;
  if (newNavEnd > oldNavEnd)
    {
      m_lastNavStart = Simulator::Now ();
      m_lastNavDuration = duration;
      return true;
    }
  return false;
}
void
MacLow::NotifyAckTimeoutStartNow (Time duration)
{
  for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->AckTimeoutStart (duration);
    }
}
void
MacLow::NotifyAckTimeoutResetNow ()
{
  for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->AckTimeoutReset ();
    }
}
void
MacLow::NotifyCtsTimeoutStartNow (Time duration)
{
  for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->CtsTimeoutStart (duration);
    }
}
void
MacLow::NotifyCtsTimeoutResetNow ()
{
  for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->CtsTimeoutReset ();
    }
}

void
MacLow::ForwardDown (Ptr<const Packet> packet, const WifiMacHeader* hdr,
                     WifiTxVector txVector, WifiPreamble preamble)
{
  NS_LOG_FUNCTION (this << packet << hdr << txVector);
  NS_LOG_DEBUG ("send " << hdr->GetTypeString () <<
                ", to=" << hdr->GetAddr1 () <<
                ", size=" << packet->GetSize () <<
                ", mode=" << txVector.GetMode() <<
                ", duration=" << hdr->GetDuration () <<
                ", seq=0x" << std::hex << m_currentHdr.GetSequenceControl () << std::dec);
  m_phy->SendPacket (packet, txVector.GetMode(), preamble, txVector);
}

void
MacLow::CtsTimeout (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("cts timeout");
  /// \todo should check that there was no rx start before now.
  /// we should restart a new cts timeout now until the expected
  /// end of rx if there was a rx start before now.
  m_stationManager->ReportRtsFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
  m_currentPacket = 0;
  MacLowTransmissionListener *listener = m_listener;
  m_listener = 0;
  listener->MissedCts ();
}
void
MacLow::NormalAckTimeout (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("normal ack timeout");
  /// \todo should check that there was no rx start before now.
  /// we should restart a new ack timeout now until the expected
  /// end of rx if there was a rx start before now.
  m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
  MacLowTransmissionListener *listener = m_listener;
  m_listener = 0;
  listener->MissedAck ();
}
void
MacLow::FastAckTimeout (void)
{
  NS_LOG_FUNCTION (this);
  m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
  MacLowTransmissionListener *listener = m_listener;
  m_listener = 0;
  if (m_phy->IsStateIdle ())
    {
      NS_LOG_DEBUG ("fast Ack idle missed");
      listener->MissedAck ();
    }
  else
    {
      NS_LOG_DEBUG ("fast Ack ok");
    }
}
void
MacLow::BlockAckTimeout (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("block ack timeout");

  m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
  MacLowTransmissionListener *listener = m_listener;
  m_listener = 0;
  listener->MissedBlockAck ();
}
void
MacLow::SuperFastAckTimeout ()
{
  NS_LOG_FUNCTION (this);
  m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
  MacLowTransmissionListener *listener = m_listener;
  m_listener = 0;
  if (m_phy->IsStateIdle ())
    {
      NS_LOG_DEBUG ("super fast Ack failed");
      listener->MissedAck ();
    }
  else
    {
      NS_LOG_DEBUG ("super fast Ack ok");
      listener->GotAck (0.0, WifiMode ());
    }
}

void
MacLow::SendRtsForPacket (void)
{
  NS_LOG_FUNCTION (this);
  /* send an RTS for this packet. */
  WifiMacHeader rts;
  rts.SetType (WIFI_MAC_CTL_RTS);
  rts.SetDsNotFrom ();
  rts.SetDsNotTo ();
  rts.SetNoRetry ();
  rts.SetNoMoreFragments ();
  rts.SetAddr1 (m_currentHdr.GetAddr1 ());
  rts.SetAddr2 (m_self);
  WifiTxVector rtsTxVector = GetRtsTxVector (m_currentPacket, &m_currentHdr);
  Time duration = Seconds (0);


  mainDest = m_currentHdr.GetAddr1 ();	//kdw

  WifiPreamble preamble;
  //standard says RTS packets can have GF format sec 9.6.0e.1 page 110 bullet b 2
  if ( m_phy->GetGreenfield()&& m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
    preamble= WIFI_PREAMBLE_HT_GF;
  else if (rtsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;

  if (m_txParams.HasDurationId ())
    {
      duration += m_txParams.GetDurationId ();
    }
  else
    {
      WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
      duration += GetSifs ();
      duration += GetCtsDuration (m_currentHdr.GetAddr1 (), rtsTxVector);
      duration += GetSifs ();
      duration += m_phy->CalculateTxDuration (GetSize (m_currentPacket, &m_currentHdr),
                                              dataTxVector, preamble);
      duration += GetSifs ();
      duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
    }
  rts.SetDuration (duration);

  Time txDuration = m_phy->CalculateTxDuration (GetRtsSize (), rtsTxVector, preamble);
  Time timerDelay = txDuration + GetCtsTimeout ();//kdw cts timeout

  NS_ASSERT (m_ctsTimeoutEvent.IsExpired ());
  NotifyCtsTimeoutStartNow (timerDelay);
  m_ctsTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::CtsTimeout, this);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (rts);
  WifiMacTrailer fcs;
  packet->AddTrailer (fcs);

  if(isMain == 2) //ULP radio
  {
	  packet->RemoveAllPacketTags();
	  MWLowTag rtsTag;
	  rtsTag.SetSimpleValue(this->GetDevice()->GetMainDevice()->GetNode()->GetId());
	  packet->AddPacketTag(rtsTag);
//	  std::cout<<"send :"<<sendPacketId<<" "<<rtsTag.GetSimpleValue()<<" "<<this->m_self<<std::endl;

//	  if(this->GetDevice()->GetMainDevice()->GetNode()->GetId() == 12)
//		  std::cout<<"[1] send ULP RTS to "<<rts.GetAddr1()<<" "<<Simulator::Now().GetSeconds()<<std::endl;
  }

  else
  {
//	  if(this->GetDevice()->GetNode()->GetId() == 12)
//		  std::cout<<"[3] send Main RTS to "<<rts.GetAddr1()<<" "<<Simulator::Now().GetSeconds()<<std::endl;
  }



  ForwardDown (packet, &rts, rtsTxVector,preamble);
}

void
MacLow::StartDataTxTimers (WifiTxVector dataTxVector)
{
  WifiPreamble preamble;
 
  //Since it is data then it can have format = GF
  if (m_phy->GetGreenfield() && m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
    preamble= WIFI_PREAMBLE_HT_GF;
  else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;
 
  Time txDuration = m_phy->CalculateTxDuration (GetSize (m_currentPacket, &m_currentHdr), dataTxVector, preamble);
  if (m_txParams.MustWaitNormalAck ())
    {
      Time timerDelay = txDuration + GetAckTimeout ();
      NS_ASSERT (m_normalAckTimeoutEvent.IsExpired ());
      NotifyAckTimeoutStartNow (timerDelay);
      m_normalAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::NormalAckTimeout, this);
    }
  else if (m_txParams.MustWaitFastAck ())
    {
      Time timerDelay = txDuration + GetPifs ();
      NS_ASSERT (m_fastAckTimeoutEvent.IsExpired ());
      NotifyAckTimeoutStartNow (timerDelay);
      m_fastAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::FastAckTimeout, this);
    }
  else if (m_txParams.MustWaitSuperFastAck ())
    {
      Time timerDelay = txDuration + GetPifs ();
      NS_ASSERT (m_superFastAckTimeoutEvent.IsExpired ());
      NotifyAckTimeoutStartNow (timerDelay);
      m_superFastAckTimeoutEvent = Simulator::Schedule (timerDelay,
                                                        &MacLow::SuperFastAckTimeout, this);
    }
  else if (m_txParams.MustWaitBasicBlockAck ())
    {
      Time timerDelay = txDuration + GetBasicBlockAckTimeout ();
      NS_ASSERT (m_blockAckTimeoutEvent.IsExpired ());
      m_blockAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::BlockAckTimeout, this);
    }
  else if (m_txParams.MustWaitCompressedBlockAck ())
    {
      Time timerDelay = txDuration + GetCompressedBlockAckTimeout ();
      NS_ASSERT (m_blockAckTimeoutEvent.IsExpired ());
      m_blockAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::BlockAckTimeout, this);
    }
  else if (m_txParams.HasNextPacket ())
    {
     if (m_stationManager->HasHtSupported())
       {
          Time delay = txDuration + GetRifs ();
          NS_ASSERT (m_waitRifsEvent.IsExpired ());
          m_waitRifsEvent = Simulator::Schedule (delay, &MacLow::WaitSifsAfterEndTx, this); 
       }
     else
       {
          Time delay = txDuration + GetSifs ();
          NS_ASSERT (m_waitSifsEvent.IsExpired ());
          m_waitSifsEvent = Simulator::Schedule (delay, &MacLow::WaitSifsAfterEndTx, this);
       }
    }
  else
    {
      // since we do not expect any timer to be triggered.
      Simulator::Schedule(txDuration, &MacLow::EndTxNoAck, this);
    }
}

void
MacLow::SendDataPacket (void)
{
  NS_LOG_FUNCTION (this);
  /* send this packet directly. No RTS is needed. */
  WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
  WifiPreamble preamble;
          
  if (m_phy->GetGreenfield() && m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
     //In the future has to make sure that receiver has greenfield enabled
     preamble= WIFI_PREAMBLE_HT_GF;
  else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
     preamble=WIFI_PREAMBLE_LONG;
  
  StartDataTxTimers (dataTxVector);

  Time duration = Seconds (0.0);
  if (m_txParams.HasDurationId ())
    {
      duration += m_txParams.GetDurationId ();
    }
  else
    {
      if (m_txParams.MustWaitBasicBlockAck ())
        {
          duration += GetSifs ();
          duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, BASIC_BLOCK_ACK);
        }
      else if (m_txParams.MustWaitCompressedBlockAck ())
        {
          duration += GetSifs ();
          duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, COMPRESSED_BLOCK_ACK);
        }
      else if (m_txParams.MustWaitAck ())
        {
          duration += GetSifs ();
          duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
        }
      if (m_txParams.HasNextPacket ())
        {
          duration += GetSifs ();
          duration += m_phy->CalculateTxDuration (m_txParams.GetNextPacketSize (),
                                                  dataTxVector, preamble);
          if (m_txParams.MustWaitAck ())
            {
              duration += GetSifs ();
              duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
            }
        }
    }
  m_currentHdr.SetDuration (duration);

  m_currentPacket->AddHeader (m_currentHdr);
  WifiMacTrailer fcs;
  m_currentPacket->AddTrailer (fcs);

  ForwardDown (m_currentPacket, &m_currentHdr, dataTxVector,preamble);
  m_currentPacket = 0;
}

bool
MacLow::IsNavZero (void) const
{
  if (m_lastNavStart + m_lastNavDuration < Simulator::Now ())
    {
      return true;
    }
  else
    {
      return false;
    }
}
void
MacLow::SendCtsToSelf (void)
{
  WifiMacHeader cts;
  cts.SetType (WIFI_MAC_CTL_CTS);
  cts.SetDsNotFrom ();
  cts.SetDsNotTo ();
  cts.SetNoMoreFragments ();
  cts.SetNoRetry ();
  cts.SetAddr1 (m_self);
 
  WifiTxVector ctsTxVector = GetCtsToSelfTxVector (m_currentPacket, &m_currentHdr);

  WifiPreamble preamble;
  if (ctsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;
  
  Time duration = Seconds (0);

  if (m_txParams.HasDurationId ())
    {
      duration += m_txParams.GetDurationId ();
    }
  else
    {
      WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
      duration += GetSifs ();
      duration += m_phy->CalculateTxDuration (GetSize (m_currentPacket,&m_currentHdr),
                                              dataTxVector, preamble);
      if (m_txParams.MustWaitBasicBlockAck ())
        {
          
          duration += GetSifs ();
          duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, BASIC_BLOCK_ACK);
        }
      else if (m_txParams.MustWaitCompressedBlockAck ())
        {
          duration += GetSifs ();
          duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, COMPRESSED_BLOCK_ACK);
        }
      else if (m_txParams.MustWaitAck ())
        {
          duration += GetSifs ();
          duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
        }
      if (m_txParams.HasNextPacket ())
        {
          duration += GetSifs ();
          duration += m_phy->CalculateTxDuration (m_txParams.GetNextPacketSize (),
                                                  dataTxVector, preamble);
          if (m_txParams.MustWaitCompressedBlockAck ())
            {
              duration += GetSifs ();
              duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxVector, COMPRESSED_BLOCK_ACK);
            }
          else if (m_txParams.MustWaitAck ())
            {
              duration += GetSifs ();
              duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
            }
        }
    }

  cts.SetDuration (duration);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (cts);
  WifiMacTrailer fcs;
  packet->AddTrailer (fcs);

  ForwardDown (packet, &cts, ctsTxVector,preamble);

  Time txDuration = m_phy->CalculateTxDuration (GetCtsSize (), ctsTxVector, preamble);
  txDuration += GetSifs ();
  NS_ASSERT (m_sendDataEvent.IsExpired ());
  
  m_sendDataEvent = Simulator::Schedule (txDuration,
                                         &MacLow::SendDataAfterCts, this,
                                         cts.GetAddr1 (),
                                         duration,
                                         ctsTxVector.GetMode());
}
void
MacLow::SendCtsAfterRts (Mac48Address source, Time duration, WifiMode rtsTxMode, double rtsSnr)
{
  NS_LOG_FUNCTION (this << source << duration << rtsTxMode << rtsSnr);
  /* send a CTS when you receive a RTS
   * right after SIFS.
   */
 WifiTxVector ctsTxVector = GetCtsTxVector (source, rtsTxMode);
  WifiMacHeader cts;
  cts.SetType (WIFI_MAC_CTL_CTS);
  cts.SetDsNotFrom ();
  cts.SetDsNotTo ();
  cts.SetNoMoreFragments ();
  cts.SetNoRetry ();
  cts.SetAddr1 (source);
  duration -= GetCtsDuration (source, ctsTxVector);
  duration -= GetSifs ();
  NS_ASSERT (duration >= MicroSeconds (0));
  cts.SetDuration (duration);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (cts);
  WifiMacTrailer fcs;
  packet->AddTrailer (fcs);

  SnrTag tag;
  tag.Set (rtsSnr);
  packet->AddPacketTag (tag);


  if(isMain == 2) //ULP radio
  {
	  MWLowTag ctsTag;
	  ctsTag.SetSimpleValue(this->GetDevice()->GetMainDevice()->GetNode()->GetId());
	  packet->AddPacketTag(ctsTag);
//	  std::cout<<"cts send :"<<recvPacketId<<" "<<this->m_self<<std::endl;
  }

  WifiPreamble preamble;
  if (ctsTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;
  ForwardDown (packet, &cts, ctsTxVector,preamble);
}

void
MacLow::SendDataAfterCts (Mac48Address source, Time duration, WifiMode txMode)
{
  NS_LOG_FUNCTION (this);
  /* send the third step in a
   * RTS/CTS/DATA/ACK hanshake
   */
  NS_ASSERT (m_currentPacket != 0);
  WifiTxVector dataTxVector = GetDataTxVector (m_currentPacket, &m_currentHdr);
  
  WifiPreamble preamble;       
  if (m_phy->GetGreenfield() && m_stationManager->GetGreenfieldSupported (m_currentHdr.GetAddr1 ()))
     //In the future has to make sure that receiver has greenfield enabled
     preamble= WIFI_PREAMBLE_HT_GF;
  else if (dataTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
     preamble=WIFI_PREAMBLE_LONG;
  
  StartDataTxTimers (dataTxVector);
  Time newDuration = Seconds (0);
  newDuration += GetSifs ();
  newDuration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxVector);
  Time txDuration = m_phy->CalculateTxDuration (GetSize (m_currentPacket, &m_currentHdr),
                                                dataTxVector, preamble);
  duration -= txDuration;
  duration -= GetSifs ();

  duration = std::max (duration, newDuration);
  NS_ASSERT (duration >= MicroSeconds (0));
  m_currentHdr.SetDuration (duration);

  m_currentPacket->AddHeader (m_currentHdr);
  WifiMacTrailer fcs;
  m_currentPacket->AddTrailer (fcs);


  if(this->isMain == 1)
  {

	  if (this->sendPacketId == m_currentPacket->GetUid())
	  {
		  this->GetDevice()->SetIsTXed(1);
//	  			  std::cout<<"hihihihih!!"<<std::endl;
	  }
  }

  ForwardDown (m_currentPacket, &m_currentHdr, dataTxVector,preamble);
  m_currentPacket = 0;
}

void
MacLow::WaitSifsAfterEndTx (void)
{
  m_listener->StartNext ();
}

void 
MacLow::EndTxNoAck (void)
{
  MacLowTransmissionListener *listener = m_listener;
  m_listener = 0;
  listener->EndTxNoAck ();
}

void
MacLow::FastAckFailedTimeout (void)
{
  NS_LOG_FUNCTION (this);
  MacLowTransmissionListener *listener = m_listener;
  m_listener = 0;
  listener->MissedAck ();
  NS_LOG_DEBUG ("fast Ack busy but missed");
}

void
MacLow::SendAckAfterData (Mac48Address source, Time duration, WifiMode dataTxMode, double dataSnr)
{
  NS_LOG_FUNCTION (this);
  /* send an ACK when you receive
   * a packet after SIFS.
   */
  WifiTxVector ackTxVector = GetAckTxVector (source, dataTxMode);
  WifiMacHeader ack;
  ack.SetType (WIFI_MAC_CTL_ACK);
  ack.SetDsNotFrom ();
  ack.SetDsNotTo ();
  ack.SetNoRetry ();
  ack.SetNoMoreFragments ();
  ack.SetAddr1 (source);
  duration -= GetAckDuration (ackTxVector);
  duration -= GetSifs ();
  NS_ASSERT (duration >= MicroSeconds (0));
  ack.SetDuration (duration);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (ack);
  WifiMacTrailer fcs;
  packet->AddTrailer (fcs);

  SnrTag tag;
  tag.Set (dataSnr);
  packet->AddPacketTag (tag);

   //since ACK is a control response it can't have Fomat =GF
  WifiPreamble preamble;
  if (ackTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;

  if(isMain == 1) //Main radio
  {
	  MWLowTag ackTag;
	  ackTag.SetSimpleValue(this->GetDevice()->GetNode()->GetId());
	  packet->AddPacketTag(ackTag);
  }

  ForwardDown (packet, &ack, ackTxVector, preamble);
}

bool
MacLow::StoreMpduIfNeeded (Ptr<Packet> packet, WifiMacHeader hdr)
{
  AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
  if (it != m_bAckAgreements.end ())
    {
      WifiMacTrailer fcs;
      packet->RemoveTrailer (fcs);
      BufferedPacket bufferedPacket (packet, hdr);

      uint16_t endSequence = ((*it).second.first.GetStartingSequence () + 2047) % 4096;
      uint16_t mappedSeqControl = QosUtilsMapSeqControlToUniqueInteger (hdr.GetSequenceControl (), endSequence);

      BufferedPacketI i = (*it).second.second.begin ();
      for (; i != (*it).second.second.end ()
           && QosUtilsMapSeqControlToUniqueInteger ((*i).second.GetSequenceControl (), endSequence) < mappedSeqControl; i++)
        {
          ;
        }
      (*it).second.second.insert (i, bufferedPacket);

      //Update block ack cache
      BlockAckCachesI j = m_bAckCaches.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
      NS_ASSERT (j != m_bAckCaches.end ());
      (*j).second.UpdateWithMpdu (&hdr);

      return true;
    }
  return false;
}

void
MacLow::CreateBlockAckAgreement (const MgtAddBaResponseHeader *respHdr, Mac48Address originator,
                                 uint16_t startingSeq)
{
  uint8_t tid = respHdr->GetTid ();
  BlockAckAgreement agreement (originator, tid);
  if (respHdr->IsImmediateBlockAck ())
    {
      agreement.SetImmediateBlockAck ();
    }
  else
    {
      agreement.SetDelayedBlockAck ();
    }
  agreement.SetAmsduSupport (respHdr->IsAmsduSupported ());
  agreement.SetBufferSize (respHdr->GetBufferSize () + 1);
  agreement.SetTimeout (respHdr->GetTimeout ());
  agreement.SetStartingSequence (startingSeq);

  std::list<BufferedPacket> buffer (0);
  AgreementKey key (originator, respHdr->GetTid ());
  AgreementValue value (agreement, buffer);
  m_bAckAgreements.insert (std::make_pair (key, value));

  BlockAckCache cache;
  cache.Init (startingSeq, respHdr->GetBufferSize () + 1);
  m_bAckCaches.insert (std::make_pair (key, cache));

  if (respHdr->GetTimeout () != 0)
    {
      AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, respHdr->GetTid ()));
      Time timeout = MicroSeconds (1024 * agreement.GetTimeout ());

      AcIndex ac = QosUtilsMapTidToAc (agreement.GetTid ());

      it->second.first.m_inactivityEvent = Simulator::Schedule (timeout,
                                                                &MacLowBlockAckEventListener::BlockAckInactivityTimeout,
                                                                m_edcaListeners[ac],
                                                                originator, tid);
    }
}

void
MacLow::DestroyBlockAckAgreement (Mac48Address originator, uint8_t tid)
{
  AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
  if (it != m_bAckAgreements.end ())
    {
      RxCompleteBufferedPacketsWithSmallerSequence (it->second.first.GetStartingSequence (), originator, tid);
      RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
      m_bAckAgreements.erase (it);

      BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
      NS_ASSERT (i != m_bAckCaches.end ());
      m_bAckCaches.erase (i);
    }
}

void
MacLow::RxCompleteBufferedPacketsWithSmallerSequence (uint16_t seq, Mac48Address originator, uint8_t tid)
{
  AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
  if (it != m_bAckAgreements.end ())
    {
      uint16_t endSequence = ((*it).second.first.GetStartingSequence () + 2047) % 4096;
      uint16_t mappedStart = QosUtilsMapSeqControlToUniqueInteger (seq, endSequence);
      uint16_t guard = (*it).second.second.begin ()->second.GetSequenceControl () & 0xfff0;
      BufferedPacketI last = (*it).second.second.begin ();

      BufferedPacketI i = (*it).second.second.begin ();
      for (; i != (*it).second.second.end ()
           && QosUtilsMapSeqControlToUniqueInteger ((*i).second.GetSequenceNumber (), endSequence) < mappedStart;)
        {
          if (guard == (*i).second.GetSequenceControl ())
            {
              if (!(*i).second.IsMoreFragments ())
                {
                  while (last != i)
                    {
                      m_rxCallback ((*last).first, &(*last).second);
                      last++;
                    }
                  m_rxCallback ((*last).first, &(*last).second);
                  last++;
                  /* go to next packet */
                  while (i != (*it).second.second.end () && ((guard >> 4) & 0x0fff) == (*i).second.GetSequenceNumber ())
                    {
                      i++;
                    }
                  if (i != (*it).second.second.end ())
                    {
                      guard = (*i).second.GetSequenceControl () & 0xfff0;
                      last = i;
                    }
                }
              else
                {
                  guard++;
                }
            }
          else
            {
              /* go to next packet */
              while (i != (*it).second.second.end () && ((guard >> 4) & 0x0fff) == (*i).second.GetSequenceNumber ())
                {
                  i++;
                }
              if (i != (*it).second.second.end ())
                {
                  guard = (*i).second.GetSequenceControl () & 0xfff0;
                  last = i;
                }
            }
        }
      (*it).second.second.erase ((*it).second.second.begin (), i);
    }
}

void
MacLow::RxCompleteBufferedPacketsUntilFirstLost (Mac48Address originator, uint8_t tid)
{
  AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
  if (it != m_bAckAgreements.end ())
    {
      uint16_t startingSeqCtrl = ((*it).second.first.GetStartingSequence () << 4) & 0xfff0;
      uint16_t guard = startingSeqCtrl;

      BufferedPacketI lastComplete = (*it).second.second.begin ();
      BufferedPacketI i = (*it).second.second.begin ();
      for (; i != (*it).second.second.end () && guard == (*i).second.GetSequenceControl (); i++)
        {
          if (!(*i).second.IsMoreFragments ())
            {
              while (lastComplete != i)
                {
                  m_rxCallback ((*lastComplete).first, &(*lastComplete).second);
                  lastComplete++;
                }
              m_rxCallback ((*lastComplete).first, &(*lastComplete).second);
              lastComplete++;
            }
          guard = (*i).second.IsMoreFragments () ? (guard + 1) : ((guard + 16) & 0xfff0);
        }
      (*it).second.first.SetStartingSequence ((guard >> 4) & 0x0fff);
      /* All packets already forwarded to WifiMac must be removed from buffer:
      [begin (), lastComplete) */
      (*it).second.second.erase ((*it).second.second.begin (), lastComplete);
    }
}

void
MacLow::SendBlockAckResponse (const CtrlBAckResponseHeader* blockAck, Mac48Address originator, bool immediate,
                              Time duration, WifiMode blockAckReqTxMode)
{
  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (*blockAck);

  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_CTL_BACKRESP);
  hdr.SetAddr1 (originator);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  hdr.SetNoRetry ();
  hdr.SetNoMoreFragments ();

  WifiTxVector blockAckTxVector = GetBlockAckTxVector (originator, blockAckReqTxMode);
  WifiTxVector blockAckReqTxVector;
  blockAckReqTxVector.SetMode(blockAckReqTxMode);
  blockAckReqTxVector.SetNss(1);
  blockAckReqTxVector.SetStbc(false);

  m_currentPacket = packet;
  m_currentHdr = hdr;
  if (immediate)
    {
      m_txParams.DisableAck ();
      duration -= GetSifs ();
      if (blockAck->IsBasic ())
        {
          duration -= GetBlockAckDuration (originator, blockAckReqTxVector, BASIC_BLOCK_ACK);
        }
      else if (blockAck->IsCompressed ())
        {
          duration -= GetBlockAckDuration (originator, blockAckReqTxVector, COMPRESSED_BLOCK_ACK);
        }
      else if (blockAck->IsMultiTid ())
        {
          NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
        }
    }
  else
    {
      m_txParams.EnableAck ();
      duration += GetSifs ();
      duration += GetAckDuration (originator, blockAckReqTxVector);
    }
  m_txParams.DisableNextData ();

  if (!immediate)
    {
      StartDataTxTimers (blockAckTxVector);
    }

  NS_ASSERT (duration >= MicroSeconds (0));
  hdr.SetDuration (duration);
  //here should be present a control about immediate or delayed block ack
  //for now we assume immediate
  packet->AddHeader (hdr);
  WifiMacTrailer fcs;
  packet->AddTrailer (fcs);
   WifiPreamble preamble;
  if (blockAckTxVector.GetMode().GetModulationClass () == WIFI_MOD_CLASS_HT)
    preamble= WIFI_PREAMBLE_HT_MF;
  else
    preamble=WIFI_PREAMBLE_LONG;
  ForwardDown (packet, &hdr, blockAckTxVector,preamble);
  m_currentPacket = 0;
}

void
MacLow::SendBlockAckAfterBlockAckRequest (const CtrlBAckRequestHeader reqHdr, Mac48Address originator,
                                          Time duration, WifiMode blockAckReqTxMode)
{
  NS_LOG_FUNCTION (this);
  CtrlBAckResponseHeader blockAck;
  uint8_t tid;
  bool immediate = false;
  if (!reqHdr.IsMultiTid ())
    {
      tid = reqHdr.GetTidInfo ();
      AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
      if (it != m_bAckAgreements.end ())
        {
          blockAck.SetStartingSequence (reqHdr.GetStartingSequence ());
          blockAck.SetTidInfo (tid);
          immediate = (*it).second.first.IsImmediateBlockAck ();
          if (reqHdr.IsBasic ())
            {
              blockAck.SetType (BASIC_BLOCK_ACK);
            }
          else if (reqHdr.IsCompressed ())
            {
              blockAck.SetType (COMPRESSED_BLOCK_ACK);
            }
          BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
          NS_ASSERT (i != m_bAckCaches.end ());
          (*i).second.FillBlockAckBitmap (&blockAck);

          /* All packets with smaller sequence than starting sequence control must be passed up to Wifimac
           * See 9.10.3 in IEEE8022.11e standard.
           */
          RxCompleteBufferedPacketsWithSmallerSequence (reqHdr.GetStartingSequence (), originator, tid);
          RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
        }
      else
        {
          NS_LOG_DEBUG ("there's not a valid block ack agreement with " << originator);
        }
    }
  else
    {
      NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
    }

  SendBlockAckResponse (&blockAck, originator, immediate, duration, blockAckReqTxMode);
}

void
MacLow::ResetBlockAckInactivityTimerIfNeeded (BlockAckAgreement &agreement)
{
  if (agreement.GetTimeout () != 0)
    {
      NS_ASSERT (agreement.m_inactivityEvent.IsRunning ());
      agreement.m_inactivityEvent.Cancel ();
      Time timeout = MicroSeconds (1024 * agreement.GetTimeout ());

      AcIndex ac = QosUtilsMapTidToAc (agreement.GetTid ());
      //std::map<AcIndex, MacLowTransmissionListener*>::iterator it = m_edcaListeners.find (ac);
      //NS_ASSERT (it != m_edcaListeners.end ());

      agreement.m_inactivityEvent = Simulator::Schedule (timeout,
                                                         &MacLowBlockAckEventListener::BlockAckInactivityTimeout,
                                                         m_edcaListeners[ac],
                                                         agreement.GetPeer (),
                                                         agreement.GetTid ());
    }
}

void
MacLow::RegisterBlockAckListenerForAc (enum AcIndex ac, MacLowBlockAckEventListener *listener)
{
  m_edcaListeners.insert (std::make_pair (ac, listener));
}


///////////////////////////////////////
// MWLowTag  add by LANADA
///////////////////////////////////////

TypeId
MWLowTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MWLowTag")
    .SetParent<Tag> ()
    .AddConstructor<MWLowTag> ()
    .AddAttribute ("SimpleValue",
                   "A simple value",
                   EmptyAttributeValue (),
                   MakeUintegerAccessor (&MWLowTag::GetSimpleValue),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}
TypeId
MWLowTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
MWLowTag::GetSerializedSize (void) const
{
  return 4;
}
void
MWLowTag::Serialize (TagBuffer i) const
{
  i.WriteU32 (m_simpleValue);
}
void
MWLowTag::Deserialize (TagBuffer i)
{
  m_simpleValue = i.ReadU32 ();
}
void
MWLowTag::Print (std::ostream &os) const
{
  os << "v=" << (uint32_t)m_simpleValue;
}
void
MWLowTag::SetSimpleValue (uint32_t value)
{
  m_simpleValue = value;
}
uint32_t
MWLowTag::GetSimpleValue (void) const
{
  return m_simpleValue;
}


} // namespace ns3
