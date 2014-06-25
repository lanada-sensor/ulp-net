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

#ifndef WIFI_NET_DEVICE_H
#define WIFI_NET_DEVICE_H

#include "ns3/net-device.h"
#include "ns3/packet.h"
#include "ns3/traced-callback.h"
#include "ns3/mac48-address.h"
#include "ns3/event-id.h"
#include <string>
#include "address-table.h"
#include "mw-queue.h"
#include "mac-low.h"
#include "regular-wifi-mac.h"
#include "dca-txop.h"
#include "ns3/simulator.h"
#include "ns3/net-device-container.h"

namespace ns3 {

class WifiRemoteStationManager;
class WifiChannel;
class WifiPhy;
class WifiMac;
class MWQueue;
class MWItem;

/**
 * \defgroup wifi Wifi Models
 *
 * This section documents the API of the ns-3 Wifi module. For a generic functional description, please refer to the ns-3 manual.
 */


/**
 * \brief Hold together all Wifi-related objects.
 * \ingroup wifi
 *
 * This class holds together ns3::WifiChannel, ns3::WifiPhy,
 * ns3::WifiMac, and, ns3::WifiRemoteStationManager.
 */
class WifiNetDevice : public NetDevice
{
public:
  static TypeId GetTypeId (void);

  WifiNetDevice ();
  virtual ~WifiNetDevice ();


  ///////////////////////////////////////
  // MW-MAC  add by LANADA
  ///////////////////////////////////////
  // constructor //
  WifiNetDevice (uint32_t isMain);

  // install //

  bool SetMainDevice(Ptr<WifiNetDevice> m_main);
  bool SetUlpDevice(Ptr<WifiNetDevice> m_ulp);
  Ptr<WifiNetDevice> GetUlpDevice (void);
  Ptr<WifiNetDevice> GetMainDevice (void);

  bool SetValue(uint32_t index, uint32_t value);
  uint32_t GetValue(uint32_t index);
  bool SetAddressTable(AddressTable* table);
  AddressTable* GetAddressTable();
  bool SetDataRate(uint32_t dataRate);

  bool SetMacLow();
  Ptr<MacLow> GetMacLow();

  bool SetDeviceContainer(NetDeviceContainer* netDevice);
  NetDeviceContainer* GetDeviceContainer();

  // sleep //
  bool Sleep (bool isSender);
  bool IsSleep (void);

  // send //
  uint32_t SendFromMWQueue();
  uint32_t SendAfterWakeupSignal(bool retransmission);
  uint32_t SendDoneAck();
  void SetBuffer(Ptr<Packet> packet);
  Ptr<Packet> GetBuffer(void) const;
  uint32_t GetBufferlen (void) const;
  Ptr<MWQueue> GetMWQueue (void) const;

  // retransmission //
  void RetransmissionReq(uint32_t option);
  void RetransmissionMain(uint32_t option);
  void SetIsTXed(uint32_t i);
  uint32_t GetIsTXed();
  ///////////////////////////////////////
  // MW-MAC  END!
  ///////////////////////////////////////
  /**
   * \param mac the mac layer to use.
   */
  void SetMac (Ptr<WifiMac> mac);
  /**
   * \param phy the phy layer to use.
   */
  void SetPhy (Ptr<WifiPhy> phy);
  /**
   * \param manager the manager to use.
   */
  void SetRemoteStationManager (Ptr<WifiRemoteStationManager> manager);
  /**
   * \returns the mac we are currently using.
   */
  Ptr<WifiMac> GetMac (void) const;
  /**
   * \returns the phy we are currently using.
   */
  Ptr<WifiPhy> GetPhy (void) const;
  /**
   * \returns the remote station manager we are currently using.
   */
  Ptr<WifiRemoteStationManager> GetRemoteStationManager (void) const;


  // inherited from NetDevice base class.
  virtual void SetIfIndex (const uint32_t index);
  virtual uint32_t GetIfIndex (void) const;
  virtual Ptr<Channel> GetChannel (void) const;
  virtual void SetAddress (Address address);
  virtual Address GetAddress (void) const;
  virtual bool SetMtu (const uint16_t mtu);
  virtual uint16_t GetMtu (void) const;
  virtual bool IsLinkUp (void) const;
  virtual void AddLinkChangeCallback (Callback<void> callback);
  virtual bool IsBroadcast (void) const;
  virtual Address GetBroadcast (void) const;
  virtual bool IsMulticast (void) const;
  virtual Address GetMulticast (Ipv4Address multicastGroup) const;
  virtual bool IsPointToPoint (void) const;
  virtual bool IsBridge (void) const;
  virtual bool Send (Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber);
  virtual Ptr<Node> GetNode (void) const;
  virtual void SetNode (Ptr<Node> node);
  virtual bool NeedsArp (void) const;
  virtual void SetReceiveCallback (NetDevice::ReceiveCallback cb);

  virtual Address GetMulticast (Ipv6Address addr) const;

  virtual bool SendFrom (Ptr<Packet> packet, const Address& source, const Address& dest, uint16_t protocolNumber);
  virtual void SetPromiscReceiveCallback (PromiscReceiveCallback cb);
  virtual bool SupportsSendFrom (void) const;
protected:
   virtual void DoDispose (void);
   virtual void DoInitialize (void);
  /**
   * Receive a packet from the lower layer and pass the
   * packet up the stack.
   *
   * \param packet
   * \param from
   * \param to
   */
   void ForwardUp (Ptr<Packet> packet, Mac48Address from, Mac48Address to);
private:
  // This value conforms to the 802.11 specification
  static const uint16_t MAX_MSDU_SIZE = 2304;

  /**
   * Set that the link is up. A link is always up in ad-hoc mode.
   * For a STA, a link is up when the STA is associated with an AP.
   */
  void LinkUp (void);
  /**
   * Set that the link is down (i.e. STA is not associated).
   */
  void LinkDown (void);
  /**
   * Return the WifiChannel this device is connected to.
   *
   * \return WifiChannel
   */
  Ptr<WifiChannel> DoGetChannel (void) const;
  /**
   * Complete the configuration of this Wi-Fi device by
   * connecting all lower components (e.g. MAC, WifiRemoteStation) together.
   */
  void CompleteConfig (void);

  Ptr<Node> m_node;
  Ptr<WifiPhy> m_phy;
  Ptr<WifiMac> m_mac;
  Ptr<WifiRemoteStationManager> m_stationManager;
  NetDevice::ReceiveCallback m_forwardUp;
  NetDevice::PromiscReceiveCallback m_promiscRx;

  TracedCallback<Ptr<const Packet>, Mac48Address> m_rxLogger;
  TracedCallback<Ptr<const Packet>, Mac48Address> m_txLogger;

  uint32_t m_ifIndex;
  bool m_linkUp;
  TracedCallback<> m_linkChanges;
  mutable uint16_t m_mtu;
  bool m_configComplete;

  ///////////////////////////////////////
  // MW-MAC  add by LANADA
  ///////////////////////////////////////

  bool isMwmac;

  uint32_t isMain;
  uint32_t mainType;
  uint32_t channelType;

  Ptr<WifiNetDevice> m_main;
  Ptr<WifiNetDevice> m_ulp;
  Ptr<MacLow> m_MacLow;
  uint32_t dataRate;

  NetDeviceContainer* netDevice;

  AddressTable* m_addressTable;

  //MWQueue
  Ptr<MWQueue> mwQueue;
  MWItem* tempMWItem;
  bool isWorking;

  //buffer
  Ptr<Packet> m_buffer;
  uint32_t m_bufferlen;

  uint32_t test1;
  uint32_t test2;

  //timeout

  EventId m_MWTimeoutEvent;

  // retx
  uint32_t isAcked;
  uint32_t isTXed;

};

class MWTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

  // these are our accessors to our tag structure
  void SetSimpleValue (uint8_t value);
  uint8_t GetSimpleValue (void) const;
  void SetAddressValue (Mac48Address address);
  Mac48Address GetAddressValue (void) const;
private:
  uint8_t m_simpleValue;
  Mac48Address address;
};

} // namespace ns3

#endif /* WIFI_NET_DEVICE_H */
