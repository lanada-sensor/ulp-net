/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005, 2009 INRIA
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

#include <iostream>
#include <stdio.h>
#include <map>
#include "mw-queue.h"
#include "wifi-net-device.h"
#include "ns3/trace-source-accessor.h" //JJH 0530 tracing

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (MWQueue)
  ;

TypeId
MWQueue::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MWQueue")
    .SetParent<Object> ()
    .AddConstructor<MWQueue> ()
    .AddTraceSource("MWQueueSize","The MWQueue's size",
    	     		   MakeTraceSourceAccessor (&MWQueue::m_size))
  ;
  return tid;
}

MWQueue::MWQueue()
	{
		m_maxSize = 10000;
		m_size = 0;
	}


MWQueue::~MWQueue ()
{
}


bool
MWQueue::Enqueue (Ptr<Packet> packet,const Address& dest, uint16_t protocolNumber)
{
  if (m_size == m_maxSize)
    {
//	  std::cout<<"queue overflow\n";
      return false;
    }
  MWItem* i = new MWItem (packet, dest, protocolNumber);
  m_queue.push_back (i);
  m_size++;
//	  cout<<"enqueue "<<m_size<<endl;
  return true;
}

void
MWQueue::PushFront (MWItem* mwItem)
{
  if (m_size == m_maxSize)
    {
      return;
    }
  m_queue.push_front (mwItem);
  m_size++;
}

MWItem*
MWQueue::Dequeue ()
{
  if (!m_queue.empty ())
    {
	  MWItem* i = m_queue.front ();
      m_queue.pop_front ();
      m_size--;
//	      cout<<"dequeue "<<m_size<<endl;
      return i;
    }
  return 0;
}

MWItem*
MWQueue::GetFront ()
{
  if (!m_queue.empty ())
    {
	  MWItem* i = m_queue.front ();
      return i;
    }
  return 0;
}

void
MWQueue::Cleanup (void)
{
  if (m_queue.empty ())
    {
      return;
    }

  for (PacketQueueI i = m_queue.begin (); i != m_queue.end ();)
    {
          i = m_queue.erase (i);
    }
}
uint32_t
MWQueue::GetSize(){
	return m_size;
}
} // namespace ns3
