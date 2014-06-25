/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/address.h"
#include "ns3/address-table.h"
#include <string>
#include <iostream>
#include <math.h>
#include "ns3/vector.h" //JJH 0527
#include "ns3/rng-seed-manager.h"
#define INT_MAX 9999 //JJH 0528
#define MAIN_RADIO 1 //JJH 0528
#define ULP_RADIO 2 //JJH 0528

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("UlpSimulatorV3");

int64_t *wakeupTime;
int64_t *collisionTraceResult;
int64_t *collisionTraceResultULP;
NetDeviceContainer mainDevices;
Time endTime;
Ptr<ListPositionAllocator> positionAlloc; //JJHmobility

static void
CollisionTrace (std::ofstream *ofs, uint32_t nodeId, int32_t old_value, int32_t new_value)
{
	collisionTraceResult[nodeId]++;
	*ofs<<Simulator::Now().GetMicroSeconds()<<"\t"<<nodeId<<"\t"<<std::endl;
}

static void
CollisionTraceULP (std::ofstream *ofs, uint32_t nodeId, int32_t old_value, int32_t new_value)
{
	collisionTraceResultULP[nodeId]++;
	*ofs<<Simulator::Now().GetMicroSeconds()<<"\t"<<nodeId<<"\t"<<std::endl;
}

static
void PacketSinkCallback (uint32_t* cumulativeBytes, std::ofstream *ofs, int i, Ptr<const Packet> packet, const Address& address){
	int nodeId;

	*cumulativeBytes += packet->GetSize ();

	std::ostringstream out;
	out<<InetSocketAddress::ConvertFrom(address).GetIpv4();
	std::string add_str =out.str();
	for(int i=0; i<3; i++)
		add_str = (add_str.substr(add_str.find('.'))).substr(1);

	istringstream (add_str) >>nodeId;
//	std::cout<<"sink "<<Simulator::Now().GetMicroSeconds()<<"\t"<<nodeId<<"\t"<<packet->GetUid()<<"\t"<<packet->GetSize ()<<"\t"<<*cumulativeBytes<<std::endl;
	*ofs<<Simulator::Now().GetMicroSeconds()<<"\td\t"<<nodeId<<"\t"<<packet->GetUid()<<"\t"<<packet->GetSize ()<<"\t"<<*cumulativeBytes<<std::endl;

}

static
void OnoffSourceCallback (uint32_t* sourceBytes, std::ofstream *ofs, uint32_t nodeId, Ptr<const Packet> packet){

	*sourceBytes += packet->GetSize ();
//	std::cout<<"source "<<Simulator::Now().GetMicroSeconds()<<"\t"<<nodeId<<"\t"<<packet->GetUid()<<"\t"<<packet->GetSize ()<<"\t"<<*sourceBytes<<std::endl;
	*ofs<<Simulator::Now().GetMicroSeconds()<<"\ts\t"<<nodeId<<"\t"<<packet->GetUid()<<"\t"<<packet->GetSize ()<<"\t"<<*sourceBytes<<std::endl;
}

static void
SleepWakeup (std::ofstream *ofs, uint32_t nodeId, int32_t prev_state,int32_t now_state)
{

	*ofs << Simulator::Now ().GetSeconds () << '\t' << nodeId << "\t" <<now_state <<std::endl;

	if(prev_state == now_state)
		return;

	if(prev_state == 1 && now_state == 0) // sleep
	{
		wakeupTime[nodeId] += Simulator::Now().GetMicroSeconds();
	}
	else if(prev_state == 0 && now_state == 1) // wake up
	{
		wakeupTime[nodeId] -= Simulator::Now().GetMicroSeconds();
	}
	else if (prev_state == -1 && now_state == -2)
	{
		if (wakeupTime[nodeId] < 0)
			wakeupTime[nodeId] += endTime.GetMicroSeconds();
	}
//	std::cout<<nodeId<<" Time : "<<Simulator::Now().GetMicroSeconds()<<"  State transition from "<<prev_state<<" to "<<now_state<<"total "<<wakeupTime[nodeId]<<std::endl;


}

static void
QueueTrace (std::ofstream *ofs, uint32_t nodeId, uint32_t old_value, uint32_t new_value)
{
//	std::cout<<"MWQueue size from : "<<old_value<<" to "<<new_value<<std::endl;

	Ptr<WifiNetDevice> temp_dev=DynamicCast<WifiNetDevice>(mainDevices.Get(nodeId));

	*ofs << Simulator::Now ().GetSeconds () << '\t' << nodeId << "\t" <<new_value<<std::endl;


}

void SetMobility(NodeContainer nodes, double d1, double d2, int type){

	if(type == 1) // grid mobility, d1:gridDelta, d2:gridWidth
	{
		MobilityHelper mobility;
		mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
				"MinX", DoubleValue (0.0),
				"MinY", DoubleValue (0.0),
				"DeltaX", DoubleValue (d1),
				"DeltaY", DoubleValue (d1),
				"GridWidth", UintegerValue ((uint32_t)(sqrt(nodes.GetN())+0.5)),
				"LayoutType", StringValue ("RowFirst"));
		mobility.Install (nodes);

	}
	else if (type == 2) // random mobility, d1:min, d2:max
	{
		Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable> ();
		uniformRV->SetAttribute ("Min", DoubleValue (d1));
		uniformRV->SetAttribute ("Max", DoubleValue (d2));

		MobilityHelper mobility;
		Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
		positionAlloc->Add (Vector (0.0, 0.0, 0.0));
		for(uint32_t i = 1; i < nodes.GetN(); i++){
			positionAlloc->Add (Vector (uniformRV->GetValue(), uniformRV->GetValue(), 0.0));
		}
		mobility.SetPositionAllocator (positionAlloc);
		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
		mobility.Install (nodes);
	}
	else if(type ==3 ) //mobility input from file JJHmobility
	{
		MobilityHelper mobility;
		mobility.SetPositionAllocator (positionAlloc);
		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
		mobility.Install (nodes);
	}
//	for(uint32_t i = 0; i < nodes.GetN(); i++){
//		Vector srcVector=GetPosition(nodes.Get(i));
//		std::cout<<srcVector.x<<"\t"<<srcVector.y<<std::endl;
//	}
}

void PopulateArpCache (Time duration){
	Ptr<ArpCache> arp = CreateObject<ArpCache> ();
	arp->SetAliveTimeout (duration);
	for (NodeList::Iterator i = NodeList::Begin(); i != NodeList::End(); i++) {
		Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT(ip != 0);
		ObjectVectorValue interfaces;
		ip->GetAttribute("InterfaceList", interfaces);
		for(ObjectVectorValue::Iterator j = interfaces.Begin(); j != interfaces.End (); j++) {
			Ptr<Ipv4Interface> ipIface = (j->second)->GetObject<Ipv4Interface> ();
			NS_ASSERT(ipIface != 0);
			Ptr<NetDevice> device = ipIface->GetDevice();
			NS_ASSERT(device != 0);
			Mac48Address addr = Mac48Address::ConvertFrom(device->GetAddress ());
			for(uint32_t k = 0; k < ipIface->GetNAddresses (); k ++) {
				Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();
				if(ipAddr == Ipv4Address::GetLoopback())
					continue;
				ArpCache::Entry * entry = arp->Add(ipAddr);
				entry->MarkWaitReply(0);
				entry->MarkAlive(addr);
			}
		}
	}
	for (NodeList::Iterator i = NodeList::Begin(); i != NodeList::End(); ++i) {
		Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT(ip !=0);
		ObjectVectorValue interfaces;
		ip->GetAttribute("InterfaceList", interfaces);
		for(ObjectVectorValue::Iterator j = interfaces.Begin(); j != interfaces.End (); j++) {
			Ptr<Ipv4Interface> ipIface = (j->second)->GetObject<Ipv4Interface> ();
			ipIface->SetAttribute("ArpCache", PointerValue(arp));
		}
	}
}

void StaticRouting(NodeContainer nodes, uint32_t nHops)
{
	// static routing configuration
	  Ipv4StaticRoutingHelper ipv4RoutingHelper; Ptr<Ipv4StaticRouting> staticRouting;
	  Ipv4Address nextHopAddress, dstAddress;

	  // interface1's address0 (inteface0: loopback)
	  for (uint32_t i = 0; i < nHops + 1; i++){ // from
		  staticRouting = ipv4RoutingHelper.GetStaticRouting (nodes.Get (i)->GetObject<Ipv4> ());

		  for (uint32_t j = 0; j < nHops + 1; j++){ // to
			  dstAddress = nodes.Get (j)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();

			  if (i < j){
				  nextHopAddress = nodes.Get (i + 1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
			  } else if (i > j){
				  nextHopAddress = nodes.Get (i - 1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
			  } else{
				  // do nothing
				  continue;
			  }
			  staticRouting->AddHostRouteTo (dstAddress, nextHopAddress, 1); // interface1
		  }
	  }
}

Vector
GetPosition (Ptr<Node> node)
{
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
	return mobility->GetPosition ();
}

void
SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

//reference : http://ankitstar.blogspot.kr/2011/05/dijkstras-algorithm.html
//Get best next hop to the source node based on shortest path
void
GetBestNextHop(uint32_t** graph, uint32_t* best_next_hop,uint32_t const num_nodes)
{
	uint32_t const path_len = num_nodes*3;
	uint32_t distance[num_nodes],precede[num_nodes],visit[num_nodes],path[path_len];
	uint32_t smalldist,newdist,current,distcurr,temp,dest,index,j;
	dest=0;
	for(uint32_t src = 1; src < num_nodes; src++)
	{
		for(uint32_t i = 0; i < num_nodes; i++)
		{
			distance[i]=INT_MAX;
			precede[i]=INT_MAX;
			visit[i]=0;
		}

		distance[src]=0;
		current=src;
		visit[current]=1;
		while(current != dest)
		{
			distcurr=distance[current];
			smalldist=INT_MAX;
			for(uint32_t i = 0; i < num_nodes; i++)
				if(visit[i] == 0)
				{
					newdist = distcurr + graph[current][i];
					if(newdist < distance[i])
					{
						distance[i] = newdist;
						precede[i] = current;
					}
					if(distance[i] < smalldist)
					{
						smalldist = distance[i];
						temp = i;
					}
				}
			current = temp;
			visit[current] = 1;
		}
		uint32_t final = 0;
		index = dest;
		path[final]=dest;
		final++;
		while(precede[index] != src)
		{
			j=precede[index];
			index=j;
			path[final]=index;
			final++;
		}
		best_next_hop[src]=path[final-1];
	}

}


double MaxNode(int max, int64_t* array, uint32_t nNode)
{
	int64_t* out = new int64_t[max];
	for(int i=0; i<max; i++)
		out[i]=0;
	for(uint32_t i=1; i<nNode; i++)
	{
		for(int j=0; j<max; j++)
		{
			if(out[j] < array[i])
			{
				out[j] = array[i];
				break;
			}
		}
	}
	double temp =0;
	for(int i=0; i<max; i++)
		temp += out[i];
	temp = temp/(double)max;
	return temp;
}


Ptr<ListPositionAllocator>
GetMobilityFile(std::ifstream* input) //JJHmobility
{
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
	if(!(*input).is_open())
	{
		std::cout<<"Mobility File open fail! \n";

	}
	double x_val,y_val;
	uint32_t num_mobility;
	*input>>num_mobility;
	//		std::cout<<num_mobility<<std::endl;
	for(uint32_t i=0; i<num_mobility;i++)
	{
		*input>>x_val>>y_val;
		//			std::cout<<x_val<<"\t"<<y_val<<std::endl;
		positionAlloc->Add (Vector (x_val, y_val, 0.0));
	}
	return positionAlloc;
}

int 
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nNode = 4;
  uint32_t channelType = 1;
  uint32_t gridWidth = 3;
  uint32_t mobilityType = 1;
  uint32_t randomSeed = 1;
  uint32_t dataRate = 100;
  double txpDistance = 20.0;
  double lambda = 5.0;
  double gridDelta = 10.0;
  double range = 50.0;
  std::string folderName ="result";
  std::string mobilityFolder = "mobility";
  int packetSize = 104;
  int nPacket = 0;
  endTime = Seconds(120.0);
  Time endTimeSource = endTime;
//  Time endTimeSource = Seconds(60.0);


  //RTS CTS set
  UintegerValue ctsThr = UintegerValue (10);
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);

  AddressTable* addressTable = new AddressTable();

  CommandLine cmd;

  cmd.AddValue ("v", "Show log", verbose);
  cmd.AddValue ("n", "Number of Node", nNode);
  cmd.AddValue ("c", "Shared channel=1, Separate channel=2", channelType);
  cmd.AddValue ("l", "Lambda (arrrival rate)", lambda);
  cmd.AddValue ("a", "GridDelta (distance btw nodes)", gridDelta);
  cmd.AddValue ("w", "GridWidth (num nodes in each row)", gridWidth);
  cmd.AddValue ("t", "TxpDistance (Tx range)", txpDistance);
  cmd.AddValue ("m", "MobilityType grid=1, random=2", mobilityType);
  cmd.AddValue ("g", "Range (sensor node area)", range);
  cmd.AddValue ("f", "Folder name for trace file (default:result)", folderName);
  cmd.AddValue ("p", "Number of packet for each node", nPacket);
  cmd.AddValue ("s", "Size of packet", packetSize);
  cmd.AddValue ("r", "Random seed", randomSeed);
  cmd.AddValue ("d", "Data Rate", dataRate);

  cmd.Parse (argc, argv);

  // set random seed
  SeedManager::SetSeed(randomSeed);

  // trace init
  wakeupTime = new int64_t[nNode];
  collisionTraceResult = new int64_t[nNode];
  collisionTraceResultULP = new int64_t[nNode];
  uint32_t *sourceBytes = new uint32_t[nNode];
  for(uint32_t i=0; i<nNode; i++){
	  wakeupTime[i]=0;
	  sourceBytes[i]=0;
	  collisionTraceResult[i]=0;
	  collisionTraceResultULP[i]=0;
  }
  // set trace filename
  std::ostringstream fileName;
  fileName<<"/n"<<nNode<<"_t"<<txpDistance<<"_s"<<packetSize<<"_d"<<dataRate<<"/l"<<lambda<<"_c"<<channelType<<"_r"<<randomSeed;
  std::ofstream *packetTrace = new std::ofstream ((folderName+"/"+fileName.str()+"_packet.txt").c_str());
  std::ofstream *wakeupTrace = new std::ofstream ((folderName+"/"+fileName.str()+"_wakeup.txt").c_str());
  std::ofstream *queueTrace = new std::ofstream ((folderName+"/"+fileName.str()+"_queue.txt").c_str());
  std::ofstream *collisionTrace = new std::ofstream ((folderName+"/"+fileName.str()+"_collision_main.txt").c_str());
  std::ofstream *collisionTraceULP = new std::ofstream ((folderName+"/"+fileName.str()+"_collision_ulp.txt").c_str());
  std::ofstream *resultFile = new std::ofstream ((folderName+"/"+fileName.str()+"_result.txt").c_str());
  std::ofstream *simulationFile = new std::ofstream ((folderName+"/"+fileName.str()+"_simulation.txt").c_str());
  std::cout<<folderName<<fileName.str()<<std::endl;

  std::ifstream *mobilityFile = new std::ifstream ((mobilityFolder+"/mobility_input.txt").c_str()); //JJHmobility

  if (verbose)
    {
//	   LogComponentEnable ("OnOffApplication", LOG_LEVEL_INFO);
//	   LogComponentEnable ("PacketSink", LOG_LEVEL_INFO);
//      LogComponentEnable ("MacLow", LOG_FUNCTION);
//      LogComponentEnable ("YansWifiPhy", LOG_DEBUG);
//      LogComponentEnable ("AdhocWifiMac", LOG_FUNCTION);
//      LogComponentEnable ("WifiNetDevice", LOG_LEVEL_INFO);
//      LogComponentEnable ("RegularWifiMac", LOG_FUNCTION);
//      LogComponentEnable ("DcaTxop",LOG_DEBUG);
//      LogComponentEnable ("WifiPhy",LOG_LOGIC);
//      LogComponentEnable ("WifiPhy",LOG_FUNCTION);

    }


  NodeContainer nodes;
  nodes.Create (nNode);

  ////////////////////////////////////////////////////
  // Main radio setting
  ////////////////////////////////////////////////////
  YansWifiChannelHelper mainChannel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper mainPhy = YansWifiPhyHelper::Default ();
  mainChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (txpDistance));

  Ptr<YansWifiChannel> channel = mainChannel.Create();
  mainPhy.SetChannel (channel);

  WifiHelper mainHelper = WifiHelper::Default ();
  mainHelper.SetStandard(WIFI_PHY_STANDARD_80211b);
  mainHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",StringValue("DsssMainRate250Kbps"),"ControlMode",StringValue("DsssMainRate250Kbps"));

  NqosWifiMacHelper mainMac = NqosWifiMacHelper::Default ();
  mainMac.SetType ("ns3::AdhocWifiMac");

  mainDevices = mainHelper.Install (mainPhy, mainMac, nodes, addressTable,MAIN_RADIO,1,channelType,dataRate);

  ////////////////////////////////////////////////////
  // ULP radio setting
  ////////////////////////////////////////////////////
  YansWifiChannelHelper ulpChannel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper ulpPhy = YansWifiPhyHelper::Default ();
  ulpChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (txpDistance));
  if(channelType==2){
	  channel = ulpChannel.Create();
//	  std::cout<<"[Separate Channel]"<<std::endl;
  }
  else{
//	  std::cout<<"[Shared Channel]"<<std::endl;
  }
  ulpPhy.SetChannel (channel);

  WifiHelper ulpHelper = WifiHelper::Default ();
  ulpHelper.SetStandard(WIFI_PHY_STANDARD_80211b);

  switch(dataRate)
  {
	  case 200 :
		  ulpHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",StringValue("DsssWakeupRate200Kbps"),"ControlMode",StringValue("DsssWakeupRate200Kbps"));
		  break;
	  case 100 :
		  ulpHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",StringValue("DsssWakeupRate100Kbps"),"ControlMode",StringValue("DsssWakeupRate100Kbps"));
		  break;
	  case 50 :
		  ulpHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",StringValue("DsssWakeupRate50Kbps"),"ControlMode",StringValue("DsssWakeupRate50Kbps"));
		  break;
	  case 10 :
		  ulpHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",StringValue("DsssWakeupRate10Kbps"),"ControlMode",StringValue("DsssWakeupRate10Kbps"));
		  break;

	  default :
		  std::cout<<"[Error] Invalid data rate\n";
		  return 0;
		  break;
  }

  NqosWifiMacHelper ulpMac = NqosWifiMacHelper::Default ();
  ulpMac.SetType ("ns3::AdhocWifiMac");

  NetDeviceContainer ulpDevices;
  ulpDevices = ulpHelper.Install (ulpPhy, ulpMac, nodes, addressTable,ULP_RADIO,1,channelType,dataRate);
  ////////////////////////////////////////////////////
  // set mobility
  ////////////////////////////////////////////////////
  positionAlloc=GetMobilityFile(mobilityFile); //JJHmobility
  SetMobility(nodes, gridDelta, gridWidth, mobilityType); //JJHmobility
//  SetMobility(nodes, -50.0, 50.0, 2);


  ////////////////////////////////////////////////////
  // Internet stack
  ////////////////////////////////////////////////////
  InternetStackHelper stack;
  stack.Install (nodes);

  Ipv4AddressHelper mainAddress;
  mainAddress.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer mainInterfaces;
  mainInterfaces = mainAddress.Assign (mainDevices);

  Ipv4AddressHelper ulpAddress;
  ulpAddress.SetBase ("20.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer ulpInterfaces;
  ulpInterfaces = ulpAddress.Assign (ulpDevices);



  //////////////////////////////////////////////////////////////////////////////
  // static routing using shortest path (dijsktra's algorithm)
  // Generating graph based on distance
  //////////////////////////////////////////////////////////////////////////////
  uint32_t** graph = new uint32_t*[nNode];
  for (uint32_t i=0; i < nNode; i++)
  {
	  graph[i] = new uint32_t[nNode];
  }
  uint32_t* best_next_hop = new uint32_t[nNode];
  for (uint32_t i=0; i < nNode; i++){
	  Vector srcVector=GetPosition(nodes.Get(i));
	  for (uint32_t j=i; j < nNode; j++)
	  {
		  Vector desVector = GetPosition(nodes.Get(j));
		  double dist=CalculateDistance(srcVector,desVector);
		  if(i==j)
		  {
			  graph[i][j]=INT_MAX;
		  }
		  else if(dist>txpDistance) //if distance is longer than maximum transmission range
		  {
			  graph[i][j]=INT_MAX;
			  graph[j][i]=INT_MAX; //Since graph is symmetric
		  }
		  else
		  {
			  graph[i][j]=dist;
			  graph[j][i]=dist;
		  }
	  }
  }

  GetBestNextHop(graph,best_next_hop,nNode);

  for (uint32_t i=0; i < nNode; i++)
  {
	  delete [] graph[i];
  }
  delete [] graph;

  // static routing using the best_next_hop information
  Ipv4StaticRoutingHelper ipv4RoutingHelper; Ptr<Ipv4StaticRouting> staticRouting;
  Ipv4Address nextHopAddress, dstAddress;
  for (uint32_t i = 1; i < nNode; i++)//src
  {
	  staticRouting = ipv4RoutingHelper.GetStaticRouting (nodes.Get (i)->GetObject<Ipv4> ());
	  uint32_t j=0; //dest
	  dstAddress=nodes.Get (j)->GetObject<Ipv4> ()->GetAddress(1,0).GetLocal ();
	  nextHopAddress = nodes.Get (best_next_hop[i])->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();
	  staticRouting->AddHostRouteTo (dstAddress, nextHopAddress, 1); // interface1
//	  std::cout<<i<<"\t"<<best_next_hop[i]<<std::endl;
  }
  delete [] best_next_hop;

  ///// routing end /////

  ////////////////////////////////////////////////////
  // onoff & sink application //
  ////////////////////////////////////////////////////

  std::ostringstream strs;
  strs << 1/lambda;
  std::string expRV = "ns3::ExponentialRandomVariable[Mean=";
  expRV += strs.str();
  expRV += "]";


  for(uint32_t i=1; i<nNode; i++){
	  OnOffHelper onOffSource ("ns3::UdpSocketFactory", InetSocketAddress (mainInterfaces.GetAddress(0), 80));
	  onOffSource.SetAttribute ("MaxBytes", UintegerValue (packetSize * nPacket)); // infite backlog
	  onOffSource.SetAttribute ("PacketSize", UintegerValue (packetSize));
	  onOffSource.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.00001]"));
//	  onOffSource.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"));
	  		onOffSource.SetAttribute ("OffTime", StringValue (expRV));
	  onOffSource.SetAttribute ("DataRate", DataRateValue (DataRate (packetSize*800000))); // saturated

	  ApplicationContainer sourceApp = onOffSource.Install (nodes.Get (i));
	  sourceApp.Start (Seconds (0.01));
	  sourceApp.Stop (endTimeSource);
	  sourceApp.Get(0)->GetObject<OnOffApplication>()->TraceConnectWithoutContext(
			  "Tx",MakeBoundCallback (OnoffSourceCallback, &sourceBytes[i],packetTrace,i));
  }
  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 80));
  ApplicationContainer sinkApp = sink.Install (nodes.Get (0));	// Sink APP install only on Sink Node(0)

  uint32_t *cumulativeBytes = new uint32_t[1];
  cumulativeBytes[0]=0;
  sinkApp.Get (0)->GetObject<PacketSink> ()->TraceConnectWithoutContext (
		  "Rx", MakeBoundCallback (PacketSinkCallback, &cumulativeBytes[0],packetTrace,1));

  sinkApp.Start (Seconds (0.0));
  sinkApp.Stop (endTime);

  ////////////////////////////////////////////////////
  // trace
  ////////////////////////////////////////////////////

  // SleepWakeup trace //
  for(uint32_t i=0; i<nNode; i++){
	  Ptr<WifiNetDevice> temp_dev=DynamicCast<WifiNetDevice>(mainDevices.Get(i));
	  Ptr<RegularWifiMac> temp_regwifimac= temp_dev->GetMac()->GetObject<RegularWifiMac>();
	  Ptr<DcaTxop> temp_DcaTxop = temp_regwifimac->GetDcaTxop();
	  Ptr<MacLow> temp_MacLow = temp_DcaTxop->m_low;
	  //	  Ptr<MacLow> temp_MacLow = temp_DcaTxop->GetObject<MacLow>(); //
	  temp_MacLow->TraceConnectWithoutContext("SleepCount",MakeBoundCallback (&SleepWakeup,wakeupTrace,i));
  }

  // Queue trace //
  for(uint32_t i=0; i<nNode; i++){
	  Ptr<WifiNetDevice> temp_dev=DynamicCast<WifiNetDevice>(mainDevices.Get(i));
	  temp_dev->GetMWQueue()->TraceConnectWithoutContext("MWQueueSize",MakeBoundCallback (&QueueTrace,queueTrace,i));
  }

  // Main Packet drop trace//
  for(uint32_t i=0; i<nNode; i++){
	  Ptr<WifiNetDevice> temp_dev=DynamicCast<WifiNetDevice>(mainDevices.Get(i));
	  Ptr<RegularWifiMac> temp_regwifimac= temp_dev->GetMac()->GetObject<RegularWifiMac>();
	  Ptr<DcaTxop> temp_DcaTxop = temp_regwifimac->GetDcaTxop();
	  temp_DcaTxop->TraceConnectWithoutContext("DcaTxOpCollision",MakeBoundCallback (&CollisionTrace,collisionTrace,i));
  }

  // ULP Packet drop trace//
  for(uint32_t i=0; i<nNode; i++){
	  Ptr<WifiNetDevice> temp_dev=DynamicCast<WifiNetDevice>(ulpDevices.Get(i));
	  Ptr<RegularWifiMac> temp_regwifimac= temp_dev->GetMac()->GetObject<RegularWifiMac>();
	  Ptr<DcaTxop> temp_DcaTxop = temp_regwifimac->GetDcaTxop();
	  temp_DcaTxop->TraceConnectWithoutContext("DcaTxOpCollision",MakeBoundCallback (&CollisionTraceULP,collisionTraceULP,i));
  }
////////////////////////////////////////////////////
  // ARP table
  ////////////////////////////////////////////////////
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  PopulateArpCache(endTime);


  ////////////////////////////////////////////////////
  // Simulation start
  ////////////////////////////////////////////////////
  Simulator::Stop (endTime);
  Simulator::Run ();


  ////////////////////////////////////////////////////
  // Print output
  ////////////////////////////////////////////////////

  /* Onoff source trace*/
  uint32_t totalSource = 0;
  for(uint32_t i = 1; i < nNode; i++)
  {
	  totalSource += sourceBytes[i];
  }
//  std::cout<<"total Tx : "<<totalSource;
  *resultFile<<"total Tx : "<<totalSource;
  /* packet Sink trace */
//  std::cout << "\t total Rx : " << cumulativeBytes[0] << std::endl;
  *resultFile << "\t total Rx : " << cumulativeBytes[0] << std::endl;
  std::cout << "Delivery Ratio : " << (double)cumulativeBytes[0]/totalSource * 100.0 <<" %"<< std::endl; // kdw print
  *resultFile<< "Delivery Ratio : " << (double)cumulativeBytes[0]/totalSource * 100.0 <<" %"<< std::endl;

//  delete []cumulativeBytes;

  /* Wake up time trace */
  for(uint32_t i=0; i<nNode; i++)
	  SleepWakeup (wakeupTrace, i, -1,-2);

  int64_t totalWakeupTime =0;
  int64_t totalcollisionTraceResult =0;
  int64_t totalcollisionTraceResultULP =0;
//  std::cout<<std::endl<<"nodeId"<<"\t"<<"Wakeup Time\tcollision main\tULP"<<std::endl;
  *resultFile<<std::endl<<"nodeId"<<"\t"<<"Wakeup Time\tcollision main\tULP"<<std::endl;

//  std::cout<<0<<"\t"<<wakeupTime[0]<<"\t"<<collisionTraceResult[0]<<"\t"<<collisionTraceResultULP[0]<<std::endl;
  	  *resultFile<<0<<"\t"<<wakeupTime[0]<<"\t"<<collisionTraceResult[0]<<"\t"<<collisionTraceResultULP[0]<<std::endl;
  for(uint32_t i = 1; i < nNode; i++)
  {
//	  std::cout<<i<<"\t"<<wakeupTime[i]<<"\t"<<collisionTraceResult[i]<<"\t"<<collisionTraceResultULP[i]<<std::endl;
	  *resultFile<<i<<"\t"<<wakeupTime[i]<<"\t"<<collisionTraceResult[i]<<"\t"<<collisionTraceResultULP[i]<<std::endl;
	  totalWakeupTime += wakeupTime[i];
	  totalcollisionTraceResult += collisionTraceResult[i];
	  totalcollisionTraceResultULP += collisionTraceResultULP[i];
  }
//  std::cout<<"total wake up time : "<<totalWakeupTime<<std::endl;
  *resultFile<<"total wake up time : "<<totalWakeupTime<<std::endl;
//  std::cout<<"total collision (main) : "<<totalcollisionTraceResult<<std::endl;
  *resultFile<<"total collision (main) : "<<totalWakeupTime<<std::endl;
//  std::cout<<"total collision (ULP) : "<<totalcollisionTraceResultULP<<std::endl;
  *resultFile<<"total collision (ULP) : "<<totalWakeupTime<<std::endl;

  for(uint32_t i=0; i<nNode; i++){
	  Ptr<WifiNetDevice> temp_dev=DynamicCast<WifiNetDevice>(mainDevices.Get(i));
	  Ptr<RegularWifiMac> temp_regwifimac= temp_dev->GetMac()->GetObject<RegularWifiMac>();
	  Ptr<DcaTxop> temp_DcaTxop = temp_regwifimac->GetDcaTxop();
	  Ptr<MacLow> temp_MacLow = temp_DcaTxop->m_low;
//	  std::cout<<temp_MacLow->IsSleep();
	  *resultFile<<temp_MacLow->IsSleep();
  }
  std::cout<<std::endl;
  *resultFile<<std::endl;
  for(uint32_t i=0; i<nNode; i++){
	  Ptr<WifiNetDevice> temp_dev=DynamicCast<WifiNetDevice>(mainDevices.Get(i));
//	  std::cout<<i<<" "<<temp_dev->GetMWQueue()->GetSize()<<std::endl;
	  *resultFile<<i<<" "<<temp_dev->GetMWQueue()->GetSize()<<std::endl;

  }
//  std::cout<<"totalWakeupTime"<<"\t"<<"RX" <<std::endl;
//  std::cout<<totalWakeupTime<<"\t"<<cumulativeBytes[0] <<std::endl;

  *resultFile<<"totalWakeupTime"<<"\t"<<"RX" <<std::endl;
  *resultFile<<totalWakeupTime<<"\t"<<cumulativeBytes[0] <<std::endl;


  double temp = MaxNode(4, wakeupTime, nNode);

  *simulationFile<<(double)totalWakeupTime/1000000.0<<"\t"<<(int)((double)cumulativeBytes[0]/endTime.GetSeconds())<<"\t"<<temp/1000000.0<<std::endl;
  std::cout<<(double)totalWakeupTime/1000000.0<<"\t"<<(int)((double)cumulativeBytes[0]/endTime.GetSeconds())<<"\t"<<temp/1000000.0<<std::endl;
  ////////////////////////////////////////////////////
  // Simulation End
  ////////////////////////////////////////////////////
  Simulator::Destroy ();
  return 0;
}


