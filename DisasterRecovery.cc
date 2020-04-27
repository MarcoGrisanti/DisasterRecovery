#include <fstream>
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/double.h"
#include "ns3/internet-module.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "Utils.h"

using namespace dr;
using namespace ns3;
using namespace std;

uint32_t sequenceNumber = 0;
uint32_t sentPackets = 0;
uint32_t receivedPackets = 0;

static void CSMA_ReceivePacket(Ptr<Socket> socket) {
  Ptr<Packet> packet;
  while (packet = socket->Recv()) {
    uint8_t buffer[RCV_BUFFER_SIZE];
    packet->CopyData(buffer, RCV_BUFFER_SIZE);
  }
}

static void GenerateTraffic(uint32_t sourceID, uint32_t sourceIndex, Time trafficInterval) {
  string content = EXAMPLE_MESSAGE + to_string(sequenceNumber);
  sentPackets++;
  sequenceNumber++;
  const uint8_t* tmp = reinterpret_cast<const uint8_t*>(&content[0]);
  Ipv4Address sourceIP = wifiStaNodes.Get(sourceIndex)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
  ForwardPacket(Create<Packet>(tmp, sizeof(content) + 1), sourceID, sourceIP);
  Simulator::Schedule(trafficInterval, &GenerateTraffic, sourceID, sourceIndex, trafficInterval);
}

static void WirelessReceivePacket(Ptr<Socket> socket) {
  Address adr;
  Ptr<Packet> packet;
  while (packet = socket->RecvFrom(adr)) {
    uint8_t buffer[RCV_BUFFER_SIZE];
    packet->CopyData(buffer, RCV_BUFFER_SIZE);
    uint32_t localNodeID = socket->GetNode()->GetId();
    string message((char*)buffer);
    if (strcasestr((char*)buffer, HELLO_MESSAGE)) {
      if (localNodeID != ACCESS_POINT_ID) {
        UpdateGraph(message, localNodeID, interHelloTime);
      }
    }
    else {
      if (localNodeID != ACCESS_POINT_ID) {
        Ipv4Address senderIP = InetSocketAddress::ConvertFrom(adr).GetIpv4();
        ForwardPacket(packet, localNodeID, senderIP);
      }
      else {
        uint32_t actualSequenceNumber = stoi(message.substr(message.find(":") + 1));
        if (!receivedSequenceNumbers[actualSequenceNumber]) {
          receivedPackets++;
          receivedSequenceNumbers[actualSequenceNumber] = true;
          CSMA_Forward(packet);
        }
      }
    }
  }
}

static void SendHello(Ptr<Socket> socket, Time packetInterval) {
  string hello = HELLO_MESSAGE + to_string(socket->GetNode()->GetId());
  const uint8_t* tmp = reinterpret_cast<const uint8_t*>(&hello[0]);
  socket->Send(Create<Packet>(tmp, sizeof(hello) + 1));
  Simulator::Schedule(packetInterval, &SendHello, socket, packetInterval);
}

static void SwitchInterface(Ptr<Ipv4Interface> iface, bool switchOn, uint32_t nodeID) {
  Time t;
  if (switchOn) {
    iface->SetUp();
    t = Seconds(onPeriodExpDistrib->GetValue());
    Simulator::Schedule(t, &SwitchInterface, iface, false, nodeID);
  }
  else {
    iface->SetDown();
    t = Seconds(offPeriodExpDistrib->GetValue());
    Simulator::Schedule(t, &SwitchInterface, iface, true, nodeID);
  }
}

NS_LOG_COMPONENT_DEFINE("DisasterRecovery");

int main(int argc, char* argv[]) {

  interHelloTime = INTER_HELLO_TIME;
  double propagationRange = PROPAGATION_RANGE;
  double simulationTime = SIMULATION_TIME;
  double squareSide = SCENARIO_SIDE;
  uint32_t mobileNodes = MOBILE_NODES;
  uint32_t rngSeed = RNG_SEED;
  const uint32_t fixedNodesPerSide = 5;
  string outFile = RESULTS_FILENAME;  

  CommandLine cmd;
  cmd.AddValue("helloInterval", "", interHelloTime);
  cmd.AddValue("mobileNodes", "", mobileNodes);
  cmd.AddValue("outFile", "", outFile);
  cmd.AddValue("propagationRange", "", propagationRange);
  cmd.AddValue("rngSeed", "", rngSeed);
  cmd.AddValue("scenarioSide", "", squareSide);
  cmd.AddValue("simulationTime", "", simulationTime);
  cmd.Parse(argc, argv);

  NS_ASSERT(mobileNodes > 0);
  NS_ASSERT(propagationRange > 0);
  NS_ASSERT(squareSide > 0);
  NS_ASSERT(interHelloTime > 0);

  std::ofstream results;
  results.open(outFile, ofstream::out | ofstream::app);

  const double monospacedDistance = squareSide / (fixedNodesPerSide - 1);
  const uint32_t fixedNodes = fixedNodesPerSide * 4 - 4;

  ns3::RngSeedManager::SetSeed(rngSeed);

  csmaNodes.Create(fixedNodes);

  CsmaHelper csma;
  csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
  csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));

  NetDeviceContainer csmaDevices;
  csmaDevices = csma.Install(csmaNodes);

  wifiStaNodes.Create(mobileNodes);

  uint32_t* accessPointIndex = GetNodeIndexById(csmaNodes, ACCESS_POINT_ID);
  if (!accessPointIndex) {
    cout << "Error: Node " << ACCESS_POINT_ID << " As AP" << endl;
    return -1;
  }
  wifiApNode = csmaNodes.Get(*accessPointIndex);
  cout << "Access Point ID: " << ACCESS_POINT_ID << endl;

  YansWifiChannelHelper channel;
  channel = YansWifiChannelHelper();
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(propagationRange));

  YansWifiPhyHelper phy;
  phy = YansWifiPhyHelper::Default();
  phy.SetChannel(channel.Create());

  WifiHelper wifi;
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("DsssRate1Mbps"), "ControlMode", StringValue("DsssRate1Mbps"));
  wifi.SetStandard(WIFI_PHY_STANDARD_80211b);

  WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  NetDeviceContainer staDevices;
  NetDeviceContainer apDevices;
  staDevices = wifi.Install(phy, mac, wifiStaNodes);
  apDevices = wifi.Install(phy, mac, wifiApNode);

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(csmaNodes);

  vector<pair<double, double>> positions = GeneratePerimeterPositions(squareSide, monospacedDistance);
  for (uint32_t i = 0; i < positions.size(); i++) {
    csmaNodes.Get(i)->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(positions[i].first, positions[i].second, 0));
  }

  const double minX = 0;
  const double minY = 0;
  const double maxX = squareSide;
  const double maxY = squareSide;

  Ptr<PositionAllocator> m_position;
  m_position = CreateObjectWithAttributes<RandomRectanglePositionAllocator>("X", StringValue("ns3::UniformRandomVariable[Min=" + to_string(minX) + "|Max=" + to_string(maxX) + "]"), "Y", StringValue("ns3::UniformRandomVariable[Min=" + to_string(minY) + "|Max=" + to_string(maxY) + "]"));
  
  Ptr<ListPositionAllocator> list1;
  Ptr<ListPositionAllocator> list2;
  list1 = CreateObjectWithAttributes<ListPositionAllocator>();
  list2 = CreateObjectWithAttributes<ListPositionAllocator>();
  for (uint32_t i = 0; i < mobileNodes; i++) {
    Vector v = m_position->GetNext();
    list1->Add(v);
    list2->Add(v);
  }
  
  mobility.SetPositionAllocator(list1);
  const double boundSide = 0.01 * squareSide * 2;
  const double minSpeed = 0.01 * squareSide;
  const double maxSpeed = 0.1 * squareSide;
  for (NodeContainer::Iterator j = wifiStaNodes.Begin(); j != wifiStaNodes.End(); j++) {
    Ptr<Node> object = *j;
    Vector position = list2->GetNext();
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue(Rectangle(position.x - boundSide / 2, position.x + boundSide / 2, position.y - boundSide / 2, position.y + boundSide / 2)), "Mode", EnumValue(RandomWalk2dMobilityModel::MODE_TIME), "Speed", StringValue("ns3::UniformRandomVariable[Min=" + to_string(minSpeed) + "|Max=" + to_string(maxSpeed) + "]"));
    mobility.Install(object);
  }

  InternetStackHelper stack;
  stack.Install(csmaNodes);
  stack.Install(wifiStaNodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");

  Ipv4InterfaceContainer wifiInterfaces = ipv4.Assign(staDevices);

  ipv4.Assign(apDevices);
  ipv4.SetBase("15.1.1.0", "255.255.255.0");
  ipv4.Assign(csmaDevices);

  InetSocketAddress localFixed = InetSocketAddress(Ipv4Address::GetAny(), CSMA_PORT);

  for (uint32_t i = 0; i < fixedNodes; i++) {
    if (csmaNodes.Get(i)->GetId() == ACCESS_POINT_ID) {
      continue;
    }
    Ptr<Socket> recvSocket = Socket::CreateSocket(csmaNodes.Get(i), tid);
    recvSocket->Bind(localFixed);
    recvSocket->SetRecvCallback(MakeCallback(&CSMA_ReceivePacket));
  }

  onPeriodExpDistrib->SetAttribute("Mean", DoubleValue(MEAN_ON_TIME));
  onPeriodExpDistrib->SetAttribute("Bound", DoubleValue(EXP_BOUND));
  offPeriodExpDistrib->SetAttribute("Mean", DoubleValue(MEAN_OFF_TIME));
  offPeriodExpDistrib->SetAttribute("Bound", DoubleValue(EXP_BOUND));

  InetSocketAddress localHelloMobile = InetSocketAddress(Ipv4Address::GetAny(), HELLO_PORT);
  InetSocketAddress remoteHelloMobile = InetSocketAddress(Ipv4Address("255.255.255.255"), HELLO_PORT);
  InetSocketAddress localTrafficMobile = InetSocketAddress(Ipv4Address::GetAny(), TRAFFIC_PORT);

  uint32_t sourceID = csmaNodes.GetN();
  uint32_t* sourceIndex = GetNodeIndexById(wifiStaNodes, sourceID);
  if (!sourceIndex) {
    cerr << "No Matching Node With ID " << sourceID << endl;
    sourceID = wifiStaNodes.Get(0)->GetId();
    sourceIndex = new uint32_t;
    *sourceIndex = 0;
  }
  cout << "Source ID: " << sourceID << endl;

  uint32_t i;
  for (i = 0; i < mobileNodes; i++) {
    Ptr<Socket> recvHelloSocket = Socket::CreateSocket(wifiStaNodes.Get(i), tid);
    recvHelloSocket->Bind(localHelloMobile);
    recvHelloSocket->SetRecvCallback(MakeCallback(&WirelessReceivePacket));
    Ptr<Socket> recvTrafficSocket = Socket::CreateSocket(wifiStaNodes.Get(i), tid);
    recvTrafficSocket->Bind(localTrafficMobile);
    recvTrafficSocket->SetRecvCallback(MakeCallback(&WirelessReceivePacket));
    Ptr<Socket> sendHelloSocket = Socket::CreateSocket(wifiStaNodes.Get(i), tid);
    sendHelloSocket->SetAllowBroadcast(true);
    sendHelloSocket->Connect(remoteHelloMobile);
    Simulator::ScheduleWithContext(wifiStaNodes.Get(i)->GetId(), MilliSeconds(i + 10), SendHello, sendHelloSocket, MilliSeconds(interHelloTime));
    if (wifiStaNodes.Get(i)->GetId() == sourceID) {
      continue;
    }
    pair<Ptr<Ipv4>, uint32_t> returnValue = wifiInterfaces.Get(i);
    Ptr<Ipv4> ipv4 = returnValue.first;
    uint32_t ifaceIndex = returnValue.second;
    Ptr<Ipv4Interface> iface = ipv4->GetObject<Ipv4L3Protocol>()->GetInterface(ifaceIndex);
    Simulator::ScheduleWithContext(wifiStaNodes.Get(i)->GetId(), MilliSeconds(i), SwitchInterface, iface, true, wifiStaNodes.Get(i)->GetId());
  }

  Ptr<Socket> LRASink = Socket::CreateSocket(wifiApNode.Get(0), tid);
  LRASink->Bind(localTrafficMobile);
  LRASink->SetRecvCallback(MakeCallback(&WirelessReceivePacket));

  Ptr<Socket> sendHelloSocket = Socket::CreateSocket(wifiApNode.Get(0), tid);
  sendHelloSocket->SetAllowBroadcast(true);
  sendHelloSocket->Connect(remoteHelloMobile);
  sendHelloSocket->BindToNetDevice(wifiApNode.Get(0)->GetDevice(1));

  Simulator::ScheduleWithContext(wifiApNode.Get(0)->GetId(), MilliSeconds(i + 10), SendHello, sendHelloSocket, MilliSeconds(interHelloTime));

  Simulator::ScheduleWithContext(sourceID, MilliSeconds(500), GenerateTraffic, sourceID, *sourceIndex, MilliSeconds(INTER_TRAFFIC_TIME));

  string animationName = "./scratch/.Animations/DisasterRecovery[Nodes=" + to_string(mobileNodes) + "_Range=" + to_string(propagationRange) + "_Side=" + to_string(squareSide) + "_Seed=" + to_string(rngSeed) + "_Hello=" + to_string(interHelloTime) +"].xml";
  AnimationInterface anim(animationName);
  anim.UpdateNodeDescription(ACCESS_POINT_ID, "AP");
  anim.UpdateNodeDescription(sourceID, "Source");

  Simulator::Stop(Seconds(simulationTime));
  Simulator::Run();
  Simulator::Destroy();

  cout << endl << endl << "Sent Packets: " << sentPackets << endl;
  cout << "Received Packets: " << receivedPackets << endl;

  results << interHelloTime << ";" << mobileNodes << ";" << propagationRange << ";" << squareSide * squareSide << ";" << rngSeed << ";" << sentPackets << ";" << receivedPackets << "\n";
  results.close();

  return 0;
  
}