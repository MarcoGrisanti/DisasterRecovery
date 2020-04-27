#include <algorithm>
#include <climits>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ns3;
using namespace std;

namespace dr {

#define ACCESS_POINT_ID 13
#define AFTER_HELLO_TOLERANCE 100
#define CSMA_PORT 2000
#define EXAMPLE_MESSAGE "Example:"
#define EXP_BOUND 0
#define HELLO_MESSAGE "Hello:"
#define HELLO_PORT 80
#define INTER_HELLO_TIME 1000
#define INTER_TRAFFIC_TIME 1000
#define MEAN_OFF_TIME 0.5
#define MEAN_ON_TIME 5
#define MOBILE_NODES 30
#define PROPAGATION_RANGE 1000
#define RCV_BUFFER_SIZE 1024
#define RESULTS_FILENAME "Simulation_Results.csv"
#define RNG_SEED 1234
#define SCENARIO_SIDE 500
#define SIMULATION_TIME 10
#define TRAFFIC_PORT 3000

unordered_map<uint32_t, vector<uint32_t>> graph;
unordered_map<uint32_t, bool> receivedSequenceNumbers;
unordered_map<uint32_t, unordered_map<uint32_t, EventId>> timeouts;

double interHelloTime;
const TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

NodeContainer csmaNodes;
NodeContainer wifiApNode;
NodeContainer wifiStaNodes;

Ptr<ExponentialRandomVariable> onPeriodExpDistrib = CreateObject<ExponentialRandomVariable>();
Ptr<ExponentialRandomVariable> offPeriodExpDistrib = CreateObject<ExponentialRandomVariable>();

ostream& operator<<(ostream& os, vector<uint32_t> v) {
  for (vector<uint32_t>::const_iterator i = v.begin(); i != v.end(); ++i) {
    os << *i << ' ';
  }
  return os;
}

static void ClearLink(uint32_t remoteNodeID, uint32_t localNodeID) {
  uint32_t index = 0;
  for (vector<uint32_t>::iterator it = graph[localNodeID].begin(); it != graph[localNodeID].end(); ++it) {
    if (*it == remoteNodeID) {
      graph[localNodeID].erase(graph[localNodeID].begin() + index);
      return;
    }
    index++;
  }
}

void checkDAG(uint32_t realSource, uint32_t sourceID, bool& checked) {
  vector<uint32_t> sourceList = graph[sourceID];
  if (sourceList.size() == 0) {
    return;
  }
  for (uint32_t i = 0; i < sourceList.size(); i++) {
    if (graph[sourceID][i] == realSource) {
      checked = false;
      return;
    }
    checkDAG(realSource, graph[sourceID][i], checked);
  }
}

static void CSMA_Forward(Ptr<Packet> packet) {
  Ptr<Socket> newSocket = Socket::CreateSocket(wifiApNode.Get(0), tid);
  newSocket->BindToNetDevice(wifiApNode.Get(0)->GetDevice(0));  // Bind con la sola interfaccia dell'access point sulla rete CSMA
  InetSocketAddress remote = InetSocketAddress(Ipv4Address("255.255.255.255"), CSMA_PORT);
  newSocket->SetAllowBroadcast(true);
  newSocket->Connect(remote);
  newSocket->Send(packet);
  newSocket->Close();
}

uint32_t* GetNodeIndexById(NodeContainer& container, uint32_t id) {
  for (uint32_t i = 0; i < container.GetN(); i++) {
    if (container.Get(i)->GetId() == id) {
      uint32_t* value = new uint32_t;
      *value = i;
      return value;
    }
  }
  return NULL;
}

static void ForwardPacket(Ptr<Packet> packet, uint32_t localNodeID, Ipv4Address senderIP) {
  if (graph[localNodeID].size() > 0) {
    for (vector<uint32_t>::iterator it = graph[localNodeID].begin(); it != graph[localNodeID].end(); ++it) {
      Ptr<Socket> newSocket = Socket::CreateSocket(wifiStaNodes.Get(localNodeID - csmaNodes.GetN()), tid);
      Ptr<Node> dest;
      if (*it == ACCESS_POINT_ID) {
        dest = wifiApNode.Get(0);
      }
      else {
        dest = wifiStaNodes.Get(*it - csmaNodes.GetN());
      }
      Ipv4Address adr = dest->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
      InetSocketAddress remote = InetSocketAddress(adr, TRAFFIC_PORT);
      newSocket->Connect(remote);
      newSocket->Send(packet);
      newSocket->Close();
    }
  }
  else {
    for (auto& pair : graph) {
      if (pair.first == localNodeID) {
        continue;
      }
      auto it = find(graph[pair.first].begin(), graph[pair.first].end(), localNodeID);
      if (it != graph[pair.first].end()) {
        auto index = distance(graph[pair.first].begin(), it);
        graph[pair.first].erase(graph[pair.first].begin() + index);
        graph[localNodeID].push_back(pair.first);
      }
    }
    if (graph[localNodeID].size() > 0) {
      ForwardPacket(packet, localNodeID, senderIP);
    }
  }
}

vector<pair<double, double>> GeneratePerimeterPositions(double squareSide, double monospacedDistance) {
  vector<pair<double, double>> positions;
  pair<double, double> couple;
  double x = 0;
  while (x <= squareSide) {
    couple.first = x;
    couple.second = 0;
    positions.push_back(couple);
    couple.first = x;
    couple.second = squareSide;
    positions.push_back(couple);
    x = x + monospacedDistance;
  }
  double y = monospacedDistance;
  while (y <= squareSide - monospacedDistance) {
    couple.first = 0;
    couple.second = y;
    positions.push_back(couple);
    couple.first = squareSide;
    couple.second = y;
    positions.push_back(couple);
    y = y + monospacedDistance;
  }
  return positions;
}

void PrintMap(unordered_map<uint32_t, vector<uint32_t>>& map) {
  for (auto const& pair : map) {
    cout << "{" << pair.first << ": " << pair.second << "}\n";
  }
}

static void UpdateGraph(string message, uint32_t localNodeID, uint32_t interHelloTime) {
  uint32_t remoteNodeID = stoi(message.substr(message.find(":") + 1));
  if (count(graph[remoteNodeID].begin(), graph[remoteNodeID].end(), localNodeID) > 0) {
    return;
  }
  else {
    if (count(graph[localNodeID].begin(), graph[localNodeID].end(), remoteNodeID) == 0) {
      graph[localNodeID].push_back(remoteNodeID);
    }
    timeouts[localNodeID][remoteNodeID].Cancel();
    timeouts[localNodeID][remoteNodeID] = Simulator::Schedule(MilliSeconds(interHelloTime + AFTER_HELLO_TOLERANCE), &ClearLink, remoteNodeID, localNodeID);
  }
}

}