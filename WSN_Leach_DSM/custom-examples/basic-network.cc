#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include <map>
#include <vector>
#include <cmath>

using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE("LeachDmsNetworkSimulation");

// Cluster structure and function declarations
struct Cluster {
    Ptr<Node> clusterHead;
    std::vector<Ptr<Node>> members;
};

std::map<uint32_t, Cluster> clusters;
std::map<uint32_t, double> nodeEnergyLevels;
double clusterHeadProbability = 0.2;
double initialEnergy = 100.0;

void ClearClusters();
void ElectClusterHeads(NodeContainer nodes);
void FormClusters(NodeContainer nodes);
void SetupClusterCommunications(NodeContainer nodes, Ptr<Node> baseStation);
void ScheduleClusterFormation(NodeContainer nodes);
void IntraClusterCommunication(Ptr<Node> memberNode, Ptr<Node> clusterHead);
void InterClusterCommunication(Ptr<Node> clusterHead, Ptr<Node> baseStation);
void SetupNodes(NodeContainer nodes, NetDeviceContainer& devices);
void InitializeNodeEnergyLevels(NodeContainer nodes);
void UpdateEnergy(uint32_t nodeId, double energyUsed);
double CalculateTransmissionPower(double distance);
void SetMobility(NodeContainer nodes);
void LogPeriodicEnergyLevels();

void InitializeNodeEnergyLevels(NodeContainer nodes) {
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> node = *it;
        nodeEnergyLevels[node->GetId()] = initialEnergy;
    }
}

void UpdateEnergy(uint32_t nodeId, double energyUsed) {
    if (nodeEnergyLevels.find(nodeId) != nodeEnergyLevels.end()) {
        nodeEnergyLevels[nodeId] = std::max(0.0, nodeEnergyLevels[nodeId] - energyUsed);
    }
}

double CalculateTransmissionPower(double distance) {
    double basePower = 1.0;
    if (distance > 50.0) {
        return basePower * 1.5;
    } else if (distance > 20.0) {
        return basePower * 1.2;
    } else {
        return basePower * 0.8;
    }
}

void ClearClusters() {
    clusters.clear();
}

void ElectClusterHeads(NodeContainer nodes) {
    ClearClusters();
    NS_LOG_INFO("Starting a new round of cluster head elections...");
    bool anyClusterHeadElected = false;
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> node = *it;
        
        if ((double)rand() / RAND_MAX <= clusterHeadProbability && nodeEnergyLevels[node->GetId()] > 10.0) {
            Cluster newCluster;
            newCluster.clusterHead = node;
            clusters[node->GetId()] = newCluster;
            NS_LOG_INFO("Node " << node->GetId() << " elected as cluster head with energy: " 
                                << nodeEnergyLevels[node->GetId()]);
            anyClusterHeadElected = true;
        }
    }
    if (!anyClusterHeadElected) {
        NS_LOG_INFO("No cluster heads elected this round.");
    }
}


void FormClusters(NodeContainer nodes) {
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> node = *it;
        if (clusters.find(node->GetId()) == clusters.end()) {
            double minDistance = std::numeric_limits<double>::max();
            Ptr<Node> closestClusterHead = nullptr;
            
            for (auto& entry : clusters) {
                Ptr<Node> clusterHead = entry.second.clusterHead;
                double distance = node->GetObject<MobilityModel>()->GetDistanceFrom(clusterHead->GetObject<MobilityModel>());
                
                if (distance < minDistance) {
                    minDistance = distance;
                    closestClusterHead = clusterHead;
                }
            }
            
            if (closestClusterHead) {
                clusters[closestClusterHead->GetId()].members.push_back(node);
                NS_LOG_INFO("Node " << node->GetId() << " joined cluster with head " << closestClusterHead->GetId());
            }
        }
    }
}

void IntraClusterCommunication(Ptr<Node> memberNode, Ptr<Node> clusterHead) {
    double distance = memberNode->GetObject<MobilityModel>()->GetDistanceFrom(clusterHead->GetObject<MobilityModel>());
    double txPower = CalculateTransmissionPower(distance);

    // Deduct energy based on transmission power
    UpdateEnergy(memberNode->GetId(), 0.1 * txPower);

    Simulator::Schedule(Seconds(1.0), &IntraClusterCommunication, memberNode, clusterHead);
}

void InterClusterCommunication(Ptr<Node> clusterHead, Ptr<Node> baseStation) {
    static int roundCounter = 0;
    roundCounter++;
    if (roundCounter % 5 == 0) { // Only log every 5 rounds to reduce output
        double distance = clusterHead->GetObject<MobilityModel>()->GetDistanceFrom(baseStation->GetObject<MobilityModel>());
        double txPower = CalculateTransmissionPower(distance);

        NS_LOG_INFO("Cluster Head " << clusterHead->GetId() 
                    << " sends aggregated data to Base Station with power level: " << txPower);

        // Deduct energy based on transmission power
        UpdateEnergy(clusterHead->GetId(), 0.2 * txPower);
    }
    Simulator::Schedule(Seconds(5.0), &InterClusterCommunication, clusterHead, baseStation);
}

void ScheduleClusterFormation(NodeContainer nodes) {
    Simulator::Schedule(Seconds(20.0), &ElectClusterHeads, nodes);
    Simulator::Schedule(Seconds(20.0), &FormClusters, nodes);
    Simulator::Schedule(Seconds(20.0), &SetupClusterCommunications, nodes, nodes.Get(0));
    Simulator::Schedule(Seconds(20.0), &ScheduleClusterFormation, nodes);
}

void SetupClusterCommunications(NodeContainer nodes, Ptr<Node> baseStation) {
    for (auto& entry : clusters) {
        Ptr<Node> clusterHead = entry.second.clusterHead;
        
        for (Ptr<Node> member : entry.second.members) {
            IntraClusterCommunication(member, clusterHead);
        }

        InterClusterCommunication(clusterHead, baseStation);
    }
}

void LogPeriodicEnergyLevels() {
    double totalEnergy = 0.0;
    for (const auto& entry : nodeEnergyLevels) {
        totalEnergy += entry.second;
    }
    double averageEnergy = totalEnergy / nodeEnergyLevels.size();
    NS_LOG_INFO("Average node energy level: " << averageEnergy << " J");
    Simulator::Schedule(Seconds(100.0), &LogPeriodicEnergyLevels);
}


void SetupNodes(NodeContainer nodes, NetDeviceContainer& devices) {
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    devices = wifi.Install(wifiPhy, wifiMac, nodes);

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    address.Assign(devices);
}

void SetMobility(NodeContainer nodes) {
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        positionAlloc->Add(Vector(10.0 * i, 10.0 * i, 0.0));
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
}

int main(int argc, char *argv[]) {
    LogComponentEnable("LeachDmsNetworkSimulation", LOG_LEVEL_INFO);

    NodeContainer sensorNodes;
    sensorNodes.Create(10);

    NodeContainer baseStationContainer;
    baseStationContainer.Create(1);
    Ptr<Node> baseStation = baseStationContainer.Get(0);

    NetDeviceContainer devices;
    SetupNodes(sensorNodes, devices);
    SetMobility(sensorNodes);

    InitializeNodeEnergyLevels(sensorNodes);
    LogPeriodicEnergyLevels(); // Start logging energy levels periodically

    ScheduleClusterFormation(sensorNodes);

    Simulator::Stop(Seconds(600.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
