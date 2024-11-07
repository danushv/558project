#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/energy-module.h"
#include <map>
#include <vector>
#include <cmath>

using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE("LeachDmsNetworkSimulation");

// Cluster structure with backup cluster head
struct Cluster {
    Ptr<Node> clusterHead;
    Ptr<Node> backupHead; // Backup cluster head
    std::vector<Ptr<Node>> members;
};

std::map<uint32_t, Cluster> clusters;
std::map<uint32_t, double> nodeEnergyLevels; // Map to track energy levels manually
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
void CheckNodeFailure(NodeContainer nodes);
void ScheduleFailureCheck(NodeContainer nodes);
Ptr<Node> FindNodeWithHighEnergy(const std::vector<Ptr<Node>>& members);

// Initialize energy levels for each node
void InitializeNodeEnergyLevels(NodeContainer nodes) {
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> node = *it;
        nodeEnergyLevels[node->GetId()] = initialEnergy;
    }
}

// Function to log and update energy level
void UpdateEnergy(uint32_t nodeId, double energyUsed) {
    if (nodeEnergyLevels.find(nodeId) != nodeEnergyLevels.end()) {
        nodeEnergyLevels[nodeId] = std::max(0.0, nodeEnergyLevels[nodeId] - energyUsed);
        
        // Only log energy levels below 10 J to reduce log size
        if (nodeEnergyLevels[nodeId] < 10.0) {
            NS_LOG_INFO("Node " << nodeId << " energy level: " << nodeEnergyLevels[nodeId] << " J");
        }
    }
}

// DMS Function to determine transmission power based on distance
double CalculateTransmissionPower(double distance) {
    double basePower = 1.0; // Base power level
    if (distance > 50.0) {
        return basePower * 1.5; // Higher power for long distances
    } else if (distance > 20.0) {
        return basePower * 1.2; // Moderate power for medium distances
    } else {
        return basePower * 0.8; // Lower power for short distances
    }
}

// Clear previous clusters
void ClearClusters() {
    clusters.clear();
}

// Elect cluster heads with backup designation
void ElectClusterHeads(NodeContainer nodes) {
    ClearClusters();
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> node = *it;
        
        if ((double)rand() / RAND_MAX <= clusterHeadProbability && nodeEnergyLevels[node->GetId()] > 10.0) {
            Cluster newCluster;
            newCluster.clusterHead = node;
            clusters[node->GetId()] = newCluster;

            // Select a backup cluster head with high energy within the cluster
            Ptr<Node> backup = FindNodeWithHighEnergy(newCluster.members);
            if (backup) {
                clusters[node->GetId()].backupHead = backup;
            }

            NS_LOG_INFO("Node " << node->GetId() << " elected as cluster head with backup: "
                                << (backup ? backup->GetId() : -1));
        }
    }
}

// Utility function to find a node with high energy to act as backup
Ptr<Node> FindNodeWithHighEnergy(const std::vector<Ptr<Node>>& members) {
    Ptr<Node> backupNode = nullptr;
    double maxEnergy = 0.0;
    for (auto& member : members) {
        double memberEnergy = nodeEnergyLevels[member->GetId()];
        if (memberEnergy > maxEnergy) {
            maxEnergy = memberEnergy;
            backupNode = member;
        }
    }
    return backupNode;
}

// Form clusters around cluster heads
void FormClusters(NodeContainer nodes) {
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> node = *it;
        if (clusters.find(node->GetId()) == clusters.end()) {  // Only add non-cluster-head nodes
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

// Intra-cluster communication with DMS applied
void IntraClusterCommunication(Ptr<Node> memberNode, Ptr<Node> clusterHead) {
    double distance = memberNode->GetObject<MobilityModel>()->GetDistanceFrom(clusterHead->GetObject<MobilityModel>());
    double txPower = CalculateTransmissionPower(distance);

    NS_LOG_INFO("Node " << memberNode->GetId() << " sends data to Cluster Head " << clusterHead->GetId()
                << " with power level: " << txPower);

    // Deduct energy based on transmission power
    UpdateEnergy(memberNode->GetId(), 0.1 * txPower); // Deduct energy based on power level

    Simulator::Schedule(Seconds(1.0), &IntraClusterCommunication, memberNode, clusterHead);
}

// Inter-cluster communication with DMS applied
void InterClusterCommunication(Ptr<Node> clusterHead, Ptr<Node> baseStation) {
    double distance = clusterHead->GetObject<MobilityModel>()->GetDistanceFrom(baseStation->GetObject<MobilityModel>());
    double txPower = CalculateTransmissionPower(distance);

    NS_LOG_INFO("Cluster Head " << clusterHead->GetId() << " sends aggregated data to Base Station with power level: " 
                << txPower);

    // Deduct energy based on transmission power
    UpdateEnergy(clusterHead->GetId(), 0.2 * txPower); // Deduct energy based on power level

    Simulator::Schedule(Seconds(5.0), &InterClusterCommunication, clusterHead, baseStation);
}

// Schedule cluster formation
void ScheduleClusterFormation(NodeContainer nodes) {
    Simulator::Schedule(Seconds(20.0), &ElectClusterHeads, nodes);
    Simulator::Schedule(Seconds(20.0), &FormClusters, nodes);
    Simulator::Schedule(Seconds(20.0), &SetupClusterCommunications, nodes, nodes.Get(0)); // Base station as node 0
    Simulator::Schedule(Seconds(20.0), &ScheduleClusterFormation, nodes);
}

// Set up communication within and between clusters
void SetupClusterCommunications(NodeContainer nodes, Ptr<Node> baseStation) {
    for (auto& entry : clusters) {
        Ptr<Node> clusterHead = entry.second.clusterHead;
        
        for (Ptr<Node> member : entry.second.members) {
            IntraClusterCommunication(member, clusterHead);
        }

        InterClusterCommunication(clusterHead, baseStation);
    }
}

// Simulate node failure by removing failed nodes from clusters
void CheckNodeFailure(NodeContainer nodes) {
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> node = *it;
        uint32_t nodeId = node->GetId();

        // Simulate failure if energy is below threshold
        if (nodeEnergyLevels[nodeId] < 5.0) {
            NS_LOG_INFO("Node " << nodeId << " has failed due to low energy.");
            for (auto &entry : clusters) {
                auto &members = entry.second.members;
                members.erase(std::remove(members.begin(), members.end(), node), members.end());
            }
        }
    }
}

// Periodically check for node failures
void ScheduleFailureCheck(NodeContainer nodes) {
    Simulator::Schedule(Seconds(10.0), &CheckNodeFailure, nodes);
    Simulator::Schedule(Seconds(10.0), &ScheduleFailureCheck, nodes);  // Re-schedule to keep checking
}

// Basic node setup
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

// Set node mobility
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

    InitializeNodeEnergyLevels(sensorNodes); // Initialize manual energy tracking

    ScheduleClusterFormation(sensorNodes);
    ScheduleFailureCheck(sensorNodes); // Schedule periodic failure checks

    Simulator::Stop(Seconds(600.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
