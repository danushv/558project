#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/energy-module.h"
#include <map>
#include <vector>
#include <fstream>

using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE("LeachDmsNetworkSimulation");

struct Cluster {
    Ptr<Node> clusterHead;
    std::vector<Ptr<Node>> members;
};

std::map<uint32_t, Cluster> clusters;
double clusterHeadProbability = 0.2;
std::map<uint32_t, double> lastReportedEnergy;

// Declare functions at the beginning
void ClearClusters();
void ElectClusterHeads(NodeContainer nodes, EnergySourceContainer energySources);
void FormClusters(NodeContainer nodes);
void SetupClusterCommunications(NodeContainer nodes, Ptr<Node> baseStation);
void LogEnergyLevels(EnergySourceContainer energySources);
void ScheduleClusterFormation(NodeContainer nodes, EnergySourceContainer energySources);
void BatchIntraClusterCommunication(NodeContainer nodes);
void BatchInterClusterCommunication(NodeContainer nodes, Ptr<Node> baseStation);
void SetupNodes(NodeContainer nodes, NetDeviceContainer& devices);
EnergySourceContainer SetupEnergyModel(NodeContainer nodes, NetDeviceContainer devices);
void SetMobility(NodeContainer nodes);

// Clear previous clusters
void ClearClusters() {
    clusters.clear();
}

// Elect cluster heads
void ElectClusterHeads(NodeContainer nodes, EnergySourceContainer energySources) {
    ClearClusters();

    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> node = *it;
        Ptr<BasicEnergySource> energySource = DynamicCast<BasicEnergySource>(energySources.Get(node->GetId()));
        
        if ((double)rand() / RAND_MAX <= clusterHeadProbability && energySource->GetRemainingEnergy() > 10.0) {
            Cluster newCluster;
            newCluster.clusterHead = node;
            clusters[node->GetId()] = newCluster;
            NS_LOG_INFO("Node " << node->GetId() << " elected as cluster head with energy: " 
                                << energySource->GetRemainingEnergy());
        }
    }
}

// Form clusters
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

// Batch intra-cluster communication
void BatchIntraClusterCommunication(NodeContainer nodes) {
    for (auto& entry : clusters) {
        Ptr<Node> clusterHead = entry.second.clusterHead;
        uint32_t transmissionCount = 0;
        
        for (Ptr<Node> member : entry.second.members) {
            transmissionCount++;
        }
        
        if (transmissionCount > 0) {
            NS_LOG_INFO("Cluster Head " << clusterHead->GetId() << " received " << transmissionCount 
                         << " transmissions from its members.");
        }
    }
    Simulator::Schedule(Seconds(20.0), &BatchIntraClusterCommunication, nodes); // Schedule next report
}

// Batch inter-cluster communication
void BatchInterClusterCommunication(NodeContainer nodes, Ptr<Node> baseStation) {
    uint32_t clusterHeadTransmissionCount = clusters.size();
    NS_LOG_INFO("Base Station received data from " << clusterHeadTransmissionCount << " cluster heads.");
    Simulator::Schedule(Seconds(20.0), &BatchInterClusterCommunication, nodes, baseStation); // Schedule next report
}

// Schedule cluster formation and periodic reporting
void ScheduleClusterFormation(NodeContainer nodes, EnergySourceContainer energySources) {
    Simulator::Schedule(Seconds(20.0), &ElectClusterHeads, nodes, energySources);
    Simulator::Schedule(Seconds(20.0), &FormClusters, nodes);
    Simulator::Schedule(Seconds(50.0), &LogEnergyLevels, energySources); // Log energy levels every 50 seconds
    Simulator::Schedule(Seconds(50.0), &ScheduleClusterFormation, nodes, energySources); // Schedule next formation
}

// Log energy levels only when there's a significant change
void LogEnergyLevels(EnergySourceContainer energySources) {
    for (uint32_t i = 0; i < energySources.GetN(); i++) {
        Ptr<BasicEnergySource> energySource = DynamicCast<BasicEnergySource>(energySources.Get(i));
        double currentEnergy = energySource->GetRemainingEnergy();

        if (lastReportedEnergy.find(i) == lastReportedEnergy.end() || 
            (lastReportedEnergy[i] - currentEnergy) >= (lastReportedEnergy[i] * 0.05)) {
            NS_LOG_INFO("Node " << i << " energy level: " << currentEnergy << " J");
            lastReportedEnergy[i] = currentEnergy;
        }
    }
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

// Energy model setup
EnergySourceContainer SetupEnergyModel(NodeContainer nodes, NetDeviceContainer devices) {
    BasicEnergySourceHelper energySourceHelper;
    energySourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100.0));

    EnergySourceContainer energySources = energySourceHelper.Install(nodes);

    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.017));
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.019));
    radioEnergyHelper.Install(devices, energySources);

    for (uint32_t i = 0; i < energySources.GetN(); i++) {
        lastReportedEnergy[i] = energySources.Get(i)->GetRemainingEnergy();
    }

    return energySources;
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
    EnergySourceContainer energySources = SetupEnergyModel(sensorNodes, devices);
    SetMobility(sensorNodes);

    ScheduleClusterFormation(sensorNodes, energySources);
    BatchIntraClusterCommunication(sensorNodes); // Start periodic intra-cluster logging
    BatchInterClusterCommunication(sensorNodes, baseStation); // Start periodic inter-cluster logging

    Simulator::Stop(Seconds(600.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
