#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/energy-module.h"
#include <fstream>

using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE("LeachDmsNetworkSimulation");

// Function to set transmission power based on DMS
double SetTransmissionPower(double distance, bool isHighPriority) {
    double basePower = 1.0;  // Base transmission power level
    if (isHighPriority) {
        return basePower * 1.5;  // Higher power for urgent data
    } else if (distance > 50.0) {
        return basePower * 1.2;  // Moderate power for longer distances
    } else {
        return basePower * 0.8;  // Low power for short distances
    }
}

// Function to log energy level with a check for significant drop (5%)
void LogEnergyLevel(Ptr<Node> node, Ptr<BasicEnergySource> energySource, double *lastReportedEnergy) {
    double currentEnergy = energySource->GetRemainingEnergy();
    double thresholdDrop = *lastReportedEnergy * 0.05;

    // Report only if there is a significant drop (5%)
    if (*lastReportedEnergy - currentEnergy >= thresholdDrop) {
        NS_LOG_INFO("Time: " << Simulator::Now().GetSeconds() 
                              << "s, Node " << node->GetId() 
                              << " Energy: " << currentEnergy << "J");
        
        // Log energy to a file
        std::ofstream energyLog("energy_log.txt", std::ios::app);
        energyLog << "Time: " << Simulator::Now().GetSeconds()
                  << "s, Node " << node->GetId() 
                  << " Energy: " << currentEnergy << "J" << std::endl;
        energyLog.close();

        // Update the last reported energy level
        *lastReportedEnergy = currentEnergy;
    }

    // Schedule the next energy log check
    Simulator::Schedule(Seconds(1.0), &LogEnergyLevel, node, energySource, lastReportedEnergy);
}

// Function to check if a node's energy is depleted
void CheckNodeEnergyDepletion(Ptr<Node> node, Ptr<BasicEnergySource> energySource) {
    if (energySource->GetRemainingEnergy() <= 0) {
        NS_LOG_INFO("Node " << node->GetId() << " has depleted its energy at time: " 
                            << Simulator::Now().GetSeconds() << "s");
    } else {
        // Schedule the next check
        Simulator::Schedule(Seconds(1.0), &CheckNodeEnergyDepletion, node, energySource);
    }
}

void SetupNodes(NodeContainer nodes, NetDeviceContainer& devices) {
    // Install WiFi and IP on nodes
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");  // Ad-hoc mode for direct communication

    devices = wifi.Install(wifiPhy, wifiMac, nodes);

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    address.Assign(devices);
}

void SetupEnergyModel(NodeContainer nodes, NetDeviceContainer devices) {
    // Configure energy sources and consumption models
    BasicEnergySourceHelper energySourceHelper;
    energySourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100.0));

    EnergySourceContainer energySources = energySourceHelper.Install(nodes);

    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.017));  // Transmit current
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.019));  // Receive current
    radioEnergyHelper.Install(devices, energySources);

    // Schedule energy logging and depletion checks for each node
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<Node> node = *it;
        Ptr<BasicEnergySource> energySource = DynamicCast<BasicEnergySource>(energySources.Get(node->GetId()));
        
        // Initialize the last reported energy level with the initial energy level
        double initialEnergy = energySource->GetInitialEnergy();
        Simulator::Schedule(Seconds(1.0), &LogEnergyLevel, node, energySource, new double(initialEnergy));
        
        // Check energy depletion
        Simulator::Schedule(Seconds(1.0), &CheckNodeEnergyDepletion, node, energySource);
    }
}

void SetMobility(NodeContainer nodes) {
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        positionAlloc->Add(Vector(10.0 * i, 10.0 * i, 0.0));  // Arrange in a grid
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
}

void SimulateLeachProtocol(NodeContainer nodes) {
    // Placeholder function to simulate LEACH behavior
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<Node> node = nodes.Get(i);
        double distanceToBaseStation = 50.0 + 10.0 * i;  // Example distance
        bool isHighPriority = (i % 5 == 0);  // Assign priority based on node index

        double txPower = SetTransmissionPower(distanceToBaseStation, isHighPriority);
        NS_LOG_INFO("Node " << node->GetId() << ": Transmission power set to " << txPower);
    }
}

int main(int argc, char *argv[]) {
    LogComponentEnable("LeachDmsNetworkSimulation", LOG_LEVEL_INFO);

    // Set the simulation end time (e.g., 600 seconds)
    Simulator::Stop(Seconds(600.0));

    // Step 1: Create and configure nodes
    NodeContainer sensorNodes;
    sensorNodes.Create(10);
    NS_LOG_INFO("Creating sensor nodes...");

    NetDeviceContainer devices;
    SetupNodes(sensorNodes, devices);      // Configure WiFi and IP
    SetupEnergyModel(sensorNodes, devices); // Add energy models
    SetMobility(sensorNodes);               // Set mobility

    // Step 2: Simulate LEACH with DMS
    NS_LOG_INFO("Simulating LEACH protocol with DMS...");
    SimulateLeachProtocol(sensorNodes);

    // Step 3: Run simulation
    Simulator::Run();
    Simulator::Destroy();
    NS_LOG_INFO("Simulation complete.");

    return 0;
}
