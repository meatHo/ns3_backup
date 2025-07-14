// RSU (UE) <--> OBU (UE) : PC5
// RSU <--> gNB : Uu
// gNB + Core network present, RSU registered via gNB

// gnb설정 먼지

#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

int
main(int argc, char* argv[])
{
    Time simTime = Seconds(10);
    double centralFreq = 3.5e9;
    uint16_t bandwidth = 400; // 40 MHz

    NodeContainer gnbNode, rsuNode, obuNode;
    gnbNode.Create(1);
    rsuNode.Create(1);
    obuNode.Create(1);

    // Mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel"); // 움직이지 않음

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    // gnb-------rsu-------obu
    posAlloc->Add(Vector(0, 0, 1.5));   // gNB
    posAlloc->Add(Vector(0, 50, 1.5));  // RSU
    posAlloc->Add(Vector(0, 100, 1.5)); // OBU
    mobility.SetPositionAllocator(posAlloc);
    mobility.Install(gnbNode);
    mobility.Install(rsuNode);
    mobility.Install(obuNode);

    // NR Helper
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);

    // Bandwidth part and frequency configuration
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFreq,
                                                   bandwidth,
                                                   1,
                                                   BandwidthPartInfo::V2V_Highway);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    nrHelper->InitializeOperationBand(&band);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // gNB Device
    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNode, allBwps);
    nrHelper->GetGnbPhy(gnbDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(30.0));

    // RSU and OBU act as UE
    NetDeviceContainer ueDevices;
    ueDevices.Add(nrHelper->InstallUeDevice(rsuNode, allBwps));
    ueDevices.Add(nrHelper->InstallUeDevice(obuNode, allBwps));

    // Install and get the pointers to the NetDevices
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(obuNode, allBwps);

    // Attach RSU to gNB (Uu link)
    nrHelper->AttachToClosestEnb(ueNetDev, gnbDev);

    // PC5 (Sidelink) 설정
    Ptr<NrSlHelper> slHelper = CreateObject<NrSlHelper>();
    slHelper->SetEpcHelper(nrEpcHelper);
    slHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    slHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(10));
    slHelper->PrepareUeForSidelink(ueDevices, {0});

    // IPv4 stack
    InternetStackHelper stack;
    stack.Install(rsuNode);
    stack.Install(obuNode);
    nrEpcHelper->AssignUeIpv4Address(ueDevices);

    // Traffic application (RSU to OBU)
    uint16_t port = 5000;
    OnOffHelper client("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address("225.0.0.0"), port));
    client.SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
    client.SetAttribute("PacketSize", UintegerValue(200));
    // client.Install(rsuNode.Get(0))->SetStartTime(Seconds(2.0));

    ApplicationContainer clientApps;

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    // sink.Install(obuNode.Get(0))->SetStartTime(Seconds(1.5));

    Simulator::Stop(simTime);
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
