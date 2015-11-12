#include "mynetwork.h"
#include "beaconapplication.h"

MyNetwork::MyNetwork(vns::RoadNetwork* network) : vns::NS3Network(network) {
	std::string phyMode("OfdmRate6MbpsBW10MHz");
	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", ns3::StringValue(phyMode), "ControlMode", ns3::StringValue(phyMode));
	wifi.SetStandard(ns3::WIFI_PHY_STANDARD_80211_10MHZ);

	wifiPhy = ns3::YansWifiPhyHelper::Default();
	wifiPhy.Set("EnergyDetectionThreshold", ns3::DoubleValue(-96.0));
	wifiPhy.Set("TxPowerStart", ns3::DoubleValue(10.0206));
	wifiPhy.Set("TxPowerEnd", ns3::DoubleValue(10.0206));

	ns3::YansWifiChannelHelper wifiChannel;

	wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "ReferenceLoss", ns3::DoubleValue(37.35));
	wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel", "m0", ns3::DoubleValue(1.5), "Distance1", ns3::DoubleValue(60), "m1",
									ns3::DoubleValue(0.75), "Distance2", ns3::DoubleValue(145), "m2", ns3::DoubleValue(0.0));

	wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

	wifiPhy.SetChannel(wifiChannel.Create());

	bool enableCtsRts = true;
	ns3::Config::SetDefault("ns3::Dcf::MinCw", ns3::StringValue("15"));
	ns3::Config::SetDefault("ns3::Dcf::MaxCw", ns3::StringValue("1023"));
	ns3::Config::SetDefault("ns3::Dcf::Aifsn", ns3::StringValue("9"));
	ns3::UintegerValue ctsThr = ( enableCtsRts ? ns3::UintegerValue(100) : ns3::UintegerValue(2200));

	ns3::Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
	ns3::Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", ns3::StringValue("2200"));
	ns3::Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", ns3::StringValue(phyMode));

	wifiMac = ns3::NqosWifiMacHelper::Default();
	wifiMac.SetType("ns3::AdhocWifiMac");
}

MyNetwork::~MyNetwork() {

}

void MyNetwork::onVehicleCreated(const vns::Simulator*, ns3::Ptr<ns3::Node> node) {
	ns3::NetDeviceContainer devs = wifi.Install(wifiPhy, wifiMac, node);
	internet.Install(node);

	ns3::Ptr<BeaconApplication> app = ns3::CreateObject<BeaconApplication>();
	node->AddApplication(app);
}

void MyNetwork::onVehicleRemoved(const vns::Simulator*, ns3::Ptr<ns3::Node> node) {
	ns3::Ptr<BeaconApplication> app = ns3::Ptr<BeaconApplication>(dynamic_cast<BeaconApplication*>(ns3::PeekPointer(node->GetApplication(0))));
	app->stop();
}


