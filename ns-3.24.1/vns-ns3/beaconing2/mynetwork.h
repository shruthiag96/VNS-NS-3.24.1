#ifndef MYNETWORK_H_
#define MYNETWORK_H_

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

#include "vns/vns.h"
#include "vns/ns3networkmodule.h"

class MyNetwork : public vns::NS3Network {
private:

	ns3::NetDeviceContainer devices;
  	ns3::InternetStackHelper internet;
  	ns3::Ipv4AddressHelper ipv4;
	ns3::NqosWifiMacHelper wifiMac;

	ns3::YansWifiPhyHelper wifiPhy;
	ns3::WifiHelper wifi;

public:

	MyNetwork(vns::RoadNetwork* network);
	virtual ~MyNetwork();

	void onVehicleCreated(const vns::Simulator* simulator, ns3::Ptr<ns3::Node> node);
 	void onVehicleRemoved(const vns::Simulator* simulator, ns3::Ptr<ns3::Node>  node);
};

#endif
