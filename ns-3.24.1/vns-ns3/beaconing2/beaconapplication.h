#ifndef BEACONAPPLICATION_H_
#define BEACONAPPLICATION_H_

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

#include "beacon.h"
#include "myvehicle.h"


class BeaconApplication: public ns3::Application {
public:
	BeaconApplication() : ns3::Application() {
		timeOfLastReceivedPacket = 0.0;
		tnext = ns3::Time(ns3::Seconds(1.0));
	}
	virtual ~BeaconApplication() {
		if (socket != 0) {
			ns3::Simulator::Cancel(sendPacketEventID);
			socket->SetRecvCallback(ns3::MakeNullCallback<void, ns3::Ptr<ns3::Socket> >());
		}
		running = false;
	}
	static ns3::TypeId GetTypeId(void) {
		static ns3::TypeId tid = ns3::TypeId("BeaconApplication").SetParent<Application>().AddConstructor<BeaconApplication>();
		return tid;
	}
	void stop() {
		StopApplication();
	}
	void start() {
		StartApplication();
	}

protected:
	virtual void DoDispose(void) {
		ns3::Simulator::Cancel(sendPacketEventID);
		Application::DoDispose();
	}
	;

private:
	virtual void StartApplication(void) {

		running = true;
		if (socket == 0) {
			ns3::TypeId tid = ns3::TypeId::LookupByName("ns3::PacketSocketFactory");
			socket = ns3::Socket::CreateSocket(GetNode(), tid);
			ns3::PacketSocketAddress destAddr;
			destAddr.SetPhysicalAddress(ns3::Mac48Address("ff:ff:ff:ff:ff:ff"));
			destAddr.SetSingleDevice(0);
			//destAddr.SetProtocol(2);
			socket->Bind();
			socket->Connect(destAddr);
		}
		socket->SetRecvCallback(ns3::MakeCallback(&BeaconApplication::ReadSocket, this));
		ns3::Time nx(ns3::Seconds(vns::randomValue(0.0, 1.0)));
		sendPacketEventID = ns3::Simulator::Schedule(nx, &BeaconApplication::SendPacket, this);


	}
	virtual void StopApplication(void) {
		if (socket != 0) {
			ns3::Simulator::Cancel(sendPacketEventID);
			socket->SetRecvCallback(ns3::MakeNullCallback<void, ns3::Ptr<ns3::Socket> >());
		}
		running = false;
	}
	void ScheduleTx(void) {
		if (running) {
			ns3::Time tnext(ns3::Seconds(1.0));
			sendPacketEventID = ns3::Simulator::Schedule(tnext, &BeaconApplication::SendPacket, this);
		}

	}
	void SendPacket(void) {
		ns3::Ptr<ns3::Node> node = GetNode();
		if (node == 0) return;
		ns3::Ptr<vns::VNSMobilityModel> model = node->GetObject<vns::VNSMobilityModel>();
		if (model == 0) return;
		MyVehicle* v = dynamic_cast<MyVehicle*>(model->getVehicle());
		int32 vid = v->getID();

		Beacon beacon;
		beacon.SetId(vid);
		beacon.SetX(v->getPosition().getX());
		beacon.SetY(v->getPosition().getY());
		beacon.SetHeading(v->getDirection().dir_deg());
		beacon.SetSpeed(v->getSpeed());
		beacon.SetAccel(v->getAccel());
		beacon.SetTimestamp(ns3::Simulator::Now());

		ns3::Ptr<ns3::Packet> packet = ns3::Create<ns3::Packet> ((uint8_t*)&beacon,sizeof(beacon));
		socket->Send(packet);
		ScheduleTx();
	}

	void ReadSocket(ns3::Ptr<ns3::Socket> socket) {
		if (!running || GetNode() == 0) return;
		ns3::Ptr<vns::VNSMobilityModel> model = GetNode()->GetObject<vns::VNSMobilityModel>();
		if (model == 0) return;

		MyVehicle* vehicle = dynamic_cast<MyVehicle*>(model->getVehicle());
		if (vehicle == 0) return;

		ns3::Ptr<ns3::Packet> packet;
		while (packet = socket->Recv()) {
			Beacon beacon;
			packet->CopyData((uint8_t*)&beacon,sizeof(beacon));
	        	double time = ns3::Simulator::Now().GetSeconds();
			vehicle->addConnection(beacon.GetId(), vns::Vec(beacon.GetX(),beacon.GetY()), time);
		}
	}
	ns3::Ptr<ns3::Socket> socket;
	bool running;
	double timeOfLastReceivedPacket;
	ns3::EventId sendPacketEventID;
	ns3::Time tnext;
};


#endif
