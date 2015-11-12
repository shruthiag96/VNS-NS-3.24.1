/***************************************************************************
 *   Copyright (C) 2012 Ricardo Fernandes - All Rights Reserved            *
 *   Email: rjf@dcc.fc.up.pt or rikardojfernandez@gmail.com                *
 *                                                                         *
 *   This file is part of the project that integrates VNS and NS-3         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef VNS_NS3NETWORKMODULE_H_
#define VNS_NS3NETWORKMODULE_H_


#include "vns/networkmodule.h"
#include "vns/lock.h"
#include "vector"

#include "ns3/vns-wifi-helper.h"

namespace vns {

class RoadNetwork;

class VNSMobilityModel : public ns3::MobilityModel {

private:
	vns::Vehicle* vehicle;
	void setVehicle(vns::Vehicle* veh);

public:

	VNSMobilityModel();
	VNSMobilityModel(vns::Vehicle* veh);
	~VNSMobilityModel();

	vns::Vehicle* getVehicle();
	ns3::Vector GetPosition() const;
	ns3::Vector GetVelocity() const;
	void SetPosition( const ns3::Vector& position);
	ns3::Vector DoGetPosition (void) const;
	void DoSetPosition(const ns3::Vector& position);
	ns3::Vector DoGetVelocity (void) const;
	void DoDispose(void);
	static ns3::TypeId GetTypeId();

	friend class NS3NetworkModule;

};



class NS3Network {
public:
	NS3Network(const vns::RoadNetwork* net);
	virtual ~NS3Network();
	virtual void initialize(const vns::Simulator*){};
	virtual void onVehicleCreated(const vns::Simulator*,ns3::Ptr<ns3::Node>){};
	virtual void onVehicleRemoved(const vns::Simulator*,ns3::Ptr<ns3::Node>){};
	virtual void onSimulationStep(const vns::Simulator*){};
	virtual void finalize(const vns::Simulator*){};

	inline const vns::RoadNetwork* getRoadNetwork() const { return network; };

protected:
	ns3::VNSWifiChannelHelper createWifiChannelHelper();
	ns3::VNSWifiPhyHelper* createWifiPhyHelper();

private:
	const vns::RoadNetwork* network;
	std::vector<ns3::VNSWifiPhyHelper*> phyHelpers;
	void step(const vns::Simulator*);
	friend class NS3NetworkModule;
};



class NS3NetworkModule : public NetworkModule {

public:
	NS3NetworkModule();
	virtual ~NS3NetworkModule();

	void setNetwork(NS3Network* factory);
	inline void setNodeRemovalDelay(int delay){ disposeDelay = delay; };

public:

	void onSimulationStart(const vns::Simulator* simulator);
	void onSimulationStep(const vns::Simulator* simulator);
	void onSimulationStop(const vns::Simulator* simulator);
	void onVehicleCreated(const vns::Simulator* simulator, Vehicle* vehicle);
	void onVehicleRemoved(const vns::Simulator* simulator, Vehicle* vehicle);

	void removeNode( ns3::Ptr<ns3::Node> );

	vns::Lock lock;
	NS3Network* networkFactory;
	int disposeDelay;
};

vns::Vehicle* vehicleFromNS3Node(ns3::Ptr<ns3::Node> node);

}




#endif /* VNS_NS3NETWORKMODULE_H_ */
