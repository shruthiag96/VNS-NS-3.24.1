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

#include "ns3networkmodule.h"
#include "vns/vns.h"
#include "ns3/ptr.h"
#include "ns3/vector.h"
#include "ns3/node.h"
#include "ns3/mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/vns-wifi-helper.h"

namespace vns {

void NS3NetworkModule::removeNode( ns3::Ptr<ns3::Node> node){
	node->Dispose();
}

NS3NetworkModule::NS3NetworkModule() : NetworkModule() {
	disposeDelay = 10;
}

NS3NetworkModule::~NS3NetworkModule() {
}

void NS3NetworkModule::setNetwork(NS3Network* factory) {
	VNS_ASSERT( factory != 0 ,"NS3NetworkModule::setNetwork","NS3Network* factory == 0");
	networkFactory = factory;
}

void NS3NetworkModule::onSimulationStart( const vns::Simulator* simulator ) {
	VNS_ASSERT( networkFactory != 0 ,"NS3NetworkModule without network","Please use NS3NetworkModule::setNetwork.");
	networkFactory->initialize(simulator);
}

void NS3NetworkModule::onSimulationStep( const vns::Simulator* simulator ){
	networkFactory->step(simulator);
	ns3::Simulator::Stop( ns3::Seconds(simulator->getStepTime()) );
	ns3::Simulator::Run();
}

void NS3NetworkModule::onSimulationStop( const vns::Simulator* simulator ) {
	ns3::Simulator::Stop();
	networkFactory->finalize(simulator);
	ns3::Simulator::Destroy();
}

void NS3NetworkModule::onVehicleCreated( const vns::Simulator* simulator,vns::Vehicle* vehicle){
	lock.lock();
	ns3::Ptr<ns3::Node> node = ns3::CreateObject<ns3::Node>();
	lock.unlock();
	ns3::Ptr<vns::VNSMobilityModel> model = node->GetObject<vns::VNSMobilityModel>();
	if( model==0 ){
		model = ns3::CreateObject<VNSMobilityModel>();
	    node->AggregateObject(model);
	}
	model->setVehicle(vehicle);
	vehicle->setNetworkNode(ns3::PeekPointer(node));
	lock.lock();
	networkFactory->onVehicleCreated(simulator,node);
	lock.unlock();
}

void NS3NetworkModule::onVehicleRemoved( const vns::Simulator* simulator,vns::Vehicle* vehicle){
	ns3::Node* ptrnode = (ns3::Node*)vehicle->getNetworkNode();
	if(ptrnode){
		ns3::Ptr<ns3::Node> node(ptrnode);
		lock.lock();
		networkFactory->onVehicleRemoved( simulator, node );
		ns3::Ptr<vns::VNSMobilityModel> model = node->GetObject<vns::VNSMobilityModel>();
		model->setVehicle(0);
		vehicle->setNetworkNode(0);
		ns3::Time tremove( ns3::Seconds(disposeDelay) );
		ns3::Simulator::Schedule(tremove, &NS3NetworkModule::removeNode, this, node);
		lock.unlock();
	}
}

NS_OBJECT_ENSURE_REGISTERED (VNSMobilityModel);

VNSMobilityModel::VNSMobilityModel() : ns3::MobilityModel(){
	vehicle=0;
}

VNSMobilityModel::~VNSMobilityModel(){
	vehicle = 0;
}

VNSMobilityModel::VNSMobilityModel(vns::Vehicle* veh){
	vehicle = veh;
}

void VNSMobilityModel::setVehicle(vns::Vehicle* veh){
	vehicle = veh;
}

vns::Vehicle* VNSMobilityModel::getVehicle(){
	return vehicle;
}

ns3::Vector VNSMobilityModel::GetPosition() const {
	if(vehicle!=0){
		Vec pos = vehicle->getPosition();
		return ns3::Vector(pos.getX(),pos.getY(),pos.getZ());
	}
	return ns3::Vector(0,0,1.79769e+308);
}

ns3::Vector VNSMobilityModel::GetVelocity() const {
	if(vehicle!=0){
		Vec dir = vehicle->getDirection();
		float speed = vehicle->getSpeed();
		return ns3::Vector(speed*dir.getX(),speed*dir.getY(),speed*dir.getZ());
	}
	return ns3::Vector(0.0,0.0,0.0);
}

void VNSMobilityModel::SetPosition( const ns3::Vector& ) {

}

ns3::Vector VNSMobilityModel::DoGetPosition (void) const{
  return GetPosition();
}

void VNSMobilityModel::DoSetPosition(const ns3::Vector& ){

}

ns3::Vector VNSMobilityModel::DoGetVelocity (void) const {
  return GetVelocity();
}

void VNSMobilityModel::DoDispose(void){
	ns3::MobilityModel::DoDispose();
}

ns3::TypeId VNSMobilityModel::GetTypeId(){
	static ns3::TypeId typeId = ns3::TypeId("VNSMobilityModel")
    .SetParent<MobilityModel>()
    .SetGroupName("Mobility")
    .AddConstructor<VNSMobilityModel>();
	return typeId;
}

NS3Network::NS3Network(const vns::RoadNetwork* net){
	network = net;
}

NS3Network::~NS3Network(){
	network = 0;
}

ns3::VNSWifiChannelHelper NS3Network::createWifiChannelHelper(){
	VNS_ASSERT( network != 0 ,"NS3Network::NS3Network","vns::RoadNetwork == 0");
	const vns::BBox bb = network->getExtents();
	ns3::VNSWifiChannelHelper helper(bb.getMinX(),bb.getMinY(),bb.getMaxX(),bb.getMaxY());
	return helper;
}

ns3::VNSWifiPhyHelper* NS3Network::createWifiPhyHelper(){
	int n = phyHelpers.size();
	phyHelpers.resize( n+1 );
	phyHelpers[n] = new ns3::VNSWifiPhyHelper();
	phyHelpers[n]->SetErrorRateModel("ns3::NistErrorRateModel");
	return phyHelpers[n];
}

void NS3Network::step( const vns::Simulator* simulator){
	std::vector<ns3::VNSWifiPhyHelper*>::iterator it = phyHelpers.begin();
	while( it != phyHelpers.end()){
		ns3::VNSWifiPhyHelper* phy = *it;
	    phy->updateStep();
	    it++;
	}
	onSimulationStep(simulator);
}

vns::Vehicle* vehicleFromNS3Node(ns3::Ptr<ns3::Node> node){
	ns3::Ptr<vns::VNSMobilityModel> model = node->GetObject<vns::VNSMobilityModel>();
	return model->getVehicle();
}

}

