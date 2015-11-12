/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage, <mathieu.lacage@sophia.inria.fr>
 *
 * Modified by Ricardo Fernandes <rjf@dcc.fc.up.pt> for the VNS project.
 *
 */

#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/object-factory.h"
#include "vns-wifi-channel.h"
#include "vns-wifi-phy.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "wifi-tx-vector.h"
#include <stdio.h>

NS_LOG_COMPONENT_DEFINE ("VNSWifiChannel");

#define MAX(a,b) ((a>b)?(a):(b))
#define MIN(a,b) ((a<b)?(a):(b))

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (VNSWifiChannel);


double VNSWifiChannel::CalcDistanceToPower( double pr ) const {
	const double PI = 3.14159265358979323846;
	double m_lambda = (300000000.0 / 5.150e9);
	double m_systemLoss = (1.0);
	double L = 10*log10( m_lambda * m_lambda );
	double Q = 10*log10( 16 * PI * PI * m_systemLoss );
	double vv = L - Q - pr;
	double v = vv/20.0;
	double d = pow(10,v);
	return d;
}


TypeId
VNSWifiChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::VNSWifiChannel")
    .SetParent<WifiChannel> ()
    .AddConstructor<VNSWifiChannel> ()
    .AddAttribute ("PropagationLossModel", "A pointer to the propagation loss model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&VNSWifiChannel::m_loss),
                   MakePointerChecker<PropagationLossModel> ())
    .AddAttribute ("PropagationDelayModel", "A pointer to the propagation delay model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&VNSWifiChannel::m_delay),
                   MakePointerChecker<PropagationDelayModel> ())
  ;
  return tid;
}

VNSWifiChannel::VNSWifiChannel(){
	maxRxGainDbm = -10000;
	maxEdTresholdDbm = -10000;
	searchR = -1;
	debug = false;
}

void VNSWifiChannel::setExtents(double minx, double miny, double maxx, double maxy){
        float offset = 50.0;
	minX = minx-offset;
	minY = miny-offset;
	maxX = maxx+offset;
	maxY = maxy+offset;
	m_phyTree = new QTree(minX,minY,maxX,maxY);
}

VNSWifiChannel::~VNSWifiChannel(){
  NS_LOG_FUNCTION_NOARGS ();
  //m_phyList.clear ();
  m_phyTree->clear();
}

void VNSWifiChannel::SetPropagationLossModel (Ptr<PropagationLossModel> loss){
  m_loss = loss;
}

void VNSWifiChannel::SetPropagationDelayModel (Ptr<PropagationDelayModel> delay){
  m_delay = delay;
}

void
VNSWifiChannel::Send (Ptr<VNSWifiPhy> sender, Ptr<const Packet> packet, double txPowerDbm, WifiTxVector txVector, WifiPreamble preamble) const
{

	VNSWifiPhy* phy = PeekPointer(sender);
    double tx = txPowerDbm+maxRxGainDbm;
    double range;
    if(searchR<=0){
    	range = CalcDistanceToPower(maxEdTresholdDbm-tx);
    }else{
    	range = searchR;
    }

	DeliverVisitor visitor(this,phy,m_loss, m_delay,packet,txPowerDbm,txVector,preamble,range);

	m_phyTree->applyVisitorBottomUp(phy,visitor);
}

void
VNSWifiChannel::Receive(VNSWifiPhy* receiver, Ptr<Packet> packet, double rxPowerDbm, WifiTxVector txVector, WifiPreamble preamble) const
{
	receiver->StartReceivePacket (packet, rxPowerDbm, txVector, preamble);
}

uint32_t
VNSWifiChannel::GetNDevices (void) const
{
  return m_phyTree->size();
}

Ptr<NetDevice> VNSWifiChannel::GetDevice( uint32_t) const{
	return 0;
}

void
VNSWifiChannel::Add( Ptr<VNSWifiPhy> phy ){
	maxRxGainDbm = MAX(phy->GetRxGain(),maxRxGainDbm);
	maxEdTresholdDbm = MAX(phy->GetEdThreshold(),maxEdTresholdDbm);
	m_phyTree->addItem( PeekPointer(phy) );
	//fprintf(stderr,"RXGAIN: %lf ED: %lf\n",maxRxGainDbm,maxEdTresholdDbm);
}

int64_t
VNSWifiChannel::AssignStreams (int64_t stream)
{
  int64_t currentStream = stream;
  currentStream += m_loss->AssignStreams (stream);
  return (currentStream - stream);
}


void VNSWifiChannel::updateStep(){
	m_phyTree->update();
}


DeliverVisitor::DeliverVisitor(const VNSWifiChannel* channel,
                                VNSWifiPhy* phy,
                                const Ptr<PropagationLossModel>& loss,
                                const Ptr<PropagationDelayModel>& delay,
                                Ptr<const Packet>& packet,
                                double txPowerDbm,
                                WifiTxVector& txVector,
                                WifiPreamble& preamble,
                                double range) : txVector(txVector){
	this->phy = phy;
	this->channel=channel;
	this->packet=packet;
	this->txPowerDbm=txPowerDbm;
	this->txVector=txVector;
	this->preamble=preamble;
	m_loss = loss;
	m_delay = delay;
	senderMobility = phy->GetDevice()->GetNode()->GetObject<MobilityModel>();
	r = range;
	ns3::Vector p = phy->getPoint();
	minX = MAX(channel->minX,p.x-r);
	maxX = MIN(channel->maxX,p.x+r);
	minY = MAX(channel->minY,p.y-r);
	maxY = MIN(channel->maxY,p.y+r);
}

bool DeliverVisitor::accept(ns3::QTreeNode* node){
    if( maxX < node->minX || minX > node->maxX)
        return false;
    if( maxY < node->minY || minY > node->maxY)
        return false;
    return true;
}

bool DeliverVisitor::acceptParent(QTreeNode* n){
	if(n->parent==0) return false;
	if( minX >= n->minX && maxX <= n->maxX && minY >= n->minY && maxY <= n->maxX){
		return false;
	}
	return true;
};

bool DeliverVisitor::DeliverVisitor::accept(ns3::QTreeItem* item){
	VNSWifiPhy* recphy = dynamic_cast<VNSWifiPhy*>(item);
	if(recphy == 0 || phy==recphy || phy->GetChannelNumber() != recphy->GetChannelNumber()){
		return false;
	}
	Ptr<MobilityModel> receiverMobility = recphy->GetDevice()->GetNode()->GetObject<MobilityModel>();
	if( senderMobility->GetDistanceFrom(receiverMobility)>=r){
		return false;
	}
	return true;
}

void DeliverVisitor::visit(ns3::QTreeItem* item){
	VNSWifiPhy* recphy = dynamic_cast<VNSWifiPhy*>(item);
	Ptr<MobilityModel> receiverMobility = recphy->GetDevice()->GetNode()->GetObject<MobilityModel>();

    Time delay = m_delay->GetDelay (senderMobility, receiverMobility);
    double rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility);
    NS_LOG_DEBUG ("propagation: txPower=" << txPowerDbm << "dbm, rxPower=" << rxPowerDbm << "dbm, " << "distance=" << senderMobility->GetDistanceFrom (receiverMobility) << "m, delay=" << delay);


    //double dd = this->channel->CalcDistanceToPower(threshold-tx);


    if(rxPowerDbm + recphy->GetRxGain() <= recphy->GetEdThreshold()){
        return;
    }

    /*
    fprintf(stderr,"%lf\n",r);
    double tx = txPowerDbm+recphy->GetRxGain();
    double txloss = rxPowerDbm-txPowerDbm;
    double threshold = recphy->GetEdThreshold();
    double dd = senderMobility->GetDistanceFrom (receiverMobility);
    fprintf(stderr,"OK - MIND: %lf GAINS: TX=%lf LOSS=%lf RX=%lf Threshold=%lf Distance=%lf\n",r,tx,txloss,tx+txloss,threshold,dd);
*/
    Ptr<Packet> copy = packet->Copy();
    Ptr<Object> dstNetDevice = recphy->GetDevice();
    uint32_t dstNode;
    if (dstNetDevice == 0)
      {
        dstNode = 0xffffffff;
      }
    else
      {
        dstNode = dstNetDevice->GetObject<NetDevice> ()->GetNode ()->GetId ();
      }
    Simulator::ScheduleWithContext(dstNode,delay, &VNSWifiChannel::Receive, channel, recphy, copy, rxPowerDbm, txVector, preamble);
}



} // namespace ns3
