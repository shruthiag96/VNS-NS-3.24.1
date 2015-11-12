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
 * Modified by Ricardo Fernandes <rjf@dcc.fc.up.pt> for the project VNS.
 *
 */

#ifndef VNS_WIFI_CHANNEL_H
#define VNS_WIFI_CHANNEL_H

#include <vector>
#include <stdint.h>
#include "ns3/packet.h"
#include "wifi-channel.h"
#include "wifi-mode.h"
#include "wifi-tx-vector.h"
#include "wifi-preamble.h"

#include "ns3/wifi-net-device.h"
#include "ns3/mobility-model.h"
#include "qtree.h"

namespace ns3 {

class PropagationLossModel;
class PropagationDelayModel;
class VNSWifiPhy;
class VNSWifiChannel;

class DeliverVisitor : public ns3::QTreeVisitor {
public:
	DeliverVisitor(const VNSWifiChannel* channel,VNSWifiPhy* phy,
			const Ptr<PropagationLossModel>& loss,
			const Ptr<PropagationDelayModel>& delay,
			Ptr<const Packet>& packet,
			double txPowerDbm,
	        WifiTxVector& txVector,
	        WifiPreamble& preamble,
	        double range);
	bool accept(ns3::QTreeNode* node);
	bool accept(ns3::QTreeItem* item);
	bool acceptParent(ns3::QTreeNode* node);
	void visit(ns3::QTreeItem* item);

	VNSWifiPhy* phy;
	const VNSWifiChannel* channel;
	Ptr<PropagationLossModel> m_loss;
	Ptr<PropagationDelayModel> m_delay;
	Ptr<const Packet> packet;
	double txPowerDbm;
    WifiTxVector& txVector;
    WifiPreamble preamble;
    Ptr<MobilityModel> senderMobility;
	double r;
	bool debug;
	int neighbours;
};


/**
 * \brief A Yans wifi channel
 * \ingroup wifi
 *
 * This wifi channel implements the propagation model described in
 * "Yet Another Network Simulator", (http://cutebugs.net/files/wns2-yans.pdf).
 *
 * This class is expected to be used in tandem with the ns3::YansWifiPhy
 * class and contains a ns3::PropagationLossModel and a ns3::PropagationDelayModel.
 * By default, no propagation models are set so, it is the caller's responsability
 * to set them before using the channel.
 */
class VNSWifiChannel : public WifiChannel
{
public:

	  double minX;
	  double minY;
	  double maxX;
	  double maxY;

  static TypeId GetTypeId (void);

  VNSWifiChannel();
  virtual ~VNSWifiChannel ();

  // inherited from Channel.
  virtual uint32_t GetNDevices (void) const;
  virtual Ptr<NetDevice> GetDevice( uint32_t i ) const;

  void Add(Ptr<VNSWifiPhy> phy);
  void Remove(Ptr<VNSWifiPhy> phy);

  /**
   * \param loss the new propagation loss model.
   */
  void SetPropagationLossModel (Ptr<PropagationLossModel> loss);
  /**
   * \param delay the new propagation delay model.
   */
  void SetPropagationDelayModel (Ptr<PropagationDelayModel> delay);

  /**
   * \param sender the device from which the packet is originating.
   * \param packet the packet to send
   * \param txPowerDbm the tx power associated to the packet
   * \param wifiMode the tx mode associated to the packet
   * \param preamble the preamble associated to the packet
   *
   * This method should not be invoked by normal users. It is
   * currently invoked only from WifiPhy::Send. YansWifiChannel
   * delivers packets only between PHYs with the same m_channelNumber,
   * e.g. PHYs that are operating on the same channel.
   */
  void Send (Ptr<VNSWifiPhy> sender, Ptr<const Packet> packet, double txPowerDbm,
             WifiTxVector txVector, WifiPreamble preamble) const;

  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
   int64_t AssignStreams (int64_t stream);

  void updateStep();

  inline QTree* getTree(){ return m_phyTree; };

  void Receive(VNSWifiPhy* receiver, Ptr<Packet> packet, double rxPowerDbm,
                WifiTxVector txVector, WifiPreamble preamble) const;


  double CalcDistanceToPower( double pr ) const ;
  inline void setMaximumSearchDistance(double range){ searchR = range; };
  inline void setDebug(bool v){ debug = v; };
  void setExtents(double minX, double minY, double maxX, double maxY);

private:
  VNSWifiChannel& operator = (const VNSWifiChannel &);
  VNSWifiChannel (const VNSWifiChannel &);

  //typedef std::vector<Ptr<YansWifiPhy> > PhyList;
  //void Receive (uint32_t i, Ptr<Packet> packet, double rxPowerDbm,
  //              WifiMode txMode, WifiPreamble preamble) const;


  Ptr<PropagationLossModel> m_loss;
  Ptr<PropagationDelayModel> m_delay;
  QTree* m_phyTree;
  double maxRxGainDbm;
  double maxEdTresholdDbm;
  double searchR;
  bool debug;

};

}


#endif /* VNS_WIFI_CHANNEL_H */
