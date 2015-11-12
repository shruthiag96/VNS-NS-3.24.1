/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *
 * Modified by Ricardo Fernandes <rjf@dcc.fc.up.pt> for the project VNS.
 *
 */

#include "vns-wifi-phy.h"
#include "vns-wifi-channel.h"
#include "wifi-mode.h"
#include "wifi-preamble.h"
#include "wifi-phy-state-helper.h"
#include "error-rate-model.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/pointer.h"
#include "ns3/net-device.h"
#include "ns3/trace-source-accessor.h"
#include <cmath>

#include "ns3/mobility-model.h"
#include "ns3/node.h"


NS_LOG_COMPONENT_DEFINE ("VNSWifiPhy");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (VNSWifiPhy);

TypeId
VNSWifiPhy::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::VNSWifiPhy")
    .SetParent<WifiPhy> ()
    .AddConstructor<VNSWifiPhy> ()
    .AddAttribute ("EnergyDetectionThreshold",
                   "The energy of a received signal should be higher than "
                   "this threshold (dbm) to allow the PHY layer to detect the signal.",
                   DoubleValue (-96.0),
                   MakeDoubleAccessor (&VNSWifiPhy::SetEdThreshold,
                                       &VNSWifiPhy::GetEdThreshold),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("CcaMode1Threshold",
                   "The energy of a received signal should be higher than "
                   "this threshold (dbm) to allow the PHY layer to declare CCA BUSY state",
                   DoubleValue (-99.0),
                   MakeDoubleAccessor (&VNSWifiPhy::SetCcaMode1Threshold,
                                       &VNSWifiPhy::GetCcaMode1Threshold),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxGain",
                   "Transmission gain (dB).",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&VNSWifiPhy::SetTxGain,
                                       &VNSWifiPhy::GetTxGain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxGain",
                   "Reception gain (dB).",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&VNSWifiPhy::SetRxGain,
                                       &VNSWifiPhy::GetRxGain),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxPowerLevels",
                   "Number of transmission power levels available between "
                   "TxPowerStart and TxPowerEnd included.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&VNSWifiPhy::m_nTxPower),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("TxPowerEnd",
                   "Maximum available transmission level (dbm).",
                   DoubleValue (16.0206),
                   MakeDoubleAccessor (&VNSWifiPhy::SetTxPowerEnd,
                                       &VNSWifiPhy::GetTxPowerEnd),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxPowerStart",
                   "Minimum available transmission level (dbm).",
                   DoubleValue (16.0206),
                   MakeDoubleAccessor (&VNSWifiPhy::SetTxPowerStart,
                                       &VNSWifiPhy::GetTxPowerStart),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxNoiseFigure",
                   "Loss (dB) in the Signal-to-Noise-Ratio due to non-idealities in the receiver."
                   " According to Wikipedia (http://en.wikipedia.org/wiki/Noise_figure), this is "
                   "\"the difference in decibels (dB) between"
                   " the noise output of the actual receiver to the noise output of an "
                   " ideal receiver with the same overall gain and bandwidth when the receivers "
                   " are connected to sources at the standard noise temperature T0 (usually 290 K)\"."
                   " For",
                   DoubleValue (7),
                   MakeDoubleAccessor (&VNSWifiPhy::SetRxNoiseFigure,
                                       &VNSWifiPhy::GetRxNoiseFigure),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("State", "The state of the PHY layer",
                   PointerValue (),
                   MakePointerAccessor (&VNSWifiPhy::m_state),
                   MakePointerChecker<WifiPhyStateHelper> ())
    .AddAttribute ("ChannelSwitchDelay",
                   "Delay between two short frames transmitted on different frequencies. NOTE: Unused now.",
                   TimeValue (MicroSeconds (250)),
                   MakeTimeAccessor (&VNSWifiPhy::m_channelSwitchDelay),
                   MakeTimeChecker ())
    .AddAttribute ("ChannelNumber",
                   "Channel center frequency = Channel starting frequency + 5 MHz * (nch - 1)",
                   UintegerValue (1),
                   MakeUintegerAccessor (&VNSWifiPhy::SetChannelNumber,
                                         &VNSWifiPhy::GetChannelNumber),
                   MakeUintegerChecker<uint16_t> ())

  ;
  return tid;
}

VNSWifiPhy::VNSWifiPhy ()
  :  m_channelNumber (1),
    m_endRxEvent (),
    m_channelStartingFrequency (0)
{
  NS_LOG_FUNCTION (this);
  m_state = CreateObject<WifiPhyStateHelper> ();
  m_random = CreateObject<UniformRandomVariable> ();
  disposed = false;
}

VNSWifiPhy::~VNSWifiPhy ()
{
  NS_LOG_FUNCTION (this);
}

void
VNSWifiPhy::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_channel->getTree()->remove(this);
  m_channel = 0;
  m_deviceRateSet.clear();
  m_device = 0;
  m_mobility = 0;
  m_state = 0;
  disposed = true;
}

void
VNSWifiPhy::ConfigureStandard (enum WifiPhyStandard standard)
{
  NS_LOG_FUNCTION (this << standard);
  switch (standard)
    {
    case WIFI_PHY_STANDARD_80211a:
      Configure80211a ();
      break;
    case WIFI_PHY_STANDARD_80211b:
      Configure80211b ();
      break;
    case WIFI_PHY_STANDARD_80211g:
      Configure80211g ();
      break;
    case WIFI_PHY_STANDARD_80211_10MHZ:
      Configure80211_10Mhz ();
      break;
    case WIFI_PHY_STANDARD_80211_5MHZ:
      Configure80211_5Mhz ();
      break;
    case WIFI_PHY_STANDARD_holland:
      ConfigureHolland ();
      break;
    default:
      NS_ASSERT (false);
      break;
    }
}


void
VNSWifiPhy::SetRxNoiseFigure (double noiseFigureDb)
{
  NS_LOG_FUNCTION (this << noiseFigureDb);
  m_interference.SetNoiseFigure (DbToRatio (noiseFigureDb));
}
void
VNSWifiPhy::SetTxPowerStart (double start)
{
  NS_LOG_FUNCTION (this << start);
  m_txPowerBaseDbm = start;
}
void
VNSWifiPhy::SetTxPowerEnd (double end)
{
  NS_LOG_FUNCTION (this << end);
  m_txPowerEndDbm = end;
}
void
VNSWifiPhy::SetNTxPower (uint32_t n)
{
  NS_LOG_FUNCTION (this << n);
  m_nTxPower = n;
}
void
VNSWifiPhy::SetTxGain (double gain)
{
  NS_LOG_FUNCTION (this << gain);
  m_txGainDb = gain;
}
void
VNSWifiPhy::SetRxGain (double gain)
{
  NS_LOG_FUNCTION (this << gain);
  m_rxGainDb = gain;
}
void
VNSWifiPhy::SetEdThreshold (double threshold)
{
  NS_LOG_FUNCTION (this << threshold);
  m_edThresholdW = DbmToW (threshold);
}
void
VNSWifiPhy::SetCcaMode1Threshold (double threshold)
{
  NS_LOG_FUNCTION (this << threshold);
  m_ccaMode1ThresholdW = DbmToW (threshold);
}
void
VNSWifiPhy::SetErrorRateModel (Ptr<ErrorRateModel> rate)
{
  m_interference.SetErrorRateModel (rate);
}
void
VNSWifiPhy::SetDevice(Ptr<WifiNetDevice> device)
{
  m_device = device;
}
void
VNSWifiPhy::SetMobility (Ptr<Object> mobility)
{
  m_mobility = mobility;
}

double
VNSWifiPhy::GetRxNoiseFigure (void) const
{
  return RatioToDb (m_interference.GetNoiseFigure ());
}
double
VNSWifiPhy::GetTxPowerStart (void) const
{
  return m_txPowerBaseDbm;
}
double
VNSWifiPhy::GetTxPowerEnd (void) const
{
  return m_txPowerEndDbm;
}
double
VNSWifiPhy::GetTxGain (void) const
{
  return m_txGainDb;
}
double
VNSWifiPhy::GetRxGain (void) const
{
  return m_rxGainDb;
}

double
VNSWifiPhy::GetEdThreshold (void) const
{
  return WToDbm (m_edThresholdW);
}

double
VNSWifiPhy::GetCcaMode1Threshold (void) const
{
  return WToDbm (m_ccaMode1ThresholdW);
}

Ptr<ErrorRateModel>
VNSWifiPhy::GetErrorRateModel (void) const
{
  return m_interference.GetErrorRateModel ();
}
Ptr<WifiNetDevice>
VNSWifiPhy::GetDevice (void) const
{
  return m_device;
}
Ptr<Object>
VNSWifiPhy::GetMobility (void)
{
  return m_mobility;
}

double
VNSWifiPhy::CalculateSnr (WifiMode txMode, double ber) const
{
  return m_interference.GetErrorRateModel ()->CalculateSnr (txMode, ber);
}

Ptr<WifiChannel>
VNSWifiPhy::GetChannel (void) const
{
  return m_channel;
}
void
VNSWifiPhy::SetChannel (Ptr<VNSWifiChannel> channel)
{
  m_channel = channel;
  m_channel->Add (this);
}

void
VNSWifiPhy::SetChannelNumber (uint16_t nch)
{
  if (Simulator::Now () == Seconds (0))
    {
      // this is not channel switch, this is initialization
      NS_LOG_DEBUG ("start at channel " << nch);
      m_channelNumber = nch;
      return;
    }

  NS_ASSERT (!IsStateSwitching ());
  switch (m_state->GetState ())
    {
    case VNSWifiPhy::RX:
      NS_LOG_DEBUG ("drop packet because of channel switching while reception");
      m_endRxEvent.Cancel ();
      goto switchChannel;
      break;
    case VNSWifiPhy::TX:
      NS_LOG_DEBUG ("channel switching postponed until end of current transmission");
      Simulator::Schedule (GetDelayUntilIdle (), &VNSWifiPhy::SetChannelNumber, this, nch);
      break;
    case VNSWifiPhy::CCA_BUSY:
    case VNSWifiPhy::IDLE:
      goto switchChannel;
      break;
    default:
      NS_ASSERT (false);
      break;
    }

  return;

switchChannel:

  NS_LOG_DEBUG ("switching channel " << m_channelNumber << " -> " << nch);
  m_state->SwitchToChannelSwitching (m_channelSwitchDelay);
  m_interference.EraseEvents ();
  /*
   * Needed here to be able to correctly sensed the medium for the first
   * time after the switching. The actual switching is not performed until
   * after m_channelSwitchDelay. Packets received during the switching
   * state are added to the event list and are employed later to figure
   * out the state of the medium after the switching.
   */
  m_channelNumber = nch;
}

uint16_t 
VNSWifiPhy::GetChannelNumber (void) const{
        return m_channelNumber;
}


double
VNSWifiPhy::GetChannelFrequencyMhz () const
{
  return m_channelStartingFrequency + 5 * GetChannelNumber ();
}

void
VNSWifiPhy::SetReceiveOkCallback (RxOkCallback callback)
{
  m_state->SetReceiveOkCallback (callback);
}
void
VNSWifiPhy::SetReceiveErrorCallback (RxErrorCallback callback)
{
  m_state->SetReceiveErrorCallback (callback);
}


void VNSWifiPhy::StartReceive(Time rxTime,Ptr<Packet> packet,double rxPowerDbm,WifiTxVector TxVector,enum WifiPreamble preamble){
	if(m_device != 0){
		Simulator::Schedule(rxTime, &VNSWifiPhy::StartReceivePacket, this, packet, rxPowerDbm, TxVector, preamble);
	}
}

void
VNSWifiPhy::StartReceivePacket (Ptr<Packet> packet,
                                 double rxPowerDbm,
                                 WifiTxVector txVector,
                                 enum WifiPreamble preamble)
{
  uint8_t packetType=0,incFlag=0;
  NS_LOG_FUNCTION (this << packet << rxPowerDbm << txVector.GetMode() << preamble);
  rxPowerDbm += m_rxGainDb;
  double rxPowerW = DbmToW (rxPowerDbm);
  Time rxDuration = CalculateTxDuration (packet->GetSize (), txVector, preamble,0.0,packetType,incFlag);
  Time endRx = Simulator::Now () + rxDuration;

  Ptr<InterferenceHelper::Event> event;
  event = m_interference.Add (packet->GetSize (),
                              txVector,
                              preamble,
                              rxDuration,
                              rxPowerW);


  switch (m_state->GetState ())
    {
    case VNSWifiPhy::SLEEP: break;
    case VNSWifiPhy::SWITCHING:
      NS_LOG_DEBUG ("drop packet because of channel switching");
      NotifyRxDrop (packet);
      /*
       * Packets received on the upcoming channel are added to the event list
       * during the switching state. This way the medium can be correctly sensed
       * when the device listens to the channel for the first time after the
       * switching e.g. after channel switching, the channel may be sensed as
       * busy due to other devices' tramissions started before the end of
       * the switching.
       */
      if (endRx > Simulator::Now () + m_state->GetDelayUntilIdle ())
        {
          // that packet will be noise _after_ the completion of the
          // channel switching.
          goto maybeCcaBusy;
        }
      break;
    case VNSWifiPhy::RX:
      NS_LOG_DEBUG ("drop packet because already in Rx (power=" << rxPowerW << "W)");
      NotifyRxDrop (packet);
      if (endRx > Simulator::Now () + m_state->GetDelayUntilIdle ())
        {
          // that packet will be noise _after_ the reception of the
          // currently-received packet.
          goto maybeCcaBusy;
        }
      break;
    case VNSWifiPhy::TX:
      NS_LOG_DEBUG ("drop packet because already in Tx (power=" << rxPowerW << "W)");
      NotifyRxDrop (packet);
      if (endRx > Simulator::Now () + m_state->GetDelayUntilIdle ())
        {
          // that packet will be noise _after_ the transmission of the
          // currently-transmitted packet.
          goto maybeCcaBusy;
        }
      break;
    case VNSWifiPhy::CCA_BUSY:
    case VNSWifiPhy::IDLE:
      if (rxPowerDbm > WToDbm(m_edThresholdW))
        {
          NS_LOG_DEBUG ("sync to signal (power=" << rxPowerW << "W)");
          // sync to signal
          m_state->SwitchToRx (rxDuration);
          NS_ASSERT (m_endRxEvent.IsExpired ());
          NotifyRxBegin (packet);
          m_interference.NotifyRxStart ();
          m_endRxEvent = Simulator::Schedule (rxDuration, &VNSWifiPhy::EndReceive, this,
                                              packet,
                                              event);
        }
      else
        {
    	  //fprintf(stderr,"DROPPED: %lf %lf\n",rxPowerDbm,WToDbm(m_edThresholdW));
          NS_LOG_DEBUG ("drop packet because signal power too Small (" <<
                        rxPowerW << "<" << m_edThresholdW << ")");
          NotifyRxDrop (packet);
          goto maybeCcaBusy;
        }
      break;
    }

  return;

maybeCcaBusy:
  // We are here because we have received the first bit of a packet and we are
  // not going to be able to synchronize on it
  // In this model, CCA becomes busy when the aggregation of all signals as
  // tracked by the InterferenceHelper class is higher than the CcaBusyThreshold

  Time delayUntilCcaEnd = m_interference.GetEnergyDuration (m_ccaMode1ThresholdW);
  if (!delayUntilCcaEnd.IsZero ())
    {
      m_state->SwitchMaybeToCcaBusy (delayUntilCcaEnd);
    }
}

void
VNSWifiPhy::SendPacket (Ptr<const Packet> packet, WifiTxVector txVector, WifiPreamble preamble, uint8_t packetType,uint32_t mpduReferenceNumber)
{
	if(disposed) return;

  NS_LOG_FUNCTION (this << packet << txVector.GetMode() << preamble << (uint32_t)txVector.GetTxPowerLevel());
  /* Transmission can happen if:
   *  - we are syncing on a packet. It is the responsability of the
   *    MAC layer to avoid doing this but the PHY does nothing to
   *    prevent it.
   *  - we are idle
   */
  NS_ASSERT (!m_state->IsStateTx () && !m_state->IsStateSwitching ());



  Time txDuration = CalculateTxDuration (packet->GetSize (), txVector, preamble,0.0,(uint8_t)0,(uint8_t)0);
  if (m_state->IsStateRx ())
    {
      m_endRxEvent.Cancel ();
      m_interference.NotifyRxEnd ();
    }
  NotifyTxBegin (packet);
  uint32_t dataRate500KbpsUnits = txVector.GetMode().GetDataRate (txVector.GetRetries(),txVector.IsShortGuardInterval(),txVector.GetNss()) / 500000;
   struct mpduInfo mpduInformation;
   //struct signalNoiseDbm signalNoise;
  //bool isShortPreamble = (WIFI_PREAMBLE_SHORT == preamble);
  NotifyMonitorSniffTx (packet, (uint16_t)GetChannelFrequencyMhz (), GetChannelNumber (), dataRate500KbpsUnits, preamble,
                             txVector, mpduInformation);
  m_state->SwitchToTx (txDuration, packet, txVector.GetTxPowerLevel(), txVector ,preamble );
  m_channel->Send (this, packet, GetPowerDbm (txVector.GetTxPowerLevel()) + m_txGainDb, txVector, preamble);
}

uint32_t
VNSWifiPhy::GetNModes (void) const
{
  return m_deviceRateSet.size ();
}
WifiMode
VNSWifiPhy::GetMode (uint32_t mode) const
{
  return m_deviceRateSet[mode];
}
uint32_t
VNSWifiPhy::GetNTxPower (void) const
{
  return m_nTxPower;
}

uint8_t
VNSWifiPhy::GetNMcs (void) const
{
  return (uint8_t)0;
}
WifiMode 
VNSWifiPhy::GetMcs (uint8_t mcs) const{
   return m_deviceRateSet[mcs];
}



Time 
VNSWifiPhy::GetChannelSwitchDelay (void) const{
        return m_state->GetLastRxStartTime ();
}




void 
VNSWifiPhy::SetFrequency (uint32_t freq){
    return;
}

uint32_t 
VNSWifiPhy::GetFrequency (void) const{
   return (uint32_t)0;
}

void 
VNSWifiPhy::SetNumberOfTransmitAntennas (uint32_t tx){
  return;
}

uint32_t
VNSWifiPhy::GetNumberOfTransmitAntennas (void) const{
        return (uint32_t)0;
}

void 
VNSWifiPhy::SetNumberOfReceiveAntennas (uint32_t rx){
        return ;
}

void 
VNSWifiPhy::SetGuardInterval (bool guardInterval){
        return;
}

bool 
VNSWifiPhy::GetGuardInterval (void) const{
        return true;
}

void 
VNSWifiPhy::SetLdpc (bool ldpc){
        return ;
}

bool 
VNSWifiPhy::GetLdpc (void) const{
        return true;
}

void 
VNSWifiPhy::SetStbc (bool stbc){
        return ;
}

bool 
VNSWifiPhy::GetStbc (void) const{
        return true;
}

void 
VNSWifiPhy::SetGreenfield (bool greenfield){
        return ;
}

bool 
VNSWifiPhy::GetGreenfield (void) const{
        return true;
}

uint32_t 
VNSWifiPhy::GetChannelWidth (void) const{
        return (uint32_t)0;
}

void 
VNSWifiPhy::SetChannelWidth (uint32_t channelwidth){
        return ;
}

uint32_t 
VNSWifiPhy::GetNumberOfReceiveAntennas (void) const{
        return (uint32_t)0;
}

uint32_t
VNSWifiPhy::GetNBssMembershipSelectors (void) const
{
  return (uint32_t)0;
}

uint32_t
VNSWifiPhy::GetBssMembershipSelector (uint32_t selector) const
{
  return (uint32_t)0;
}

WifiModeList
VNSWifiPhy::GetMembershipSelectorModes (uint32_t selector) 
{
  return m_deviceRateSet;
}


void
VNSWifiPhy::Configure80211a (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 5.000 GHz

  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate36Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate48Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate54Mbps ());
}


void
VNSWifiPhy::Configure80211b (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 2407; // 2.407 GHz

  m_deviceRateSet.push_back (WifiPhy::GetDsssRate1Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate2Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate5_5Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate11Mbps ());
}

void
VNSWifiPhy::Configure80211g (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 2407; // 2.407 GHz

  m_deviceRateSet.push_back (WifiPhy::GetDsssRate1Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate2Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate5_5Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate6Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate9Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetDsssRate11Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate12Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate18Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate24Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate36Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate48Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate54Mbps ());
}

void
VNSWifiPhy::Configure80211_10Mhz (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 5.000 GHz, suppose 802.11a

  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
}

void
VNSWifiPhy::Configure80211_5Mhz (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 5.000 GHz, suppose 802.11a

  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate1_5MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate2_25MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW5MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate13_5MbpsBW5MHz ());
}

void
VNSWifiPhy::ConfigureHolland (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 5.000 GHz
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate36Mbps ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate54Mbps ());
}

void
VNSWifiPhy::Configure80211p_CCH (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 802.11p works over the 5Ghz freq range

  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
}

void
VNSWifiPhy::Configure80211p_SCH (void)
{
  NS_LOG_FUNCTION (this);
  m_channelStartingFrequency = 5e3; // 802.11p works over the 5Ghz freq range

  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
  m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
}

void
VNSWifiPhy::RegisterListener (WifiPhyListener *listener)
{
  m_state->RegisterListener (listener);
}

void
VNSWifiPhy::UnregisterListener (WifiPhyListener *listener)
{
  m_state->UnregisterListener (listener);
}

void
VNSWifiPhy::SetSleepMode (void){
 }

void
VNSWifiPhy::ResumeFromSleep (void){}

bool
VNSWifiPhy::IsStateCcaBusy (void)
{
  return m_state->IsStateCcaBusy ();
}

bool
VNSWifiPhy::IsStateIdle (void)
{
  return m_state->IsStateIdle ();
}
bool
VNSWifiPhy::IsStateBusy (void)
{
  return m_state->IsStateBusy ();
}
bool
VNSWifiPhy::IsStateRx (void)
{
  return m_state->IsStateRx ();
}
bool
VNSWifiPhy::IsStateTx (void)
{
  return m_state->IsStateTx ();
}
bool
VNSWifiPhy::IsStateSwitching (void)
{
  return m_state->IsStateSwitching ();
}

bool
VNSWifiPhy::IsModeSupported (WifiMode mode) const
{
  return true;
}


bool
VNSWifiPhy::IsStateSleep (void)
{
  return m_state->IsStateSleep ();
}

Time
VNSWifiPhy::GetStateDuration (void)
{
  return m_state->GetStateDuration ();
}
Time
VNSWifiPhy::GetDelayUntilIdle (void)
{
  return m_state->GetDelayUntilIdle ();
}

Time
VNSWifiPhy::GetLastRxStartTime (void) const
{
  return m_state->GetLastRxStartTime ();
}

double
VNSWifiPhy::DbToRatio (double dB) const
{
  double ratio = pow (10.0,dB / 10.0);
  return ratio;
}

double
VNSWifiPhy::DbmToW (double dBm) const
{
  double mW = pow (10.0,dBm / 10.0);
  return mW / 1000.0;
}

double
VNSWifiPhy::WToDbm (double w) const
{
  return 10.0 * log10 (w * 1000.0);
}

double
VNSWifiPhy::RatioToDb (double ratio) const
{
  return 10.0 * log10 (ratio);
}

double
VNSWifiPhy::GetEdThresholdW (void) const
{
  return m_edThresholdW;
}

double
VNSWifiPhy::GetPowerDbm (uint8_t power) const
{
  NS_ASSERT (m_txPowerBaseDbm <= m_txPowerEndDbm);
  NS_ASSERT (m_nTxPower > 0);
  double dbm;
  if (m_nTxPower > 1)
    {
      dbm = m_txPowerBaseDbm + power * (m_txPowerEndDbm - m_txPowerBaseDbm) / (m_nTxPower - 1);
    }
  else
    {
      NS_ASSERT_MSG (m_txPowerBaseDbm == m_txPowerEndDbm, "cannot have TxPowerEnd != TxPowerStart with TxPowerLevels == 1");
      dbm = m_txPowerBaseDbm;
    }
  return dbm;
}

void
VNSWifiPhy::EndReceive (Ptr<Packet> packet, Ptr<InterferenceHelper::Event> event)
{
  NS_LOG_FUNCTION (this << packet << event);
  NS_ASSERT (IsStateRx ());
  NS_ASSERT (event->GetEndTime () == Simulator::Now ());

  struct InterferenceHelper::SnrPer snrPer;
  struct mpduInfo aMpdu;
  struct signalNoiseDbm signalNoise;
  snrPer = m_interference.CalculatePlcpPayloadSnrPer(event);
  m_interference.NotifyRxEnd ();
  
  //channelWidth=event->GetTxVector.GetChannelWidth;

  NS_LOG_DEBUG ("mode=" << (event->GetPayloadMode ().GetDataRate (((event->GetTxVector()).GetChannelWidth()),((event->GetTxVector()).IsShortGuardInterval()),((event->GetTxVector()).GetNss()))) <<
                ", snr=" << snrPer.snr << ", per=" << snrPer.per << ", size=" << packet->GetSize ()); 
  if (m_random->GetValue () > snrPer.per)
    {
      NotifyRxEnd (packet);
      uint32_t dataRate500KbpsUnits = event->GetPayloadMode ().GetDataRate(event->GetTxVector().GetChannelWidth(),event->GetTxVector().IsShortGuardInterval(),event->GetTxVector().GetNss()) / 500000;
      //bool isShortPreamble = (WIFI_PREAMBLE_SHORT == event->GetPreambleType ());
      //double signalDbm = RatioToDb (event->GetRxPowerW ()) + 30;
      //double noiseDbm = RatioToDb (event->GetRxPowerW () / snrPer.snr) - GetRxNoiseFigure () + 30;
      NotifyMonitorSniffRx (packet, (uint16_t)GetChannelFrequencyMhz (), GetChannelNumber (), dataRate500KbpsUnits,event->GetPreambleType(),event->GetTxVector(), aMpdu, signalNoise);
      m_state->SwitchFromRxEndOk (packet, snrPer.snr, event->GetTxVector(), event->GetPreambleType ());
    }
  else
    {
      /* failure. */
      NotifyRxDrop (packet);
      m_state->SwitchFromRxEndError (packet, snrPer.snr);
    }
}

int64_t
VNSWifiPhy::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_random->SetStream (stream);
  return 1;
}

Vector VNSWifiPhy::getPoint(){
	Ptr<MobilityModel> model = m_device->GetNode()->GetObject<MobilityModel>();
	return model->GetPosition();
}


}  // namespace ns3
