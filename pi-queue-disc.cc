/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 NITK Surathkal
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
 * Authors: Priya S Tavarmani <priyast663@gmail.com>
 *          Viyom Mittal <viyommittal@gmail.com>
 *          Mohit P. Tahiliani <tahiliani@nitk.edu.in>
 */

/*
 * PORT NOTE: This code was ported from ns-2.36rc1 (queue/pi.cc).
 * Most of the comments are also ported from the same.
 */
#include <iostream>
#include <string>
#include <fstream>
#include "ns3/log.h"
#include "ns3/enum.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/simulator.h"
#include "ns3/abort.h"
#include "pi-queue-disc.h"
#include "ns3/drop-tail-queue.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("PiQueueDisc");

NS_OBJECT_ENSURE_REGISTERED (PiQueueDisc);

TypeId PiQueueDisc::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PiQueueDisc")
    .SetParent<QueueDisc> ()
    .SetGroupName ("TrafficControl")
    .AddConstructor<PiQueueDisc> ()
    .AddAttribute ("MeanPktSize",
                   "Average of packet size",
                   UintegerValue (500),
                   MakeUintegerAccessor (&PiQueueDisc::m_meanPktSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("QueueRef",
                   "Desired queue size",
                   DoubleValue (50),
                   MakeDoubleAccessor (&PiQueueDisc::m_qRef),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("A",
                   "Value of alpha",
                   DoubleValue (0.00001822),
                   MakeDoubleAccessor (&PiQueueDisc::m_a),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("B",
                   "Value of beta",
                   DoubleValue (0.00001816),
                   MakeDoubleAccessor (&PiQueueDisc::m_b),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("W",
                   "Sampling frequency",
                   DoubleValue (170),
                   MakeDoubleAccessor (&PiQueueDisc::m_w),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("QueueLimit",
                   "Queue limit in bytes/packets",
                   DoubleValue (50),
                   MakeDoubleAccessor (&PiQueueDisc::SetQueueLimit),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MaxSize",
                   "The maximum number of packets accepted by this queue disc",
                   QueueSizeValue (QueueSize ("500p")),
                   MakeQueueSizeAccessor (&QueueDisc::SetMaxSize,
                                          &QueueDisc::GetMaxSize),
                   MakeQueueSizeChecker ())
    //Self Tuning PI
    .AddAttribute ("STPI",
                   "True to enable Self Tuning PI",
                   BooleanValue (false),
                   MakeBooleanAccessor (&PiQueueDisc::m_isSTPI),
                   MakeBooleanChecker ())
    .AddAttribute ("LinkCapacity",
                   "The STPI Link Capacity",
                   DoubleValue (0),
                   MakeDoubleAccessor (&PiQueueDisc::m_capacity),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Kc",
                   "Filter time constant to smoothen capacity",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&PiQueueDisc::m_kc),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Knrc",
                   "Filter time constant to smoothen N/R*C",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&PiQueueDisc::m_knrc),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BPI",
                   "Controls AQM responsiveness",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&PiQueueDisc::m_bpi),
                   MakeDoubleChecker<double> (0,0.85))
    .AddAttribute ("Thc",
                   "Smoothened estimate of C",
                   DoubleValue (0),
                   MakeDoubleAccessor (&PiQueueDisc::m_thc),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Thnrc",
                   "Smoothened estimate of N/R*C",
                   DoubleValue (0),
                   MakeDoubleAccessor (&PiQueueDisc::m_thnrc),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RTT",
                   "estimated round trip time",
                   DoubleValue (0),
                   MakeDoubleAccessor (&PiQueueDisc::m_rtt),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Kp",
                   "PI parameter",
                   DoubleValue (0.00001816),
                   MakeDoubleAccessor (&PiQueueDisc::m_kp),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Ki",
                   "PI parameter",
                   DoubleValue (0.00001822),
                   MakeDoubleAccessor (&PiQueueDisc::m_ki),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("UseEcn",
                   "True to use ECN (packets are marked instead of being dropped)",
                   BooleanValue (false),
                   MakeBooleanAccessor (&PiQueueDisc::m_useEcn),
                   MakeBooleanChecker ())

  ;

  return tid;
}

PiQueueDisc::PiQueueDisc ()
  : QueueDisc ()
{
  NS_LOG_FUNCTION (this);
  m_uv = CreateObject<UniformRandomVariable> ();
  m_rtrsEvent = Simulator::Schedule (Time (Seconds (1.0 / m_w)), &PiQueueDisc::CalculateP, this);

}

PiQueueDisc::~PiQueueDisc ()
{
  NS_LOG_FUNCTION (this);
}

void
PiQueueDisc::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_uv = 0;
  Simulator::Remove (m_rtrsEvent);
  QueueDisc::DoDispose ();
}

void
PiQueueDisc::SetQueueLimit (uint32_t lim)
{
  NS_LOG_FUNCTION (this << lim);
  SetMaxSize (QueueSize (GetMaxSize ().GetUnit (), lim));
}

uint32_t
PiQueueDisc::GetQueueSize (void)
{
  NS_LOG_FUNCTION (this);
  return GetInternalQueue (0)->GetCurrentSize ().GetValue ();
}

int64_t
PiQueueDisc::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uv->SetStream (stream);
  return 1;
}

bool
PiQueueDisc::DoEnqueue (Ptr<QueueDiscItem> item)
{
  NS_LOG_FUNCTION (this << item);

  QueueSize nQueued = GetCurrentSize ();

  if (m_idle)
    {
      Time m_idleEndTime = Simulator :: Now ();
      m_totalIdleTime += (m_idleEndTime - m_idleStartTime);
      m_idleEndTime = NanoSeconds (0);
      m_idleStartTime = NanoSeconds (0);
      m_idle = false;
    }


  if (nQueued + item > GetMaxSize ())
    {
      // Drops due to queue limit: reactive
      DropBeforeEnqueue (item, FORCED_DROP);
      return false;
    }
  else if (DropEarly (item, nQueued.GetValue ()))
    {
      if (!m_useEcn || !Mark (item, UNFORCED_MARK))
        {
          // Early probability drop: proactive
          DropBeforeEnqueue (item, UNFORCED_DROP);
          return false;
        }
    }

  // No drop
  bool retval = GetInternalQueue (0)->Enqueue (item);

  //Self Tuning PI

  // If Queue::Enqueue fails, QueueDisc::Drop is called by the internal queue
  // because QueueDisc::AddInternalQueue sets the drop callback

  NS_LOG_LOGIC ("\t bytesInQueue  " << GetInternalQueue (0)->GetNBytes ());
  NS_LOG_LOGIC ("\t packetsInQueue  " << GetInternalQueue (0)->GetNPackets ());

  return retval;
}

void
PiQueueDisc::InitializeParams (void)
{
  m_dropProb = 0;
  m_qOld = 0;

  //Self-Tuning PI
  if (m_isSTPI)
    {
      m_oldThc = 0;
      m_oldThnrc = 0;
      m_idle = true;
      m_idleStartTime = NanoSeconds (0);
      m_idleEndTime = NanoSeconds (0);
      m_totalIdleTime = NanoSeconds (0);
      m_oldRoutBusyTime = NanoSeconds (0);
    }
}

bool PiQueueDisc::DropEarly (Ptr<QueueDiscItem> item, uint32_t qSize)
{
  NS_LOG_FUNCTION (this << item << qSize);

  double p = m_dropProb;
  bool earlyDrop = true;

  if (GetMaxSize ().GetUnit () == QueueSizeUnit::BYTES)
    {
      p = p * item->GetSize () / m_meanPktSize;
    }
  p = p > 1 ? 1 : p;

  double u =  m_uv->GetValue ();

  if (u > p)
    {
      earlyDrop = false;
    }
  if (!earlyDrop)
    {
      return false;
    }

  return true;
}

void PiQueueDisc::CalculateP ()
{
  NS_LOG_FUNCTION (this);
  double p = 0.0;
  uint32_t qlen = GetQueueSize ();

  //Self Tuning PI (STPI)
  if (m_isSTPI)
    {
      if (m_idle)
        {
          m_idleEndTime = Simulator :: Now ();
          m_totalIdleTime += (m_idleEndTime - m_idleStartTime);
          m_idleEndTime = NanoSeconds (0);
          m_idleStartTime = NanoSeconds (0);
        }

      m_rtt = 0.0025 / 1000.0;
      m_routerBusyTime = double (((Seconds (1.0 / m_w)) - ((m_totalIdleTime))).GetSeconds ());
      if (m_routerBusyTime > 0)
        {
          m_capacity = (m_departedPkts * m_meanPktSize * 8.0) / (m_routerBusyTime);
          m_thc = ((m_oldThc * (1 - m_kc)) + (m_kc * m_capacity));
          if (m_dropProb > 0)
            {
              m_thnrc = ((m_oldThnrc * (1 - m_knrc)) + (m_knrc * (std :: sqrt (m_dropProb / 2))));
              //m_rtt = (((m_thnrc / m_thc)) / (std :: sqrt (m_dropProb / 2)));
              //m_ki is alpha and m_Kp is beta
              m_kp = (2 * m_bpi * (std :: sqrt ((m_bpi * m_bpi) + 1)) * m_thnrc ) / (m_rtt * m_thc);
              m_ki = ((2 * m_thnrc) / m_rtt) * m_kp;
            }
        }

      if (GetMaxSize ().GetUnit () == QueueSizeUnit::BYTES)
        {
          p = (m_ki * (qlen * 1.0 / m_meanPktSize) - m_qRef) + (m_kp * (qlen * 1.0 / m_meanPktSize));
          //p = (m_ki * ((qlen * 1.0 / m_meanPktSize) - m_qRef)) - (m_kp * ((m_qOld * 1.0 / m_meanPktSize) - m_qRef)) + m_dropProb;

        }
      else
        {
          p = (m_ki * (qlen - m_qRef)) + (m_kp * qlen);
          //p = (m_ki * (qlen - m_qRef)) - (m_kp * (m_qOld - m_qRef)) + m_dropProb;
        }
      m_idleStartTime = NanoSeconds (0);
      m_idleEndTime = NanoSeconds (0);
      if (m_idle)
        {
          m_idleStartTime = Simulator :: Now ();
        }


      m_departedPkts = 0;
      m_oldThnrc = m_thnrc;
      m_oldThc = m_thc;

    }

  // PI
  else
    {
      if (GetMaxSize ().GetUnit () == QueueSizeUnit::BYTES)

        {
          p = m_a * ((qlen * 1.0 / m_meanPktSize) - m_qRef) - m_b * ((m_qOld * 1.0 / m_meanPktSize) - m_qRef) + m_dropProb;
        }
      else
        {
          p = m_a * (qlen - m_qRef) - m_b * (m_qOld - m_qRef) + m_dropProb;
        }
    }
  p = (p < 0) ? 0 : p;
  p = (p > 1) ? 1 : p;

  m_dropProb = p;
  m_totalIdleTime = NanoSeconds (0);
  m_qOld = qlen;
  m_rtrsEvent = Simulator::Schedule (Time (Seconds (1.0 / m_w)), &PiQueueDisc::CalculateP, this);


}

Ptr<QueueDiscItem>
PiQueueDisc::DoDequeue ()
{
  NS_LOG_FUNCTION (this);

  if (!GetInternalQueue (0)->IsEmpty ())
    {
      m_idle = false;
      Ptr<QueueDiscItem> item = StaticCast<QueueDiscItem> (GetInternalQueue (0)->Dequeue ());
      m_departedPkts++;
      return item;
    }
  else
    {
      NS_LOG_LOGIC ("Queue empty");
      //Self-Tuning PI
      m_idle = true;
      m_idleStartTime = Simulator::Now ();
      return 0;
    }
}

Ptr<const QueueDiscItem>
PiQueueDisc::DoPeek () const
{
  NS_LOG_FUNCTION (this);
  if (GetInternalQueue (0)->IsEmpty ())
    {
      NS_LOG_LOGIC ("Queue empty");
      return 0;
    }

  Ptr<const QueueDiscItem> item = StaticCast<const QueueDiscItem> (GetInternalQueue (0)->Peek ());

  NS_LOG_LOGIC ("Number packets " << GetInternalQueue (0)->GetNPackets ());
  NS_LOG_LOGIC ("Number bytes " << GetInternalQueue (0)->GetNBytes ());

  return item;
}

bool
PiQueueDisc::CheckConfig (void)
{
  NS_LOG_FUNCTION (this);
  if (GetNQueueDiscClasses () > 0)
    {
      NS_LOG_ERROR ("PiQueueDisc cannot have classes");
      return false;
    }

  if (GetNPacketFilters () > 0)
    {
      NS_LOG_ERROR ("PiQueueDisc cannot have packet filters");
      return false;
    }

  if (GetNInternalQueues () == 0)
    {
      // create a DropTail queue
      AddInternalQueue (CreateObjectWithAttributes<DropTailQueue<QueueDiscItem> >
                          ("MaxSize", QueueSizeValue (GetMaxSize ())));
    }

  if (GetNInternalQueues () != 1)
    {
      NS_LOG_ERROR ("PiQueueDisc needs 1 internal queue");
      return false;
    }

  return true;
}

} //namespace ns3
