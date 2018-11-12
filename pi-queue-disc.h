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
 * PORT NOTE: This code was ported from ns-2.36rc1 (queue/pi.h).
 * Most of the comments are also ported from the same.
 */

#ifndef PI_QUEUE_DISC_H
#define PI_QUEUE_DISC_H

#include <iostream>
#include <string>
#include <fstream>
#include <queue>
#include "ns3/packet.h"
#include "ns3/queue-disc.h"
#include "ns3/nstime.h"
#include "ns3/boolean.h"
#include "ns3/data-rate.h"
#include "ns3/timer.h"
#include "ns3/event-id.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {

class TraceContainer;
class UniformRandomVariable;

/**
 * \ingroup traffic-control
 *
 * \brief Implements PI Active Queue Management discipline
 */
class PiQueueDisc : public QueueDisc
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * \brief PiQueueDisc Constructor
   */
  PiQueueDisc ();

  /**
   * \brief PiQueueDisc Destructor
   */
  virtual ~PiQueueDisc ();

  /**
   * \brief Stats
   */
  typedef struct
  {
    uint32_t unforcedDrop;      //!< Early probability drops: proactive
    uint32_t forcedDrop;        //!< Drops due to queue limit: reactive
    uint32_t packetsDequeued;
  } Stats;

  /**
   * \brief Drop types
   */
  enum
  {
    DTYPE_NONE,        //!< Ok, no drop
    DTYPE_FORCED,      //!< A "forced" drop
    DTYPE_UNFORCED,    //!< An "unforced" (random) drop
  };

  uint32_t GetQueueSize (void);

  /**
   * \brief Set the limit of the queue in bytes or packets.
   *
   * \param lim The limit in bytes or packets.
   */
  void SetQueueLimit (uint32_t lim);

  /**
   * \brief Get PI statistics after running.
   *
   * \returns The drop statistics.
   */
  Stats GetStats ();

  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);

  // Reasons for dropping packets
  static constexpr const char* UNFORCED_DROP = "Unforced drop";  //!< Early probability drops
  static constexpr const char* FORCED_DROP = "Forced drop";      //!< Forced drop
  // Reasons for marking packets
  static constexpr const char* UNFORCED_MARK = "Unforced mark";  //!< Early probability marks
  static constexpr const char* FORCED_MARK = "Forced mark";      //!< Forced mark

protected:
  /**
   * \brief Dispose of the object
   */
  virtual void DoDispose (void);

private:
  virtual bool DoEnqueue (Ptr<QueueDiscItem> item);
  virtual Ptr<QueueDiscItem> DoDequeue (void);
  virtual Ptr<const QueueDiscItem> DoPeek (void) const;
  virtual bool CheckConfig (void);

  /**
   * \brief Initialize the queue parameters.
   */
  virtual void InitializeParams (void);

  /**
   * \brief Check if a packet needs to be dropped due to probability drop
   * \param item queue item
   * \param qSize queue size
   * \returns 0 for no drop, 1 for drop
   */
  bool DropEarly (Ptr<QueueDiscItem> item, uint32_t qSize);

  /**
   * Periodically update the drop probability based on the delay samples:
   * not only the current delay sample but also the trend where the delay
   * is going, up or down
   */
  void CalculateP ();

  Stats m_stats;                                //!< PI statistics

  // ** Variables supplied by user
  //Queue::QueueMode m_mode;                      //!< Mode (bytes or packets)
  double m_queueLimit;                          //!< Queue limit in bytes / packets
  uint32_t m_meanPktSize;                       //!< Average packet size in bytes
  double m_qRef;                                //!< Desired queue size
  double m_a;                                   //!< Parameter to PI controller
  double m_b;                                   //!< Parameter to PI controller
  double m_w;                                   //!< Sampling frequency (Number of times per second)

  // ** Variables maintained by PI
  double m_dropProb;                            //!< Variable used in calculation of drop probability
  uint32_t m_qOld;                              //!< Old value of queue length
  EventId m_rtrsEvent;                          //!< Event used to decide the decision of interval of drop probability calculation
  Ptr<UniformRandomVariable> m_uv;              //!< Rng stream

  // **Self Tuning PI
  
  std::ofstream myfile;
  
  bool m_useEcn;                                //!< True if ECN is used (packets are marked instead of being dropped)
  bool m_idle;                                  //!< Idle status
  bool m_isSTPI;                                //!< To enable STPI
  double m_capacity;                            //!< link capacity
  double m_kc;                                  //!< filter time constant to smoothen capacity
  double m_knrc;                                //!< filter time constant to smoothen N/R*C
  double m_bpi;                                 //!< controls AQM Responsiveness
  double m_thc;                                 //!< Smoothened estimate of Capacity
  double m_thnrc;                               //!< Smoothened estimate of N/R*C
  double m_oldThc;                              //!< old Smoothened estimate of Capacity
  double m_oldThnrc;                            //!< old Smoothened estimate of N/R*C
  Time m_oldRoutBusyTime;                       //!< Router's old busy time
  double m_rtt;                                 //!< estimated round trip time
  double m_kp;                                  //!< PI parameter
  double m_ki;                                  //!< PI parameter
  Time m_totalIdleTime;                         //!< Router's total idle Time
  Time m_idleStartTime;                         //!< Router's idle Start Time
  Time m_idleEndTime;                           //!< Router's idle Start Time
  double m_routerBusyTime;                    //!< Router's Busy Time
  uint32_t m_departedPkts;                      //!< No. of departed packets since the last probability calculation


};

}    // namespace ns3

#endif
