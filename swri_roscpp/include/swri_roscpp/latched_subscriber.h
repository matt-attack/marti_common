// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef SWRI_ROSCPP_LATCHED_SUBSCRIBER_H_
#define SWRI_ROSCPP_LATCHED_SUBSCRIBER_H_


#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <swri_roscpp/subscriber.h>

namespace swri
{
// THIS CLASS IS DEPRECATED.  This class has been replaced with a
// secondary interface to Subscriber that is initialized with the
// address of a boost::shared_ptr<M const> variable.  When a message
// is received, it's value is assigned to this pointer.  This approach
// provides the same simplicity as LatchedSubscriber without confusing
// semantics of faking a pointer type.
// 
// This is an extension of the swri::Subscriber class to simplify the
// common case where you only care about the most recent value of a
// message rather than trying to queue up messages or responding to
// events.  It has its own callback that is provided to
// swri::Subscriber and simply stores the message for the user to
// access whenever necessary.
template <class M>
class LatchedSubscriber : public Subscriber
{
 private:
  template <class M2>
  struct LatchedReceiver
  {
    std::shared_ptr<M2 const> msg_;    
    void handleMessage(const std::shared_ptr<M2> msg) {
      msg_ = msg;
    }    
  };  // struct LatchedReceiver

  std::shared_ptr<LatchedReceiver<M> > receiver_;
  M empty_;
  
 public:
  LatchedSubscriber();

  // Create a new latched subscriber.  This is most like the
  // swri::Subscriber interface, but you have to provide the template
  // argument twice (once in your class declaration and at
  // construction).  See the initialize() method for a simpler
  // alternative.
  LatchedSubscriber(swri::Node* nh,
                    const std::string &topic);//,
                    //const ros::TransportHints &transport_hints=ros::TransportHints());

  // Creates a latched subscriber in place.  This is more convenient
  // because you don't have to provide the template argument a second
  // time.
  void initialize(swri::Node* nh,
                  const std::string &topic);//,
                  //const ros::TransportHints &transport_hints=ros::TransportHints());

  LatchedSubscriber<M>& operator=(const LatchedSubscriber<M> &other);

  // Return the value of the most recent message.  This is guaranteed
  // to be non-NULL if the messageCount() is non-zero, otherwise it
  // may be null.
  const std::shared_ptr<M const>& data() const;
  M const * operator->() const;
  
  void reset();  
};  // class LatchedSubscriber

template<class M>
LatchedSubscriber<M>::LatchedSubscriber()
{
  // Setup an empty receiver so that we can assume receiver_ is
  // non-null and avoid a lot of unnecessary NULL checks.
  receiver_ = std::shared_ptr<LatchedReceiver<M> >(new LatchedReceiver<M>());
}

template<class M>
LatchedSubscriber<M>::LatchedSubscriber(
  swri::Node* nh,
  const std::string &topic)//,
//  const ros::TransportHints &transport_hints)
{
  ROS_WARN("swri_roscpp::LatchedSubscriber has been deprecated.  See header for information.");
  receiver_ = std::shared_ptr<LatchedReceiver<M> >(new LatchedReceiver<M>());
  // It seems like there should be a better way to do this?
  Subscriber::operator=(
    Subscriber(nh, topic, 1,
               &LatchedReceiver<M>::handleMessage, receiver_.get()));//, transport_hints));
}

template<class M>
void LatchedSubscriber<M>::initialize(
  swri::Node* nh,
  const std::string &topic)//,
//  const ros::TransportHints &transport_hints)
{
  *this = LatchedSubscriber<M>(nh, topic);//, transport_hints);
}


template<class M>
LatchedSubscriber<M>& LatchedSubscriber<M>::operator=(const LatchedSubscriber<M> &other)
{
  Subscriber::operator=(other);
  receiver_ = other.receiver_;
  return *this;
}
    
template<class M>
const std::shared_ptr<M const>& LatchedSubscriber<M>::data() const
{
  return receiver_->msg_;
}

template<class M>
M const * LatchedSubscriber<M>::operator->() const
{
  if (receiver_->msg_) {
    return receiver_->msg_.get();
  } else {
    return &empty_;
  }
}

template<class M>
void LatchedSubscriber<M>::reset()
{
  receiver_->msg_.reset();
  resetStatistics();
}
}  // namespace swri
#endif  // SWRI_ROSCPP_SUBSCRIBER_H_

