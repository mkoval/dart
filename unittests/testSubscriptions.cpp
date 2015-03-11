/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <functional>

#include <gtest/gtest.h>
#include <boost/signals2.hpp>
#include <boost/signals2/signal_type.hpp>

#include "dart/common/sub_ptr.h"
#include "dart/common/Signal.h"
#include "dart/common/Timer.h"
#include "dart/dynamics/SimpleFrame.h"
#include "dart/dynamics/BoxShape.h"

using namespace dart;
using namespace dynamics;

namespace bs2 = boost::signals2;
using namespace bs2::keywords;
using bs2::dummy_mutex;
using bs2::signal_type;

enum Notification { TALKER_SOMETHING_UPDATED };

size_t foo()
{
  size_t sum = 0;
  for (size_t i = 0; i < 1e+1; ++i)
    sum += i;
  return sum;
}

class Listener;
class Talker;

class Listener : public common::Subscriber
{
public:
  Listener() : common::Subscriber() { }
  virtual ~Listener() {}

  virtual void receiveNotification(const common::Publisher* _subscription,
                                   int _notice)
  {
    if (_notice == TALKER_SOMETHING_UPDATED) { foo(); }
  }
  void receiveNotificationSignal() { foo(); }
};

class Talker : public common::Publisher
{
public:
  Talker() : common::Publisher() {}
  virtual ~Talker() {}

  void connect(Listener* _listener)
  {
    addSubscriber(_listener);
    mSignal.connect(std::bind(&Listener::receiveNotificationSignal, _listener));
    mBoostSignal.connect(boost::bind(&Listener::receiveNotificationSignal, _listener));
    mBoostSignalDummyMutex.connect(boost::bind(&Listener::receiveNotificationSignal, _listener));
  }

  void updateSomethingPublisher() { sendNotification(TALKER_SOMETHING_UPDATED); }
  void updateSomethingSignal() { mSignal.raise(); }
  void updateSomethingBoostSignal() { mBoostSignal(); }
  void updateSomethingBoostSignalDummyMutex() { mBoostSignalDummyMutex(); }

protected:
  typedef bs2::signal_type<void(), bs2::keywords::mutex_type<bs2::dummy_mutex> >::type signal_type;
  bs2::signal<void()> mBoostSignal;
  signal_type mBoostSignalDummyMutex;
  common::Signal<void()> mSignal;
};

void testListeners(size_t _numTests, size_t _numListeners)
{
  common::Timer t;

  Talker talker;
  std::vector<Listener> listeners(_numListeners);

  for (auto& listener : listeners)
    talker.connect(&listener);

  std::cout << "[ Number of listeners: " << _numListeners << "]" << std::endl;

  t.start();
  for (size_t i = 0; i < _numTests; ++i)
  {
    talker.updateSomethingPublisher();
  }
  t.stop();
  std::cout << "Publisher                 : " << t.getLastElapsedTime() << std::endl;

  t.start();
  for (size_t i = 0; i < _numTests; ++i)
  {
    talker.updateSomethingSignal();
  }
  t.stop();
  std::cout << "Signal                    : " << t.getLastElapsedTime() << std::endl;

  t.start();
  for (size_t i = 0; i < _numTests; ++i)
  {
    talker.updateSomethingBoostSignal();
  }
  t.stop();
  std::cout << "Boost.Signal              : " << t.getLastElapsedTime() << std::endl;

  t.start();
  for (size_t i = 0; i < _numTests; ++i)
  {
    talker.updateSomethingBoostSignalDummyMutex();
  }
  t.stop();
  std::cout << "Boost.Signal (dummy mutex): " << t.getLastElapsedTime() << std::endl;

  std::cout << std::endl;
}

TEST(Subscriptions, PerformanceTest)
{
  const size_t numTests = 1e+5;

  testListeners(numTests, 0);
  testListeners(numTests, 1e+0);
  testListeners(numTests, 1e+1);
  testListeners(numTests, 1e+2);
  testListeners(numTests, 1e+3);
  testListeners(numTests, 1e+4);
}

TEST(Subscriptions, Notifications)
{
  sub_ptr<Detachable> entity_ptr = new Detachable(Frame::World(), "entity", false);
  sub_ptr<SimpleFrame> frame_ptr = new SimpleFrame(Frame::World(), "frame");

  EXPECT_TRUE(entity_ptr.valid());
  EXPECT_TRUE(frame_ptr.valid());

  entity_ptr->setParentFrame(frame_ptr);
  EXPECT_TRUE(entity_ptr.getLatestNotification() == Entity::FRAME_CHANGED_NOTICE);

  entity_ptr->setName("new entity name");
  EXPECT_TRUE(entity_ptr.getLatestNotification() == Entity::NAME_CHANGED_NOTICE);

  entity_ptr->addVisualizationShape(new BoxShape(Eigen::Vector3d::Ones()));
  EXPECT_TRUE(entity_ptr.getLatestNotification() == Entity::VISUALIZATION_CHANGE_NOTICE);

  delete entity_ptr;
  delete frame_ptr;

  EXPECT_FALSE(entity_ptr.valid());
  EXPECT_FALSE(frame_ptr.valid());

  EXPECT_TRUE(entity_ptr.get() == nullptr);
  EXPECT_TRUE(frame_ptr.get() == nullptr);
}

Entity* getPointer(Entity* _ptr)
{
  return _ptr;
}

TEST(Subscriptions, ImplicitConversion)
{
  sub_ptr<Entity> entity_ptr = new Entity(Frame::World(), "entity", false);

  // This checks whether the sub_ptr class can successfully be implicitly
  // converted to the type of class it's supposed to be pointing to
  EXPECT_TRUE( getPointer(entity_ptr) == entity_ptr.get() );

  delete entity_ptr;
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
