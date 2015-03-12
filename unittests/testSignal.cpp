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

#include <gtest/gtest.h>

#include "dart/dart.h"

using namespace dart;
using namespace common;

static int callCount0 = 0;
static int callCount1 = 0;
static int callCount2 = 0;

//==============================================================================
void foo0() { callCount0++; }

//==============================================================================
void foo1(int /*_val*/) { callCount1++; }

//==============================================================================
void foo2(int /*_val1*/, float /*_val2*/) { callCount2++; }

//==============================================================================
class Viewer
{
public:
  static void onSignal1Static(int /*_val*/) { callCount1++; }
  static void onSignal2Static(int /*_val1*/, float /*_val2*/) { callCount2++; }
  void onSignal1(int /*_val*/) { callCount1++; }
  void onSignal2(int /*_val1*/, float /*_val2*/) { callCount2++; }
};

//==============================================================================
TEST(Signal, Basic)
{
  Signal<void()> signal0;
  Signal<void(int)> signal1;
  Signal<void(int, float)> signal2;
  Connection connection0 = signal0.connect(&foo0);
  Connection connection1 = signal1.connect(&foo1);
  Connection connection2 = signal2.connect(&foo2);

  EXPECT_EQ(signal0.getNumConnections(), 1);
  EXPECT_EQ(signal1.getNumConnections(), 1);
  EXPECT_EQ(signal2.getNumConnections(), 1);

  EXPECT_TRUE(connection0.isConnected());
  EXPECT_TRUE(connection1.isConnected());
  EXPECT_TRUE(connection2.isConnected());

  connection0.disconnect();
  connection1.disconnect();
  connection2.disconnect();

  // The connections are still exist in the signal as disconnected state.
  // The disconnected connections will be removed when the signal raises or
  // flushDisconnections() is explictly called.
  EXPECT_EQ(signal0.getNumConnections(), 1);
  EXPECT_EQ(signal1.getNumConnections(), 1);
  EXPECT_EQ(signal2.getNumConnections(), 1);

  signal0();
  signal1.flushDisconnections();
  signal2.flushDisconnections();

  EXPECT_FALSE(connection0.isConnected());
  EXPECT_FALSE(connection1.isConnected());
  EXPECT_FALSE(connection2.isConnected());
}

//==============================================================================
TEST(Signal, Id)
{
  Signal<void()> signal;
  Connection connection0 = signal.connect(&foo0);
  Connection connection1 = signal.connect(&foo0);
  Connection connection2 = signal.connect(&foo0);

  EXPECT_EQ(signal.getNumConnections(), 3);
  EXPECT_TRUE(connection0.isConnected());
  EXPECT_TRUE(connection1.isConnected());
  EXPECT_TRUE(connection2.isConnected());

  EXPECT_EQ(connection0.getId(), 0);
  EXPECT_EQ(connection1.getId(), 1);
  EXPECT_EQ(connection2.getId(), 2);

  // As disconnecting, the ID of connection1 is returned to the signal as free
  // ID to be assigned for new connection.
  connection1.disconnect();
  signal.flushDisconnections();

  EXPECT_EQ(signal.getNumConnections(), 2);
  EXPECT_TRUE(connection0.isConnected());
  EXPECT_FALSE(connection1.isConnected());
  EXPECT_TRUE(connection2.isConnected());

  // connection1 still has the previous ID but it's meaningless.
  EXPECT_EQ(connection0.getId(), 0);
  EXPECT_EQ(connection1.getId(), 1);
  EXPECT_EQ(connection2.getId(), 2);

  // Since ID 1 is now available, connection will get ID 1.
  Connection connection3 = signal.connect(&foo0);

  EXPECT_EQ(signal.getNumConnections(), 3);
  EXPECT_TRUE(connection0.isConnected());
  EXPECT_FALSE(connection1.isConnected());
  EXPECT_TRUE(connection2.isConnected());
  EXPECT_TRUE(connection3.isConnected());

  EXPECT_EQ(connection0.getId(), 0);
  EXPECT_EQ(connection1.getId(), 1);
  EXPECT_EQ(connection2.getId(), 2);
  EXPECT_EQ(connection3.getId(), 1);
}

//==============================================================================
TEST(Signal, NonStaticMemberFunction)
{
  Signal<void(int)> signal1;
  Signal<void(int, float)> signal2;
  Viewer viewer;

  // Connect static member function
  signal1.connect(&Viewer::onSignal1Static);
  signal2.connect(&Viewer::onSignal2Static);

  // Connect non-static member function
  using std::placeholders::_1;
  using std::placeholders::_2;
  signal1.connect(std::bind(&Viewer::onSignal1, &viewer, _1));
  signal2.connect(std::bind(&Viewer::onSignal2, &viewer, _1, _2));

  // The signal should have the maximum number of listeners
  EXPECT_EQ(signal1.getNumConnections(), 2);
  EXPECT_EQ(signal2.getNumConnections(), 2);

  // Check the number of calls
  callCount1 = 0;
  callCount2 = 0;
  signal1.raise(0);
  signal2.raise(0, 0);
  EXPECT_EQ(callCount1, 2);
  EXPECT_EQ(callCount2, 2);
}

//==============================================================================
float product(float x, float y) { return x * y; }
float quotient(float x, float y) { return x / y; }
float sum(float x, float y) { return x + y; }
float difference(float x, float y) { return x - y; }

// combiner which returns the maximum value returned by all slots
template <typename T>
struct maximum
{
  typedef T result_type;

  template <typename InputIterator>
  T operator()(InputIterator first, InputIterator last) const
  {
    // If there are no slots to call, just return the
    // default-constructed value
    if (first == last)
      return T();

    T max_value = *first++;

    while (first != last)
    {
      if (max_value < *first)
        max_value = *first;
      ++first;
    }

    return max_value;
  }
};

//==============================================================================
TEST(Signal, ReturnValues)
{
  Signal<float(float, float)> signal1;

  signal1.connect(&product);
  signal1.connect(&quotient);
  signal1.connect(&sum);
  signal1.connect(&difference);

  EXPECT_EQ(signal1(5, 3), 2);

  Signal<float(float, float), maximum> signal2;

  signal2.connect(&product);
  signal2.connect(&quotient);
  signal2.connect(&sum);
  signal2.connect(&difference);

  EXPECT_EQ(signal2(5, 3), 15);
}

//==============================================================================
TEST(Signal, SignalToSignal)
{
  Signal<void(int)> signal1;
  Signal<void(int)> signal2;

  Connection connection = signal1.connect(&signal2);
  signal2.connect(foo1);
  signal2.connect(foo1);
  signal2.connect(foo1);
  signal2.connect(foo1);

  signal1.raise(0);

  // Check the number of calls
  callCount1 = 0;
  signal1.raise(0);
  EXPECT_EQ(callCount1, 4);

  // Check the number of calls
  callCount1 = 0;
  signal1(0);
  EXPECT_EQ(callCount1, 4);

  connection.disconnect();

  // Check the number of calls
  callCount1 = 0;
  signal1.raise(0);
  EXPECT_EQ(callCount1, 0);

  // Check the number of calls
  callCount1 = 0;
  signal1(0);
  EXPECT_EQ(callCount1, 0);

  signal1.disconnectAll();
  EXPECT_EQ(signal1.getNumConnections(), 0);
  auto connection2 = signal1.connect(&signal1);
  EXPECT_EQ(signal1.getNumConnections(), 0);
  EXPECT_FALSE(connection2.isConnected());
  signal1(0);
  EXPECT_EQ(callCount1, 0);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
