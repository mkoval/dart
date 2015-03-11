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
  static void onEvent1Static(int /*_val*/) { callCount1++; }
  static void onEvent2Static(int /*_val1*/, float /*_val2*/) { callCount2++; }
  void onEvent1(int /*_val*/) { callCount1++; }
  void onEvent2(int /*_val1*/, float /*_val2*/) { callCount2++; }
};

//==============================================================================
TEST(Signal, Basic)
{
  Signal<void()> event0;
  Signal<void(int)> event1;
  Signal<void(int, float)> event2;
  Signal<void()>::Connection connection0 = event0.connect(&foo0);
  Signal<void(int)>::Connection connection1 = event1.connect(&foo1);
  Signal<void(int, float)>::Connection connection2 = event2.connect(&foo2);

  EXPECT_EQ(event0.getNumSlots(), 1);
  EXPECT_EQ(event1.getNumSlots(), 1);
  EXPECT_EQ(event2.getNumSlots(), 1);

  EXPECT_TRUE(connection0.connected());
  EXPECT_TRUE(connection1.connected());
  EXPECT_TRUE(connection2.connected());

  connection0.disconnect();
  connection1.disconnect();
  connection2.disconnect();
  EXPECT_EQ(event0.getNumSlots(), 0);
  EXPECT_EQ(event1.getNumSlots(), 0);
  EXPECT_EQ(event2.getNumSlots(), 0);

  EXPECT_FALSE(connection0.connected());
  EXPECT_FALSE(connection1.connected());
  EXPECT_FALSE(connection2.connected());
}

//==============================================================================
TEST(Signal, NonStaticMemberFunction)
{
  Signal<void(int)> event1;
  Signal<void(int, float)> event2;
  Viewer viewer;

  // Connect static member function
  event1.connect(&Viewer::onEvent1Static);
  event2.connect(&Viewer::onEvent2Static);

  // Connect non-static member function
  using std::placeholders::_1;
  using std::placeholders::_2;
  event1.connect(std::bind(&Viewer::onEvent1, &viewer, _1));
  event2.connect(std::bind(&Viewer::onEvent2, &viewer, _1, _2));

  // The event should have the maximum number of listeners
  EXPECT_EQ(event1.getNumSlots(), 2);
  EXPECT_EQ(event2.getNumSlots(), 2);

  // Check the number of calls
  callCount1 = 0;
  callCount2 = 0;
  event1.raise(0);
  event2.raise(0, 0);
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
TEST(Signal, EventToEvent)
{
  Signal<void(int)> event1;
  Signal<void(int)> event2;

  Signal<void(int)>::Connection connection = event1.connect(&event2);
  event2.connect(foo1);
  event2.connect(foo1);
  event2.connect(foo1);
  event2.connect(foo1);

  event1.raise(0);

  // Check the number of calls
  callCount1 = 0;
  event1.raise(0);
  EXPECT_EQ(callCount1, 4);

  // Check the number of calls
  callCount1 = 0;
  event1(0);
  EXPECT_EQ(callCount1, 4);

  connection.disconnect();

  // Check the number of calls
  callCount1 = 0;
  event1.raise(0);
  EXPECT_EQ(callCount1, 0);

  // Check the number of calls
  callCount1 = 0;
  event1(0);
  EXPECT_EQ(callCount1, 0);

  event1.disconnectAll();
  EXPECT_EQ(event1.getNumSlots(), 0);
  auto connection2 = event1.connect(&event1);
  EXPECT_EQ(event1.getNumSlots(), 0);
  EXPECT_FALSE(connection2.connected());
  event1(0);
  EXPECT_EQ(callCount1, 0);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
