/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_COMMON_SIGNAL_H_
#define DART_COMMON_SIGNAL_H_

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>

namespace dart {
namespace common {

namespace signal {
namespace detail {

template <typename T> struct DefaultCombiner;

}  // namespace detail
}  // namespace signal

template <typename _Signature, template<class> class Combiner = signal::detail::DefaultCombiner>
class Signal;

/// Non-thread safe implementation of Signal for general slots
template <typename _Res, typename... _ArgTypes, template<class> class Combiner>
class Signal<_Res(_ArgTypes...), Combiner>
{
public:
  using ResultType = _Res;
  using SlotType   = std::function<ResultType(_ArgTypes...)>;

  class Connection;
  friend class Connection;

  /// Constructor
  Signal() {}

  /// Destructor
  ~Signal() {}

  /// Connect a slot to the signal
  Connection connect(const SlotType& _slot)
  {
    const SlotType* slotPtr = new SlotType(_slot);

    mSlots.push_back(slotPtr);

    return Connection(this, slotPtr);
  }

  /// Connect another signal
  Connection connect(Signal<ResultType(_ArgTypes...), Combiner>* _signal)
  {
    if (this == _signal)
      return Connection();

    using std::placeholders::_1;
    return connect(std::bind(std::mem_fn(&Signal<ResultType(_ArgTypes...), Combiner>::raise), _signal, _1));
  }

  /// Connect another signal
  Connection connect(const Signal<ResultType(_ArgTypes...), Combiner>& _signal)
  {
    if (this == &_signal)
      return;

    using std::placeholders::_1;
    return connect(std::bind(std::mem_fn(&Signal<ResultType(_ArgTypes...), Combiner>::raise), &_signal, _1));
  }

  // Note: To implement void disconnect(const SlotType&) function,
  // std::function should provide == operator, which is not included in C++11
  // In the meantime, we use Connection to disconnect slots.

  /// Disconnect all the slots
  void disconnectAll()
  {
    for (const auto& slot : mSlots)
      delete slot;

    mSlots.clear();
  }

  /// Return the number of slots
  size_t getNumSlots() const { return mSlots.size(); }

  /// Raise signal
  ResultType raise(const _ArgTypes&... _args)
  {
    std::vector<ResultType> res(mSlots.size());

    size_t i = 0;
    for (const auto& slot : mSlots)
      res[i++] = (*slot)(_args...);

    Combiner<ResultType> getResult;
    return getResult(res.begin(), res.end());
  }

  /// Invoke the signal
  template <class ...Args>
  ResultType operator()(const _ArgTypes&... _args)
  {
    return raise(_args...);
  }

protected:
  /// Disconnect
  void disconnect(const SlotType* _slotPtr)
  {
    const auto& res
        = std::find(mSlots.begin(), mSlots.end(), _slotPtr);

    if (mSlots.end() == res)
      return;

    mSlots.erase(
          std::remove(mSlots.begin(), mSlots.end(), _slotPtr));

    delete _slotPtr;
  }

private:
  /// List of slots
  std::vector<const SlotType*> mSlots;

public:
  /// Connection provides useful feature
  class Connection
  {
  public:
    /// Default constructor
    Connection()
      : mIsConnected(false),
        mSignal(nullptr),
        mSlotPtr(nullptr) {}

    /// Constructor
    Connection(Signal<ResultType(_ArgTypes...), Combiner>* _signal, const SlotType* _slotPtr)
      : mIsConnected(true),
        mSignal(_signal),
        mSlotPtr(_slotPtr) {}

    /// Destructor
    ~Connection() {}

    /// Get true if the slot is connected
    bool connected() const
    {
      if (nullptr == mSignal || nullptr == mSlotPtr)
        return false;

      return mIsConnected;
    }

    /// Disconnect the slot
    void disconnect()
    {
      if (!connected())
        return;

      mSignal->disconnect(mSlotPtr);
      mIsConnected = false;
    }

  protected:
    /// True
    bool mIsConnected;

    /// Signal
    Signal<ResultType(_ArgTypes...), Combiner>* mSignal;

    /// Slot
    const SlotType* mSlotPtr;
  };
};

/// Non-thread safe implementation of Signal for slots not returning value
template <typename... _ArgTypes>
class Signal<void(_ArgTypes...)>
{
public:
  using SlotType = std::function<void(_ArgTypes...)>;

  class Connection;
  friend class Connection;

  /// Constructor
  Signal() {}

  /// Destructor
  ~Signal() {}

  /// Connect a slot to the signal
  Connection connect(const SlotType& _slot)
  {
    const SlotType* slotPtr = new SlotType(_slot);

    mSlots.push_back(slotPtr);

    return Connection(this, slotPtr);
  }

  /// Connect another signal
  Connection connect(Signal<void(_ArgTypes...)>* _signal)
  {
    if (this == _signal)
      return Connection();

    using std::placeholders::_1;
    return connect(std::bind(std::mem_fn(&Signal<void(_ArgTypes...)>::raise), _signal, _1));
  }

  /// Connect another signal
  Connection connect(const Signal<void(_ArgTypes...)>& _signal)
  {
    if (this == &_signal)
      return;

    using std::placeholders::_1;
    return connect(std::bind(std::mem_fn(&Signal<void(_ArgTypes...)>::raise), &_signal, _1));
  }

  // Note: To implement void disconnect(const SlotType&) function,
  // std::function should provide == operator, which is not included in C++11
  // In the meantime, we use Connection to disconnect slots.

  /// Disconnect all the slots
  void disconnectAll()
  {
    for (const auto& slot : mSlots)
      delete slot;

    mSlots.clear();
  }

  /// Return the number of slots
  size_t getNumSlots() const { return mSlots.size(); }

  /// Raise signal
  void raise(const _ArgTypes&... _args)
  {
    for (const auto& slot : mSlots)
      (*slot)(_args...);
  }

  /// Invoke the signal
  template <class ...Args>
  void operator()(const _ArgTypes&... _args)
  {
    raise(_args...);
  }

  protected:
  /// Disconnect
  void disconnect(const SlotType* _slotPtr)
  {
    const auto& res
        = std::find(mSlots.begin(), mSlots.end(), _slotPtr);

    if (mSlots.end() == res)
      return;

    mSlots.erase(
          std::remove(mSlots.begin(), mSlots.end(), _slotPtr));

    delete _slotPtr;
  }

  private:
  /// List of slots
  std::vector<const SlotType*> mSlots;

  public:
  /// Connection provides useful feature
  class Connection
  {
  public:
    /// Default constructor
    Connection()
      : mIsConnected(false),
        mSignal(nullptr),
        mSlotPtr(nullptr) {}

    /// Constructor
    Connection(Signal<void(_ArgTypes...)>* _signal, const SlotType* _slotPtr)
      : mIsConnected(true),
        mSignal(_signal),
        mSlotPtr(_slotPtr) {}

    /// Destructor
    ~Connection() {}

    /// Get true if the slot is connected
    bool connected() const
    {
      if (nullptr == mSignal || nullptr == mSlotPtr)
        return false;

      return mIsConnected;
    }

    /// Disconnect the slot
    void disconnect()
    {
      if (!connected())
        return;

      mSignal->disconnect(mSlotPtr);
      mIsConnected = false;
    }

  protected:
    /// True
    bool mIsConnected;

    /// Signal
    Signal<void(_ArgTypes...)>* mSignal;

    /// Slot
    const SlotType* mSlotPtr;
  };
};

namespace signal {
namespace detail {

/// Return the last result
template <typename T>
struct DefaultCombiner
{
  typedef T result_type;

  template <typename InputIterator>
  T operator()(InputIterator first, InputIterator last) const
  {
    // If there are no slots to call, just return the
    // default-constructed value
    if (first == last)
      return T();

    return *(--last);
  }
};

}  // namespace detail
}  // namespace signal

}  // namespace common
}  // namespace dart

#endif  // DART_COMMON_SIGNAL_H_
