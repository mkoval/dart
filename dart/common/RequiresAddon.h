/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#ifndef DART_COMMON_REQUIRESADDON_H_
#define DART_COMMON_REQUIRESADDON_H_

#include "dart/common/SpecializedForAddon.h"

namespace dart {
namespace common {

//==============================================================================
/// RequiresAddon allows classes that inherit AddonManager to know which Addons
/// are required for their operation. This guarantees that there is no way for
/// a required Addon do not get unexpectedly removed from their manager.
///
/// Required Addons are also automatically specialized for.
template <class... OtherRequiredAddons>
class RequiresAddon { };

//==============================================================================
template <class ReqAddon>
class RequiresAddon<ReqAddon> : public virtual SpecializedForAddon<ReqAddon>
{
public:

  /// Default constructor. This is where the base AddonManager is informed that
  /// the Addon type is required.
  RequiresAddon();

};

//==============================================================================
template <class ReqAddon1, class... OtherReqAddons>
class RequiresAddon<ReqAddon1, OtherReqAddons...> :
    public AddonManagerJoiner< Virtual< RequiresAddon<ReqAddon1> >,
                               Virtual< RequiresAddon<OtherReqAddons...> > > { };

} // namespace common
} // namespace dart

#include "dart/common/detail/RequiresAddon.h"

#endif // DART_COMMON_REQUIRESADDON_H_
