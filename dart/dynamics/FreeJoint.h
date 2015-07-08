/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_FREEJOINT_H_
#define DART_DYNAMICS_FREEJOINT_H_

#include <string>

#include <Eigen/Dense>

#include "dart/dynamics/MultiDofJoint.h"

namespace dart {
namespace dynamics {

/// class FreeJoint
class FreeJoint : public MultiDofJoint<6>
{
public:

  friend class Skeleton;

  struct Properties : MultiDofJoint<6>::Properties
  {
    Properties(const MultiDofJoint<6>::Properties& _properties =
                                                MultiDofJoint<6>::Properties());

    virtual ~Properties() = default;
  };

  /// Destructor
  virtual ~FreeJoint();

  /// Get the Properties of this FreeJoint
  Properties getFreeJointProperties() const;

  // Documentation inherited
  virtual const std::string& getType() const override;

  /// Get joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  virtual bool isCyclic(size_t _index) const override;

  /// Convert a transform into a 6D vector that can be used to set the positions
  /// of a FreeJoint. The positions returned by this function will result in a
  /// relative transform of
  /// getTransformFromParentBodyNode() * _tf * getTransformFromChildBodyNode().inverse()
  /// between the parent BodyNode and the child BodyNode frames when applied to
  /// a FreeJoint.
  static Eigen::Vector6d convertToPositions(const Eigen::Isometry3d& _tf);

  /// Convert a FreeJoint-style 6D vector into a transform
  static Eigen::Isometry3d convertToTransform(const Eigen::Vector6d& _positions);

  // Documentation inherited
  Eigen::Matrix6d getLocalJacobianStatic(
      const Eigen::Vector6d& _positions) const override;

  // Documentation inherited
  Eigen::Vector6d getPositionDifferencesStatic(
      const Eigen::Vector6d& _q2, const Eigen::Vector6d& _q1) const override;

protected:

  /// Constructor called by Skeleton class
  FreeJoint(const Properties& _properties);

  // Documentation inherited
  virtual Joint* clone() const override;

  using MultiDofJoint::getLocalJacobianStatic;

  // Documentation inherited
  virtual void integratePositions(double _dt) override;

  // Documentation inherited
  virtual void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  virtual void updateLocalTransform() const override;

  // Documentation inherited
  virtual void updateLocalJacobian(bool =true) const override;

  // Documentation inherited
  virtual void updateLocalJacobianTimeDeriv() const override;

protected:

  /// Access mQ, which is an auto-updating variable
  const Eigen::Isometry3d& getQ() const;

  /// Transformation matrix dependent on generalized coordinates
  ///
  /// Do not use directly! Use getQ() to access this
  mutable Eigen::Isometry3d mQ;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_FREEJOINT_H_
