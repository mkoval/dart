/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"

using namespace dart;
using namespace dart::math;
using namespace dart::dynamics;
using namespace dart::simulation;

using std::cout;
using std::endl;

#define JOINT_TOL 0.01

//==============================================================================
class Joints : public testing::Test
{
public:
  void kinematicsTest(Joint* _joint);
};

//==============================================================================
void Joints::kinematicsTest(Joint* _joint)
{
  assert(_joint);

  int numTests = 1;

  _joint->setTransformFromChildBodyNode(
        math::expMap(Eigen::Vector6d::Random()));
  _joint->setTransformFromParentBodyNode(
        math::expMap(Eigen::Vector6d::Random()));

  BodyNode* bodyNode = new BodyNode();
  bodyNode->setParentJoint(_joint);

  Skeleton skeleton;
  skeleton.addBodyNode(bodyNode);
  skeleton.init();

  int dof = _joint->getNumDofs();

  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  VectorXd q = VectorXd::Zero(dof);
  VectorXd dq = VectorXd::Zero(dof);

  for (int idxTest = 0; idxTest < numTests; ++idxTest)
  {
    double q_delta = 0.000001;

    for (int i = 0; i < dof; ++i)
    {
      q(i) = random(-DART_PI*1.0, DART_PI*1.0);
      dq(i) = random(-DART_PI*1.0, DART_PI*1.0);
    }

    skeleton.setPositions(q);
    skeleton.setVelocities(dq);
    skeleton.computeForwardKinematics(true, true, false);

    if (_joint->getNumDofs() == 0)
      return;

    Eigen::Isometry3d T = _joint->getLocalTransform();
    Jacobian J = _joint->getLocalJacobian();
    Jacobian dJ = _joint->getLocalJacobianTimeDeriv();

    //--------------------------------------------------------------------------
    // Test T
    //--------------------------------------------------------------------------
    EXPECT_TRUE(math::verifyTransform(T));

    //--------------------------------------------------------------------------
    // Test analytic Jacobian and numerical Jacobian
    // J == numericalJ
    //--------------------------------------------------------------------------
    Jacobian numericJ = Jacobian::Zero(6,dof);
    for (int i = 0; i < dof; ++i)
    {
      // a
      Eigen::VectorXd q_a = q;
      _joint->setPositions(q_a);
      skeleton.computeForwardKinematics(true, false, false);
      Eigen::Isometry3d T_a = _joint->getLocalTransform();

      // b
      Eigen::VectorXd q_b = q;
      q_b(i) += q_delta;
      _joint->setPositions(q_b);
      skeleton.computeForwardKinematics(true, false, false);
      Eigen::Isometry3d T_b = _joint->getLocalTransform();

      //
      Eigen::Isometry3d Tinv_a = T_a.inverse();
      Eigen::Matrix4d Tinv_a_eigen = Tinv_a.matrix();

      // dTdq
      Eigen::Matrix4d T_a_eigen = T_a.matrix();
      Eigen::Matrix4d T_b_eigen = T_b.matrix();
      Eigen::Matrix4d dTdq_eigen = (T_b_eigen - T_a_eigen) / q_delta;
      //Matrix4d dTdq_eigen = (T_b_eigen * T_a_eigen.inverse()) / dt;

      // J(i)
      Eigen::Matrix4d Ji_4x4matrix_eigen = Tinv_a_eigen * dTdq_eigen;
      Eigen::Vector6d Ji;
      Ji[0] = Ji_4x4matrix_eigen(2,1);
      Ji[1] = Ji_4x4matrix_eigen(0,2);
      Ji[2] = Ji_4x4matrix_eigen(1,0);
      Ji[3] = Ji_4x4matrix_eigen(0,3);
      Ji[4] = Ji_4x4matrix_eigen(1,3);
      Ji[5] = Ji_4x4matrix_eigen(2,3);
      numericJ.col(i) = Ji;
    }

    if (/*dynamic_cast<BallJoint*>(_joint) || */dynamic_cast<FreeJoint*>(_joint))
    {
      // Skip this test for BallJoint and FreeJoint since Jacobian of BallJoint
      // and FreeJoint is not obtained by the above method.
    }
    else
    {
      for (int i = 0; i < dof; ++i)
        for (int j = 0; j < 6; ++j)
          EXPECT_NEAR(J.col(i)(j), numericJ.col(i)(j), JOINT_TOL);
    }

    //--------------------------------------------------------------------------
    // Test first time derivative of analytic Jacobian and numerical Jacobian
    // dJ == numerical_dJ
    //--------------------------------------------------------------------------
    Jacobian numeric_dJ = Jacobian::Zero(6,dof);
    for (int i = 0; i < dof; ++i)
    {
      // a
      Eigen::VectorXd q_a = q;
      _joint->setPositions(q_a);
      skeleton.computeForwardKinematics(true, false, false);
      Jacobian J_a = _joint->getLocalJacobian();

      // b
      Eigen::VectorXd q_b = q;
      q_b(i) += q_delta;
      _joint->setPositions(q_b);
      skeleton.computeForwardKinematics(true, false, false);
      Jacobian J_b = _joint->getLocalJacobian();

      //
      Jacobian dJ_dq = (J_b - J_a) / q_delta;

      // J(i)
      numeric_dJ += dJ_dq * dq(i);
    }


    if (/*dynamic_cast<BallJoint*>(_joint) || */dynamic_cast<FreeJoint*>(_joint))
    {
      // Skip this test for BallJoint and FreeJoint since time derivative of
      // Jacobian of BallJoint and FreeJoint is not obtained by the above
      // method.
    }
    else
    {
      for (int i = 0; i < dof; ++i)
        for (int j = 0; j < 6; ++j)
          EXPECT_NEAR(dJ.col(i)(j), numeric_dJ.col(i)(j), JOINT_TOL);
    }
  }

  // Forward kinematics test with high joint position
  double posMin = -1e+64;
  double posMax = +1e+64;

  for (int idxTest = 0; idxTest < numTests; ++idxTest)
  {
    for (int i = 0; i < dof; ++i)
      q(i) = random(posMin, posMax);

    skeleton.setPositions(q);
    skeleton.computeForwardKinematics(true, false, false);

    if (_joint->getNumDofs() == 0)
      return;

    Eigen::Isometry3d T = _joint->getLocalTransform();
    EXPECT_TRUE(math::verifyTransform(T));
  }
}

// 0-dof joint
TEST_F(Joints, WELD_JOINT)
{
  WeldJoint* weldJoint = new WeldJoint;

  kinematicsTest(weldJoint);
}

// 1-dof joint
TEST_F(Joints, REVOLUTE_JOINT)
{
  RevoluteJoint* revJoint = new RevoluteJoint;

  kinematicsTest(revJoint);
}

// 1-dof joint
TEST_F(Joints, PRISMATIC_JOINT)
{
  PrismaticJoint* priJoint = new PrismaticJoint;

  kinematicsTest(priJoint);
}

// 1-dof joint
TEST_F(Joints, SCREW_JOINT)
{
  ScrewJoint* screwJoint = new ScrewJoint;

  kinematicsTest(screwJoint);
}

// 2-dof joint
TEST_F(Joints, UNIVERSAL_JOINT)
{
  UniversalJoint* univJoint = new UniversalJoint;

  kinematicsTest(univJoint);
}

// 3-dof joint
TEST_F(Joints, BALL_JOINT)
{
  BallJoint* ballJoint = new BallJoint;

  kinematicsTest(ballJoint);
}

// 3-dof joint
TEST_F(Joints, EULER_JOINT)
{
  EulerJoint* eulerJoint1 = new EulerJoint;

  eulerJoint1->setAxisOrder(EulerJoint::AO_XYZ);
  kinematicsTest(eulerJoint1);

  EulerJoint* eulerJoint2 = new EulerJoint;

  eulerJoint2->setAxisOrder(EulerJoint::AO_ZYX);
  kinematicsTest(eulerJoint2);
}

// 3-dof joint
TEST_F(Joints, TRANSLATIONAL_JOINT)
{
  TranslationalJoint* translationalJoint = new TranslationalJoint;

  kinematicsTest(translationalJoint);
}

// 3-dof joint
TEST_F(Joints, PLANAR_JOINT)
{
  PlanarJoint* planarJoint = new PlanarJoint;

  kinematicsTest(planarJoint);
}

// 6-dof joint
TEST_F(Joints, FREE_JOINT)
{
  FreeJoint* freeJoint = new FreeJoint;

  kinematicsTest(freeJoint);
}

//==============================================================================
TEST_F(Joints, POSITION_LIMIT)
{
  double tol = 1e-3;

  simulation::World* myWorld
      = utils::SkelParser::readWorld(
          DART_DATA_PATH"/skel/test/joint_limit_test.skel");
  EXPECT_TRUE(myWorld != NULL);

  myWorld->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));

  dynamics::Skeleton* pendulum = myWorld->getSkeleton("double_pendulum");
  EXPECT_TRUE(pendulum != NULL);

  dynamics::Joint* joint0 = pendulum->getJoint("joint0");
  dynamics::Joint* joint1 = pendulum->getJoint("joint1");

  EXPECT_TRUE(joint0 != NULL);
  EXPECT_TRUE(joint1 != NULL);

  double limit0 = DART_PI / 6.0;
  double limit1 = DART_PI / 6.0;

  joint0->setPositionLimited(true);
  joint0->setPositionLowerLimit(0, -limit0);
  joint0->setPositionUpperLimit(0, limit0);

  joint1->setPositionLimited(true);
  joint1->setPositionLowerLimit(0, -limit1);
  joint1->setPositionUpperLimit(0, limit1);

  double simTime = 2.0;
  double timeStep = myWorld->getTimeStep();
  int nSteps = simTime / timeStep;

  // Two seconds with positive control forces
  for (int i = 0; i < nSteps; i++)
  {
    joint0->setForce(0, 0.1);
    joint1->setForce(0, 0.1);
    myWorld->step();

    double jointPos0 = joint0->getPosition(0);
    double jointPos1 = joint1->getPosition(0);

    EXPECT_GE(jointPos0, -limit0 - tol);
    EXPECT_GE(jointPos1, -limit1 - tol);

    EXPECT_LE(jointPos0, limit0 + tol);
    EXPECT_LE(jointPos1, limit1 + tol);
  }

  // Two more seconds with negative control forces
  for (int i = 0; i < nSteps; i++)
  {
    joint0->setForce(0, -0.1);
    joint1->setForce(0, -0.1);
    myWorld->step();

    double jointPos0 = joint0->getPosition(0);
    double jointPos1 = joint1->getPosition(0);

    EXPECT_GE(jointPos0, -limit0 - tol);
    EXPECT_GE(jointPos1, -limit1 - tol);

    EXPECT_LE(jointPos0, limit0 + tol);
    EXPECT_LE(jointPos1, limit1 + tol);
  }
}

//==============================================================================
TEST_F(Joints, RotationGenCoordTypes)
{
  // Build test skeleton
  const double tol = 1e-12;
  Skeleton* skel = new Skeleton();
  BodyNode* bodyNode = new BodyNode();
  BallJoint* ballJoint = new BallJoint();
  bodyNode->setParentJoint(ballJoint);
  skel->addBodyNode(bodyNode);
  skel->init();

  // Set random positions for the ball joint
  Eigen::Matrix3d R = math::expMapRot(Eigen::Vector3d::Random());
  skel->setRotationGenCoordType(RotationGenCoordType::SO3_LIE_ALGEBRA);
  skel->setPositions(math::logMap(R));

  //----------------------------------------------------------------------------
  // Test for all the rotation generalized coordinate types
  //----------------------------------------------------------------------------
  Eigen::Vector3d genCoords1;
  Eigen::Vector3d genCoords2;

  skel->setRotationGenCoordType(RotationGenCoordType::SO3_LIE_ALGEBRA);
  genCoords1 = skel->getPositions();
  genCoords2 = math::logMap(R);
  EXPECT_NEAR(genCoords1[0], genCoords2[0], tol);
  EXPECT_NEAR(genCoords1[1], genCoords2[1], tol);
  EXPECT_NEAR(genCoords1[2], genCoords2[2], tol);

  skel->setRotationGenCoordType(RotationGenCoordType::EULER_INTRINSIC_XYZ);
  genCoords1 = skel->getPositions();
  genCoords2 = math::matrixToEulerXYZ(R);
  EXPECT_NEAR(genCoords1[0], genCoords2[0], tol);
  EXPECT_NEAR(genCoords1[1], genCoords2[1], tol);
  EXPECT_NEAR(genCoords1[2], genCoords2[2], tol);

  skel->setRotationGenCoordType(RotationGenCoordType::EULER_INTRINSIC_ZYX);
  genCoords1 = skel->getPositions();
  genCoords2 = math::matrixToEulerZYX(R);
  EXPECT_NEAR(genCoords1[0], genCoords2[0], tol);
  EXPECT_NEAR(genCoords1[1], genCoords2[1], tol);
  EXPECT_NEAR(genCoords1[2], genCoords2[2], tol);

  skel->setRotationGenCoordType(RotationGenCoordType::EULER_EXTRINSIC_XYZ);
  genCoords1 = skel->getPositions();
  genCoords2 = math::matrixToEulerZYX(R).reverse();
  EXPECT_NEAR(genCoords1[0], genCoords2[0], tol);
  EXPECT_NEAR(genCoords1[1], genCoords2[1], tol);
  EXPECT_NEAR(genCoords1[2], genCoords2[2], tol);

  skel->setRotationGenCoordType(RotationGenCoordType::EULER_EXTRINSIC_ZYX);
  genCoords1 = skel->getPositions();
  genCoords2 = math::matrixToEulerXYZ(R).reverse();
  EXPECT_NEAR(genCoords1[0], genCoords2[0], tol);
  EXPECT_NEAR(genCoords1[1], genCoords2[1], tol);
  EXPECT_NEAR(genCoords1[2], genCoords2[2], tol);

  delete skel;
}

//==============================================================================
TEST_F(Joints, TransformGenCoordTypes)
{
  // Build test skeleton
  const double tol = 1e-12;
  Skeleton* skel = new Skeleton();
  BodyNode* bodyNode = new BodyNode();
  FreeJoint* ballJoint = new FreeJoint();
  bodyNode->setParentJoint(ballJoint);
  skel->addBodyNode(bodyNode);
  skel->init();

  // Set random positions for the ball joint
  Eigen::Isometry3d T;
  T.linear() = math::expMapRot(Eigen::Vector3d::Random());
  T.translation() = Eigen::Vector3d::Random();
  skel->setTransformGenCoordType(TransformGenCoordType::SE3_LIE_ALGEBRA);
  skel->setPositions(math::logMap(T));

  //----------------------------------------------------------------------------
  // Test for all the rotation generalized coordinate types
  //----------------------------------------------------------------------------
  Eigen::Vector6d genCoords1;
  Eigen::Vector6d genCoords2;

  skel->setTransformGenCoordType(TransformGenCoordType::SE3_LIE_ALGEBRA);
  genCoords1 = skel->getPositions();
  genCoords2 = math::logMap(T);
  EXPECT_TRUE(equals(genCoords1, genCoords2, tol));

  skel->setTransformGenCoordType(TransformGenCoordType::SO3_LIE_ALGEBRA_AND_POSITION);
  genCoords1 = skel->getPositions();
  genCoords2.head<3>() = math::logMap(T.linear());
  genCoords2.tail<3>() = T.translation();
  EXPECT_TRUE(equals(genCoords1, genCoords2, tol));

  skel->setTransformGenCoordType(TransformGenCoordType::POSITION_AND_SO3_LIE_ALGEBRA);
  genCoords1 = skel->getPositions();
  genCoords2.head<3>() = T.translation();
  genCoords2.tail<3>() = math::logMap(T.linear());
  EXPECT_TRUE(equals(genCoords1, genCoords2, tol));

  skel->setTransformGenCoordType(TransformGenCoordType::POSITION_AND_EULER_INTRINSIC_XYZ);
  genCoords1 = skel->getPositions();
  genCoords2.head<3>() = T.translation();
  genCoords2.tail<3>() = math::matrixToEulerXYZ(T.linear());
  EXPECT_TRUE(equals(genCoords1, genCoords2, tol));

  skel->setTransformGenCoordType(TransformGenCoordType::POSITION_AND_EULER_INTRINSIC_ZYX);
  genCoords1 = skel->getPositions();
  genCoords2.head<3>() = T.translation();
  genCoords2.tail<3>() = math::matrixToEulerZYX(T.linear());
  EXPECT_TRUE(equals(genCoords1, genCoords2, tol));

  skel->setTransformGenCoordType(TransformGenCoordType::POSITION_AND_EULER_EXTRINSIC_XYZ);
  genCoords1 = skel->getPositions();
  genCoords2.head<3>() = T.translation();
  genCoords2.tail<3>() = math::matrixToEulerZYX(T.linear()).reverse();
  EXPECT_TRUE(equals(genCoords1, genCoords2, tol));

  skel->setTransformGenCoordType(TransformGenCoordType::POSITION_AND_EULER_EXTRINSIC_ZYX);
  genCoords1 = skel->getPositions();
  genCoords2.head<3>() = T.translation();
  genCoords2.tail<3>() = math::matrixToEulerXYZ(T.linear()).reverse();
  EXPECT_TRUE(equals(genCoords1, genCoords2, tol));

  delete skel;
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

