/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/Shape.h"

#define PRIMITIVE_MAGIC_NUMBER 1000

namespace dart {
namespace dynamics {
//==============================================================================
Shape::Shape(ShapeType _type)
  : mBoundingBox(),
    mVolume(0.0),
    mID(mCounter++),
    mVariance(STATIC),
    mType(_type)
{
}

//==============================================================================
Shape::~Shape()
{
  // Do nothing
}

//==============================================================================
const math::BoundingBox& Shape::getBoundingBox() const
{
    return mBoundingBox;
}

//==============================================================================
double Shape::getVolume() const
{
  return mVolume;
}

//==============================================================================
int Shape::getID() const
{
  return mID;
}

//==============================================================================
Shape::ShapeType Shape::getShapeType() const
{
  return mType;
}

//==============================================================================
void Shape::setDataVariance(unsigned int _variance)
{
  mVariance = _variance;
}

//==============================================================================
void Shape::addDataVariance(unsigned int _variance)
{
  mVariance |= _variance;
}

//==============================================================================
void Shape::removeDataVariance(unsigned int _variance)
{
  mVariance &= ~_variance;
}

//==============================================================================
unsigned int Shape::getDataVariance() const
{
  return mVariance;
}

//==============================================================================
bool Shape::checkDataVariance(DataVariance type) const
{
  if(STATIC == type)
    return STATIC == mVariance;

  return (type & mVariance) != 0x00;
}

//==============================================================================
void Shape::refreshData()
{
  // Do nothing
}

//==============================================================================
void Shape::notifyAlphaUpdate(double /*alpha*/)
{
  // Do nothing
}

//==============================================================================
void Shape::notifyColorUpdate(const Eigen::Vector4d& /*color*/)
{
  // Do nothing
}

//==============================================================================
int Shape::mCounter = PRIMITIVE_MAGIC_NUMBER;

}  // namespace dynamics
}  // namespace dart
