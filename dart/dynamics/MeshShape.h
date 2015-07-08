/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s):
 * Date:
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

#ifndef DART_DYNAMICS_MESHSHAPE_H_
#define DART_DYNAMICS_MESHSHAPE_H_

#include <string>

#include <assimp/scene.h>

#include "dart/dynamics/Shape.h"

namespace dart {
namespace dynamics {

/// \brief
class MeshShape : public Shape {
public:
  /// \brief Constructor.
  MeshShape(const Eigen::Vector3d& _scale, const aiScene* _mesh,
            const std::string &path = std::string());

  /// \brief Destructor.
  virtual ~MeshShape();

  /// \brief
  const aiScene* getMesh() const;

  /// \brief
  void setMesh(const aiScene* _mesh, const std::string &path = std::string());

  /// \brief Path to the mesh on disk; an empty string if unavailable.
  const std::string &getMeshPath() const;

  /// \brief
  void setScale(const Eigen::Vector3d& _scale);

  /// \brief
  const Eigen::Vector3d& getScale() const;

  /// \brief
  int getDisplayList() const;

  /// \brief
  void setDisplayList(int _index);

  // Documentation inherited.
  void draw(renderer::RenderInterface* _ri = nullptr,
            const Eigen::Vector4d& _col = Eigen::Vector4d::Ones(),
            bool _default = true) const;

  /// \brief
  static const aiScene* loadMesh(const uint8_t* _data, size_t length);

  /// \brief
  static const aiScene* loadMesh(const std::string& _fileName);

  // Documentation inherited.
  virtual Eigen::Matrix3d computeInertia(double _mass) const;

protected:
  // Documentation inherited.
  virtual void computeVolume();

private:
  /// \brief
  void _updateBoundingBoxDim();

protected:
  /// \brief
  const aiScene* mMesh;

  /// \brief
  std::string mMeshPath;

  /// \brief OpenGL DisplayList id for rendering
  int mDisplayList;

  /// \brief Scale
  Eigen::Vector3d mScale;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_MESHSHAPE_H_
