/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael Koval <mkoval@cs.cmu.edu>
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
#ifndef DART_UTILS_PACKAGERESOURCERETRIEVER_H_
#define DART_UTILS_PACKAGERESOURCERETRIEVER_H_
#include <unordered_map>
#include <vector>
#include "dart/utils/ResourceRetriever.h"

namespace dart {
namespace utils {

/// \brief Retrieve local resources specified by package:// URIs.
class PackageResourceRetriever : public virtual ResourceRetriever
{
public:
  PackageResourceRetriever(
    const ResourceRetrieverPtr& localRetriever = nullptr);

  virtual ~PackageResourceRetriever() = default;

  /// Specify the directory of a ROS package. In your URDF files, you may see
  /// strings with a package URI pattern such as:
  ///
  /// @code
  /// "package://my_robot/meshes/mesh_for_my_robot.stl"
  ///  \______/  \______/\___________________________/
  ///      |        |                 |
  ///   package  package   file path with respect to
  ///   keyword   name       the package directory
  /// @endcode
  ///
  /// For us to successfully parse a URDF, we need to be told what the path
  /// to the package directory is, using addPackageDirectory(). In this case,
  /// suppose the path to the my_robot package is /path/to/my_robot. Then you
  /// should use addPackageDirectory("my_robot", "/path/to/my_robot").
  /// Altogether, this implies that a file named
  /// "/path/to/my_robot/meshes/mesh_for_my_robot.stl" exists. Whatever you
  /// specify as the package directory will end up replacing the 'package
  /// keyword' and 'package name' components of the URI string.
  ///
  /// 
  void addPackageDirectory(const std::string& _packageName,
                           const std::string& _packageDirectory);

  virtual ConstMemoryResourcePtr retrieve(const std::string& _uri) override;

private:
  ResourceRetrieverPtr mLocalRetriever;
  std::unordered_map<std::string, std::vector<std::string> > mPackageMap;
};

typedef std::shared_ptr<PackageResourceRetriever> PackageResourceRetrieverPtr;

} // namespace utils
} // namespace dart

#endif // ifndef DART_UTILS_PACKAGERESOURCERETRIEVER_H_