/**
 * @file DartLoader.h
 */

#ifndef DART_UTILS_URDF_LOADER_H
#define DART_UTILS_URDF_LOADER_H

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <string>

#include "dart/common/Signal.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/simulation/World.h"

namespace urdf
{
  class ModelInterface;
  class Link;
  class Joint;
  class Pose;
  class Vector3;
}

namespace dart {

namespace dynamics
{
  class Skeleton;
  class BodyNode;
  class Joint;
  class Shape;
}
namespace simulation
{
  class World;
}

namespace utils {

namespace detail {

template <class T>
struct ResourceCombiner 
{
  typedef T result_type;

  template <typename InputIterator>
  static T process(InputIterator first, InputIterator last)
  {
    // Find the last element that is non-empty. 
    std::reverse_iterator<InputIterator> rbegin(last), rend(first);
    const auto it = std::find_if(rbegin, rend,
      [](const T &resourceData)
      {
        return !!resourceData;
      }
    );

    if (it != rend)
      return *it;
    else
      return T(); // Everything was empty.
  }
};

} // namespace detail


struct MemoryResource {
    MemoryResource()
        : mData(nullptr)
        , mSize(0)
    {
    }

    ~MemoryResource()
    {
      if(mData)
        delete mData;
    }

    std::string mPath;
    uint8_t *mData;
    size_t mSize;
};

typedef std::shared_ptr<MemoryResource> MemoryResourcePtr;


/**
 * @class DartLoader
 */
class DartLoader {
public:
    using ResourceRetrievalSignal
        = common::Signal<MemoryResourcePtr (const std::string &),
                         detail::ResourceCombiner>;

    DartLoader();

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
    void addPackageDirectory(const std::string& _packageName,
                             const std::string& _packageDirectory);

    /// Restore the default URI resolver that uses addPackageDirectory(~) to
    /// resolve URIs with the package:// schema. This should only be necessary
    /// if you modified the onResourceRetrieval slot.
    void restoreDefaultUriResolvers(); 

    /// Parse a file to produce a Skeleton
    dynamics::SkeletonPtr parseSkeleton(const std::string& _urdfFileName);

    /// Parse a text string to produce a Skeleton
    dynamics::SkeletonPtr parseSkeletonString(const std::string& _urdfString,
                                              const std::string& _urdfFileDirectory);

    /// Parse a file to produce a World
    dart::simulation::WorldPtr parseWorld(const std::string& _urdfFileName);

    /// Parse a text string to produce a World
    dart::simulation::WorldPtr parseWorldString(const std::string& _urdfString,
                                        const std::string& _urdfFileDirectory);

private:

    typedef std::shared_ptr<dynamics::BodyNode::Properties> BodyPropPtr;
    typedef std::shared_ptr<dynamics::Joint::Properties> JointPropPtr;

    void parseWorldToEntityPaths(const std::string& _xml_string);

    dart::dynamics::SkeletonPtr modelInterfaceToSkeleton(const urdf::ModelInterface* _model);
    bool createSkeletonRecursive(dynamics::SkeletonPtr _skel, const urdf::Link* _lk, dynamics::BodyNode* _parent);

    template <class VisualOrCollision>
    dynamics::ShapePtr createShape(const VisualOrCollision* _vizOrCol);

    dynamics::BodyNode* createDartJointAndNode(
        const urdf::Joint* _jt,
        const dynamics::BodyNode::Properties& _body,
        dynamics::BodyNode* _parent,
        dynamics::SkeletonPtr _skeleton);

    bool createDartNodeProperties(
        const urdf::Link* _lk, dynamics::BodyNode::Properties *properties);

    MemoryResourcePtr resolvePackageURI(const std::string &_filename) const;
    MemoryResourcePtr resolveRelativePath(const std::string &_filename) const;

    Eigen::Isometry3d toEigen(const urdf::Pose& _pose);
    Eigen::Vector3d toEigen(const urdf::Vector3& _vector);

    std::string readFileToString(std::string _xmlFile);
    MemoryResourcePtr readFileToResource(const std::string& _file) const;

    ResourceRetrievalSignal mResourceRetrievalSignal;
    std::map<std::string, std::string> mWorld_To_Entity_Paths;
    std::map<std::string, std::string> mPackageDirectories;
    std::string mRootToSkelPath;
    std::string mRootToWorldPath;

public:
  //----------------------------------------------------------------------------
  /// \{ \name Slot registers
  //----------------------------------------------------------------------------

  /// Slot register for package:// URI resolution.
  common::SlotRegister<ResourceRetrievalSignal> onResourceRetrieval;

  /// \}
};

}
}

#endif /** DART_UTILS_URDF_LOADER_H */
