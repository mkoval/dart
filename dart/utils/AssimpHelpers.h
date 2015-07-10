#ifndef DART_UTILS_ASSIMPHELPERS_H_
#define DART_UTILS_ASSIMPHELPERS_H_
#include <string>

class aiScene;

namespace dart {
namespace utils {

bool embedTextures(aiScene &scene, std::string const &scenePath);

} // namespace utils
} // namespace dart

#endif // ifndef DART_UTILS_ASSIMPHELPERS_H_
