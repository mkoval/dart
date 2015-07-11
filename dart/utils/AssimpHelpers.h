#ifndef DART_UTILS_ASSIMPHELPERS_H_
#define DART_UTILS_ASSIMPHELPERS_H_
#include <string>
#include "urdf/DartLoader.h"
// TODO: Move ResourceLoader into a separate file.

class aiScene;

namespace dart {
namespace utils {

bool embedTextures(aiScene &scene, std::string const &scenePath,
                   ResourceLoader const &resourceLoader);

} // namespace utils
} // namespace dart

#endif // ifndef DART_UTILS_ASSIMPHELPERS_H_
