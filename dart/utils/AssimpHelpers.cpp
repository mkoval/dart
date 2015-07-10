#include <iostream>
#include <vector>
#include <assimp/material.h>
#include <assimp/scene.h>
#include <assimp/cexport.h>
#include <assimp/version.h>
#include <boost/filesystem.hpp>
#include <dart/common/Console.h>
#include "AssimpHelpers.h"

namespace {

struct TextureEntry {
  unsigned int mMeshIndex;
  aiTextureType mTextureType;
  unsigned int mTextureIndex;
  std::string mPath;
};

}

namespace dart {
namespace utils {

static void getTextures(aiScene const &scene, std::string const &scenePath,
                        std::vector<TextureEntry> &textures)
{
  using boost::filesystem::path;

  static std::vector<aiTextureType> const materialTypes {
    aiTextureType_AMBIENT,
    aiTextureType_DIFFUSE,
    aiTextureType_DISPLACEMENT,
    aiTextureType_EMISSIVE,
    aiTextureType_HEIGHT,
    aiTextureType_LIGHTMAP,
    aiTextureType_NONE,
    aiTextureType_NORMALS,
    aiTextureType_OPACITY,
    aiTextureType_REFLECTION,
    aiTextureType_SHININESS,
    aiTextureType_SPECULAR,
    aiTextureType_UNKNOWN
  };

  if (scenePath.empty()) {
    return;
  }

  path const basePath = path(scenePath).parent_path();

  for (unsigned int imesh = 0; imesh < scene.mNumMeshes; ++imesh) {
    aiMesh const &mesh = *scene.mMeshes[imesh];
    aiMaterial const &material = *scene.mMaterials[mesh.mMaterialIndex];

    for (aiTextureType const &textureType : materialTypes) {
      unsigned int numTextures = material.GetTextureCount(textureType);

      for (unsigned int itexture = 0; itexture < numTextures; ++itexture) {
        aiString textureAssimpPath;
        material.GetTexture(textureType, itexture, &textureAssimpPath, nullptr);

        TextureEntry entry;
        entry.mMeshIndex= imesh;
        entry.mTextureType = textureType;
        entry.mTextureIndex = itexture;
        entry.mPath = textureAssimpPath.C_Str();
        textures.push_back(entry);
      }
    }
  }
}

static bool getFormatHint(std::string const &texturePath, char *formatHint)
{
  using boost::filesystem::path;

  // Fill the format will NUL bytes, just to be safe.
  memset(formatHint, '\0', 4);

  std::string textureExtension = path(texturePath).extension().string();
  if (textureExtension.empty()) {
    dtwarn << "[getFormatHint] Unable to extract file extension from '"
           << texturePath << "'. We are not providing a format hint.\n";
    return false;
  }

  // Boost.Filesystem returns file extensions that includes the '.'.
  assert(textureExtension[0] == '.');
  textureExtension = textureExtension.substr(1);

  if (textureExtension.size() > 3) {
    dtwarn << "[getFormatHint] Texture '" << texturePath << "' has the file"
              " extension '" << textureExtension << "', which contains more"
              " than three characters. This is not supported by Assimp,"
              " We are not providing a format hint.\n";
    return false;
  }

  // Copy the extension into the buffer.
  memcpy(formatHint, textureExtension.c_str(), textureExtension.size());
  return true;
}

bool embedTextures(aiScene &scene, std::string const &scenePath)
{
  std::vector<TextureEntry> textureEntries;
  getTextures(scene, scenePath, textureEntries);

  if (!textureEntries.empty() && scene.mNumTextures > 0) {
    dterr << "[embedTextures] Scene already contains " << scene.mNumTextures
          << " embedded textures. Embedding additional textures is not"
             " currently supported.\n";
    return false;
  }

  scene.mTextures = new aiTexture*[textureEntries.size()];

  for (size_t i = 0; i < textureEntries.size(); ++i) {
    TextureEntry const &entry = textureEntries[i];
    aiTexture*& texture = scene.mTextures[i];

    std::cout << "Processing texture[" << i << "]" << std::endl;
    std::cout << "path: '" << entry.mPath << "'" << std::endl;

    // TODO: Load the texture.
    size_t texture_size = 0;
    uint8_t *texture_data = nullptr;

    // Create a compressed texture. From the Assimp documentation, this must
    // have: (1) mWidth = 0, (2) mHeight equal to the number of bytes, (3)
    // mData a pointer to a buffer of size mHeight, and (4) the achFormatHint
    // string populated.
    texture = new aiTexture;
    texture->mHeight = texture_size;
    texture->mWidth = 0;

    getFormatHint(entry.mPath, texture->achFormatHint);

    texture->pcData = reinterpret_cast<aiTexel *>(new uint8_t[texture_size]);
    memcpy(texture->pcData, texture_data, texture_size);

    // TODO: Change the aiMaterials that reference this path to point to
    aiMesh &mesh = *scene.mMeshes[entry.mMeshIndex];
    aiMaterial &material = *scene.mMaterials[mesh.mMaterialIndex];

    aiString embeddedPath;
    embeddedPath.Set(AI_MAKE_EMBEDDED_TEXNAME(i));
    material.AddProperty(&embeddedPath,
      AI_MATKEY_TEXTURE(entry.mTextureType, entry.mTextureIndex));
  }
  return true;
}

} // namespace utils
} // namespace dart
