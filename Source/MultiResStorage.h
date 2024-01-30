//
// Created by dallin on 10/4/23.
//

#ifndef TESSELLATION_MULTIRESSTORAGE_H
#define TESSELLATION_MULTIRESSTORAGE_H

#include <glm/glm.hpp>
#include <vector>
#include "MeshImpl.h"

namespace MultiResStorage {
    void assignAttributes(std::vector<TessLevelData>& meshes, OGLData& data);
};


#endif //TESSELLATION_MULTIRESSTORAGE_H
