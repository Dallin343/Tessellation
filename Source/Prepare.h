//
// Created by dallin on 3/5/23.
//

#ifndef TESSELLATION_PREPARE_H
#define TESSELLATION_PREPARE_H

#include "MeshImpl.h"
#include "MultiResStorage.h"

namespace Prepare {
    OGLData toOGL(const SurfaceMeshPtr& sm);
    OGLData toOGL(std::vector<TessLevelData>& meshes);
};


#endif //TESSELLATION_PREPARE_H
