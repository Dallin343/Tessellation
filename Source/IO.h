//
// Created by dallin on 2/28/23.
//

#ifndef TESSELLATION_IO_H
#define TESSELLATION_IO_H

#include "MeshImpl.h"
#include <iomanip>

namespace IO {
    void toOBJ(const SurfaceMesh& sm, std::ostream& out);
    //Convenience method
    std::pair<SurfaceMeshPtr, SeamMeshPtr> fromOBJ(const std::string& filename, const std::string& seamFilename);

    SurfaceMeshPtr LoadMesh(const std::string& filename);

    void ReadUV(const SurfaceMeshPtr& mesh, const std::string& filename);

    SeamMeshPtr AddSeams(const SurfaceMeshPtr& sm, const std::string& selectionsFile);

    void WriteTexture(const std::vector<glm::vec3>& tex, unsigned int w, unsigned int h, unsigned int max, const std::string& file);
};


#endif //TESSELLATION_IO_H
