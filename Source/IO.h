//
// Created by dallin on 2/28/23.
//

#ifndef TESSELLATION_IO_H
#define TESSELLATION_IO_H

#include "MeshImpl.h"
#include <iomanip>

namespace IO {
    typedef std::unordered_map<unsigned long, unsigned int> VdHdMap;
    typedef std::unordered_map<unsigned int, unsigned int> VdMap;

    inline unsigned long key(unsigned int i, unsigned int j) {
        return ((unsigned long) i) << 32 | j;
    }

    void toOBJ(const SurfaceMesh& sm, std::ostream& out, const std::string& outVD, const std::string& outHD, const std::string& outSeams);
    void toOBJ(const SurfaceMesh& sm, std::ostream& out);

    //Convenience method
    std::pair<SurfaceMeshPtr, SeamMeshPtr> fromOBJ(const std::string& filename, const std::string& seamFilename);
    std::tuple<SurfaceMeshPtr, VdMap, VdHdMap>  fromOBJ(const std::string& filename, const std::string& vdMapFilename,
                                                        const std::string& hdMapFilename, const std::string& seamsFilename);
    SurfaceMeshPtr fromOBJ(const std::string& filename, bool hasUV = false, bool lightmapUV = false);

    SurfaceMeshPtr LoadMesh(const std::string& filename);

    void ReadUV(const SurfaceMeshPtr& mesh, const std::string& filename);

    SeamMeshPtr AddSeams(const SurfaceMeshPtr& sm, const std::string& selectionsFile);

    void WriteTexture(const std::vector<glm::vec3>& tex, unsigned int w, unsigned int h, float offsetVal, unsigned int max, const std::string& file);
};


#endif //TESSELLATION_IO_H
