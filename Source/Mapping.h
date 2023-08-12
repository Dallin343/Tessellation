//
// Created by dallin on 6/11/23.
//

#ifndef TESSELLATION_MAPPING_H
#define TESSELLATION_MAPPING_H
#include <MeshImpl.h>
#include <cereal/cereal.hpp>

struct BarycentricPoint {
    double alpha, beta, gamma;
    int v0, v1, v2, hd;
};

struct MapVert {
    int vd, vh0, vh1;
    BarycentricPoint bary;
    int mesh_idx;

    template<class Archive>
    void save(Archive & archive) const
    {
        int i0 = this->vd;
        int ih0 = this->vh0;
        int ih1 = this->vh1;
        int heh = this->bary.hd;
        double alpha = this->bary.alpha;
        double beta = this->bary.beta;
        archive(i0, ih0, ih1, heh, alpha, beta, mesh_idx);
    }

    template<class Archive>
    void load(Archive & archive)
    {
        this->bary = {};
        archive(vd, vh0, vh1, bary.hd, bary.alpha, bary.beta, mesh_idx);
        bary.gamma = 1.0 - bary.alpha - bary.beta;
        bary.v0 = vh0;
        bary.v1 = vh1;
    }
};

typedef std::unordered_map<int, std::vector<MapVert>> FaceVertMapping;

struct SMBarycentricPoint {
    double alpha, beta, gamma;
    SM_vertex_descriptor v0, v1, v2;
    SM_halfedge_descriptor hd;
};

struct VertMapping {
    SMBarycentricPoint bary;
    SM_face_descriptor inFace;
};

struct FaceMapping {
    SMBarycentricPoint bary;
    SM_vertex_descriptor vd;
};
typedef std::unordered_map<SM_vertex_descriptor, VertMapping> VertToFaceMap;
typedef std::unordered_map<SM_face_descriptor, std::vector<FaceMapping>> FaceToVertsMap;

FaceToVertsMap loadMap(const std::string& filename, const SurfaceMesh& A, const SurfaceMesh& B);
#endif //TESSELLATION_MAPPING_H
