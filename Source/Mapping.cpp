//
// Created by dallin on 8/5/23.
//

#include "Mapping.h"

FaceToVertsMap loadMap(const std::string& filename, const SurfaceMesh& A, const SurfaceMesh& B) {
    std::ifstream infile(filename);
    std::string line;
    FaceToVertsMap f2vmap;

    for (const auto f : B.faces()) {
        f2vmap.insert({f, {}});
    }

    for (int i = 0; i < A.number_of_vertices(); i++) {
        int vA, vB[3];
        double bary[3];
        SMBarycentricPoint baryPoint{};
        for (int j = 0; j < 3; j++) {
            std::getline(infile, line);
            std::istringstream ss(line);
            ss >> vA >> vB[j] >> bary[j];
        }
        vA--;
        vB[0]--;
        vB[1]--;
        vB[2]--;

        auto hd = B.halfedge(SM_vertex_descriptor(vB[0]), SM_vertex_descriptor(vB[1]));

        if (hd == B.null_halfedge()) {
            std::cerr << "ERRORRRRR" << std::endl;
        }
        if (B.target(B.next(hd)) != SM_vertex_descriptor(vB[2])) {
            int temp = vB[2];
            vB[2] = vB[1];
            vB[2] = temp;

            double btemp = bary[2];
            bary[2] = bary[1];
            bary[2] = btemp;
        }

        hd = B.halfedge(SM_vertex_descriptor(vB[0]), SM_vertex_descriptor(vB[1]));
        if (B.target(B.next(hd)) != SM_vertex_descriptor(vB[2])) {
            int x =1;
        }

        baryPoint.v0 = SM_vertex_descriptor(vB[0]);
        baryPoint.v1 = SM_vertex_descriptor(vB[1]);
        baryPoint.v2 = SM_vertex_descriptor(vB[2]);
        baryPoint.alpha = bary[0];
        baryPoint.beta = bary[1];
        baryPoint.gamma = bary[2];
        baryPoint.hd = B.halfedge(SM_vertex_descriptor(vB[0]), SM_vertex_descriptor(vB[1]));
        SM_face_descriptor face = B.face(SM_halfedge_descriptor(baryPoint.hd));
        FaceMapping faceMapping {baryPoint, SM_vertex_descriptor(vA)};
        f2vmap.at(face).push_back(faceMapping);
    }

    return f2vmap;
}