//
// Created by dallin on 6/27/23.
//

#ifndef TESSELLATION_MESH_H
#define TESSELLATION_MESH_H

#include <vector>
#include "Heap.h"

namespace FastDecimator {
    class Mesh {
    public:
        explicit Mesh(const std::string& filename);
        void loadOBJ(const std::string& filename);

        void decimate(double ratio, int max_face_feats = 10);

    private:
        void init();
        static double calculate_error(const EPtr& edge, glm::dvec3& optimalPoint);
        static double vertex_error(const SymmetricMatrix& q, glm::dvec3& p);
        static EPtr get_edge(std::unordered_map<unsigned long, EPtr>& pte, unsigned long k, const VPtr& v0, const VPtr& v1);
    public:
        std::vector<FPtr> faces;
        std::vector<EPtr> edges;
        std::vector<VPtr> vertices;
    };
}
#endif //TESSELLATION_MESH_H
