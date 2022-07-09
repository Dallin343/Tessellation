//
// Created by dallin on 5/4/22.
//

#include "MeshProcessor.h"
#include <glm/glm.hpp>

void MeshProcessor::DecimateInit(Mesh *hdMesh) {
    this->mesh = hdMesh;
    for (auto& vertex : this->mesh->GetVertices()) {
        glm::dmat4 Q;

    }
}

glm::dmat4 MeshProcessor::ComputeQ(const VPtr& vertex) {

}
