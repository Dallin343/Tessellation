//
// Created by dallin on 10/4/23.
//

#include "MultiResStorage.h"
#include "Utils.h"

namespace MultiResStorage {
    int calculateVertexIndex(glm::ivec4 tessLevels, glm::vec3 bary) {
        int innerLevel = tessLevels.w, outerLevel = tessLevels.x;
        auto alpha = bary.x, beta = bary.y, gamma = bary.z;
        auto b_min = std::min(alpha, std::min(beta, gamma));
        int ring = int(round(1.5 * innerLevel * b_min));

        int edge;
        float b_prime;
        if (Utils::nearly_equal(b_min, alpha)) {
            // On edge 1 side of triangle between v1, v2
            edge = 1;
            b_prime = gamma;
            if (Utils::nearly_equal(b_min, beta)) {
                //Gamma corner, we need to change edge to reflect that
                edge = 2;
                b_prime = alpha;
            }
        }
        else if (Utils::nearly_equal(b_min, beta)) {
            // On edge 2 side of triangle between v2, v0
            edge = 2;
            b_prime = alpha;
            if (Utils::nearly_equal(b_min, gamma)) {
                //alpha corner
                edge = 0;
                b_prime = beta;
            }
        }
        else if (Utils::nearly_equal(b_min, gamma)) {
            // On edge 0 side of triangle between v0, v1
            edge = 0;
            b_prime = beta;
            if (Utils::nearly_equal(b_min, beta)) {
                //beta corner
                edge = 1;
                b_prime = gamma;
            }
        }

        int T = ring == 0 ? outerLevel : innerLevel;
        int vertsPerEdge = T - 2*ring;

        //If vertsPerEdge is 0, then this is the centermost/last inner vertex
        float vertex = vertsPerEdge == 0 ? 0.0f : (b_prime / (1.0f - 3.0f*b_min)) * float(vertsPerEdge);
        int vertexIndexOnEdge = int(vertex);

        int relativeVertexIndex;
        if (ring != 0) {
            // This is an inner face vertex
            // Add any outer ring indices to start at the next sequential index
            int ringRelativeStartIndex = 0;
            for (int i = 1; i < ring; i++) {
                ringRelativeStartIndex += 3 * (T - 2 * i); // Add number of vertices in previous ring
            }

            // If all barycentric components are equal, this is the center/last inner vertex
            if (Utils::nearly_equal(alpha, beta) && Utils::nearly_equal(beta, gamma)) {
                relativeVertexIndex = ringRelativeStartIndex;
            }
            else {
                // Get the index for the start of this edge
                ringRelativeStartIndex += edge * vertsPerEdge;
                //Add vertex index on this edge
                relativeVertexIndex = ringRelativeStartIndex + vertexIndexOnEdge;
            }
        }
        else {
            // This vertex is on an edge, so it is just the index on that edge.
            relativeVertexIndex = vertexIndexOnEdge - 1;
        }

        return relativeVertexIndex;
    }

    void assignAttributes(std::vector<TessLevelData>& meshes, OGLData& data) {
        std::unordered_map<SM_vertex_descriptor, int> assignedCorners{};
        std::unordered_map<SM_edge_descriptor, int> assignedEdges{};

        int totalPositiveEdges = 0, totalNegativeEdges = 0;
        auto& corner = data.cornerVertexData;
        auto& inner = data.innerVertexData;
        auto& edges = data.edgeVertexData;
        auto& faces = data.faceData;

        auto baseMesh = meshes.at(0).mesh;
        auto baseVNorms = baseMesh->property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        for (auto face : baseMesh->faces()) {
            //Assign corner vertices first
            std::array<int, 3> firstCornerIndices{};
            int arrayIndex = 0;
            for (const auto& vd : baseMesh->vertices_around_face(baseMesh->halfedge(face))) {
                if (assignedCorners.find(vd) != assignedCorners.end()) {
                    // This edge has already been ordered, so this face accesses it in reverse order so its CCW.
                    firstCornerIndices.at(arrayIndex) = assignedCorners.at(vd);
                    arrayIndex++;
                    continue;
                }

                firstCornerIndices.at(arrayIndex++) = corner.size();
                assignedCorners.insert({vd, corner.size()});

                for (auto tessLevel : meshes) {
                    if (tessLevel.level.isIdentity()) {
                        corner.emplace_back(glm::vec3(1.0f, 0.0f, 0.0f), Utils::toGLM(get(baseVNorms, vd)));
                        continue;
                    }

                    auto tessFace = tessLevel.processedFaces.at(face);
                    for (auto tessVd : tessFace->vds) {
                        auto tessVert = tessFace->vdToTessVert.at(tessVd);
                        corner.emplace_back(tessVert->newCoords - tessVert->origCoords, tessVert->normal);
                    }
                }
            }

            //Assign inner indices next since they are a little easier
            int firstInnerIndex = inner.size();
            for (auto tessLevel : meshes) {
                if (tessLevel.level.isIdentity()) continue; //Skip the base simplified mesh

                auto tessFace = tessLevel.processedFaces.at(face);
                VAttrs faceVerticesInOrder;
                faceVerticesInOrder.resize(tessFace->innerVerts.size());
                for (const auto& innerVert : tessFace->innerVerts) {
                    // Calculate the index for each inner vertex and order them in the vector
                    int relativeIndex = calculateVertexIndex(tessLevel.level.toGLM(), innerVert->baryCoords);
                    faceVerticesInOrder.at(relativeIndex).displacement = glm::vec4(innerVert->newCoords - innerVert->origCoords, 1.0f);
                    faceVerticesInOrder.at(relativeIndex).normal = glm::vec4(innerVert->normal, 1.0f);
                }
                inner.insert(inner.end(), faceVerticesInOrder.begin(), faceVerticesInOrder.end());
            }

            //Assign edge vertices next
            //If an edge has already been done, we need to give a negative start index
            std::array<int, 3> firstEdgeIndices{};
            arrayIndex = 0;
            for (const auto& hd : baseMesh->halfedges_around_face(baseMesh->halfedge(face))) {
                auto edge = baseMesh->edge(hd);
                if (assignedEdges.find(edge) != assignedEdges.end()) {
                    // This edge has already been ordered, so this face accesses it in reverse order so its CCW.
                    firstEdgeIndices.at(arrayIndex) = -assignedEdges.at(edge);
                    arrayIndex++;
                    totalNegativeEdges++;
                    continue;
                }

                //New edge in the list
                firstEdgeIndices.at(arrayIndex++) = edges.size();
                assignedEdges.insert({edge, edges.size()});
                totalPositiveEdges++;
                // For this edge, we order its attributes for each tessLevel all together
                for (auto tessLevel : meshes) {
                    if (tessLevel.level.isIdentity()) continue; //Skip the base simplified mesh
                    auto tessFace = tessLevel.processedFaces.at(face);

                    VAttrs edgeVerticesInOrder;
                    auto edgeVertices = tessFace->edgeVertices(hd);
                    edgeVerticesInOrder.resize(edgeVertices.size());
                    for (const auto& edgeVert : edgeVertices) {
                        // Calculate the index for each inner vertex and order them in the vector
                        int relativeIndex = calculateVertexIndex(tessLevel.level.toGLM(), edgeVert->baryCoords);
                        edgeVerticesInOrder.at(relativeIndex).displacement = glm::vec4(edgeVert->newCoords - edgeVert->origCoords, 1.0f);
                        edgeVerticesInOrder.at(relativeIndex).normal = glm::vec4(edgeVert->normal, 1.0f);
                    }
                    edges.insert(edges.end(), edgeVerticesInOrder.begin(), edgeVerticesInOrder.end());
                }
            }

            faces.emplace_back(firstCornerIndices, firstInnerIndex, firstEdgeIndices);
        }

        std::cout << "POSITIVE: " << totalPositiveEdges << "\nNegative: " << totalNegativeEdges << std::endl;
    }
}

