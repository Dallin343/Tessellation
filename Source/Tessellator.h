//
// Created by dallin on 1/10/23.
//

#ifndef TESSELLATION_TESSELLATOR_H
#define TESSELLATION_TESSELLATOR_H

#include <array>
#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <optional>
#include <unordered_set>

typedef typename std::array<glm::dvec3, 3> Triangle;
typedef typename std::array<unsigned int, 3> FaceIndices;
typedef typename std::array<std::vector<glm::dvec3>, 3> Ring;
typedef typename std::array<std::vector<int>, 3> RingVertexIndices;

class TessellatedTriangle {
public:
    std::vector<glm::dvec3> vertices;
    std::unordered_set<unsigned int> innerVertexIndices;
    std::vector<FaceIndices> faces;
};

class Tessellator {
public:
    static std::shared_ptr<TessellatedTriangle> TessellateTriangle(const Triangle &tri, int ol0, int ol1, int ol2, int il);

private:
    static std::vector<glm::dvec3> subdivideEdge(glm::dvec3 a, glm::dvec3 b, int subdivision);
    static Ring generateInnerRing(const Ring &outerRing);
    static std::optional<glm::dvec3> intersect(glm::dvec3 c, glm::dvec3 e, glm::dvec3 d, glm::dvec3 f);
    static glm::dvec3 project(glm::dvec3 point, glm::dvec3 a, glm::dvec3 b);
    static void generateFacesBetween(const  std::vector<unsigned int>& outerVertexIndices,
                                     const  std::vector<unsigned int>& innerVertexIndices,
                                     const  std::vector<glm::dvec3>& vertices,
                                     glm::dvec3 normal, std::vector<FaceIndices>& faces);
    static FaceIndices buildFace(unsigned int a, unsigned int b, unsigned int c,
                                 const std::vector<glm::dvec3>& vertices, glm::dvec3 normal);
    static bool isCCW(glm::dvec3 a, glm::dvec3 b, glm::dvec3 c, glm::dvec3 normal);
};


#endif //TESSELLATION_TESSELLATOR_H
