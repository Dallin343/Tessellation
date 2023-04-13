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

typedef typename std::array<glm::vec3, 3> Triangle;
typedef typename std::array<unsigned int, 3> FaceIndices;
typedef typename std::array<std::vector<glm::vec3>, 3> Ring;
typedef typename std::array<std::vector<int>, 3> RingVertexIndices;

class TessellatedTriangle {
public:
    std::vector<glm::vec3> vertices;
    std::unordered_set<unsigned int> innerVertexIndices;
    std::vector<FaceIndices> faces;
};

class Tessellator {
public:
    static std::shared_ptr<TessellatedTriangle> TessellateTriangle(const Triangle &tri, int ol0, int ol1, int ol2, int il);

private:
    static std::vector<glm::vec3> subdivideEdge(glm::vec3 a, glm::vec3 b, int subdivision);
    static Ring generateInnerRing(const Ring &outerRing);
    static std::optional<glm::vec3> intersect(glm::vec3 c, glm::vec3 e, glm::vec3 d, glm::vec3 f);
    static glm::vec3 project(glm::vec3 point, glm::vec3 a, glm::vec3 b);
    static void generateFacesBetween(const  std::vector<unsigned int>& outerVertexIndices,
                                     const  std::vector<unsigned int>& innerVertexIndices,
                                     const  std::vector<glm::vec3>& vertices,
                                     glm::vec3 normal, std::vector<FaceIndices>& faces);
    static FaceIndices buildFace(unsigned int a, unsigned int b, unsigned int c,
                                 const std::vector<glm::vec3>& vertices, glm::vec3 normal);
    static bool isCCW(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 normal);
};


#endif //TESSELLATION_TESSELLATOR_H
