//
// Created by dallin on 1/10/23.
//
// OpenGL tessellation simulator
// Ported from https://github.com/teropa/triangle-tessellation

#include <iostream>
#include "Tessellator.h"

std::shared_ptr<TessellatedTriangle>
Tessellator::TessellateTriangle(const Triangle &tri, int ol0, int ol1, int ol2, int il) {
    auto [u, v, w] = tri;
    auto tessellated = std::make_shared<TessellatedTriangle>();

    if (ol0 == 1 && ol1 == 1 && ol2 == 1 && il == 1) {
        tessellated->vertices.insert(tessellated->vertices.end(), {u, v, w});
        tessellated->faces.insert(tessellated->faces.end(), {0, 1, 2});
        return tessellated;
    }

    const int innerLevel = il == 1 ? 2 : il;

    // Normal vector of the plane formed by the triangle
    glm::dvec3 planeNrm = glm::normalize(glm::cross(v - u, w - u));

    // Start by subdividing the outer edges according to the inner tessellation level
    std::vector<Ring> rings {
            {
                    subdivideEdge(u, v, innerLevel),
                    subdivideEdge(v, w, innerLevel),
                    subdivideEdge(w, u, innerLevel),
            }
    };

    // Generate inner rings until there are no more edge subdivisions
    while (rings.at(rings.size() - 1).at(0).size() > 3) {
        rings.push_back(generateInnerRing(rings.at(rings.size()-1)));
    }

    // Now replace the outermost ring with one that uses the outer subdivisions
    rings.at(0) = {
            subdivideEdge(u, v, ol0),
            subdivideEdge(v, w, ol1),
            subdivideEdge(w, u, ol2),
    };

    // Generate vertex array and generate version of ring data structure with indexes
    std::vector<glm::dvec3> vertices;
    std::vector<std::array<std::vector<unsigned int>, 3>> ringVertexIndices;
    for (const auto& ring : rings) {
        auto [ringUVPoints, ringVWPoints, ringWUPoints] = ring;

        std::vector<unsigned int> ringUVIndices(ringUVPoints.size(), 0);
        std::vector<unsigned int> ringVWIndices(ringVWPoints.size(), 0);
        std::vector<unsigned int> ringWUIndices(ringWUPoints.size(), 0);

        vertices.push_back(ringUVPoints.at(0));
        ringUVIndices.at(0) = vertices.size() - 1;
        ringWUIndices.at(ringWUPoints.size() - 1) = vertices.size() - 1;

        vertices.push_back(ringVWPoints.at(0));
        ringVWIndices.at(0) = vertices.size() - 1;
        ringUVIndices.at(ringUVPoints.size() - 1) = vertices.size() - 1;

        vertices.push_back(ringWUPoints.at(0));
        ringWUIndices.at(0) = vertices.size() - 1;
        ringVWIndices.at(ringVWPoints.size() - 1) = vertices.size() - 1;

        for (int i = 1; i < ringUVPoints.size() - 1; i++) {
            vertices.push_back(ringUVPoints.at(i));
            ringUVIndices.at(i) = vertices.size() - 1;
        }
        for (int i = 1; i < ringVWPoints.size() - 1; i++) {
            vertices.push_back(ringVWPoints.at(i));
            ringVWIndices.at(i) = vertices.size() - 1;
        }
        for (int i = 1; i < ringWUPoints.size() - 1; i++) {
            vertices.push_back(ringWUPoints.at(i));
            ringWUIndices.at(i) = vertices.size() - 1;
        }

        ringVertexIndices.push_back({ringUVIndices, ringVWIndices, ringWUIndices});
    }

    // Generate faces to fill each consecutive ring pair
    std::vector<FaceIndices> faces;
    for (int i = 0; i < rings.size() - 1; i++) {
        auto [outerRingUVVertexIndices,
              outerRingVWVertexIndices,
              outerRingWUVertexIndices] = ringVertexIndices.at(i);

        auto [innerRingUVVertexIndices,
              innerRingVWVertexIndices,
              innerRingWUVertexIndices] = ringVertexIndices[i + 1];

        generateFacesBetween(outerRingUVVertexIndices, innerRingUVVertexIndices,
                             vertices, planeNrm, faces);

        generateFacesBetween(outerRingVWVertexIndices, innerRingVWVertexIndices,
                             vertices, planeNrm, faces);

        generateFacesBetween(outerRingWUVertexIndices, innerRingWUVertexIndices,
                             vertices, planeNrm, faces);
    }

    auto [innermostUVVertexIndices,
          innermostVWVertexIndices,
          innermostWUVertexIndices] = ringVertexIndices.at(rings.size() - 1);

    if (innermostUVVertexIndices.size() == 2 && innermostVWVertexIndices.size() == 2 &&
        innermostWUVertexIndices.size() == 2) {
        // Innermost ring has no subdivisions; use it as a triangle as-is
        faces.push_back(buildFace(innermostUVVertexIndices.at(0), innermostVWVertexIndices.at(0),
                                  innermostWUVertexIndices.at(0), vertices, planeNrm));
    } else {
        // Innermost triangle is subdivided, generate faces between all the points on the ring and the centerpoint
        glm::dvec3 centerVertex = ((u + v) + w) / 3.0;
        vertices.push_back(centerVertex);
        unsigned int centerVertexIdx = vertices.size() - 1;
        for (int i = 0; i < innermostUVVertexIndices.size() - 1; i++) {
            faces.push_back(buildFace(innermostUVVertexIndices.at(i), innermostUVVertexIndices.at(i + 1),
                                      centerVertexIdx, vertices, planeNrm));
        }
        for (int i = 0; i < innermostVWVertexIndices.size() - 1; i++) {
            faces.push_back(buildFace(innermostVWVertexIndices.at(i), innermostVWVertexIndices.at(i + 1),
                                      centerVertexIdx, vertices, planeNrm));
        }
        for (int i = 0; i < innermostWUVertexIndices.size() - 1; i++) {
            faces.push_back(buildFace(innermostWUVertexIndices.at(i), innermostWUVertexIndices.at(i + 1),
                                      centerVertexIdx, vertices, planeNrm));
        }
    }
    tessellated->vertices = vertices;
    tessellated->faces = faces;
    return tessellated;
}

std::vector<glm::dvec3> Tessellator::subdivideEdge(glm::dvec3 a, glm::dvec3 b, int subdivision) {
    auto newVertices = std::vector<glm::dvec3>();
    newVertices.reserve(subdivision + 2);
    newVertices.push_back(a);

    glm::dvec3 aToB = a - b;
    glm::dvec3 aToBNorm = glm::normalize(aToB);
    double len = glm::length(aToB) / subdivision;

    for (int i = 0; i < subdivision - 1; i++) {
        newVertices.push_back(a + (aToBNorm * (len * (i+1))));
    }
    newVertices.push_back(b);
    return newVertices;
}

Ring Tessellator::generateInnerRing(const Ring &outerRing) {
    auto [outerUVPoints, outerVWPoints, outerWUPoints] = outerRing;
    glm::dvec3 outerU = outerUVPoints.at(0);
    glm::dvec3 outerV = outerVWPoints.at(0);
    glm::dvec3 outerW = outerWUPoints.at(0);

    // Outer edge vectors
    glm::dvec3 outerUV = outerV - outerU;
    glm::dvec3 outerVW = outerW - outerV;
    glm::dvec3 outerWU = outerU - outerW;

    // Normal vector of the plane formed by the triangle
    glm::dvec3 planeNrm = glm::normalize(glm::cross(outerUV, outerVW));

    // Edge normal vectors
    glm::dvec3 outerUVNrm = glm::normalize(glm::cross(outerUV, planeNrm));
    glm::dvec3 outerVWNrm = glm::normalize(glm::cross(outerVW, planeNrm));
    glm::dvec3 outerWUNrm = glm::normalize(glm::cross(outerWU, planeNrm));

    // Vertices on each subdivided outer edge closest to the vertices of the original outer triangle
    glm::dvec3 uv = outerUVPoints.at(1);
    glm::dvec3 uw = outerWUPoints.at(outerWUPoints.size() - 2);
    glm::dvec3 vw = outerVWPoints.at(1);
    glm::dvec3 vu = outerUVPoints.at(outerUVPoints.size() - 2);
    glm::dvec3 wu = outerWUPoints.at(1);
    glm::dvec3 wv = outerVWPoints.at(outerVWPoints.size() - 2);

    auto innerUOpt = intersect(uv, outerUVNrm, uw, outerWUNrm);
    auto innerVOpt = intersect(vw, outerVWNrm, vu, outerUVNrm);
    auto innerWOpt = intersect(wu, outerWUNrm, wv, outerVWNrm);

    if (!(innerUOpt.has_value() && innerVOpt.has_value() && innerWOpt.has_value())) {
        std::cerr << "Something weird happened with the intersections" << std::endl;
    }

    glm::dvec3 innerU = innerUOpt.value();
    glm::dvec3 innerV = innerVOpt.value();
    glm::dvec3 innerW = innerWOpt.value();

    std::vector<glm::dvec3> innerUVPoints { innerU };
    innerUVPoints.reserve(outerUVPoints.size() - 2);
    for (int i = 2; i < outerUVPoints.size() - 2; i++) {
        innerUVPoints.push_back(project(outerUVPoints.at(i), innerU, innerV));
    }
    innerUVPoints.push_back(innerV);

    std::vector<glm::dvec3> innerVWPoints { innerV };
    innerVWPoints.reserve(outerVWPoints.size() - 2);
    for (int i = 2; i < outerVWPoints.size() - 2; i++) {
        innerVWPoints.push_back(project(outerVWPoints.at(i), innerV, innerW));
    }
    innerVWPoints.push_back(innerW);

    std::vector<glm::dvec3> innerWUPoints { innerW };
    innerWUPoints.reserve(outerWUPoints.size() - 2);
    for (int i = 2; i < outerWUPoints.size() - 2; i++) {
        innerWUPoints.push_back(project(outerWUPoints.at(i), innerW, innerU));
    }
    innerWUPoints.push_back(innerU);

    return {innerUVPoints, innerVWPoints, innerWUPoints};
}

std::optional<glm::dvec3> Tessellator::intersect(glm::dvec3 c, glm::dvec3 e, glm::dvec3 d, glm::dvec3 f) {
    glm::dvec3 g = d - c;
    glm::dvec3 h = glm::cross(f, g);
    double hLen = glm::length(h);
    if (hLen == 0) {
        return std::nullopt;
    }

    glm::dvec3 k = glm::cross(f, e);
    double kLen = glm::length(k);
    if (kLen == 0) {
        return std::nullopt;
    }

    glm::dvec3 i = e * (hLen / kLen);

    if (glm::length(glm::cross(h, k)) > 0.0000001) {
        return c + i;
    }

    return c - i;
}

glm::dvec3 Tessellator::project(glm::dvec3 point, glm::dvec3 a, glm::dvec3 b) {
    glm::dvec3 ab = b - a;
    double t = glm::dot(ab, point - a) / glm::dot(ab, ab);
    return (ab * t) + a;
}

void Tessellator::generateFacesBetween(const std::vector<unsigned int>& outerVertexIndices,
                                       const std::vector<unsigned int>& innerVertexIndices,
                                       const std::vector<glm::dvec3> &vertices, glm::dvec3 normal,
                                       std::vector<FaceIndices>& faces) {
        bool outerEdge = true;
        int outerIdx = 0;
        int innerIdx = 0;

        while (outerIdx < outerVertexIndices.size() - 1 || innerIdx < innerVertexIndices.size() - 1) {
            if (outerEdge && outerIdx < outerVertexIndices.size() - 1) {
                faces.push_back(buildFace(outerVertexIndices.at(outerIdx), outerVertexIndices.at(outerIdx + 1),
                                          innerVertexIndices.at(innerIdx), vertices, normal));
                outerIdx++;
            } else if (!outerEdge && innerIdx < innerVertexIndices.size() - 1) {
                faces.push_back(buildFace(innerVertexIndices.at(innerIdx), innerVertexIndices.at(innerIdx + 1),
                                          outerVertexIndices.at(outerIdx), vertices, normal));
                innerIdx++;
            }
            outerEdge = !outerEdge;
        }
}

FaceIndices Tessellator::buildFace(unsigned int a, unsigned int b, unsigned int c,
                                   const std::vector<glm::dvec3>& vertices, glm::dvec3 normal) {
    if (isCCW(vertices.at(a), vertices.at(b), vertices.at(c), normal)) {
        return {a, b, c};
    } else {
        return {a, c, b};
    }
}

bool Tessellator::isCCW(glm::dvec3 a, glm::dvec3 b, glm::dvec3 c, glm::dvec3 normal) {
    auto triNormal = glm::cross(b - a, c - a);

    return glm::dot(triNormal, normal) > 0;
}
