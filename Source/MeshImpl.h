//
// Created by dallin on 1/28/23.
//

#ifndef TESSELLATION_MESHIMPL_H
#define TESSELLATION_MESHIMPL_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/Seam_mesh.h>
#include <CGAL/Surface_mesh_parameterization/Orbifold_Tutte_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/ARAP_parameterizer_3.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/boost/graph/properties.h>

#include <unordered_map>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <utility>
#include <vector>

#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Surface_mesh_parameterization/Circular_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_authalic_parameterizer_3.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>
#include <CGAL/Polygon_mesh_processing/shape_predicates.h>
#include <CGAL/Point_set_2.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

#include <glm/glm.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/unordered_set.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/array.hpp>

typedef CGAL::Simple_cartesian<double>            Kernel;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Ray_3 Ray;
typedef Kernel::Point_2                           Point_2;
typedef Kernel::Point_3                           Point_3;
typedef CGAL::Surface_mesh<Kernel::Point_3>       SurfaceMesh;
typedef std::shared_ptr<SurfaceMesh> SurfaceMeshPtr;

typedef CGAL::AABB_face_graph_triangle_primitive<SurfaceMesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;
typedef Tree::Primitive_id Primitive_id;

typedef std::pair<unsigned int, unsigned int> VertexUVDescriptor;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Point_set_2<Kernel, Tds> Point2Set;
typedef Point2Set::Vertex_handle  Vertex_handle;

typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor     SM_vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor   SM_halfedge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor       SM_edge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor       SM_face_descriptor;

typedef SurfaceMesh::Property_map<SM_edge_descriptor, bool>           Seam_edge_pmap;
typedef SurfaceMesh::Property_map<SM_vertex_descriptor, bool>         Seam_vertex_pmap;
typedef SurfaceMesh::Property_map<SM_vertex_descriptor, Vector>   Normal_vertex_pmap;
typedef SurfaceMesh::Property_map<SM_face_descriptor, Vector>   Normal_face_pmap;
typedef SurfaceMesh::Property_map<SM_vertex_descriptor, double> Gauss_vertex_pmap;


////////// Custom Simplification
class SymmetricMatrix {

public:

    // Constructor

    SymmetricMatrix(double c = 0) {
        for (double &i : m) i = c;
    }

    SymmetricMatrix(	double m11, double m12, double m13, double m14,
                        double m22, double m23, double m24,
                        double m33, double m34,
                        double m44) {
        m[0] = m11;  m[1] = m12;  m[2] = m13;  m[3] = m14;
        m[4] = m22;  m[5] = m23;  m[6] = m24;
        m[7] = m33;  m[8] = m34;
        m[9] = m44;
    }

    // Make plane

    SymmetricMatrix(double a,double b,double c,double d)
    {
        m[0] = a*a;  m[1] = a*b;  m[2] = a*c;  m[3] = a*d;
        m[4] = b*b;  m[5] = b*c;  m[6] = b*d;
        m[7 ] =c*c; m[8 ] = c*d;
        m[9 ] = d*d;
    }

    double operator[](int c) const { return m[c]; }

    // Determinant

    double det(	int a11, int a12, int a13,
                   int a21, int a22, int a23,
                   int a31, int a32, int a33)
    {
        double det =  m[a11]*m[a22]*m[a33] + m[a13]*m[a21]*m[a32] + m[a12]*m[a23]*m[a31]
                      - m[a13]*m[a22]*m[a31] - m[a11]*m[a23]*m[a32]- m[a12]*m[a21]*m[a33];
        return det;
    }

    SymmetricMatrix operator+(const SymmetricMatrix& n) const
    {
        return SymmetricMatrix( m[0]+n[0],   m[1]+n[1],   m[2]+n[2],   m[3]+n[3],
                                m[4]+n[4],   m[5]+n[5],   m[6]+n[6],
                                m[ 7]+n[ 7], m[ 8]+n[8 ],
                                m[ 9]+n[9 ]);
    }

    SymmetricMatrix& operator+=(const SymmetricMatrix& n)
    {
        m[0]+=n[0];   m[1]+=n[1];   m[2]+=n[2];   m[3]+=n[3];
        m[4]+=n[4];   m[5]+=n[5];   m[6]+=n[6];   m[7]+=n[7];
        m[8]+=n[8];   m[9]+=n[9];
        return *this;
    }

    std::array<double, 10> m;
};

struct FeaturePoint {
    FeaturePoint(Point_3 p, Vector n) : p(p), n(n) {};
    Point_3 p;
    Vector n;
};
typedef std::shared_ptr<FeaturePoint> FeaturePointPtr;
typedef SurfaceMesh::Property_map<SM_face_descriptor, std::unordered_set<FeaturePointPtr>> Feature_face_pmap;
typedef SurfaceMesh::Property_map<SM_edge_descriptor, double> Edge_cost_pmap;
typedef SurfaceMesh::Property_map<SM_vertex_descriptor, SymmetricMatrix> Vertex_Q_pmap;

struct PairCost {
    PairCost(SM_vertex_descriptor src, SM_vertex_descriptor tgt, double cost) : pair(src, tgt), cost(cost) {}

    std::pair<SM_vertex_descriptor, SM_vertex_descriptor> pair;
    double cost;
};

class MinCompare
{
public:
    bool operator()(const PairCost l, const PairCost r) const {
        return l.cost > r.cost;
    }
};

class EdgeCostHeap : public std::priority_queue<PairCost, std::vector<PairCost>, MinCompare>
{
private:
    bool _remove(const SM_vertex_descriptor& searchVd, bool& need_to_heap) {
        auto it = std::find_if(this->c.begin(), this->c.end(), [&](const PairCost& item) {
            return item.pair.first == searchVd || item.pair.second == searchVd;
        });

        if (it == this->c.end()) {
            return false;
        }
        if (it == this->c.begin()) {
            // deque the top element
            this->pop();
        }
        else {
            // remove element and re-heap
            this->c.erase(it);
            need_to_heap = true;
        }
        return true;
    }

    bool _remove(const SM_vertex_descriptor& searchVd1, const SM_vertex_descriptor& searchVd2) {
        auto it = std::find_if(this->c.begin(), this->c.end(), [&](const PairCost& item) {
            return (item.pair.first == searchVd1 && item.pair.second == searchVd2) ||
                    (item.pair.first == searchVd2 && item.pair.second == searchVd1);
        });

        if (it == this->c.end()) {
            return false;
        }
        else if (it == this->c.begin()) {
            // deque the top element
            this->pop();
        }
        else {
            // remove element and re-heap
            this->c.erase(it);
            return true;
        }
        return false;
    }
public:

    bool remove(const SM_vertex_descriptor& searchVd, bool remove_all) {
        bool need_to_heap = false;
        if (remove_all) {
            while (this->_remove(searchVd, need_to_heap)) {}
        }
        else {
            this->_remove(searchVd, need_to_heap);
        }

        if (need_to_heap) {
            std::make_heap(this->c.begin(), this->c.end(), this->comp);
        }
        return true;
    }

    bool remove(const SM_vertex_descriptor& searchVd1, const SM_vertex_descriptor& searchVd2) {
        if (this->_remove(searchVd1, searchVd2)) {
            std::make_heap(this->c.begin(), this->c.end(), this->comp);
        }
        return true;
    }

    bool remove(const std::vector<SM_vertex_descriptor>& searchVds) {
        bool need_to_heap = false;
        for (const auto& searchVd : searchVds) {
            this->_remove(searchVd, need_to_heap);
        }

        if (need_to_heap) {
            std::make_heap(this->c.begin(), this->c.end(), this->comp);
        }
        return true;
    }

    bool remove(const std::vector<std::pair<SM_vertex_descriptor, SM_vertex_descriptor>>& searchVds) {
        bool need_to_heap = false;
        for (const auto& [vd1, vd2] : searchVds) {
            if (this->_remove(vd1, vd2)) {
                need_to_heap = true;
            }
        }

        if (need_to_heap) {
            std::make_heap(this->c.begin(), this->c.end(), this->comp);
        }
        return true;
    }
};
//////////

typedef CGAL::Seam_mesh<SurfaceMesh, Seam_edge_pmap, Seam_vertex_pmap>  SeamMesh;
typedef boost::graph_traits<SeamMesh>::vertex_descriptor                    vertex_descriptor;
typedef boost::graph_traits<SeamMesh>::halfedge_descriptor                  halfedge_descriptor;
typedef std::shared_ptr<SeamMesh> SeamMeshPtr;

namespace SMP = CGAL::Surface_mesh_parameterization;
namespace SMS = CGAL::Surface_mesh_simplification;

typedef SMS::Edge_profile<SurfaceMesh>                                 EdgeProfile;
typedef SurfaceMesh::Property_map<SM_halfedge_descriptor, Point_2>      UV_pmap;

typedef SMS::GarlandHeckbert_plane_policies<SurfaceMesh, Kernel>                  Classic_plane;
typedef SMS::GarlandHeckbert_probabilistic_plane_policies<SurfaceMesh, Kernel>    Prob_plane;
typedef SMS::GarlandHeckbert_triangle_policies<SurfaceMesh, Kernel>               Classic_tri;
typedef SMS::GarlandHeckbert_probabilistic_triangle_policies<SurfaceMesh, Kernel> Prob_tri;

// BGL property map which indicates whether an edge is marked as non-removable
struct Seam_is_constrained_edge_map
{
    const SeamMesh* seamMesh_ptr;
    Feature_face_pmap& faceFeatures;
    const UV_pmap& uvmap;
    typedef SM_edge_descriptor                                       key_type;
    typedef bool                                                  value_type;
    typedef value_type                                            reference;
    typedef boost::readable_property_map_tag                      category;
    Seam_is_constrained_edge_map(const SeamMesh& seamMesh, Feature_face_pmap& faceFeatures, const UV_pmap& uvmap) :
    seamMesh_ptr(&seamMesh), faceFeatures(faceFeatures), uvmap(uvmap) {
    }

    friend value_type get(const Seam_is_constrained_edge_map& m, const key_type& edge) {
        const int FEATS = 10;

        auto& sm = m.seamMesh_ptr->mesh();
//        auto v = get(m.vSeamMap, sm.source(sm.halfedge(edge))) || get(m.vSeamMap, sm.target(sm.halfedge(edge)));
//        auto e = get(m.eSeamMap, edge);
        auto f1 = sm.face(sm.halfedge(edge));
        auto f2 = sm.face(sm.opposite(sm.halfedge(edge)));
        auto f1Points = get(m.faceFeatures, f1);
        auto f2Points = get(m.faceFeatures, f2);
        if (f1Points.size() != 0 || f2Points.size() != 0) {
            int lll = 0;
        }
        if (f1Points.size() >= FEATS || f2Points.size() >= FEATS) {
            return true;
        } else {
            return false;
//            std::array<Point_2, 3> f1_uvs, f2_uvs;
//            std::list<Vertex_handle> vertexList;
//            int i = 0;
//            for (auto hd : sm.halfedges_around_face(sm.halfedge(edge))) {
//                f1_uvs.at(i++) = get(m.uvmap, hd);
//            }
//
//            i = 0;
//            for (auto hd : sm.halfedges_around_face(sm.opposite(sm.halfedge(edge)))) {
//                f2_uvs.at(i++) = get(m.uvmap, hd);
//            }
//
//            bool ret = false;
//            m.pointSet.range_search(f1_uvs.at(0), f1_uvs.at(1), f1_uvs.at(2), std::back_inserter(vertexList));
//            if (vertexList.size() >= FEATS) {
//                m.finishedFaces.insert(f1);
//                ret = true;
//            }
//
//            vertexList.clear();
//            m.pointSet.range_search(f2_uvs.at(0), f2_uvs.at(1), f2_uvs.at(2), std::back_inserter(vertexList));
//            if (vertexList.size() >= FEATS) {
//                m.finishedFaces.insert(f2);
//                ret = true;
//            }
//            return ret;
        }

//        auto result = m.seamMesh_ptr->has_on_seam(edge);
//        return result;
    }
};

class FeatureVert {
public:
    FeatureVert() = default;
    FeatureVert(const SM_halfedge_descriptor &hd, const glm::vec3 &cartCoords, const glm::vec3 &baryCoords,
                const glm::vec2 &uv) : hd(hd), cartCoords(cartCoords), baryCoords(baryCoords), uv(uv) {}

    SM_halfedge_descriptor hd;
    glm::vec3 cartCoords;
    glm::vec3 baryCoords;
    glm::vec2 uv;

    template<class Archive>
    void serialize(Archive& archive) {
        archive(hd, cartCoords, baryCoords, uv);
    }
};
typedef std::shared_ptr<FeatureVert> FeatureVertPtr;

enum AssigningSection {
    ProjectEdge,
    MoveValidate,
    MVEdgeCase,
    ProjectValidate,
    PVEdgeCase,
    InterpolateUnmatched,
    Undone,
    Never
};

class TessellatedVert {
public:
    TessellatedVert() {
        undoMove();
    }

    TessellatedVert(FeatureVertPtr matchingFeature,
                    const glm::vec3 &origCoords, const glm::vec3 &baryCoords) :
            matchingFeature(std::move(matchingFeature)), origCoords(origCoords), baryCoords(baryCoords) {
        undoMove();
        uv = glm::vec2(minVal, minVal);
    }

    TessellatedVert(const glm::vec3 &origCoords, const glm::vec3 &baryCoords) :
            origCoords(origCoords), baryCoords(baryCoords) {
        undoMove();
        uv = glm::vec2(minVal, minVal);
    }

    void undoMove() {
        newCoords = {minVal, minVal, minVal};
    }

    bool isAssigned() const {
        return newCoords != glm::vec3(minVal, minVal, minVal);
    }

    bool hasUV() const {
        return uv != glm::vec2(minVal, minVal);
    }

    FeatureVertPtr matchingFeature = nullptr;
    SM_halfedge_descriptor edgeHd;
    glm::vec3 origCoords;
    glm::vec3 newCoords{};
    glm::vec3 baryCoords;
    glm::vec3 otherBaryCoords;
    glm::vec2 uv{};
    glm::vec3 normal{};
    bool isInner = false;
    bool anchored = false;
    SM_vertex_descriptor vd;
    bool isOnSeam = false;
    AssigningSection assignedBy = Never;

    glm::vec3 getBaryCoords(const SM_halfedge_descriptor& hd) {
        if (hd == this->edgeHd) return baryCoords;
        return otherBaryCoords;
    }

    template<class Archive>
    void serialize(Archive& archive) {
        archive(matchingFeature, origCoords, newCoords, baryCoords, uv, isInner, anchored, vd, isOnSeam, assignedBy);
    }

private:
    float minVal = std::numeric_limits<float>::min();
};
typedef std::shared_ptr<TessellatedVert> TessVertPtr;

struct TessellatedFace {
    TessellatedFace()  = default;
    SM_face_descriptor fd;
    std::array<TessVertPtr, 3> vertices;

    template<class Archive>
    void serialize(Archive& archive) {
        archive(fd, vertices);
    }
};
typedef std::shared_ptr<TessellatedFace> TessFacePtr;
typedef std::unordered_map<SM_vertex_descriptor, TessVertPtr> VDToTessVert;

class ProcessEdge {
public:
    ProcessEdge() = default;
    ProcessEdge(const SM_edge_descriptor& ed) : ed(ed) {}
    ProcessEdge(const SM_edge_descriptor& ed, const SM_halfedge_descriptor& hd, const SM_halfedge_descriptor& other) : ed(ed), hd(hd), otherHd(other) {}

    SM_halfedge_descriptor hd, otherHd;
    SM_edge_descriptor ed;
    TessVertPtr v0, v1;
    std::vector<TessVertPtr> tessVerts;
    std::vector<TessVertPtr> otherTessVerts;

    const std::vector<TessVertPtr>& getTessVerts(const SM_halfedge_descriptor& h) {
        if (h == hd) return tessVerts;
        return otherTessVerts;
    }

    void sort() {
        otherTessVerts.reserve(tessVerts.size());
        otherTessVerts.insert(otherTessVerts.end(), tessVerts.rbegin(), tessVerts.rend());
    }

    void push_back(const TessVertPtr& tessVert) {
        // Add temporary bary coordinates for the other direction.
        // x gets set to the component of the bary coord for v1
        if (tessVert->baryCoords.x == 0.0f) {
//            tessVert->otherBaryCoords = {0.0f, tessVert->baryCoords.z, tessVert->baryCoords.y};
            tessVert->otherBaryCoords = {tessVert->baryCoords.z, 0.0f, 0.0f};
        }
        else if (tessVert->baryCoords.y == 0.0f) {
//            tessVert->otherBaryCoords = {tessVert->baryCoords.z, 0.0f, tessVert->baryCoords.x};
            tessVert->otherBaryCoords = {tessVert->baryCoords.x, 0.0f, 0.0f};
        }
        else if (tessVert->baryCoords.z == 0.0f) {
            tessVert->otherBaryCoords = {tessVert->baryCoords.y, 0.0f, 0.0f};
        }
        tessVert->edgeHd = hd;
        tessVerts.push_back(tessVert);
    }

    template<class Archive>
    void serialize(Archive& archive) {
        archive(hd, otherHd, ed, v0, v1, tessVerts, otherTessVerts);
    }
};
typedef std::shared_ptr<ProcessEdge> ProcessEdgePtr;

class ProcessFace {
public:
    ProcessFace() = default;

    ProcessFace(const SM_face_descriptor &fd, const std::array<SM_vertex_descriptor, 3> &vds, const std::array<Point_2, 3> &uvs,
                const std::array<Point_3, 3> &coords) : fd(fd), vds(vds), uvs(uvs), coords(coords) {}

    SM_face_descriptor fd {};
    std::array<SM_vertex_descriptor, 3> vds {};
    std::array<Point_2, 3> uvs {};
    std::array<Point_3, 3> coords {};
    std::vector<TessVertPtr> tessVerts;
    std::vector<TessVertPtr> innerVerts;
    std::vector<TessFacePtr> tessFaces;
    ProcessEdgePtr e01, e02, e12;
    VDToTessVert vdToTessVert;

    std::vector<TessVertPtr> edgeVertices(const SM_halfedge_descriptor& hd) {
        for (const auto& edge : {e01, e02, e12}) {
            if (edge->hd == hd) {
                return edge->tessVerts;
            }
            else if (edge->otherHd == hd) {
                return edge->otherTessVerts;
            }
        }
        return {};
    }

    template<class Archive>
    void serialize(Archive& archive) {
        archive(fd, vds, uvs, coords, tessVerts, innerVerts, tessFaces, e01, e02, e12, vdToTessVert);
    }
};
typedef std::shared_ptr<ProcessFace> ProcessFacePtr;

typedef std::vector<FeatureVertPtr> FeatureVerts;
typedef std::vector<TessVertPtr> TessellatedVerts;
typedef std::unordered_set<TessVertPtr> VertSet;
typedef std::unordered_map<SM_edge_descriptor, ProcessEdgePtr> TessEdgeMap;
typedef std::unordered_map<SM_face_descriptor, ProcessFacePtr> ProcessFaceMap;

struct TessLevel {
    unsigned int ol0;
    unsigned int ol1;
    unsigned int ol2;
    unsigned int il;

    glm::ivec4 toGLM() const {
        return {ol0, ol1, ol2, il};
    }

    bool isIdentity() const {
        return ol0 == 1 && ol1 == 1 && ol2 == 1 && il == 1;
    };

    std::string str() const {
        std::stringstream tessLevelStr;
        tessLevelStr << ol0 << "-" << ol1 << "-" << ol2 << "-" << il;
        return tessLevelStr.str();
    }

    template<class Archive>
    void serialize(Archive& archive) {
        archive(ol0, ol1, ol2, il);
    }
};

namespace glm {
    template<class Archive>
    void serialize(Archive& archive, glm::vec3& m) {
        archive(m.x, m.y, m.z);
    }

    template<class Archive>
    void serialize(Archive& archive, glm::vec2& m) {
        archive(m.x, m.y);
    }
}

struct TessLevelData {
    SurfaceMeshPtr mesh;
    TessLevel level;
    TessEdgeMap processedEdges;
    ProcessFaceMap processedFaces;
    VertSet interpolateVerts;

    template<class Archive>
    void serialize(Archive& archive) {
        archive(level, processedEdges, processedFaces, interpolateVerts);
    }
};


struct VertexAttributes {
    glm::vec4 displacement;
    glm::vec4 normal;
    VertexAttributes(glm::vec3 d, glm::vec3 n): displacement(glm::vec4(d, 1.0f)), normal(glm::dvec4(n, 1.0f)) {}
    VertexAttributes(): displacement(), normal() {}
};

struct FaceLookupIndices {
    int firstCorner0, firstCorner1, firstCorner2;
    int firstInner;
    int firstEdge0, firstEdge1, firstEdge2;

    FaceLookupIndices(const std::array<int,3>& corners, int inner, const std::array<int,3> edges) {
        firstCorner0 = corners[0], firstCorner1 = corners[1], firstCorner2 = corners[2];
        firstInner = inner;
        firstEdge0 = edges[0], firstEdge1 = edges[1], firstEdge2 = edges[2];
    }
};

typedef std::vector<VertexAttributes> VAttrs;
typedef std::vector<FaceLookupIndices> FIdxs;

struct VertexData {
    VertexData(const glm::vec3 &pos) : pos(pos), normal() {}
    VertexData(const glm::vec3 &pos, const glm::vec3& norm) : pos(pos), normal(norm) {}

    glm::vec3 pos;
    glm::vec3 normal;
};

struct FaceData {
    FaceData(unsigned int v0, unsigned int v1, unsigned int v2) : v0(v0), v1(v1), v2(v2) {}

    unsigned int v0, v1, v2;
};

struct OGLData {
    std::vector<VertexData> vertices{};
    std::vector<FaceData> faces{};
    VAttrs cornerVertexData{};
    VAttrs edgeVertexData{};
    VAttrs innerVertexData{};
    FIdxs faceData{};
};

#endif //TESSELLATION_MESHIMPL_H
