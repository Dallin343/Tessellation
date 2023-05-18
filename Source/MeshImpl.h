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
    const Seam_vertex_pmap& vSeamMap;
    const Seam_edge_pmap& eSeamMap;
    typedef SM_edge_descriptor                                       key_type;
    typedef bool                                                  value_type;
    typedef value_type                                            reference;
    typedef boost::readable_property_map_tag                      category;
    Seam_is_constrained_edge_map(const SeamMesh& seamMesh, const Seam_vertex_pmap& vSeamMap, const Seam_edge_pmap& eSeamMap) :
    seamMesh_ptr(&seamMesh), vSeamMap(vSeamMap), eSeamMap(eSeamMap) {}

    friend value_type get(const Seam_is_constrained_edge_map& m, const key_type& edge) {
//        auto sm = m.seamMesh_ptr->mesh();
//        auto v = get(m.vSeamMap, sm.source(sm.halfedge(edge))) || get(m.vSeamMap, sm.target(sm.halfedge(edge)));
//        auto e = get(m.eSeamMap, edge);
        auto result = m.seamMesh_ptr->has_on_seam(edge);
        if (result) {
            int x = 0;
        }
        return result;
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
    glm::vec3 origCoords;
    glm::vec3 newCoords{};
    glm::vec3 baryCoords;
    glm::vec2 uv{};
    bool isInner = false;
    bool anchored = false;
    SM_vertex_descriptor vd;
    bool isOnSeam = false;
    AssigningSection assignedBy = Never;

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

    SM_edge_descriptor ed;
    TessVertPtr v0, v1;
    std::vector<TessVertPtr> tessVerts;

    template<class Archive>
    void serialize(Archive& archive) {
        archive(ed, v0, v1, tessVerts);
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
    TessEdgeMap processedEdges;
    ProcessFaceMap processedFaces;
    VertSet interpolateVerts;
};

#endif //TESSELLATION_MESHIMPL_H
