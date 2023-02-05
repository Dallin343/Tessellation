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
#include <CGAL/boost/graph/properties.h>

#include <unordered_map>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <utility>
#include <vector>

#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>
#include <CGAL/Polygon_mesh_processing/shape_predicates.h>
#include <CGAL/Point_set_2.h>

typedef CGAL::Simple_cartesian<double>            Kernel;
typedef Kernel::Point_2                           Point_2;
typedef Kernel::Point_3                           Point_3;
typedef CGAL::Surface_mesh<Kernel::Point_3>       SurfaceMesh;

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

typedef CGAL::Seam_mesh<SurfaceMesh, Seam_edge_pmap, Seam_vertex_pmap>  SeamMesh;
typedef boost::graph_traits<SeamMesh>::vertex_descriptor                    vertex_descriptor;
typedef boost::graph_traits<SeamMesh>::halfedge_descriptor                  halfedge_descriptor;

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
    typedef SM_edge_descriptor                                       key_type;
    typedef bool                                                  value_type;
    typedef value_type                                            reference;
    typedef boost::readable_property_map_tag                      category;
    Seam_is_constrained_edge_map(const SeamMesh& seamMesh) : seamMesh_ptr(&seamMesh) {}
    friend value_type get(const Seam_is_constrained_edge_map& m, const key_type& edge) {
        return m.seamMesh_ptr->has_on_seam(edge);
    }
};

#endif //TESSELLATION_MESHIMPL_H
