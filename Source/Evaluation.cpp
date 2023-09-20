//
// Created by dallin on 9/2/23.
//

#include "Evaluation.h"
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/tags.h>

std::vector<double> Evaluation::error(const SurfaceMeshPtr &original, const std::vector<SurfaceMeshPtr> &tests) {
    std::cout << "Running evaluation for " << tests.size() << " meshes...\n";
    std::cout << "Calculating Hausdorff distances...\n";
    int meshIndex = 0;
    std::vector<double> errors;
    errors.reserve(tests.size());
    for (const auto& test : tests) {
        std::cout << "\tStarting Mesh " << meshIndex << "...\n";
//        auto hausdorff = CGAL::Polygon_mesh_processing::bounded_error_Hausdorff_distance<CGAL::Sequential_tag>(*test, *original, 0.01);
        auto hausdorff = CGAL::Polygon_mesh_processing::approximate_Hausdorff_distance<CGAL::Sequential_tag>(*test, *original);
        errors.push_back(hausdorff);
        std::cout << "\tMesh " << meshIndex << " error = " << hausdorff << "\n\n";

        meshIndex++;
    }

    return errors;
}
