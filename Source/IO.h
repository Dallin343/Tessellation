//
// Created by dallin on 2/28/23.
//

#ifndef TESSELLATION_IO_H
#define TESSELLATION_IO_H

#include "MeshImpl.h"
#include <iomanip>

class IO {
public:
    static void toOBJ(const SurfaceMesh& sm, std::ostream& out);
};


#endif //TESSELLATION_IO_H
