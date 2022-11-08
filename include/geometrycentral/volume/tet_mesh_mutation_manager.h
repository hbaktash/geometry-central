#include "geometrycentral/volume/surface_mesh.h"
#include "geometrycentral/volume/vertex_position_geometry.h"
#include "geometrycentral/volume/tet_mesh.h"
#include "string.h"

// Mutating tet meshes while handling geometry

namespace geometrycentral{
namespace volume{

// simple splitting by inserting a vertex
Vertex split_tet(TetMesh* tet_mesh, VertexPositionGeometry* geometry, Tet tIn, std::vector<double> bary_weights);

}
}



