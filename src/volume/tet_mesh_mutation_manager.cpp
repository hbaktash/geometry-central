#include "geometrycentral/volume/tet_mesh_mutation_manager.h"
// #include "set"

namespace geometrycentral{
namespace volume{


Vertex split_tet(TetMesh* tet_mesh, VertexPositionGeometry* geometry, 
                 Tet tIn, std::vector<double> bary_weights){
    // center vertex position
    Vector3 baryCenter = {0., 0., 0.};
    std::vector<Vertex> adjVs = tIn.adjVertices();
    double weight_sum = 0.;
    for(int i = 0; i < bary_weights.size(); i++){ // size is just 4..
        baryCenter += bary_weights[i] * geometry->inputVertexPositions[adjVs[i]];
        weight_sum += bary_weights[i];
    }
    baryCenter = baryCenter/weight_sum;

    // build connectivity 
    Vertex v = tet_mesh->splitTet(tIn);
    tet_mesh->compress(); // otherwise vector size is doubles and we get lots of meaningless indices? (and the polyscope part fails?)
    tet_mesh->compressTets();
    // updating position vector, for the sake of size
    VertexData<Vector3> newPositions(*tet_mesh);
    for(Vertex vv: tet_mesh->vertices()){
        if(vv.getIndex() != v.getIndex()){
            newPositions[vv] = geometry->inputVertexPositions[vv];
        }
    }
    newPositions[v] = baryCenter;
    geometry->inputVertexPositions = newPositions;
    geometry->refreshQuantities();
    return v;
}


Vertex split_edge(TetMesh* tet_mesh, VertexPositionGeometry* geometry, Edge e, std::pair<double, double> bary_weights){
    Vector3 baryCenter = {0., 0., 0.};
    baryCenter += (geometry->inputVertexPositions[e.firstVertex()]  * bary_weights.first + 
                   geometry->inputVertexPositions[e.secondVertex()] * bary_weights.second)/(bary_weights.first + bary_weights.second);
    
    Vertex v = tet_mesh->splitEdge(e);
    // tet_mesh->validateConnectivity();
    std::cout<<"compressing..\n";
    tet_mesh->compress(); // otherwise? vector size is doubles and we get lots of meaningless indices?
    tet_mesh->compressTets();
    std::cout<<"compressed.\n";
    VertexData<Vector3> newPositions(*tet_mesh);
    for(Vertex vv: tet_mesh->vertices()){
        if(vv.getIndex() != v.getIndex()){
            newPositions[vv] = geometry->inputVertexPositions[vv];
        }
    }
    newPositions[v] = baryCenter;
    geometry->inputVertexPositions = newPositions;
    geometry->refreshQuantities();
    return v;
}

}
}
