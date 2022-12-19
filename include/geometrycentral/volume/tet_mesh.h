#pragma once

#include "geometrycentral/volume/simple_polygon_mesh.h"
#include "geometrycentral/volume/surface_mesh.h"
#include "geometrycentral/volume/tet_element_types.h"


namespace geometrycentral{
namespace volume{
// using namespace geometrycentral::surface::volume;

//container template
template <typename T>
using TetData = MeshData<Tet, T>;


// helper functions
std::vector<std::vector<size_t>> triangles_from_tets(std::vector<std::vector<size_t>> tets);

// classes
class SimpleTetMesh : public SimplePolygonMesh{
public:
    std::vector<std::vector<size_t>> tets;
    
    SimpleTetMesh(const std::vector<std::vector<size_t>>& tets_,
                  const std::vector<Vector3>& vertexCoordinates_);
    
    ~SimpleTetMesh(){}

    // merging functions
    void mergeIdenticalVertices(); // overriding parent function; just changing some minor stuff.
    void mergeIdenticalFaces(); // merging faces with the same set of vertices (by index).

};

class TetMesh : public SurfaceMesh{
public:
    
    TetMesh(const std::vector<std::vector<size_t>>& tet_v_inds_)
            :TetMesh(tet_v_inds_, triangles_from_tets(tet_v_inds_)){}
            
    TetMesh(const std::vector<std::vector<size_t>>& tets_,
            const std::vector<std::vector<size_t>>& triangles_); // assuming there are no identical triangles or vertices
    
    ~TetMesh(){}
    
    // dirty iterators
    // to be removed later
    std::vector<std::vector<size_t>> fAdjTs;
    std::vector<std::vector<size_t>> tAdjVs;
    
    // halfedges of faces on the same tet, should be siblings
    bool siblings_are_ordered = false;

    // navigation helpers; all can be integrated as mesh element operators
    Face get_connecting_face(Vertex v1, Vertex v2, Vertex v3); // assuming we dont have non-manifold(?!) edges or duplicat(!?) or.. faces (3-manifold) ; generally we should return a vector<Face>.
    Tet get_connecting_tet(Vertex v1, Vertex v2, Vertex v3, Vertex v4); // assuming we dont have duplicate(?!) tets..; otherwise ..//..
    Halfedge get_he_of_edge_on_face(Edge e, Face f);
    Edge common_edge_of_faces(Face f1, Face f2);
    Halfedge boundary_he_of_edge(Edge e); // Halfedge() if not exists
    
    // added for sibling ordering
    bool face_is_boundary(Face f);
    Tet next_tet_along_face(Tet t, Face f);
    
    // to have faces on the same tet be siblings
    void order_siblings_of_edge(Edge e);
    void order_all_siblings();

    // Range-based loops
    TetSet tets();
    
    // mesh resize routines
    Face getNewFace();
    Tet getNewTet();

    // compression 
    void compressTets();

    // mutation routines; should be added to mutation_manager later?
    Vertex buildVolOnFace(Face fIn);
    Vertex splitTet(Tet tIn);
    Vertex splitEdge(Edge e);

    // Expanding Callbacks
    std::list<std::function<void(size_t)>> tetExpandCallbackList;
    // Compression callbacks
    std::list<std::function<void(const std::vector<size_t>&)>> tetPermuteCallbackList;
    
    // sanity checks
    void validateConnectivity();


    // element counters getters
    size_t nTets() const;
    size_t nTetsCapacity() const;
    size_t nTetsFill() const;

    // get some element
    Tet tet(size_t index);
    
protected:
    // element counters
    
    size_t nTetsCount = 0; // number of tets
    size_t nTetsFillCount = 0; // last index of legal tets in t->X vectors
    size_t nTetsCapacityCount = 0; // size of t->X vectors
};

}
}

#include <geometrycentral/volume/tet_element_types.ipp>
#include <geometrycentral/volume/tet_element_logic_templates.ipp>
#include <geometrycentral/volume/tet_mesh.ipp>