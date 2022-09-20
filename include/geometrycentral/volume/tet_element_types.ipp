#pragma once


namespace std {
// clang-format off
template <> struct hash<geometrycentral::volume::Tet>            { std::size_t operator()(const geometrycentral::volume::Tet& e)            const { return std::hash<size_t>{}(e.getIndex()); } };
// clang-format on
} // namespace std

namespace geometrycentral {
namespace volume {

// ==========================================================
// ================          Tet          ==================
// ==========================================================

// Constructors
inline Tet::Tet() {}
inline Tet::Tet(TetMesh* mesh_, size_t ind_) : Element(mesh_,ind_) {}
inline Tet::Tet(TetMesh* mesh_, size_t ind_, std::vector<Vertex> vertices_) : Element(mesh_,ind_), adjVertices(vertices_) {} // there is no cool halfEdge indicators for tets, so..

inline void Tet::buildAdjEdges(){
  if(adjVertices.size() == 0) throw std::logic_error("vertices should have been initialized first. (size=0)"); //logic?
  adjEdges.reserve((size_t)6); // general case? in which this function won't exist like this
  for(Vertex v1: adjVertices){
    for(Vertex v2: adjVertices){
      if(v1 != v2) adjEdges.push_back(mesh->connectingEdge(v1, v2));
    }
  }
}

inline void Tet::buildAdjFaces(){
  if(adjVertices.size() == 0) throw std::logic_error("vertices should have been initialized first. (size=0)"); //logic?
  adjFaces.reserve(4); // general case? in which this function won't exist like this
  for(Vertex v1: adjVertices){
    // TODO: should encapsulate these set operations in utils 
    std::vector<Vertex> triplet; // instead, we should have some set operations added to utils. 
    std::vector<Vertex> boring_solo_set{v1};
    triplet.reserve(3);
    std::set_difference(adjVertices.begin(), adjVertices.end(),  // fancy diff
                        boring_solo_set.begin(), boring_solo_set.end(),
                        std::inserter(triplet, triplet.begin()));
    // for(Vertex v2: adjVertices){ // not very fancy diff
    //   if(v1 != v2) triplet.push_back(v2);
    // }
    adjFaces.push_back(mesh->get_connecting_face(triplet[0], triplet[1], triplet[2]));
  }
}
  
inline bool Tet::isDead() const {
  return adjVertices.size() < 4; // hmm, not really sure for now.
}

// Properties
inline bool Tet::isTet() const {
  return adjVertices.size() == 4;
}
inline size_t Tet::degree() const{
  return adjVertices.size();
}

// == Range iterators
inline bool TetRangeF::elementOkay(const TetMesh& mesh, size_t ind) {
  return !mesh.tet_objects[ind].isDead();
}

}
}