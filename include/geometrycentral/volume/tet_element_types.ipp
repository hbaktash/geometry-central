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
inline Tet::Tet(TetMesh* mesh_, size_t ind_, std::vector<size_t> vertices_) : Element(mesh_,ind_) {

}

inline void Tet::buildAdjVertices(std::vector<size_t> vertices){
  std::vector<size_t> adjVs;
  adjVs.reserve(4);
  for(size_t v_ind: vertices){
    adjVs.push_back(v_ind);
  }
  mesh->tAdjVs[getIndex()] = adjVs;
}

inline void Tet::buildAdjEdges(){
  std::vector<Vertex> adjVs = adjVertices();
  std::vector<size_t> adjEs;
  adjEs.reserve(6);
  if(adjVs.size() == 0) throw std::logic_error("vertices should have been initialized first. (size=0)"); //logic?
  adjEs.reserve(6);
  for(Vertex v1: adjVs){
    for(Vertex v2: adjVs){
      if(v1 != v2) adjEs.push_back(mesh->connectingEdge(v1, v2).getIndex());
    }
  }
  mesh->tAdjEs[getIndex()] = adjEs;
}

inline void Tet::buildAdjFaces(){
  std::vector<Vertex> adjVs = adjVertices();
  std::vector<size_t> adjFs;
  if(adjVs.size() == 0) throw std::logic_error("vertices should have been initialized first. (size=0)"); //logic?
  adjFs.reserve(4); // general case? in which this function won't exist like this
  for(Vertex v1: adjVs){
    // TODO: should encapsulate these set operations in utils 
    std::vector<Vertex> triplet; // instead, we should have some set operations added to utils. 
    std::vector<Vertex> boring_solo_set{v1};
    triplet.reserve(3);
    std::set_difference(adjVs.begin(), adjVs.end(),  // fancy diff
                        boring_solo_set.begin(), boring_solo_set.end(),
                        std::inserter(triplet, triplet.begin()));
    adjFs.push_back(mesh->get_connecting_face(triplet[0], triplet[1], triplet[2]).getIndex());
  }
  mesh->tAdjFs[getIndex()] = adjFs;
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