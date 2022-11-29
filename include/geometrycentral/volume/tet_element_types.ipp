#pragma once


namespace std {
// clang-format off
template <> struct hash<geometrycentral::volume::Tet>            { std::size_t operator()(const geometrycentral::volume::Tet& e)            const { return std::hash<size_t>{}(e.getIndex()); } };
// clang-format on
} // namespace std

namespace geometrycentral {
namespace volume {


// ======== left over from surface elements ===========
//lazy iterators
inline std::vector<Tet> Face::adjacentTets() const{
  TetMesh* tet_mesh = (TetMesh*)mesh;
  std::vector<size_t> adjTInds = tet_mesh->fAdjTs[getIndex()];
  std::vector<Tet> adjTs;
  adjTs.reserve(adjTInds.size());
  for(size_t t_ind: adjTInds) 
    adjTs.push_back(Tet(tet_mesh, t_ind));
  return adjTs;
}

//lazy Tet iterator for Vertex
inline std::vector<Tet> Vertex::adjacentTets() const {
  std::unordered_set<Tet> adjTset;
  for(Face f: adjacentFaces()){
    for(Tet t: f.adjacentTets()){
      adjTset.insert(t);
    }
  }
  std::vector<Tet> adjTs;
  // adjTs.reserve(adjTset.size());
  // for(Tet t: adjTset){
  //   adjTs.push_back(t);
  // }
  adjTs.insert(adjTs.end(), adjTset.begin(), adjTset.end());
  return adjTs;
}

//lazy Tet iterators for Edge 
inline std::vector<Tet> Edge::adjacentTets() const{
  std::unordered_set<Tet> adjTset;
  for(Face f: adjacentFaces()){
    for(Tet t: f.adjacentTets()){
      adjTset.insert(t);
    }
  }
  std::vector<Tet> adjTs;
  // adjTs.reserve(adjTset.size());
  // for(Tet t: adjTset){
  //   adjTs.push_back(t);
  // }
  adjTs.insert(adjTs.end(), adjTset.begin(), adjTset.end());
  return adjTs;
}

// ==========================================================
// ================          Tet          ==================
// ==========================================================

// Constructors
inline Tet::Tet() {}
inline Tet::Tet(TetMesh* mesh_, size_t ind_) : Element(mesh_,ind_) {}

inline void Tet::buildAdjVertices(std::vector<size_t> vertices){
  std::vector<size_t> adjVs;
  adjVs.reserve(4);
  for(size_t v_ind: vertices){
    adjVs.push_back(v_ind);
  }
  mesh->tAdjVs[getIndex()] = adjVs;
}

inline std::vector<Vertex> Tet::adjVertices(){
  std::vector<size_t> adjVs = mesh->tAdjVs[getIndex()];
  return std::vector<Vertex>{Vertex(mesh, adjVs[0]), Vertex(mesh, adjVs[1]), Vertex(mesh, adjVs[2]), Vertex(mesh, adjVs[3])};
}

inline std::vector<Edge> Tet::adjEdges(){
  std::vector<Vertex> adjVs = adjVertices();
  std::vector<Edge> adjEs;
  adjEs.reserve(6);
  for(Vertex v1: adjVs){
    for(Vertex v2: adjVs){
      if(v1 < v2){
        Edge tmp_e = mesh->connectingEdge(v1, v2);
        adjEs.push_back(tmp_e);
      }
    }
  }
  return adjEs;
}

inline std::vector<Face> Tet::adjFaces(){
  std::vector<Vertex> adjVs = adjVertices();
  std::vector<Face> adjFs;
  adjFs.reserve(4);
  for(Vertex v1: adjVs){
    std::vector<Vertex> triplet; // instead, we should have some set operations added to utils.
    triplet.reserve(3);
    for(Vertex v : adjVs) 
      if (v != v1) 
        triplet.push_back(v);
    Face tmp_f = mesh->get_connecting_face(Vertex(triplet[0]), Vertex(triplet[1]), Vertex(triplet[2]));
    adjFs.push_back(tmp_f);
  }
  return adjFs;
}
  
inline bool Tet::isDead() const {
  if (getIndex() == INVALID_IND) 
      return true;
  return mesh->tAdjVs[getIndex()].size() < 4; // hmm, not really sure about the tombstone for now.
}

// Properties
inline bool Tet::isTet() const {
  return mesh->tAdjVs[getIndex()].size() == 4;
}
inline size_t Tet::degree() const{
  return mesh->tAdjVs[getIndex()].size();
}

// == Range iterators
inline bool TetRangeF::elementOkay(const TetMesh& mesh, size_t ind) {
  return mesh.tAdjVs[ind].size() < 4;
}

}
}