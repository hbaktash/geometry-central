#pragma once

namespace geometrycentral {
namespace volume {

inline size_t TetMesh::nTets()         const { return nTetsCount;}
inline size_t TetMesh::nTetsCapacity() const { return nTetsCapacityCount;}
inline size_t TetMesh::nTetsFill() const { return nTetsFillCount;}


inline Tet TetMesh::tet(size_t index) { return Tet(this, index); }

// Range set iters
inline TetSet TetMesh::tets() { return TetSet(this, 0, nTetsCount); }

inline bool TetMesh::face_is_boundary(Face f){
  size_t adjTCnt = fAdjTs.size();
  if (adjTCnt == 0) throw std::logic_error("This face has no adj tets!");
  return fAdjTs.size() == 1;
}

}
}