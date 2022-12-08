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
  size_t adjTCnt = f.adjacentTets().size();
  if (adjTCnt == 0) throw std::logic_error("This face has no adj tets!\n");
  if (adjTCnt > 2) {
    for (Tet t: f.adjacentTets()){
      printf("neigh tet: %d\n", t.getIndex());
    }
    throw std::logic_error("This face is on more than 2 tets!\n");
  }
  // else{
  //   printf("size of adjs for face %d is %d \n", f.getIndex(), f.adjacentTets().size());
  //   for (Tet t: f.adjacentTets()) printf(" bfcheck ** adj tet: %d\n", t.getIndex());
  // }
  return adjTCnt == 1;
}

}
}