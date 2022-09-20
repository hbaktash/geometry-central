
// === Helpers which will allow us abstract over types
// The corresponding template declarations are given in halfedge_element_types.h

namespace geometrycentral {

// clang-format off

template<> inline size_t nElements<volume::Tet          >(volume::TetMesh*     mesh)   { return mesh->nTets();    }

template<> inline size_t elementCapacity<volume::Tet         >(volume::TetMesh*     mesh)   { return mesh->nTetsCapacity();    }

template<> inline size_t dataIndexOfElement<volume::Tet             >(volume::TetMesh* mesh    , volume::Tet e)            { return e.getIndex(); }

// template<> inline volume::TetSet            iterateElements<volume::Tet         >(volume::TetMesh*     mesh)   { return mesh->tets();          }

// template<> inline std::list<std::function<void(size_t)>>& getExpandCallbackList<volume::Tet         >(volume::TetMesh* mesh    )   { return mesh->tetExpandCallbackList;   } // TODO

// template<> inline std::list<std::function<void(const std::vector<size_t>&)>>& getPermuteCallbackList<volume::Tet          >(volume::TetMesh* mesh    )   { return mesh->tetPermuteCallbackList;   }

template<> inline std::string typeShortName<volume::Tet          >()            { return "t";   }

// clang-format on

} // namespace geometrycentral
