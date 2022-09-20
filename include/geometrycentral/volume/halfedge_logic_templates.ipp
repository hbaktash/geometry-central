
// === Helpers which will allow us abstract over types
// The corresponding template declarations are given in halfedge_element_types.h

namespace geometrycentral {

// clang-format off

template<> inline size_t nElements<volume::Vertex       >(volume::SurfaceMesh* mesh)   { return mesh->nVertices();   }
template<> inline size_t nElements<volume::Face         >(volume::SurfaceMesh* mesh)   { return mesh->nFaces();      }
template<> inline size_t nElements<volume::Edge         >(volume::SurfaceMesh* mesh)   { return mesh->nEdges();      }
template<> inline size_t nElements<volume::Halfedge     >(volume::SurfaceMesh* mesh)   { return mesh->nHalfedges();  }
template<> inline size_t nElements<volume::Corner       >(volume::SurfaceMesh* mesh)   { return mesh->nCorners();    }
template<> inline size_t nElements<volume::BoundaryLoop >(volume::SurfaceMesh* mesh)   { return mesh->nBoundaryLoops();    }
// template<> inline size_t nElements<volume::Tet          >(volume::TetMesh*     mesh)   { return mesh->nTets();    }

template<> inline size_t elementCapacity<volume::Vertex      >(volume::SurfaceMesh* mesh)   { return mesh->nVerticesCapacity();   }
template<> inline size_t elementCapacity<volume::Face        >(volume::SurfaceMesh* mesh)   { return mesh->nFacesCapacity(); }
template<> inline size_t elementCapacity<volume::Edge        >(volume::SurfaceMesh* mesh)   { return mesh->nEdgesCapacity();      }
template<> inline size_t elementCapacity<volume::Halfedge    >(volume::SurfaceMesh* mesh)   { return mesh->nHalfedgesCapacity();}
template<> inline size_t elementCapacity<volume::Corner      >(volume::SurfaceMesh* mesh)   { return mesh->nHalfedgesCapacity();    }
template<> inline size_t elementCapacity<volume::BoundaryLoop>(volume::SurfaceMesh* mesh)   { return mesh->nBoundaryLoopsCapacity();    }
// template<> inline size_t elementCapacity<volume::Tet         >(volume::TetMesh*     mesh)   { return mesh->nTetsCapacity();    }

template<> inline size_t dataIndexOfElement<volume::Vertex          >(volume::SurfaceMesh* mesh, volume::Vertex e)         { return e.getIndex(); }
template<> inline size_t dataIndexOfElement<volume::Face            >(volume::SurfaceMesh* mesh, volume::Face e)           { return e.getIndex(); }
template<> inline size_t dataIndexOfElement<volume::Edge            >(volume::SurfaceMesh* mesh, volume::Edge e)           { return e.getIndex(); }
template<> inline size_t dataIndexOfElement<volume::Halfedge        >(volume::SurfaceMesh* mesh, volume::Halfedge e)       { return e.getIndex(); }
template<> inline size_t dataIndexOfElement<volume::Corner          >(volume::SurfaceMesh* mesh, volume::Corner e)         { return e.getIndex(); }
template<> inline size_t dataIndexOfElement<volume::BoundaryLoop    >(volume::SurfaceMesh* mesh, volume::BoundaryLoop e)   { return e.getIndex(); }
// template<> inline size_t dataIndexOfElement<volume::Tet             >(volume::TetMesh* mesh    , volume::Tet e)            { return e.getIndex(); }

template<> inline volume::VertexSet         iterateElements<volume::Vertex      >(volume::SurfaceMesh* mesh)   { return mesh->vertices();      }
template<> inline volume::HalfedgeSet       iterateElements<volume::Halfedge    >(volume::SurfaceMesh* mesh)   { return mesh->halfedges();     }
template<> inline volume::CornerSet         iterateElements<volume::Corner      >(volume::SurfaceMesh* mesh)   { return mesh->corners();       }
template<> inline volume::EdgeSet           iterateElements<volume::Edge        >(volume::SurfaceMesh* mesh)   { return mesh->edges();         }
template<> inline volume::FaceSet           iterateElements<volume::Face        >(volume::SurfaceMesh* mesh)   { return mesh->faces();         }
template<> inline volume::BoundaryLoopSet   iterateElements<volume::BoundaryLoop>(volume::SurfaceMesh* mesh)   { return mesh->boundaryLoops(); }
// template<> inline volume::TetSet            iterateElements<volume::Tet         >(volume::TetMesh*     mesh)   { return mesh->tets();          }

template<> inline std::list<std::function<void(size_t)>>& getExpandCallbackList<volume::Vertex      >(volume::SurfaceMesh* mesh)   { return mesh->vertexExpandCallbackList;   }
template<> inline std::list<std::function<void(size_t)>>& getExpandCallbackList<volume::Halfedge    >(volume::SurfaceMesh* mesh)   { return mesh->halfedgeExpandCallbackList;   }
template<> inline std::list<std::function<void(size_t)>>& getExpandCallbackList<volume::Corner      >(volume::SurfaceMesh* mesh)   { return mesh->halfedgeExpandCallbackList;   }
template<> inline std::list<std::function<void(size_t)>>& getExpandCallbackList<volume::Edge        >(volume::SurfaceMesh* mesh)   { return mesh->edgeExpandCallbackList;   }
template<> inline std::list<std::function<void(size_t)>>& getExpandCallbackList<volume::Face        >(volume::SurfaceMesh* mesh)   { return mesh->faceExpandCallbackList;   }
template<> inline std::list<std::function<void(size_t)>>& getExpandCallbackList<volume::BoundaryLoop>(volume::SurfaceMesh* mesh)   { return mesh->faceExpandCallbackList;   }
// template<> inline std::list<std::function<void(size_t)>>& getExpandCallbackList<volume::Tet         >(volume::TetMesh* mesh    )   { return mesh->tetExpandCallbackList;   } // TODO

template<> inline std::list<std::function<void(const std::vector<size_t>&)>>& getPermuteCallbackList<volume::Vertex       >(volume::SurfaceMesh* mesh)   { return mesh->vertexPermuteCallbackList;   }
template<> inline std::list<std::function<void(const std::vector<size_t>&)>>& getPermuteCallbackList<volume::Halfedge     >(volume::SurfaceMesh* mesh)   { return mesh->halfedgePermuteCallbackList;   }
template<> inline std::list<std::function<void(const std::vector<size_t>&)>>& getPermuteCallbackList<volume::Corner       >(volume::SurfaceMesh* mesh)   { return mesh->halfedgePermuteCallbackList;   }
template<> inline std::list<std::function<void(const std::vector<size_t>&)>>& getPermuteCallbackList<volume::Edge         >(volume::SurfaceMesh* mesh)   { return mesh->edgePermuteCallbackList;   }
template<> inline std::list<std::function<void(const std::vector<size_t>&)>>& getPermuteCallbackList<volume::Face         >(volume::SurfaceMesh* mesh)   { return mesh->facePermuteCallbackList;   }
template<> inline std::list<std::function<void(const std::vector<size_t>&)>>& getPermuteCallbackList<volume::BoundaryLoop >(volume::SurfaceMesh* mesh)   { return mesh->boundaryLoopPermuteCallbackList;   }
// template<> inline std::list<std::function<void(const std::vector<size_t>&)>>& getPermuteCallbackList<volume::Tet          >(volume::TetMesh* mesh    )   { return mesh->tetPermuteCallbackList;   }


template<> inline std::string typeShortName<volume::Vertex       >()            { return "v";    }
template<> inline std::string typeShortName<volume::Halfedge     >()            { return "he";   }
template<> inline std::string typeShortName<volume::Corner       >()            { return "c";    }
template<> inline std::string typeShortName<volume::Edge         >()            { return "e";    }
template<> inline std::string typeShortName<volume::Face         >()            { return "f";    }
template<> inline std::string typeShortName<volume::BoundaryLoop >()            { return "bl";   }
// template<> inline std::string typeShortName<volume::Tet          >()            { return "t";   }


// clang-format on

} // namespace geometrycentral
