#pragma once

#include "geometrycentral/volume/halfedge_element_types.h"
#include "geometrycentral/utilities/element.h"
#include "geometrycentral/utilities/element_iterators.h"
#include "geometrycentral/utilities/mesh_data.h"
#include "geometrycentral/utilities/utilities.h"

#include <cstddef>
#include <iostream>
#include <list>
#include <typeindex>
#include <array>
#include <unordered_set>

namespace geometrycentral {
namespace volume {

// === Types and inline methods for the halfedge mesh pointer and datatypes
// class TetMesh;
// class Tet;  

// // ==========================================================
// // ================          Tet          ==================
// // ==========================================================
// class Tet : public Element<Tet, TetMesh> {
// public:
//   // Constructors
//   Tet();                              // construct an empty (null) element
//   Tet(TetMesh* mesh, size_t ind); // construct pointing to the i'th element of that type on a mesh.
//   Tet(TetMesh* mesh, size_t ind, std::vector<Vertex> vertices); // there is no cool halfEdge indicators for tets, so..

//   // Lazy iterators; probably should be protected (use getter/setter)
//   std::vector<Vertex> adjVertices;
//   std::vector<Edge> adjEdges;
//   std::vector<Face> adjFaces;

//   void buildAdjEdges(); // should only work if adjVertices is populated
//   void buildAdjFaces(); // ..same..
  
//   bool isDead() const;

//   // Properties
//   bool isTet() const; // not handling polyhedras at the moment, so..
//   size_t degree() const;

//   // Iterators
//   // NavigationSetBase<TetAdjacentVertexNavigator> adjacentVertices() const;
//   // NavigationSetBase<TetAdjacentHalfedgeNavigator> adjacentHalfedges() const;
//   // NavigationSetBase<TetAdjacentCornerNavigator> adjacentCorners() const;
//   // NavigationSetBase<TetAdjacentEdgeNavigator> adjacentEdges() const;
//   // NavigationSetBase<TetAdjacentFaceNavigator> adjacentFaces() const;
// };
// // using DynamicTet = DynamicElement<Tet>;

// // == Range iterators

// // All tets
// struct TetRangeF {
//   static bool elementOkay(const TetMesh& mesh, size_t ind);
//   typedef Tet Etype;
//   typedef TetMesh ParentMeshT;
// };
// typedef RangeSetBase<TetRangeF> TetSet;


// ==========================================================
// ===============   Navigation Iterators   =================
// ==========================================================



} // namespace surface

// Declare specializations of the logic templates. This is important, because these need to be declared before any of
// the templates using them are instantiated.

// clang-format off

template<> inline size_t nElements<volume::Tet          >(volume::TetMesh*     mesh);

template<> inline size_t elementCapacity<volume::Tet         >(volume::TetMesh*     mesh);

template<> inline size_t dataIndexOfElement<volume::Tet             >(volume::TetMesh*     mesh, volume::Tet e              );

template<> struct ElementSetType<volume::Tet           >   { typedef volume::Tet             type; };

// template<> inline volume::TetSet            iterateElements<volume::Tet         >(volume::TetMesh*     mesh);

template<> inline std::list<std::function<void(size_t)>>& getExpandCallbackList<volume::Tet         >(volume::TetMesh*     mesh);

template<> inline std::list<std::function<void(const std::vector<size_t>&)>>& getPermuteCallbackList<volume::Tet          >(volume::TetMesh*     mesh);

template<> inline std::string typeShortName<volume::Tet          >();


// clang-format on

} // namespace geometrycentral

