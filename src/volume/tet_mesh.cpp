#include "geometrycentral/volume/tet_mesh.h"
// #include "set"

namespace geometrycentral{
namespace volume{

SimpleTetMesh::SimpleTetMesh(const std::vector<std::vector<size_t>>& tets_, const std::vector<Vector3>& vertexCoordinates_)
    :SimplePolygonMesh(triangles_from_tets(tets_), vertexCoordinates_), tets(tets_){}

void SimpleTetMesh::mergeIdenticalVertices(){
  std::vector<Vector3> compressedPositions;
  // Store mapping from original vertex index to merged vertex index
  std::vector<size_t> compressVertex;
  compressVertex.reserve(vertexCoordinates.size());

  std::unordered_map<Vector3, size_t> canonicalIndex;

  for (size_t iV = 0; iV < vertexCoordinates.size(); ++iV) {
    Vector3 v = vertexCoordinates[iV];
    auto it = canonicalIndex.find(v);

    // Check if vertex exists in map or not
    if (it == canonicalIndex.end()) {
      compressedPositions.push_back(v);
      size_t vecIndex = compressedPositions.size() - 1;
      canonicalIndex[v] = vecIndex;
      compressVertex.push_back(vecIndex);
    } else {
      size_t vecIndex = it->second;
      compressVertex.push_back(vecIndex);
    }
  }

  vertexCoordinates = std::move(compressedPositions);

  // Update face indices.
  for (std::vector<size_t>& face : polygons) {
    for (size_t& iV : face) {
      iV = compressVertex[iV];
    }
  }
  // update tet indices.
  for (std::vector<size_t>& tet : tets) {
    for (size_t& iV : tet) {
      iV = compressVertex[iV];
    }
  }
}

void SimpleTetMesh::mergeIdenticalFaces(){
    std::unordered_set<Vector3> triangle_set; // Could have a int type Vector3 kind of thing to avoid the double casts. Tuple is not Hashable?
    for(std::vector<size_t> triangle: polygons){
        size_t min = *std::min_element(triangle.begin(), triangle.end()), 
               max = *std::max_element(triangle.begin(), triangle.end()), 
               mid = triangle[0] + triangle[1] + triangle[2] - max - min;
        Vector3 t0{(double)min, (double)mid, (double)max}; // sorry
        triangle_set.insert(t0);
    }
    std::vector<std::vector<size_t>> compressed_triangles;
    compressed_triangles.reserve(triangle_set.size());
    
    //compressing triangle vector set
    for (auto itr = triangle_set.begin(); itr != triangle_set.end(); ++itr) {
        Vector3 tri_vec3 = (*itr);
        std::vector<size_t> tri_vec = {(size_t)tri_vec3[0], (size_t)tri_vec3[1], (size_t)tri_vec3[2]};
        compressed_triangles.push_back(tri_vec);
    }
    polygons = std::vector<std::vector<size_t>>(compressed_triangles);
}


TetMesh::TetMesh(const std::vector<std::vector<size_t>>& tet_v_inds_,
                 const std::vector<std::vector<size_t>>& triangles_)
      :SurfaceMesh(triangles_) {
  // The nonmanifold surface skeleton is already constructed.
  // Faces, vertices, halfedges and edges are initiated.
  nTetsCount = tet_v_inds_.size();
  nTetsCapacityCount = nTetsCount;
  nTetsFillCount = nTetsCount;
  
  tet_objects.reserve(nTetsCount);
  
  // tAdjVs = std::vector<std::vector<size_t>>(n_tets, {INVALID_IND});
  // fAdjTs = std::vector<std::vector<size_t>>(nFaces(), {INVALID_IND});
  tAdjVs.resize(nTetsCapacityCount);
  fAdjTs.resize(nFaces());

  size_t tet_ind = 0;
  for(std::vector<size_t> tet_v_inds: tet_v_inds_){
    
    Tet new_tet(this, tet_ind);
    
    //populating the index-based iterators on Tets
    new_tet.buildAdjVertices(tet_v_inds);
    
    // populating the lazy/dirty to-Tet iterators on Faces. Vertices/Edges will use these later.
    for(size_t v_ind:tet_v_inds){
      // TODO: should encapsulate these set operations in utils. ASAP
      std::vector<size_t> triplet; // instead, we should have some set operations added to utils.
      std::vector<size_t> boring_solo_set{v_ind};
      triplet.reserve(3);
      std::set_difference(tet_v_inds.begin(), tet_v_inds.end(),  // fancy diff
                          boring_solo_set.begin(), boring_solo_set.end(),
                          std::inserter(triplet, triplet.begin()));
      Face tmp_f = get_connecting_face(Vertex(this, triplet[0]), Vertex(this,triplet[1]), Vertex(this,triplet[2]));

      fAdjTs[tmp_f.getIndex()].push_back(tet_ind);
    }
    // Tet object is ready to be pushed in. Was actually ready even before handling the elem.adjT iterators; anyway..
    tet_objects.push_back(new_tet);
    tet_ind++;
  }
}

Face TetMesh::get_connecting_face(Vertex v1, Vertex v2, Vertex v3){
  Edge e1 = connectingEdge(v1, v2);
  Edge e2 = connectingEdge(v1, v3);
  for(Face f1: e1.adjacentFaces()){
    for(Face f2: e2.adjacentFaces()){
      if(f1 == f2){
        return f1;
      }
    }
  }
  return Face();
}


Tet TetMesh::get_connecting_tet(Vertex v1, Vertex v2, Vertex v3, Vertex v4){
  Face f1 = get_connecting_face(v1, v2, v3);
  Face f2 = get_connecting_face(v1, v2, v4);

  for(Tet t1: f1.adjacentTets()){
    for(Tet t2: f2.adjacentTets()){
      if(t1 == t2){
        return t1;
      }
    }
  }
  return Tet();
}


// helper fucntions 
std::vector<std::vector<size_t>> triangles_from_tets(std::vector<std::vector<size_t>> tets_){
    // just adding tet faces. Will compress later
    int face_cnt = 0;
    std::vector<std::vector<size_t>> triangles;
    triangles.reserve(tets_.size()*4);
    for(auto tet: tets_){
        triangles.push_back({tet[0], tet[1], tet[2]});
        triangles.push_back({tet[1], tet[2], tet[3]});
        triangles.push_back({tet[0], tet[1], tet[3]});
        triangles.push_back({tet[0], tet[2], tet[3]});
        face_cnt += 4;
    }
    
    //compressing in the case of bounded volume; which always happens
    std::vector<std::vector<size_t>> compressed_triangles;
    compressed_triangles.reserve(face_cnt);
    
    for (std::vector<size_t> triangle: triangles) {
        compressed_triangles.push_back(triangle);
    }
    return compressed_triangles;
}

// mesh resize routines
Tet TetMesh::getNewTet(){
  if (nTetsFillCount < nTetsCapacityCount) {
    // No work needed
  }
  // The intesting case, where vectors resize
  else {
    size_t newCapacity = nTetsCapacityCount * 2;

    // Resize internal arrays
    tAdjVs.resize(newCapacity);

    nTetsCapacityCount = newCapacity;


    // Invoke relevant callback functions
    for (auto& f : tetExpandCallbackList) {
      f(newCapacity);
    }
  }

  nTetsFillCount++;
  nTetsCount++;

  modificationTick++;
  isCompressedFlag = false;
  return Tet(this, nTetsFillCount - 1);
}

void TetMesh::compressTets(){
  // assuming the trivial case for now
  std::vector<std::vector<size_t>> newTetAdjVs(nTetsFillCount);
  for(size_t t_ind = 0; t_ind < nTetsFillCount; t_ind++){
    newTetAdjVs[t_ind] = tAdjVs[t_ind];
  }
  nTetsCapacityCount = nTetsFillCount;
  tAdjVs = newTetAdjVs;
}


// mutation routines
Vertex TetMesh::buildVolOnFace(Face fIn){
  std::cout<<"  -- starting to build Tet on face "<< fIn.getIndex()<<"\n";
  Vertex raisedVertex = raiseVertexOnFace(fIn); // raise the surface skeleton first
  printf("surface skeleton was raised by vertex %d\n", raisedVertex.getIndex());
  Tet newT = getNewTet();

  fAdjTs.resize(nFacesFillCount + fIn.degree()); // couldnt do it in surfaceMesh::raiseVert...()
  

  // update the Tet->V and F->T iterators
  std::vector<size_t> tetVertices(fIn.degree() + 1);
  size_t i = 0;
  std::cout<<"2.5.5\n";
  for(Halfedge he: fIn.adjacentHalfedges()){
    tetVertices[i] = he.tailVertex().getIndex();
    Face tmp_raised_face = get_connecting_face(he.tailVertex(), he.tipVertex(), raisedVertex);
    if(tmp_raised_face.getIndex() == INVALID_IND) std::cout<<" raise Vertex on Face had gone wrong! :(\n";
    else{
      fAdjTs[tmp_raised_face.getIndex()].push_back(newT.getIndex());
    }
    i += 1;
  }

  tetVertices[fIn.degree()] = raisedVertex.getIndex();
  fAdjTs[fIn.getIndex()].push_back(newT.getIndex());
  tAdjVs[newT.getIndex()] = tetVertices;
  

  return raisedVertex;
}

Vertex TetMesh::splitTet(Tet tIn){
  
  // Create the new center vertex
  Vertex centerVert = getNewVertex();

  // Count degree to allocate elements; can only handle tets for now, but will attempt to keep generality as long as possible
  size_t volDegree = tIn.degree();

  // == Create new halfedges/edges/faces around the center vertex

  // Create all of the new elements first, then hook them up below
  std::vector<Tet> innerTets;
  std::vector<Face> innerFaces;
  std::vector<std::vector<Halfedge>> leadingHalfedges(volDegree); // the one that points towards the center; one per each face
  std::vector<std::vector<Halfedge>> trailingHalfedges(volDegree);
  std::vector<Edge> innerEdges(volDegree); // aligned with leading he
  for (size_t i = 0; i < volDegree; i++) {
    // Re-use first face
    if (i == 0) {
      // innerFaces.push_back(fIn);
      innerTets.push_back(tIn);
    } else {
      innerTets.push_back(getNewTet());
    }

    // Get the new edge group
    // one he per neigh face of current vertex; tet assumption -> 3
    size_t neigh_he_cnt = 3;
    Halfedge he1 = getNewEdgeTriple(false), he2, he3;
    

    Halfedge newHe = getNewEdgeTriple(false);

    leadingHalfedges[i] = newHe;
    trailingHalfedges[(i + 1) % volDegree] = newHe.twin(); // these inner edges only have 2 adj faces, so it makes sense. can always do sibling instead
    innerEdges[i] = newHe.edge();

    for (size_t j = 0; j < volDegree; j++) { // making a Face per each edge of the volume
      if(j < i) innerFaces.push_back(getNewFace());
    }  
  }

  // // Form this list before we start, because we're about to start breaking pointers
  // std::vector<Face> boundaryFaces;
  // std::vector<std::vector<Halfedge>> boundaryFacesHalfedges;
  // for ( Face f: tIn.adjFaces()){
  //   boundaryFaces.push_back(f);
  //   std::vector<Halfedge> tmp_boundary_halfedges;
  //   for (Halfedge he : f.adjacentHalfedges()) {
  //     tmp_boundary_halfedges.push_back(he);
  //   }
  //   boundaryFacesHalfedges.push_back(tmp_boundary_halfedges);
  // }

  // // Connect up all the pointers
  // modificationTick++;
  // return centerVert;
  return Vertex(this, INVALID_IND);
}

} // namespace volume
} // namespace 