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

Halfedge TetMesh::get_he_of_edge_on_face(Edge e, Face f){
  Halfedge he = e.halfedge();
  Halfedge first_he = he;
  while(true) {
    if(he.face() == f) return he;
    he = he.sibling();
    if(he == first_he) break;
  }
  return Halfedge();
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
  Vertex raisedVertex = raiseVertexOnFace(fIn); // raise the surface skeleton first
  printf("surface skeleton was raised by vertex %d\n", raisedVertex.getIndex());
  Tet newT = getNewTet();
  fAdjTs.resize(nFacesFillCount + fIn.degree()); // couldnt do it in surfaceMesh::raiseVert...()
  

  // update the Tet->V and F->T iterators
  std::vector<size_t> tetVertices(fIn.degree() + 1);
  size_t i = 0;
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


Vertex TetMesh::splitTet(Tet tIn){ // An implementation I will go to hell for..
  
  // Create the new center vertex
  Vertex centerVert = getNewVertex();

  // going for the assumption that faces are triangles and cells are tets.
  // ===== Creating elements ======
  // - boundary vertices
  std::vector<Vertex> vertices = tIn.adjVertices();
  Vertex v1 = vertices[0], v2 = vertices[1], v3 = vertices[2], v4 = vertices[3]; // vertices of tIn
  printf(" *** Tet vertices are (%d,%d,%d,%d)", v1.getIndex(), v2.getIndex(), v3.getIndex(), v4.getIndex());
  // - boundary faces
  Face f234 = get_connecting_face(v2,v3,v4), f134 = get_connecting_face(v1,v3,v4),
       f124 = get_connecting_face(v1,v2,v4), f123 = get_connecting_face(v1,v2,v3);
  // - duplicating boundary edges
  Edge bE12 = connectingEdge(v1, v2), bE13 = connectingEdge(v1, v3), bE14 = connectingEdge(v1, v4),
       bE23 = connectingEdge(v2, v3), bE24 = connectingEdge(v2, v4),
       bE34 = connectingEdge(v3, v4);
  
  // Create all of the new elements (may some god forgive me)
  // - new halfedges on boundary edges
  Halfedge bhe12 = getNewHalfedge(true), bhe13 = getNewHalfedge(true), bhe14 = getNewHalfedge(true), 
           bhe23 = getNewHalfedge(true), bhe24 = getNewHalfedge(true), 
           bhe34 = getNewHalfedge(true);

  printf(" *** bounary he inds first, third, last: %d, %d , %d\n", bhe12.getIndex(), bhe14.getIndex(), bhe34.getIndex());
  // - new tets
  Tet t0123 = tIn, t0124 = getNewTet(), t0134 = getNewTet(), t0234 = getNewTet(); 
  // - new inner faces; one per boundary edge
  Face f012 = getNewFace(), f013 = getNewFace(), f014 = getNewFace(),
       f023 = getNewFace(), f024 = getNewFace(), 
       f034 = getNewFace();
  // - new inner halfEdges
  Halfedge he01_012 = getNewHalfedge(true), he01_013 = getNewHalfedge(true), he01_014 = getNewHalfedge(true),
           he02_021 = getNewHalfedge(true), he02_023 = getNewHalfedge(true), he02_024 = getNewHalfedge(true),
           he03_031 = getNewHalfedge(true), he03_032 = getNewHalfedge(true), he03_034 = getNewHalfedge(true),
           he04_041 = getNewHalfedge(true), he04_042 = getNewHalfedge(true), he04_043 = getNewHalfedge(true);
  Edge e01 = getNewEdge(), e02 = getNewEdge(), e03 = getNewEdge(), e04 = getNewEdge();
  // ======= hooking up elements ======= (may some god forgive me)
  // first for higher level stuff 
  // tet -> vertex && face -> tet
  printf(" *** inner he inds first, third, last: %d, %d , %d\n", he01_012.getIndex(), he01_014.getIndex(), he04_043.getIndex());
  fAdjTs.resize(nFacesFillCount + 4);
  for(int i = 0 ; i < fAdjTs[f123.getIndex()].size() ; i++){
    if(fAdjTs[f123.getIndex()][i] == tIn.getIndex()) fAdjTs[f123.getIndex()][i] = t0123.getIndex(); 
  }
  for(int i = 0 ; i < fAdjTs[f124.getIndex()].size() ; i++){
    if(fAdjTs[f124.getIndex()][i] == tIn.getIndex()) fAdjTs[f124.getIndex()][i] = t0124.getIndex(); 
  }
  for(int i = 0 ; i < fAdjTs[f134.getIndex()].size() ; i++){
    if(fAdjTs[f134.getIndex()][i] == tIn.getIndex()) fAdjTs[f134.getIndex()][i] = t0134.getIndex(); 
  }
  for(int i = 0 ; i < fAdjTs[f234.getIndex()].size() ; i++){
    if(fAdjTs[f234.getIndex()][i] == tIn.getIndex()) fAdjTs[f234.getIndex()][i] = t0234.getIndex(); 
  }
  tAdjVs[t0123.getIndex()] = {centerVert.getIndex(), v1.getIndex(), v2.getIndex(), v3.getIndex()};
  tAdjVs[t0124.getIndex()] = {centerVert.getIndex(), v1.getIndex(), v2.getIndex(), v4.getIndex()};
  tAdjVs[t0134.getIndex()] = {centerVert.getIndex(), v1.getIndex(), v3.getIndex(), v4.getIndex()};
  tAdjVs[t0234.getIndex()] = {centerVert.getIndex(), v4.getIndex(), v2.getIndex(), v3.getIndex()};
  // vertex -> he (just for center)
  vHalfedgeArr[centerVert.getIndex()] = he01_012.getIndex();
  vHeOutStartArr[centerVert.getIndex()] = INVALID_IND; // will be handled later; TODO this should hold by default.
  vHeInStartArr[centerVert.getIndex()] = INVALID_IND; // will be handled later; TODO this should hold by default.

  // all he -> edge
  heEdgeArr[bhe12.getIndex()] = bE12.getIndex();heEdgeArr[bhe13.getIndex()] = bE13.getIndex(); heEdgeArr[bhe14.getIndex()] = bE14.getIndex();heEdgeArr[bhe23.getIndex()] = bE23.getIndex();heEdgeArr[bhe24.getIndex()] = bE24.getIndex();heEdgeArr[bhe34.getIndex()] = bE34.getIndex();
  heEdgeArr[he01_012.getIndex()] = e01.getIndex(); heEdgeArr[he01_013.getIndex()] = e01.getIndex(); heEdgeArr[he01_014.getIndex()] = e01.getIndex();
  heEdgeArr[he02_021.getIndex()] = e02.getIndex(); heEdgeArr[he02_023.getIndex()] = e02.getIndex(); heEdgeArr[he02_024.getIndex()] = e02.getIndex();
  heEdgeArr[he03_031.getIndex()] = e03.getIndex(); heEdgeArr[he03_032.getIndex()] = e03.getIndex(); heEdgeArr[he03_034.getIndex()] = e03.getIndex();
  heEdgeArr[he04_041.getIndex()] = e04.getIndex(); heEdgeArr[he04_042.getIndex()] = e04.getIndex(); heEdgeArr[he04_043.getIndex()] = e04.getIndex(); 
  // all edge -> halfedge
  eHalfedgeArr[e01.getIndex()] = he01_012.getIndex();
  eHalfedgeArr[e02.getIndex()] = he02_021.getIndex();
  eHalfedgeArr[e03.getIndex()] = he03_034.getIndex();
  eHalfedgeArr[e04.getIndex()] = he04_041.getIndex();

  // all he -> face (inner face only)
  heFaceArr[bhe12.getIndex()] = f012.getIndex(); heFaceArr[bhe13.getIndex()] = f013.getIndex(); heFaceArr[bhe14.getIndex()] = f014.getIndex(); heFaceArr[bhe23.getIndex()] = f023.getIndex(); heFaceArr[bhe24.getIndex()] = f024.getIndex(); heFaceArr[bhe34.getIndex()] = f034.getIndex();
  heFaceArr[he01_012.getIndex()] = f012.getIndex(); heFaceArr[he01_013.getIndex()] = f013.getIndex(); heFaceArr[he01_014.getIndex()] = f014.getIndex();
  heFaceArr[he02_021.getIndex()] = f012.getIndex(); heFaceArr[he02_023.getIndex()] = f023.getIndex(); heFaceArr[he02_024.getIndex()] = f024.getIndex();
  heFaceArr[he03_031.getIndex()] = f013.getIndex(); heFaceArr[he03_032.getIndex()] = f023.getIndex(); heFaceArr[he03_034.getIndex()] = f034.getIndex();
  heFaceArr[he04_041.getIndex()] = f014.getIndex(); heFaceArr[he04_042.getIndex()] = f024.getIndex(); heFaceArr[he04_043.getIndex()] = f034.getIndex();
  // [inner] face -> [boundary] halfedge
  fHalfedgeArr[f012.getIndex()] = bhe12.getIndex(); fHalfedgeArr[f013.getIndex()] = bhe13.getIndex(); fHalfedgeArr[f014.getIndex()] = bhe14.getIndex(); 
  fHalfedgeArr[f023.getIndex()] = bhe23.getIndex(); fHalfedgeArr[f024.getIndex()] = bhe24.getIndex(); 
  fHalfedgeArr[f034.getIndex()] = bhe34.getIndex();
  
  // all he -> vertex  &&  all he -> next 
  //    012 face
  heVertexArr[bhe12.getIndex()] = v1.getIndex();
  heNextArr[bhe12.getIndex()] = he02_021.getIndex();
  heVertexArr[he02_021.getIndex()] = v2.getIndex();
  heNextArr[he02_021.getIndex()] = he01_012.getIndex();
  heVertexArr[he01_012.getIndex()] = centerVert.getIndex();
  heNextArr[he01_012.getIndex()] = bhe12.getIndex();

  heOrientArr[bhe12.getIndex()] = bE12.firstVertex() == v1;
  //    013 face
  heVertexArr[bhe13.getIndex()] = v1.getIndex();
  heNextArr[bhe13.getIndex()] = he03_031.getIndex();
  heVertexArr[he03_031.getIndex()] = v3.getIndex();
  heNextArr[he03_031.getIndex()] = he01_013.getIndex();
  heVertexArr[he01_013.getIndex()] = centerVert.getIndex();
  heNextArr[he01_013.getIndex()] = bhe13.getIndex();

  heOrientArr[bhe13.getIndex()] = bE13.firstVertex() == v1;
  //    014 face
  heVertexArr[bhe14.getIndex()] = v1.getIndex();
  heNextArr[bhe14.getIndex()] = he04_041.getIndex();
  heVertexArr[he04_041.getIndex()] = v4.getIndex();
  heNextArr[he04_041.getIndex()] = he01_014.getIndex();
  heVertexArr[he01_014.getIndex()] = centerVert.getIndex();
  heNextArr[he01_014.getIndex()] = bhe14.getIndex();

  heOrientArr[bhe14.getIndex()] = bE14.firstVertex() == v1;
  //    023 face
  heVertexArr[bhe23.getIndex()] = v2.getIndex();
  heNextArr[bhe23.getIndex()] = he03_032.getIndex();
  heVertexArr[he03_032.getIndex()] = v3.getIndex();
  heNextArr[he03_032.getIndex()] = he02_023.getIndex();
  heVertexArr[he02_023.getIndex()] = centerVert.getIndex();
  heNextArr[he02_023.getIndex()] = bhe23.getIndex();

  heOrientArr[bhe23.getIndex()] = bE23.firstVertex() == v2;
  //    024 face
  heVertexArr[bhe24.getIndex()] = v2.getIndex();
  heNextArr[bhe24.getIndex()] = he04_042.getIndex();
  heVertexArr[he04_042.getIndex()] = v4.getIndex();
  heNextArr[he04_042.getIndex()] = he02_024.getIndex();
  heVertexArr[he02_024.getIndex()] = centerVert.getIndex();
  heNextArr[he02_024.getIndex()] = bhe24.getIndex();

  heOrientArr[bhe24.getIndex()] = bE24.firstVertex() == v2;
  //    034 face
  heVertexArr[bhe34.getIndex()] = v3.getIndex();
  heNextArr[bhe34.getIndex()] = he04_043.getIndex();
  heVertexArr[he04_043.getIndex()] = v4.getIndex();
  heNextArr[he04_043.getIndex()] = he03_034.getIndex();
  heVertexArr[he03_034.getIndex()] = centerVert.getIndex();
  heNextArr[he03_034.getIndex()] = bhe34.getIndex();

  heOrientArr[bhe34.getIndex()] = bE34.firstVertex() == v3;


  // - orientation of all he
  std::cout<< "orientation \n";

  // eHalfedgeArr[e01.getIndex()] = he01_012.getIndex();
  // eHalfedgeArr[e02.getIndex()] = he02_021.getIndex();
  // eHalfedgeArr[e03.getIndex()] = he03_034.getIndex();
  // eHalfedgeArr[e04.getIndex()] = he04_041.getIndex();
  heOrientArr[he01_012.getIndex()] = true;
  heOrientArr[he01_013.getIndex()] = (e01.firstVertex() == he01_013.vertex());
  heOrientArr[he01_014.getIndex()] = (e01.firstVertex() == he01_014.vertex());

  heOrientArr[he02_021.getIndex()] = true;
  heOrientArr[he02_023.getIndex()] = (e02.firstVertex() == he02_023.vertex());
  heOrientArr[he02_024.getIndex()] = (e02.firstVertex() == he02_024.vertex());

  heOrientArr[he03_034.getIndex()] = true;
  heOrientArr[he03_031.getIndex()] = (e03.firstVertex() == he03_031.vertex());
  heOrientArr[he03_032.getIndex()] = (e03.firstVertex() == he03_032.vertex());

  heOrientArr[he04_041.getIndex()] = true;
  heOrientArr[he04_042.getIndex()] = (e04.firstVertex() == he04_042.vertex());
  heOrientArr[he04_043.getIndex()] = (e04.firstVertex() == he04_043.vertex());

  // - sibling relationships
  //    - inner ge's sibling rels
  std::cout<< "inner siblings \n";

  heSiblingArr[he01_012.getIndex()] = he01_013.getIndex(); heSiblingArr[he01_013.getIndex()] = he01_014.getIndex(); heSiblingArr[he01_014.getIndex()] = he01_012.getIndex(); 
  heSiblingArr[he02_021.getIndex()] = he02_023.getIndex(); heSiblingArr[he02_023.getIndex()] = he02_024.getIndex(); heSiblingArr[he02_024.getIndex()] = he02_021.getIndex();
  heSiblingArr[he03_031.getIndex()] = he03_032.getIndex(); heSiblingArr[he03_032.getIndex()] = he03_034.getIndex(); heSiblingArr[he03_034.getIndex()] = he03_031.getIndex();
  heSiblingArr[he04_041.getIndex()] = he04_042.getIndex(); heSiblingArr[he04_042.getIndex()] = he04_043.getIndex(); heSiblingArr[he04_043.getIndex()] = he04_041.getIndex();
  //    - inner he's sibling rels
  //      ** I'll try to respect the sibling order of older halfEdges
  
  std::cout<< "boundary siblings \n";
  Halfedge first_he, second_he;
  //    12
  Halfedge he12_f123 = get_he_of_edge_on_face(bE12, f123),
           he12_f124 = get_he_of_edge_on_face(bE12, f124);
  if     (he12_f123.sibling() == he12_f124) {  first_he = he12_f123;  second_he = he12_f124;}
  else if(he12_f124.sibling() == he12_f123) {  first_he = he12_f124;  second_he = he12_f123;}
  else{
    std::logic_error(" --------- adj faces on a tet should be siblings ----------");
  }
  heSiblingArr[first_he.getIndex()] = bhe12.getIndex();
  heSiblingArr[bhe12.getIndex()] = second_he.getIndex();
  //    13
  Halfedge he13_f132 = get_he_of_edge_on_face(bE13, f123),
           he13_f134 = get_he_of_edge_on_face(bE13, f134);
  if     (he13_f132.sibling() == he13_f134) {  first_he = he13_f132;  second_he = he13_f134;}
  else if(he13_f134.sibling() == he13_f132) {  first_he = he13_f134;  second_he = he13_f132;}
  else{
    std::logic_error(" --------- adj faces on a tet should be tailing he siblings ----------");
  }
  heSiblingArr[first_he.getIndex()] = bhe13.getIndex();
  heSiblingArr[bhe13.getIndex()] = second_he.getIndex();
  //    14
  Halfedge he14_f142 = get_he_of_edge_on_face(bE14, f124),
           he14_f143 = get_he_of_edge_on_face(bE14, f134);
  if     (he14_f142.sibling() == he14_f143) {  first_he = he14_f142;  second_he = he14_f143;}
  else if(he14_f143.sibling() == he14_f142) {  first_he = he14_f143;  second_he = he14_f142;}
  else{
    std::logic_error(" --------- adj faces on a tet should be tailing he siblings ----------");
  }
  heSiblingArr[first_he.getIndex()] = bhe14.getIndex();
  heSiblingArr[bhe14.getIndex()] = second_he.getIndex();
  // printf(" *** looking for boundary halfedge corres to faces %d : (%d,%d,%d) and %d : (%d,%d,%d) \n", 
  //                               f124.getIndex(), f124.halfedge().tailVertex().getIndex(), f124.halfedge().tipVertex().getIndex(), f124.halfedge().next().tipVertex().getIndex(),
  //                               f134.getIndex(), f134.halfedge().tailVertex().getIndex(), f134.halfedge().tipVertex().getIndex(), f134.halfedge().next().tipVertex().getIndex());
  // printf("boundary edge %d (%d,%d)'s he is %d (%d, %d)\n", bE14.getIndex(), bE14.firstVertex().getIndex(), bE14.secondVertex().getIndex(),
  //                                                       bE14.halfedge().getIndex(), bE14.halfedge().tailVertex().getIndex(), bE14.halfedge().tipVertex().getIndex());
  // Halfedge s1 = bE14.halfedge().sibling() , s2 = bE14.halfedge().sibling().sibling();
  // printf("    - and the sibling is: (%d,%d) with f %d  -- (%d,%d) with f %d \n", 
  //                                 s1.tailVertex().getIndex(), s1.tipVertex().getIndex(), s1.face().getIndex(), 
  //                                 s2.tailVertex().getIndex(), s2.tipVertex().getIndex(), s2.face().getIndex());
  // printf("boundary edge %d (%d,%d) vs boundary he (%d, %d)\n", bE14.getIndex(), bE14.firstVertex().getIndex(), bE14.secondVertex().getIndex(),
  //                                                              bhe14.tailVertex().getIndex(), bhe14.tipVertex().getIndex());
  // printf("he's getting hooked up (%d, %d) (%d, %d) (%d, %d)\n",
  //         first_he.tailVertex().getIndex(), first_he.tipVertex().getIndex(), 
  //         second_he.tailVertex().getIndex(), second_he.tipVertex().getIndex(), 
  //         bhe14.tailVertex().getIndex(), bhe14.tipVertex().getIndex());
  // //    23
  Halfedge he23_f231 = get_he_of_edge_on_face(bE23, f123),
           he23_f234 = get_he_of_edge_on_face(bE23, f234);
  if     (he23_f231.sibling() == he23_f234) {  first_he = he23_f231;  second_he = he23_f234;}
  else if(he23_f234.sibling() == he23_f231) {  first_he = he23_f234;  second_he = he23_f231;}
  else{
    std::logic_error(" --------- adj faces on a tet should be tailing he siblings ----------");
  }
  heSiblingArr[first_he.getIndex()] = bhe23.getIndex();
  heSiblingArr[bhe23.getIndex()] = second_he.getIndex();
  //    24
  Halfedge he24_f241 = get_he_of_edge_on_face(bE24, f124),
           he24_f243 = get_he_of_edge_on_face(bE24, f234);
  if     (he24_f241.sibling() == he24_f243) {  first_he = he24_f241;  second_he = he24_f243;}
  else if(he24_f243.sibling() == he24_f241) {  first_he = he24_f243;  second_he = he24_f241;}
  else{
    std::logic_error(" --------- adj faces on a tet should be tailing he siblings ----------");
  }
  heSiblingArr[first_he.getIndex()] = bhe24.getIndex();
  heSiblingArr[bhe24.getIndex()] = second_he.getIndex();
  //    34
  Halfedge he34_f341 = get_he_of_edge_on_face(bE34, f134),
           he34_f342 = get_he_of_edge_on_face(bE34, f234);
  if     (he34_f341.sibling() == he34_f342) {  first_he = he34_f341;  second_he = he34_f342;}
  else if(he34_f342.sibling() == he34_f341) {  first_he = he34_f342;  second_he = he34_f341;}
  else{
    std::logic_error(" --------- adj faces on a tet should be tailing he siblings ----------");
  }
  heSiblingArr[first_he.getIndex()] = bhe34.getIndex();
  heSiblingArr[bhe34.getIndex()] = second_he.getIndex();
  
  std::cout << "handling vertex in outs \n";

  addToVertexLists(bhe12); addToVertexLists(bhe13); addToVertexLists(bhe14); 
  addToVertexLists(bhe23); addToVertexLists(bhe24); 
  addToVertexLists(bhe34);
  addToVertexLists(he01_012); addToVertexLists(he01_013); addToVertexLists(he01_014);
  addToVertexLists(he02_021); addToVertexLists(he02_023); addToVertexLists(he02_024);
  addToVertexLists(he03_031); addToVertexLists(he03_032); addToVertexLists(he03_034);
  addToVertexLists(he04_041); addToVertexLists(he04_042); addToVertexLists(he04_043);
  return centerVert;
}

void TetMesh::validateConnectivity(){
  // for the surface skeleton first (Thanks Nick!)
  SurfaceMesh::validateConnectivity();
  // Tet stuff 
  for(Face f : faces()){
    bool found_it = false;
    for(Tet t : f.adjacentTets()){
      for(Face tf: t.adjFaces()){
        if(tf == f) found_it = true;
      }
    }
    if(!found_it) std::logic_error("face.tet did not have face in tet.faces!");
  }
}

} // namespace volume
} // namespace 