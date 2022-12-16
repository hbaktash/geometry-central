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
    for(size_t v_ind: tet_v_inds){
      // TODO: should encapsulate these set operations in utils. ASAP
      std::vector<size_t> triplet; // instead, we should have some set operations added to utils.
      for(size_t other_v_ind: tet_v_inds) if (other_v_ind != v_ind) triplet.push_back(other_v_ind);
      Face tmp_f = get_connecting_face(Vertex(this, triplet[0]), Vertex(this,triplet[1]), Vertex(this,triplet[2]));
      if (tet_ind == 46 && tmp_f.getIndex() == 73){
        printf("tet %d and face %d\n", tet_ind, tmp_f.getIndex());
      }
      fAdjTs[tmp_f.getIndex()].push_back(tet_ind);
    }
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
  Halfedge he = f.halfedge();
  Halfedge first_he = he;
  while(true) {
    if(he.edge() == e) return he;
    he = he.next();
    if(he == first_he) break;
  }
  printf("face %d\n", f.getIndex());
  for (Vertex v: f.adjacentVertices()) printf(" - v %d\n", v.getIndex());
  printf("edge %d\n", e.getIndex());
  for (Vertex v: e.adjacentVertices()) printf(" - v %d\n", v.getIndex());
  throw std::logic_error("get_he_of_edge_on_face:he of e on f FAILED!"); // throwing a logic error for now; debugging purposes
  return Halfedge();
}


Edge TetMesh::common_edge_of_faces(Face f1, Face f2){
  for (Halfedge he1: f1.adjacentHalfedges()){
    for (Halfedge he2: f2.adjacentHalfedges()){
      if (he1.edge() == he2.edge())
        return he1.edge();
    }
  }
  return Edge();
}



Tet TetMesh::next_tet_along_face(Tet t, Face f){
  if (face_is_boundary(f)) return Tet();
  for (Tet adjT: f.adjacentTets()){
    if (adjT != t){
      return adjT;
    }
  }
  printf("Tet\n");
  for (Vertex v: t.adjVertices()) printf("  -- v %d\n", v.getIndex());
  printf("Face %d\n", f.getIndex());
  for (Vertex v: f.adjacentVertices()) printf("  -- v %d\n", v.getIndex());
  throw std::logic_error("next_tet_along_face: The face is probably not on the input tet!"); // throwing a logic error for now; debugging purposes
  return Tet();
}


Halfedge TetMesh::boundary_he_of_edge(Edge e){
  // finding a boundary he -> face
  Halfedge first_he = e.halfedge(), he = e.halfedge();
  Halfedge boundary_he = Halfedge();
  while (true) {
    if (face_is_boundary(he.face())) {
      boundary_he = he;
      break;
    }
    he = he.sibling();
    if (he == first_he) break;
  }
  return boundary_he;
}


// to have faces on the same tet be siblings; like iterating over siblings without the sibling assignemnts
void TetMesh::order_siblings_of_edge(Edge e){
  // finding a boundary he -> face
  Halfedge boundary_he = boundary_he_of_edge(e);

  // select starting tet and face (for direction or iteration)
  Halfedge current_he = e.halfedge();
  Tet current_tet = e.halfedge().face().adjacentTets()[0];
  Face current_face = e.halfedge().face(); 
  if (boundary_he.getIndex() != INVALID_IND){ // we have boundary
    current_he = boundary_he;
    current_face = boundary_he.face();
    current_tet = boundary_he.face().adjacentTets()[0]; // should have exactly one element
  }
  // re-arrange siblings; by iterating over neighboring tets using 
  Face next_face;
  Halfedge first_he = current_he; // for the final hook-up / termination
  while (true) {  // each iteration needs: current_tet, current_he -> current_face 
    // getting next face
    Vertex otherV; // other vertex of current_tet, not in current_face
    std::vector<Vertex> cFVs; // current_face vertices
    for (Vertex cFV: current_he.face().adjacentVertices()) cFVs.push_back(cFV);
    for (Vertex tV: current_tet.adjVertices()) if (tV != cFVs[0] && tV != cFVs[1] && tV != cFVs[2]) otherV = tV;
    // printf("other v: %d   ---  cfVs: %d, %d, %d \n", otherV.getIndex(), cFVs[0].getIndex(), cFVs[1].getIndex(), cFVs[2].getIndex());
    next_face = get_connecting_face(otherV, e.firstVertex(), e.secondVertex()); // sorry but other V should be first, since we are disrupting siblings
    Halfedge next_he = get_he_of_edge_on_face(e, next_face);
    // next halfedge goes here
    heSiblingArr[current_he.getIndex()] = next_he.getIndex();
    // update current tet and face
    current_tet = next_tet_along_face(current_tet, next_face);
    current_face = next_face;
    current_he = next_he;
    // terminate if on boundary
    if (current_tet.getIndex() == INVALID_IND){
      heSiblingArr[current_he.getIndex()] = first_he.getIndex();
      break;
    }
    // terminate if the loop is complete
    if (current_he == first_he) break;
  }
}

void TetMesh::order_all_siblings(){
  for (Edge e: edges()){
    order_siblings_of_edge(e);
  }
  siblings_are_ordered = true;
}


Vertex TetMesh::splitEdge(Edge e){ // assumes triangularity
  // start from a boundary halfedge (h->face is boundary)
  Halfedge boundary_he = boundary_he_of_edge(e);
  bool have_boundary = boundary_he.getIndex() != INVALID_IND;
  Halfedge first_he = have_boundary ? boundary_he : e.halfedge();
  // the new vertex
  Vertex new_v = getNewVertex();

  std::vector<Tet> upper_tets, // shadow current tets
                   lower_tets; // new
  std::vector<Face> wedge_bisecting_faces; // 1 per tet
  // this are behind the tets during iteration
  std::vector<Face> upper_faces, // shadow current faces
                    lower_faces; // new
  Edge upper_pilar_edge, // shadow current faces
       lower_pilar_edge; // new
  std::vector<Halfedge> upper_pilar_hes, // shadow current faces
                        lower_pilar_hes; // new
  std::vector<Edge> bisecting_edges; // new
  std::vector<Halfedge> upper_bisecting_hes, // new
                        lower_bisecting_hes, // new
                        wedge_bisecting_hes_pre,
                        wedge_bisecting_hes_pro; // new
  std::vector<Edge> wedge_loop_edges; // old stuff
  std::vector<Halfedge> wedge_loop_hes; // new

  Edge new_pilar_edge = getNewEdge();
  lower_pilar_edge = new_pilar_edge;
  upper_pilar_edge = e;

  Halfedge current_he = first_he,
           sib_he = first_he.sibling();
  Face current_face = current_he.face(),
       sib_face  = sib_he.face();
  Tet current_tet = get_connecting_tet(current_he.tipVertex(), current_he.tailVertex(), current_he.next().tipVertex(), sib_he.next().tipVertex()); 
  // iterate till next boundary; or till we loop back
  size_t nSibs = 0;
  while (true) {
    nSibs++;
    if (!face_is_boundary(current_face) || sib_he != first_he){ // #tet = #face - 1; if we have boundary
      Tet new_tet = getNewTet();
      lower_tets.push_back(new_tet);
      upper_tets.push_back(current_tet);
      Face wedge_face = getNewFace();
      wedge_bisecting_faces.push_back(wedge_face);
      Vertex v1 = current_he.next().tipVertex(), 
             v2 = sib_he.next().tipVertex();
      wedge_loop_edges.push_back(connectingEdge(v1, v2));
      Halfedge wedge_loop_he = getNewHalfedge(true);
      wedge_loop_hes.push_back(wedge_loop_he);
      Halfedge wedge_bisecting_he_pre = getNewHalfedge(true),
               wedge_bisecting_he_pro = getNewHalfedge(true);
      wedge_bisecting_hes_pre.push_back(wedge_bisecting_he_pre);
      wedge_bisecting_hes_pro.push_back(wedge_bisecting_he_pro);
    }
    Face new_face = getNewFace();
    lower_faces.push_back(new_face);
    upper_faces.push_back(current_face);

    Halfedge new_pilar_he = getNewHalfedge(true);
    lower_pilar_hes.push_back(new_pilar_he);
    upper_pilar_hes.push_back(current_he);

    Edge bisecting_edge = getNewEdge();
    bisecting_edges.push_back(bisecting_edge);

    Halfedge upper_bisecting_he = getNewHalfedge(true), lower_bisecting_he = getNewHalfedge(true);
    upper_bisecting_hes.push_back(upper_bisecting_he);
    lower_bisecting_hes.push_back(lower_bisecting_he);

    if (sib_he == first_he) break;
    current_he = sib_he;
    sib_he = current_he.sibling();
    current_face = current_he.face();
    sib_face = sib_he.face();
    current_tet = get_connecting_tet(current_he.tipVertex(), current_he.tailVertex(), current_he.next().tipVertex(), sib_he.next().tipVertex());
  }
  
  // hook-up pointers

  Vertex v1 = e.firstVertex(), v2 = e.secondVertex();  // will select v2 as upper!
  
  // DEBUG
  printf("Spliting Edge %d v1, v2:(%d, %d):\n", e.getIndex(), v1.getIndex(), v2.getIndex());
  printf("New vertex: %d:\n", new_v.getIndex());
  
  // vertex -> he ; just for center
  vHalfedgeArr[new_v.getIndex()] = upper_pilar_hes[0].getIndex();
  vHeOutStartArr[new_v.getIndex()] = INVALID_IND; // will be handled later; TODO this should hold by default.
  vHeInStartArr[new_v.getIndex()] = INVALID_IND; // will be handled later; TODO this should hold by default.
  // edge -> halfedge ; just for the lower edge
  //   ** upper is already fine
  eHalfedgeArr[lower_pilar_edge.getIndex()] = lower_pilar_hes[0].getIndex();
  for (size_t i = 0; i < nSibs; i++) {
    Halfedge old_current_he = upper_pilar_hes[i],
             old_sib_he = upper_pilar_hes[(i + 1) % nSibs];
    
    // tet related stuff; and wedge faces
    bool wedge_condition = (!have_boundary || i < nSibs - 1);
    if (wedge_condition){
      // tet -> vertex
      tAdjVs[upper_tets[i].getIndex()] = {v2.getIndex(), new_v.getIndex(), old_current_he.next().tipVertex().getIndex(), old_sib_he.next().tipVertex().getIndex()};
      tAdjVs[lower_tets[i].getIndex()] = {v1.getIndex(), new_v.getIndex(), old_current_he.next().tipVertex().getIndex(), old_sib_he.next().tipVertex().getIndex()};
      // face -> tet
      //  ** upper faces are done already
      Face tmp_lower_face = lower_faces[i],
           tmp_next_lower_face = lower_faces[(i + 1) % nSibs];
      fAdjTs[tmp_lower_face.getIndex()].push_back(lower_tets[i].getIndex());
      fAdjTs[tmp_next_lower_face.getIndex()].push_back(lower_tets[i].getIndex());
      Face wedge_bisecting_face = wedge_bisecting_faces[i];
      fAdjTs[wedge_bisecting_face.getIndex()] = {upper_tets[i].getIndex(), lower_tets[i].getIndex()};
      // face -> halfedge ; wedge bisectors
      fHalfedgeArr[wedge_bisecting_face.getIndex()] = wedge_loop_hes[i].getIndex();
      // he -> edge ; wedge loop stuff
      heEdgeArr[wedge_loop_hes[i].getIndex()] = wedge_loop_edges[i].getIndex();
      heEdgeArr[wedge_bisecting_hes_pre[i].getIndex()] = bisecting_edges[i].getIndex();
      heEdgeArr[wedge_bisecting_hes_pro[i].getIndex()] = bisecting_edges[(i + 1) % nSibs].getIndex();
      // he -> face ; wedge faces
      heFaceArr[wedge_loop_hes[i].getIndex()] = wedge_bisecting_face.getIndex();
      heFaceArr[wedge_bisecting_hes_pre[i].getIndex()] = wedge_bisecting_face.getIndex();
      heFaceArr[wedge_bisecting_hes_pro[i].getIndex()] = wedge_bisecting_face.getIndex();
      // he -> vertex  &&  he -> next
      //  -- traversing the wedge bisecting face
      Vertex pre_v = old_current_he.next().tipVertex(),
             pro_v = old_sib_he.next().tipVertex();
      heVertexArr[wedge_loop_hes[i].getIndex()] = pre_v.getIndex();
      heNextArr[wedge_loop_hes[i].getIndex()] = wedge_bisecting_hes_pro[i].getIndex();
      heVertexArr[wedge_bisecting_hes_pro[i].getIndex()] = pro_v.getIndex();
      heNextArr[wedge_bisecting_hes_pro[i].getIndex()] = wedge_bisecting_hes_pre[i].getIndex();
      heVertexArr[wedge_bisecting_hes_pre[i].getIndex()] = new_v.getIndex();
      heNextArr[wedge_bisecting_hes_pre[i].getIndex()] = wedge_loop_hes[i].getIndex();
      
      // siblings ; for the wedge face
      heSiblingArr[lower_bisecting_hes[(i + 1) % nSibs].getIndex()] = wedge_bisecting_hes_pro[i].getIndex(); 
      heSiblingArr[wedge_bisecting_hes_pro[i].getIndex()] = upper_bisecting_hes[(i + 1) % nSibs].getIndex();
      if (i == nSibs - 2 && have_boundary) heSiblingArr[upper_bisecting_hes[(i + 1) % nSibs].getIndex()] = lower_bisecting_hes[(i + 1) % nSibs].getIndex();
      
      heSiblingArr[upper_bisecting_hes[i].getIndex()] = wedge_bisecting_hes_pre[i].getIndex(); 
      heSiblingArr[wedge_bisecting_hes_pre[i].getIndex()] = lower_bisecting_hes[i].getIndex();
      if (i == 0 && have_boundary) heSiblingArr[lower_bisecting_hes[(i + 1) % nSibs].getIndex()] = upper_bisecting_hes[(i + 1) % nSibs].getIndex();

      //     front  edge (wedge loop)
      Face upper_front_face = get_connecting_face(v2, pre_v, pro_v),
           lower_front_face = get_connecting_face(v1, pre_v, pro_v);
      if (lower_front_face.getIndex() == INVALID_IND || upper_front_face.getIndex() == INVALID_IND)
        throw std::logic_error("SplitEdge: lower and upper faces should exist!");
      Halfedge upper_front_he = get_he_of_edge_on_face(connectingEdge(pre_v, pro_v), upper_front_face),
               lower_front_he = get_he_of_edge_on_face(connectingEdge(pre_v, pro_v), lower_front_face);
      Halfedge he1, he2;
      if (siblings_are_ordered){
        if (upper_front_he.sibling() == lower_front_he) {
          he1 = upper_front_he;
          he2 = lower_front_he;
        }
        else if (lower_front_he.sibling() == upper_front_he){
          he2 = upper_front_he;
          he1 = lower_front_he;
        }
        else throw std::logic_error("SplitEdge: siblings are supposed to be ordered!");
      }
      else{
        he1 = upper_front_he;
        he2 = upper_front_he.sibling();
      }
      heSiblingArr[he1.getIndex()] = wedge_loop_hes[i].getIndex();
      heSiblingArr[wedge_loop_hes[i].getIndex()] = he2.getIndex();

      // vertex -> he In and Out loops
      addToVertexLists(wedge_loop_hes[i]);
      addToVertexLists(wedge_bisecting_hes_pre[i]);
      addToVertexLists(wedge_bisecting_hes_pro[i]);
    }
    // face -> halfedge ; pillars
    fHalfedgeArr[upper_faces[i].getIndex()] = upper_pilar_hes[i].getIndex();
    fHalfedgeArr[lower_faces[i].getIndex()] = lower_pilar_hes[i].getIndex();
    // edge -> halfedge ; bisectors
    eHalfedgeArr[bisecting_edges[i].getIndex()] = upper_bisecting_hes[i].getIndex();
    // he -> edge ; on-face stuff
    //  ** upper is fine
    heEdgeArr[lower_pilar_hes[i].getIndex()] = lower_pilar_edge.getIndex();
    heEdgeArr[upper_bisecting_hes[i].getIndex()] = bisecting_edges[i].getIndex();
    heEdgeArr[lower_bisecting_hes[i].getIndex()] = bisecting_edges[i].getIndex();
    // he -> face ; bisected sub-faces
    //  ** upper is fine
    heFaceArr[lower_pilar_hes[i].getIndex()] = lower_faces[i].getIndex();
    heFaceArr[upper_bisecting_hes[i].getIndex()] = upper_faces[i].getIndex();
    heFaceArr[lower_bisecting_hes[i].getIndex()] = lower_faces[i].getIndex();
    // cont..

    // he -> vertex  &&  he -> next
    //  -- pillars modification
    Halfedge next_he = upper_pilar_hes[i].next(),
             prev_he = upper_pilar_hes[i].next().next(),
             upper_he = upper_pilar_hes[i],
             lower_he = lower_pilar_hes[i];
    Vertex wing_v = next_he.tipVertex();
    if (upper_he.tipVertex() == v2){
      // upper face
      heVertexArr[upper_he.getIndex()] = new_v.getIndex();
      heVertexArr[upper_bisecting_hes[i].getIndex()] = wing_v.getIndex();
      heNextArr[next_he.getIndex()] = upper_bisecting_hes[i].getIndex();
      heNextArr[upper_bisecting_hes[i].getIndex()] = upper_he.getIndex();
      // lower face
      heVertexArr[lower_bisecting_hes[i].getIndex()] = new_v.getIndex();
      heVertexArr[lower_he.getIndex()] = v1.getIndex();
      heNextArr[prev_he.getIndex()] = lower_he.getIndex();
      heNextArr[lower_he.getIndex()] = lower_bisecting_hes[i].getIndex();
      heNextArr[lower_bisecting_hes[i].getIndex()] = prev_he.getIndex(); 

      // he -> face ; lower faces
      heFaceArr[prev_he.getIndex()] = lower_faces[i].getIndex();
    }
    else if (upper_he.tipVertex() == v1){
      // upper face
      heVertexArr[upper_bisecting_hes[i].getIndex()] = new_v.getIndex();
      heNextArr[upper_he.getIndex()] = upper_bisecting_hes[i].getIndex();
      heNextArr[upper_bisecting_hes[i].getIndex()] = prev_he.getIndex();
      // lower face
      heVertexArr[lower_he.getIndex()] = new_v.getIndex();
      heVertexArr[lower_bisecting_hes[i].getIndex()] = wing_v.getIndex();
      heNextArr[lower_he.getIndex()] = next_he.getIndex();
      heNextArr[next_he.getIndex()] = lower_bisecting_hes[i].getIndex();
      heNextArr[lower_bisecting_hes[i].getIndex()] = lower_he.getIndex();

      // he -> face ; lower faces
      heFaceArr[next_he.getIndex()] = lower_faces[i].getIndex();
    }
    else throw std::logic_error("upper edge should have either v1 or v2 on the tip!");

    // siblings
    //  ** upper should be fine
    heSiblingArr[lower_pilar_hes[i].getIndex()] = lower_pilar_hes[(i+1)%nSibs].getIndex();

    // vertex -> he In and Out loops
    addToVertexLists(upper_pilar_hes[i]); // will result in repetetive hes in v.hes(); since the tip/tail is already handled and we doing this here just for the new vertex
    addToVertexLists(lower_pilar_hes[i]);
    addToVertexLists(upper_bisecting_hes[i]);
    addToVertexLists(lower_bisecting_hes[i]);

    // DEBUG!
    printf("current pillar hes\n   upper %d (%d, %d)\n   lower %d (%d, %d)\n", 
           upper_pilar_hes[i].getIndex(), upper_pilar_hes[i].tailVertex().getIndex(), upper_pilar_hes[i].tipVertex().getIndex(),
           lower_pilar_hes[i].getIndex(), lower_pilar_hes[i].tailVertex().getIndex(), lower_pilar_hes[i].tipVertex().getIndex());
    printf("current bisectors\n");
    printf("   low bisec he\n    %d (%d, %d)  edge %d\n", 
           lower_bisecting_hes[i].getIndex(), lower_bisecting_hes[i].tailVertex().getIndex(), lower_bisecting_hes[i].tipVertex().getIndex(), lower_bisecting_hes[i].edge().getIndex());
    printf("   upe bisec he\n    %d (%d, %d)  edge %d\n", 
           upper_bisecting_hes[i].getIndex(), upper_bisecting_hes[i].tailVertex().getIndex(), upper_bisecting_hes[i].tipVertex().getIndex(), upper_bisecting_hes[i].edge().getIndex());
  }
  printf("lower pillar edge %d, he %d (%d, %d)\n", 
          lower_pilar_edge.getIndex(), lower_pilar_edge.halfedge().getIndex(),
          lower_pilar_edge.firstVertex().getIndex(), lower_pilar_edge.secondVertex().getIndex());
  printf("upper pillar edge %d, he %d (%d, %d)\n", 
          upper_pilar_edge.getIndex(), upper_pilar_edge.halfedge().getIndex(),
          upper_pilar_edge.firstVertex().getIndex(), upper_pilar_edge.secondVertex().getIndex());
  
  // orientation of he's ; all
  for (size_t i = 0; i < nSibs; i++) {
    //   ** upper remains ok
    heOrientArr[lower_pilar_hes[i].getIndex()] = (lower_pilar_hes[i].vertex() == lower_pilar_edge.firstVertex());
    bool wedge_condition = (!have_boundary || i < nSibs - 1);
    if (wedge_condition){
      // orientation of he's ; wedge bisecting face
      heOrientArr[wedge_loop_hes[i].getIndex()] = (wedge_loop_hes[i].vertex() == wedge_loop_edges[i].firstVertex());
      heOrientArr[wedge_bisecting_hes_pre[i].getIndex()] = (wedge_bisecting_hes_pre[i].vertex() == bisecting_edges[i].firstVertex());
      heOrientArr[wedge_bisecting_hes_pro[i].getIndex()] = (wedge_bisecting_hes_pro[i].vertex() == bisecting_edges[(i + 1) % nSibs].firstVertex());
    }
  }

  return new_v;
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
Face TetMesh::getNewFace(){
  Face new_face = SurfaceMesh::getNewFace();
  fAdjTs.resize(nFacesCount, {});
  return new_face;
}

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
  
  // face -> tet
  // fAdjTs.resize(nFacesFillCount + 6);
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
  fAdjTs[f012.getIndex()] = {t0123.getIndex(), t0124.getIndex()};
  fAdjTs[f013.getIndex()] = {t0123.getIndex(), t0134.getIndex()};
  fAdjTs[f014.getIndex()] = {t0124.getIndex(), t0134.getIndex()};
  fAdjTs[f023.getIndex()] = {t0123.getIndex(), t0234.getIndex()};
  fAdjTs[f024.getIndex()] = {t0124.getIndex(), t0234.getIndex()};
  fAdjTs[f034.getIndex()] = {t0134.getIndex(), t0234.getIndex()};
  
  // tet -> vertex
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
  
  heSiblingArr[he01_012.getIndex()] = he01_013.getIndex(); heSiblingArr[he01_013.getIndex()] = he01_014.getIndex(); heSiblingArr[he01_014.getIndex()] = he01_012.getIndex(); 
  heSiblingArr[he02_021.getIndex()] = he02_023.getIndex(); heSiblingArr[he02_023.getIndex()] = he02_024.getIndex(); heSiblingArr[he02_024.getIndex()] = he02_021.getIndex();
  heSiblingArr[he03_031.getIndex()] = he03_032.getIndex(); heSiblingArr[he03_032.getIndex()] = he03_034.getIndex(); heSiblingArr[he03_034.getIndex()] = he03_031.getIndex();
  heSiblingArr[he04_041.getIndex()] = he04_042.getIndex(); heSiblingArr[he04_042.getIndex()] = he04_043.getIndex(); heSiblingArr[he04_043.getIndex()] = he04_041.getIndex();
  //    - inner he's sibling rels
  //      ** I'll try to respect the sibling order of older halfEdges
  
  Halfedge first_he, second_he;
  //    12
  Halfedge he12_f123 = get_he_of_edge_on_face(bE12, f123),
           he12_f124 = get_he_of_edge_on_face(bE12, f124);
  if     (he12_f123.sibling() == he12_f124) {  first_he = he12_f123;  second_he = he12_f124;}
  else if(he12_f124.sibling() == he12_f123) {  first_he = he12_f124;  second_he = he12_f123;}
  else {
    if (siblings_are_ordered) throw std::logic_error(" --------- adj faces on a tet should be siblings ----------");
    else{
      first_he = he12_f123; // random choice; order doesn't matter
      second_he = first_he.sibling(); 
    }
  }
  heSiblingArr[first_he.getIndex()] = bhe12.getIndex();
  heSiblingArr[bhe12.getIndex()] = second_he.getIndex();
  //    13
  Halfedge he13_f132 = get_he_of_edge_on_face(bE13, f123),
           he13_f134 = get_he_of_edge_on_face(bE13, f134);
  if     (he13_f132.sibling() == he13_f134) {  first_he = he13_f132;  second_he = he13_f134;}
  else if(he13_f134.sibling() == he13_f132) {  first_he = he13_f134;  second_he = he13_f132;}
  else{
    if (siblings_are_ordered) throw std::logic_error(" --------- adj faces on a tet should be siblings ----------");
    else{
      first_he = he13_f132; // random choice; order doesn't matter
      second_he = first_he.sibling(); 
    }
  }
  heSiblingArr[first_he.getIndex()] = bhe13.getIndex();
  heSiblingArr[bhe13.getIndex()] = second_he.getIndex();
  //    14
  Halfedge he14_f142 = get_he_of_edge_on_face(bE14, f124),
           he14_f143 = get_he_of_edge_on_face(bE14, f134);
  if     (he14_f142.sibling() == he14_f143) {  first_he = he14_f142;  second_he = he14_f143;}
  else if(he14_f143.sibling() == he14_f142) {  first_he = he14_f143;  second_he = he14_f142;}
  else{
    if (siblings_are_ordered) throw std::logic_error(" --------- adj faces on a tet should be siblings ----------");
    else{
      first_he = he14_f142; // random choice; order doesn't matter
      second_he = first_he.sibling(); 
    }
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
    if (siblings_are_ordered) throw std::logic_error(" --------- adj faces on a tet should be siblings ----------");
    else{
      first_he = he23_f231; // random choice; order doesn't matter
      second_he = first_he.sibling(); 
    }
  }
  heSiblingArr[first_he.getIndex()] = bhe23.getIndex();
  heSiblingArr[bhe23.getIndex()] = second_he.getIndex();
  //    24
  Halfedge he24_f241 = get_he_of_edge_on_face(bE24, f124),
           he24_f243 = get_he_of_edge_on_face(bE24, f234);
  if     (he24_f241.sibling() == he24_f243) {  first_he = he24_f241;  second_he = he24_f243;}
  else if(he24_f243.sibling() == he24_f241) {  first_he = he24_f243;  second_he = he24_f241;}
  else{
    if (siblings_are_ordered) throw std::logic_error(" --------- adj faces on a tet should be siblings ----------");
    else{
      first_he = he24_f241; // random choice; order doesn't matter
      second_he = first_he.sibling(); 
    }
  }
  heSiblingArr[first_he.getIndex()] = bhe24.getIndex();
  heSiblingArr[bhe24.getIndex()] = second_he.getIndex();
  //    34
  Halfedge he34_f341 = get_he_of_edge_on_face(bE34, f134),
           he34_f342 = get_he_of_edge_on_face(bE34, f234);
  if     (he34_f341.sibling() == he34_f342) {  first_he = he34_f341;  second_he = he34_f342;}
  else if(he34_f342.sibling() == he34_f341) {  first_he = he34_f342;  second_he = he34_f341;}
  else{
    if (siblings_are_ordered) throw std::logic_error(" --------- adj faces on a tet should be siblings ----------");
    else{
      first_he = he34_f341; // random choice; order doesn't matter
      second_he = first_he.sibling(); 
    }
  }
  heSiblingArr[first_he.getIndex()] = bhe34.getIndex();
  heSiblingArr[bhe34.getIndex()] = second_he.getIndex();
  

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
    if (f.adjacentTets().size() > 2) throw std::logic_error("validateConnectivity: Face has more than 2 tets!");
    for(Tet t : f.adjacentTets()){
      for(Face tf: t.adjFaces()){
        if(tf == f) found_it = true;
      }
    }
    if(!found_it) throw std::logic_error("face.tet did not have face in tet.faces!");
  }

  if (siblings_are_ordered){
    for (Tet t: tets()){
      for (Face f1: t.adjFaces()){
        for (Face f2: t.adjFaces()){
          if (f1 < f2){
            Edge common_edge = common_edge_of_faces(f1, f2);
            Halfedge he1 = get_he_of_edge_on_face(common_edge, f1),
                     he2 = get_he_of_edge_on_face(common_edge, f2);
            if (he1.sibling() != he2 && he2.sibling() != he1){
              printf("face %d and %d are supposed to be siblings on edge %d\n", f1.getIndex(), f2.getIndex(), common_edge.getIndex());
              throw std::logic_error("Siblings are not ordered! :(\n");
            }
          }
        }
      }
    }
  }
}

} // namespace volume
} // namespace 