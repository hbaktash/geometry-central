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
      :SurfaceMesh(triangles_), tet_v_inds(tet_v_inds_) {
  // The nonmanifold surface skeleton is already constructed.
  // Faces, vertices, halfedges and edges are initiated.
  size_t n_tets = tet_v_inds_.size();
  nTetsCount = n_tets;
  nTetsCapacityCount = nTetsCount;
  
  tet_objects.reserve(n_tets);
  
  tAdjVs = std::vector<std::vector<size_t>>(n_tets, {INVALID_IND});
  fAdjTs = std::vector<std::vector<size_t>>(nFaces(), {INVALID_IND});

  size_t tet_ind = 0;
  for(std::vector<size_t> tet_v_inds: tet_v_inds_){
    
    Tet new_tet(this, tet_ind);
    //populating the lazy/dirty iterators on Tets
    new_tet.buildAdjVertices(tet_v_inds);
    // populating the lazy/dirty to-Tet iterators on Faces
    for(size_t v_ind:tet_v_inds){
      // --Faces

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

  for(Tet t1: f1.adjTets){
    for(Tet t2: f2.adjTets){
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


} // namespace volume
} // namespace 