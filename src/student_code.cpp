#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // Part 1 --Before Part 2 
    std::vector<Vector2D> res = std::vector<Vector2D>();
    for (int i = 0; i < points.size() - 1; i++) {
        res.push_back(((1 - t) * points[i]) + (t * points[i + 1]));
    }
    return res;
  }

  /**
   * Evaluates one Sstep of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
   
    std::vector<Vector3D> res = std::vector<Vector3D>();
    for (int i = 0; i < points.size() - 1; i++) {
        res.push_back(((1 - t) * points[i]) + (t * points[i + 1]));
    }
    return res;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {

    // TODO Part 2.
      std::vector<Vector3D> res = std::vector<Vector3D>();
      res = points;
      for (int i = 0; i < points.size() - 1; i++) {
          res = evaluateStep(res, t);
      }
      return res[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    std::vector<Vector3D> intermediatePoints = std::vector<Vector3D>();

    for (int i = 0; i < controlPoints.size(); i++) {
		intermediatePoints.push_back(evaluate1D(controlPoints[i], u));
    }
    return evaluate1D(intermediatePoints, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    // Calculate the vectors of the face
    HalfedgeCIter h = Vertex::halfedge();
    HalfedgeCIter startingH = h;
    std::vector<Vector3D> vectors = std::vector<Vector3D>();
    do {
        HalfedgeCIter hNext = h->next();
        HalfedgeCIter hNext2 = hNext->next();
        double x0 = hNext->vertex()->position[0] - this->position[0];
        double y0 = hNext->vertex()->position[1] - this->position[1];
        double z0 = hNext->vertex()->position[2] - this->position[2];
        Vector3D vec0 = Vector3D(x0, y0, z0);

        double x1 = hNext2->vertex()->position[0] - this->position[0];
        double y1 = hNext2->vertex()->position[1] - this->position[1];
        double z1 = hNext2->vertex()->position[2] - this->position[2];
        Vector3D vec1 = Vector3D(x1, y1, z1);

        Vector3D normVec = cross(vec0, vec1) * 0.5f;
        vectors.push_back(normVec);

        h = h->twin()->next();
    } while (h != startingH);

    // Sum the normal vectors and normalize
    Vector3D sumVec = Vector3D(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < vectors.size(); i++) {
        sumVec += vectors[i];
    }
    return sumVec / sumVec.norm();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    
    // Check for boundary loops
    if (e0->halfedge()->face()->isBoundary() || e0->halfedge()->twin()->face()->isBoundary()) {
        return EdgeIter();
    }

    /* BEFORE FLIP */

    // Halfedges:

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    // Vertices:

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    // Edges:

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    // Faces:

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    /*-----AFTER FLIP----*/

    // Halfedges:

    h0->setNeighbors(h1, h3, v3, e0, f0);
    h1->setNeighbors(h2, h7, v2, e2, f0);
    h2->setNeighbors(h0, h8, v0, e3, f0);
    h3->setNeighbors(h4, h0, v2, e0, f1);
    h4->setNeighbors(h5, h9, v3, e4, f1);
    h5->setNeighbors(h3, h6, v1, e1, f1);

    h6->twin() = h5;
    h7->twin() = h1;
    h8->twin() = h2;
    h9->twin() = h4;


    // Vertices:

    v0->halfedge() = h2;
    v1->halfedge() = h5;
    v2->halfedge() = h3;
    v3->halfedge() = h0;

    // Edges:
    e0->halfedge() = h0;
    e1->halfedge() = h5;
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h4;


    // Faces:
    f0->halfedge() = h0;
    f1->halfedge() = h3;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    // Check for boundary loops
    if (e0->halfedge()->face()->isBoundary() || e0->halfedge()->twin()->face()->isBoundary()) {
        return VertexIter();
    }

	/* BEFORE FLIP */
     
    // Halfedges:
     
    HalfedgeIter h0 = e0->halfedge();
	HalfedgeIter h1 = h0->next();
	HalfedgeIter h2 = h1->next();
	HalfedgeIter h3 = h0->twin();
	HalfedgeIter h4 = h3->next();
	HalfedgeIter h5 = h4->next();
	HalfedgeIter h6 = h1->twin();
	HalfedgeIter h7 = h2->twin();
	HalfedgeIter h8 = h4->twin();
	HalfedgeIter h9 = h5->twin();

	// Vertices:

	VertexIter v0 = h0->vertex();
	VertexIter v1 = h3->vertex();
	VertexIter v2 = h2->vertex();
	VertexIter v3 = h5->vertex();

	// Edges:

	EdgeIter e1 = h1->edge();
	EdgeIter e2 = h2->edge();
	EdgeIter e3 = h4->edge();
	EdgeIter e4 = h5->edge();

	// Faces:

	FaceIter f0 = h0->face();
	FaceIter f1 = h3->face();

	/*-----AFTER SPLIT----*/

    HalfedgeIter h10 = newHalfedge();
	h10->setNeighbors(h1, h1, v1, e1, f1);
	HalfedgeIter h11 = newHalfedge();
	h11->setNeighbors(h1, h1, v1, e1, f1);
	HalfedgeIter h12 = newHalfedge();
	h12->setNeighbors(h1, h1, v1, e1, f1);
	HalfedgeIter h13 = newHalfedge();
	h13->setNeighbors(h1, h1, v1, e1, f1);
    HalfedgeIter h14 = newHalfedge();
	h14->setNeighbors(h1, h1, v1, e1, f1);
    HalfedgeIter h15 = newHalfedge();
	h15->setNeighbors(h1, h1, v1, e1, f1);

    VertexIter v4 = newVertex();
    v4->position = (v1->position + v0->position) / 2.0f;
    v4->halfedge() = h0;

    EdgeIter e5 = newEdge();
    e5->halfedge() = h11;
    EdgeIter e6 = newEdge();
    e6->halfedge() = h2;
    EdgeIter e7 = newEdge();
    e7->halfedge() = h4;

    FaceIter f2 = newFace();
    f2->halfedge() = h3;
    FaceIter f3 = newFace();
    f3->halfedge() = h10;

    h10->setNeighbors(h11, h4, v3, e7, f3);
    h11->setNeighbors(h12, h15, v4, e5, f3);
	h12->setNeighbors(h10, h8, v0, e3, f3);
    h13->setNeighbors(h14, h2, v4, e6, f1);
	h14->setNeighbors(h15, h7, v2, e2, f1);
	h15->setNeighbors(h13, h11, v0, e5, f1);


 //   /* Reassign pointers */
 
 //   // Halfedges:

    h0->setNeighbors(h0, h3, v4, e0, f0);
    h1->setNeighbors(h2, h6, v1, e1, f0);
	h2->setNeighbors(h0, h13, v2, e6, f0);
	h3->setNeighbors(h4, h0, v1, e0, f2);
	h4->setNeighbors(h5, h10, v4, e7, f2);
	h5->setNeighbors(h3, h9, v3, e4, f2);
	h6->setNeighbors(h6->next(), h1, h6->vertex(), h6->edge(), h6->face());
	h7->setNeighbors(h7->next(), h14, h7->vertex(), h7->edge(), h7->face());
	h8->setNeighbors(h8->next(), h12, h8->vertex(), h8->edge(), h8->face());
	h9->setNeighbors(h9->next(), h5, h9->vertex(), h9->edge(), h9->face());

 //   // Vertices:

    v0->halfedge() = h15;
    v1->halfedge() = h3;
    v2->halfedge() = h2;
    v3->halfedge() = h10;
    v4->halfedge() = h0;

 //   // Edges:

    e0->halfedge() = h0;
	e1->halfedge() = h1;
    e2->halfedge() = h14;
    e3->halfedge() = h12;
    e4->halfedge() = h5;
    e5->halfedge() = h15;
    e6->halfedge() = h2;
    e7->halfedge() = h4;

 //   // Faces:

    f0->halfedge() = h0;
    f1->halfedge() = h13;
	f2->halfedge() = h3;
    f3->halfedge() = h10;

    check_for(v0);
    check_for(v1);
	check_for(v2);
    check_for(v3);
    check_for(v4);

	check_for(f0);
	check_for(f1);
	check_for(f2);
	check_for(f3);

    return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

  }
}
