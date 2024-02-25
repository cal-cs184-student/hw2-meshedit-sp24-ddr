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
        HalfedgeCIter hTwin = h->twin();
        HalfedgeCIter hNext = h->next();
        HalfedgeCIter hNext2 = hNext->next();
        /*cout << "h isBound() " << h->isBoundary() << endl;
        cout << "hTwin isBound() " << hTwin->isBoundary() << endl;*/
        cout << "Halfedge address" << &h << endl;
        /*cout << "Halfedge Twin address" << &hTwin << endl;
        cout << "Halfedge Next address" << &hNext << endl;
        cout << "Halfedge Next2 address" << &hNext2 << endl;*/
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

        h = hTwin->next();
        cout << "Halfedge update address" << &h << endl;
    } while (h != startingH);

    // Sum the normal vectors and normalize
    Vector3D sumVec = Vector3D(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < vectors.size(); i++) {
        sumVec += vectors[i];
    }
    /*cout << sumVec / vectors.size() << endl;*/
    return sumVec / vectors.size();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    return EdgeIter();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    return VertexIter();
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
