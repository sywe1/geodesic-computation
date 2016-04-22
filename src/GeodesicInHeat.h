#pragma once

#include<OpenGP/SurfaceMesh/SurfaceMesh.h>
#include<Eigen/Sparse>

using namespace OpenGP;

class GeodesicInHeat
{
	typedef SurfaceMesh::Vertex Vertex;
	typedef SurfaceMesh::Face Face;
	typedef SurfaceMesh::Halfedge Halfedge;
	typedef Eigen::Triplet<Scalar> Triplet;
	typedef Eigen::SparseMatrix<Scalar> Sparse;
private:SurfaceMesh& mesh;
		Eigen::SparseMatrix<Scalar> Laplacian;
		Eigen::SparseMatrix<Scalar> LaplacianDirichlet;
		Eigen::SparseMatrix<Scalar> Grad_x;
		Eigen::SparseMatrix<Scalar> Grad_y;
		Eigen::SparseMatrix<Scalar> Grad_z;
		Eigen::SparseMatrix<Scalar> Div_x;
		Eigen::SparseMatrix<Scalar> Div_y;
		Eigen::SparseMatrix<Scalar> Div_z;
		Eigen::SparseMatrix<Scalar> AreaMatrix;
		Eigen::SparseMatrix<Scalar> InvertAreaMatrix;
		SurfaceMesh::Vertex_property<float> HeatDisplay;
		SurfaceMesh::Vertex_property<float> HeatDistance;
		bool initialized = false;
		float avglength;
		float factor = 100;
public:	float MaxGeodesic;
		float MinGeodesic;
		float MaxEdgeSpan;
		bool Dirichlet = false;  //display Dirichlet?
		bool Neumann = false; //display Neumann?

public:GeodesicInHeat(SurfaceMesh& mesh) ;

	   //initialization construct operator;
	   void initialization();

	   void clean_up();

	   void compute_distance(const Vertex& start_vertex);

	   std::vector<std::pair<Vec3, Vec3>> draw_isolines();

	   std::vector<std::pair<Vec3, Vec3>> draw_path(const Vertex& start,const Vertex& end);

	   void set_factor(float m) { 
		   factor = m;
		   std::cout<< "M: " <<factor;
	   };

	   float Geodesic(const Vertex& v) { return HeatDistance[v];};

private: VecN compute_heat(const VecN& delta, const Sparse& LapMatrix, const Sparse& areamatrix, float t);

		 VecN compute_divergence(const Sparse& div_x, const Sparse& div_y, const Sparse& div_z, const MatMxN& X);

		 MatMxN compute_gradient(const Sparse& grad_x, const Sparse& grad_y, const Sparse& grad_z, const VecN& u);
};