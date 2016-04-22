#include "GeodesicInHeat.h"
#include<cmath>
#include<OpenGP/MLogger.h>


GeodesicInHeat::GeodesicInHeat(SurfaceMesh& mesh) :mesh(mesh)
{
	HeatDisplay = mesh.vertex_property<float>("v:quality");
	HeatDistance = mesh.vertex_property<float>("v:heat distance");
	MaxGeodesic=0.0;
	MinGeodesic = 9999;
	MaxEdgeSpan=0.0;
	avglength = 0.0;
}



void GeodesicInHeat::initialization()
{
	int n = mesh.n_vertices();
	int f = mesh.n_faces();
	int count = 0;
	Laplacian.resize(n, n);
	LaplacianDirichlet.resize(n, n);
	Grad_x.resize(f, n);
	Grad_y.resize(f, n);
	Grad_z.resize(f, n);
	Div_x.resize(n, f);
	Div_y.resize(n, f);
	Div_z.resize(n, f);
	AreaMatrix.resize(n, n);

	std::vector<Triplet> Laplist;
	std::vector<Triplet> LapDList;
	std::vector<Triplet> Arealist;
	std::vector<Triplet> Grad_x_list;
	std::vector<Triplet> Grad_y_list;
	std::vector<Triplet> Grad_z_list;
	std::vector<Triplet> Div_x_list;
	std::vector<Triplet> Div_y_list;
	std::vector<Triplet> Div_z_list;

	/*              v_k (cot_theta_1)
					/\
		div_edge_2 /  \   <----triangle
				  /____\
				v_i     v_j (cot_theta_2)
				 grad_edge	
				 div_edge_1
	
	*/

	Vertex v_j, v_k, v_alpha, v_beta;
	Halfedge alpha_edge, beta_edge;
	Face triangle;
	Point p_i, p_j, p_k, p_alpha, face_normal, grad_edge, grad_cross, div_edge_1, div_edge_2, cot_edge_1, cot_edge_2, grad_ele;
	Scalar cot_alpha, cot_beta, cot_theta_1, cot_theta_2, div_sum, tri_area,cots,epsilon=1e-10;


	
	for (auto const& v_i : mesh.vertices())
	{
		Scalar Area = 0.0;
		Scalar CotSum = 0.0;
		for (auto const& edge : mesh.halfedges(v_i))
		{
				Halfedge edge_oppo = mesh.opposite_halfedge(edge);
				v_j = mesh.to_vertex(edge);
				avglength += (mesh.position(v_j) - mesh.position(v_i)).norm();
				++count;
				if (mesh.face(edge_oppo).is_valid())
					alpha_edge = mesh.next_halfedge(mesh.opposite_halfedge(edge));
				else
					alpha_edge = mesh.next_halfedge(edge);

				if (mesh.face(edge).is_valid())
					beta_edge = mesh.next_halfedge(edge);
				else
					beta_edge = mesh.next_halfedge(mesh.opposite_halfedge(edge));

				// v_beta = v_k
				v_k = mesh.to_vertex(beta_edge);
				v_alpha = mesh.to_vertex(alpha_edge);


				p_i = mesh.position(v_i);
				p_j = mesh.position(v_j);
				p_k = mesh.position(v_k);
				p_alpha = mesh.position(v_alpha);

		//laplacian operator
				//compute cot alpha and cot beta and add to the list
				//beta
				cot_edge_1 = (p_i - p_k);
				cot_edge_2 = (p_j - p_k);
				cot_beta = cot_edge_1.dot(cot_edge_2) / cot_edge_1.cross(cot_edge_2).norm();
				//alpha
				cot_edge_1 = (p_i - p_alpha);
				cot_edge_2 = (p_j - p_alpha);
				cot_alpha = cot_edge_1.dot(cot_edge_2) / cot_edge_1.cross(cot_edge_2).norm();



				if (!mesh.face(edge).is_valid())
					cot_beta = 0;
				if (!mesh.face(edge_oppo).is_valid())
					cot_alpha = 0;

				
				cots = cot_beta + cot_alpha + epsilon;

				//Dirichlet condition, if v_j is boundary, then set it zero, do not push in the Matrix.
				if (!mesh.is_boundary(v_j))
					LapDList.push_back(Triplet(v_i.idx(), v_j.idx(), 0.5*cots));

				Laplist.push_back(Triplet(v_i.idx(), v_j.idx(), 0.5*cots));
				
				CotSum += 0.5*cots;


				//Divergence operator
				if (mesh.face(edge).is_valid())
				{
					triangle = mesh.face(edge);

					div_edge_1 = p_j - p_i;
					div_edge_2 = p_k - p_i;

					tri_area = 0.5*(div_edge_1.cross(div_edge_2)).norm();

					Area += (1 / 3.0f)*tri_area;

					face_normal = (div_edge_1.cross(div_edge_2)).normalized();

					//compute cot theta1
					cot_edge_1 = (p_j - p_k);
					cot_edge_2 = (p_i - p_k);
					cot_theta_1 = cot_edge_1.dot(cot_edge_2) / cot_edge_1.cross(cot_edge_2).norm();

					//compute cot theta2
					cot_edge_1 = (p_k - p_j);
					cot_edge_2 = (p_i - p_j);
					cot_theta_2 = cot_edge_1.dot(cot_edge_2) / cot_edge_1.cross(cot_edge_2).norm();

					//add to x ,y ,z list respectively
					div_sum = cot_theta_1*div_edge_1(0) + cot_theta_2*div_edge_2(0);
					Div_x_list.push_back(Triplet(v_i.idx(), triangle.idx(), (0.5*div_sum)));
					div_sum = cot_theta_1*div_edge_1(1) + cot_theta_2*div_edge_2(1);
					Div_y_list.push_back(Triplet(v_i.idx(), triangle.idx(), (0.5*div_sum)));
					div_sum = cot_theta_1*div_edge_1(2) + cot_theta_2*div_edge_2(2);
					Div_z_list.push_back(Triplet(v_i.idx(), triangle.idx(), (0.5*div_sum)));

			//construct Gradient operator on triangles in x , y ,z separatly

					grad_edge = p_j - p_i;
					grad_cross = face_normal.cross(grad_edge);

					grad_ele = (1 / (2 * tri_area))*grad_cross;

					Grad_x_list.push_back(Triplet(triangle.idx(), v_k.idx(), grad_ele(0)));
					Grad_y_list.push_back(Triplet(triangle.idx(), v_k.idx(), grad_ele(1)));
					Grad_z_list.push_back(Triplet(triangle.idx(), v_k.idx(), grad_ele(2)));
				}
		}

		if (!mesh.is_boundary(v_i))
			LapDList.push_back(Triplet(v_i.idx(), v_i.idx(), -CotSum));


		Arealist.push_back(Triplet(v_i.idx(), v_i.idx(), Area));
		Laplist.push_back(Triplet(v_i.idx(), v_i.idx(),-CotSum));
	}
	
	AreaMatrix.setFromTriplets(Arealist.begin(), Arealist.end());
	LaplacianDirichlet.setFromTriplets(LapDList.begin(), LapDList.end());
	Laplacian.setFromTriplets(Laplist.begin(), Laplist.end());
	Grad_x.setFromTriplets(Grad_x_list.begin(), Grad_x_list.end());
	Grad_y.setFromTriplets(Grad_y_list.begin(), Grad_y_list.end());
	Grad_z.setFromTriplets(Grad_z_list.begin(), Grad_z_list.end());
	Div_x.setFromTriplets(Div_x_list.begin(), Div_x_list.end());
	Div_y.setFromTriplets(Div_y_list.begin(), Div_y_list.end());
	Div_z.setFromTriplets(Div_z_list.begin(), Div_z_list.end());
	avglength /=float(count);
	initialized = true;
}


VecN GeodesicInHeat::compute_heat(const VecN& delta,const Sparse& LapMatrix,const Sparse& areamatrix, float t)
{
	Sparse A = areamatrix - t*LapMatrix;
	Eigen::SimplicialCholesky<Sparse, Eigen::RowMajor> solver;
	solver.compute(A);
	VecN u = solver.solve(delta);
	return u;
}

VecN GeodesicInHeat::compute_divergence(const Sparse& div_x, const Sparse& div_y, const Sparse& div_z, const MatMxN& X)
{
	//Sum x,y,z up to get  Div(X) = 0.5*sum(cottheta1*e1*Xj+cottheta2*e2*Xj)
	VecN divergence_X = div_x*(X.col(0));
	divergence_X += div_y*(X.col(1));
	divergence_X += div_z*(X.col(2));
	return divergence_X;
}

MatMxN GeodesicInHeat::compute_gradient(const Sparse& grad_x, const Sparse& grad_y, const Sparse& grad_z, const VecN& u)
{
	//compute gradient on surface in x,y,z separatly
	int m=grad_x.rows();
	VecN gradient_x = grad_x*u;
	VecN gradient_y = grad_y*u;
	VecN gradient_z = grad_z*u;
	MatMxN X(m, 3);
	X.col(0) = gradient_x;
	X.col(1) = gradient_y;
	X.col(2) = gradient_z;
	for (int i = 0; i != m; ++i)
	{
		X.row(i) =-X.row(i).normalized();
	}
	return X;
}


void GeodesicInHeat::compute_distance(const Vertex& start_vertex)
{
	int n = mesh.n_vertices();
	int start_idx = start_vertex.idx();
	VecN delta(n);
	delta.setZero();
	delta(start_idx) =1.0f;

	//The laplacian, gradient divergence matrix only need to compute once.
	if (!initialized)
		initialization();
	float timestep = avglength*avglength*factor;
	VecN uNeumann = compute_heat(delta,Laplacian, AreaMatrix,timestep);
	VecN uDirichlet = compute_heat(delta, LaplacianDirichlet, AreaMatrix, timestep);
	VecN u =0.5*uNeumann+0.5*uDirichlet;

	//Method::the three boundary function will active one, if all false, display the average.
	if (Neumann)
		u = uNeumann;
	if (Dirichlet)
		u = uDirichlet;
	MatMxN X = compute_gradient(Grad_x, Grad_y, Grad_z, u);
	VecN div_X = compute_divergence(Div_x, Div_y, Div_z, X);

	Eigen::SimplicialCholesky<Sparse, Eigen::RowMajor> Solver;
	Solver.compute(Laplacian);
	VecN phi = Solver.solve(div_X);	
	
	//shift phi and find out the Max G, Min G and the Max geodesic distance span along edge.
	for (auto const& vertex : mesh.vertices())
	{
		float d1 = phi[vertex.idx()];
		HeatDisplay[vertex] = d1;
		float d2 = 0.0;
		for (auto const& vj : mesh.vertices(vertex))
		{
			float d = std::abs(phi[vertex.idx()] - phi[vj.idx()]);
			if (d > d2)
				d2 = d;
		}
		if (d1 > MaxGeodesic)
			MaxGeodesic = d1;
		if (d2 > MaxEdgeSpan)
			MaxEdgeSpan = d2;
		if (d1 < MinGeodesic)
			MinGeodesic = d1;
	}
	
	for (auto const& vertex : mesh.vertices())
	{
		HeatDisplay[vertex] = HeatDisplay[vertex] - MinGeodesic;
		HeatDistance[vertex] = HeatDisplay[vertex];
	}
	MaxGeodesic -= MinGeodesic;
	MinGeodesic = 0;

	if (Neumann) {
		mLogger() << "Heat: Neumann Boudary Condition, M=" << factor;
		return;
	}
	if (Dirichlet) {
		mLogger() << "Heat: Dirichlet Boudary Condition, M=" << factor;
		return;
	}
	mLogger() << "Heat: Average Boudary Condition, M=" << factor;
}


void GeodesicInHeat::clean_up()
{
	for (auto const& vertex : mesh.vertices())
	{
		HeatDisplay[vertex] = 0;
		HeatDistance[vertex] = 0;
	}
	MinGeodesic = 999;
	MaxGeodesic = -999;
}

std::vector<std::pair<Vec3, Vec3>> GeodesicInHeat::draw_isolines()
{
	//make sure this is no more than one line cross the triangle (step>MaxEdgeSpan)
	int n_iso = (MaxGeodesic-MinGeodesic) / MaxEdgeSpan;
	float step = (MaxGeodesic - MinGeodesic) / float(n_iso);
	SurfaceMesh::Vertex v1, v2;
	std::vector<std::pair<Vec3, Vec3>> Lines;
	for (auto const& face : mesh.faces())
	{
		std::vector<Vec3> points;
		for (auto const& edge : mesh.halfedges(face))
		{
			v1 = mesh.from_vertex(edge);
			v2 = mesh.to_vertex(edge);
			int n1 = HeatDistance[v1] / step;
			int n2 = HeatDistance[v2] / step;
			if (n1!=n2)
			{
				int n = n1 < n2 ? n1 : n2;
					++n;
				float lamda = (n*step - HeatDistance[v1]) / (HeatDistance[v2] - HeatDistance[v1]);
				Vec3 p = (1 - lamda)*mesh.position(v1) + lamda*mesh.position(v2);
				points.push_back(p);
			}
		}
		assert(points.size() <= 2);
		if (points.size() == 1)
		{
			Lines.push_back(std::pair<Vec3, Vec3>(points[0], points[0]));
		}
		if (points.size() == 2)
		{
			Lines.push_back(std::pair<Vec3, Vec3>(points[0], points[1]));
		}
	}
	return Lines;
}

std::vector<std::pair<Vec3, Vec3>> GeodesicInHeat::draw_path(const Vertex& start, const Vertex& end)
{
	Vertex v_local = end, v_next=end;
	std::vector<std::pair<Vec3, Vec3>> Lines;
	Lines.push_back(std::pair<Vec3, Vec3>(mesh.position(v_local), mesh.position(v_next)));
	//from the end point always find the smaller Geodesic values until it reach the start point.
	while (v_next != start)
	{
		float Min = 999;
		for (auto const& e : mesh.halfedges(v_local))
		{
			Vertex v = mesh.to_vertex(e);
			float d = HeatDistance[v];
			if (d < Min)
			{
				v_next = v;
				Min = d;
			}
		}
		Lines.push_back(std::pair<Vec3, Vec3>(mesh.position(v_local), mesh.position(v_next)));
		v_local = v_next;
	}
	return Lines;
}
