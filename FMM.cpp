#include "FMM.h"
#include <OpenGP/MLogger.h>
void FMM::clean_up()
{
	for (auto const& vertex : mesh.vertices())
	{
		vfixed[vertex] = false;
		vclose[vertex] = false;
		vfar[vertex] = true;
		Distance[vertex] = -1;
		Display[vertex] = 0;
		MaxGeodesic = -999;
		MinGeodesic = 999;
	}
	Queue.clear();
}

float FMM::compute_distance_dijkstra(Vertex vi, Vertex vj, Vertex vk)
{
	assert(!vfixed[vk]);
	assert(vfixed[vi]);
	assert(vfixed[vj]);

	float d1 = Distance[vi];
	float d2 = Distance[vj];

	assert(d1 != -1);
	assert(d2 != -1);

	float e1 = (mesh.position(vk) - mesh.position(vi)).norm();
	float e2 = (mesh.position(vk) - mesh.position(vj)).norm();

	d1 += e1;
	d2 += e2;

	return d1 < d2 ? d1 : d2;
}

//not used, another method
float FMM::compute_distance2(Vertex vi, Vertex vj, Vertex vk)
{
	//not used, another method
	assert(!vfixed[vk]);
	assert(vfixed[vi]);
	assert(vfixed[vj]);

	float d1 = Distance[vi];
	float d2 = Distance[vj];

	assert(d1 != -1);
	assert(d2 != -1);


	Vec3 pi = mesh.position(vi);
	Vec3 pj = mesh.position(vj);
	Vec3 pk = mesh.position(vk);

	//transform the cordinates
	Vec3 x_tri = (pj - pi).normalized();
	Vec3 z_tri = (x_tri.cross(pk - pi)).normalized();
	Vec3 y_tri = z_tri.cross(x_tri).normalized();

	Mat3x3 A;
	A.col(0) = x_tri;
	A.col(1) = y_tri;
	A.col(2) = z_tri;

	Mat3x3 T;
	T = ((A.transpose()).inverse())*O;
	T.transposeInPlace();

	pj = T*(pj-pi);
	pk = T*(pk-pi);
	pi = Vec3(0, 0, 0);

	pj(1) =pj(2)= 0;
	pk(2) = 0;


	float pjx = pj(0);

	float vsx = (d1*d1 - d2*d2 + pjx*pjx) / (2 * pjx);

	float vsy = -1*sqrt(std::abs(d1*d1 - vsx*vsx));

	Vec3 vs(vsx, vsy, 0);

	Vec3 e1 = pj - pi;
	Vec3 e2 = pk - pj;
	Vec3 e3 = pk - vs;



	float d=-1.0;
	float sign1 = e3.cross(e1)(2);
	float sign2 = e3.cross(e2)(2);


	if (sign1 < 0 && sign2>0)
		d = e3.norm();
	if (sign1 >= 0)
		d = e1.norm() + d1;
	if (sign2 <= 0)
		d = e2.norm() + d2;

	return d;
}


double FMM::compute_distance(Vertex vi, Vertex vj, Vertex vk)
{
	assert(!vfixed[vk]);
	assert(vfixed[vi]);
	assert(vfixed[vj]);
	double a, b, u, costheta,A,B,C,di,dj,dk,t,CD;
	Vec3 e1, e2;
	e1 = mesh.position(vi) - mesh.position(vk);
	e2 = mesh.position(vj) - mesh.position(vk);
	costheta = (e1.normalized()).dot(e2.normalized());
	di = Distance[vi];
	dj = Distance[vj];
	if (di < dj)
	{
		u = dj - di;
		a = e2.norm();
		b = e1.norm();
	}
	else
	{
		u = di - dj;
		a = e1.norm();
		b = e2.norm();
	}
	A = a*a + b*b - 2 * a*b*costheta;
	B = 2 * b*u*(a*costheta - b);
	C = b*b*(u*u - a*a*(1 - costheta*costheta));

	float root = B*B - 4 * A*C;

	if (root < 0)
		root = 0;
	t = (-B + sqrt(root)) / (2 * A);


	CD = b*(t - u) / t;

	
	if (t > u&&(CD<a/costheta)&&(CD>a*costheta)) {
		double T = di < dj ? di : dj;
		dk = T + t;
	}
	else
	{
		double d1 = e1.norm() + di;
		double d2 = e2.norm() + dj;
		dk = d1 < d2 ? d1 : d2;
	}


	return dk;

}

void FMM::compute_geodesic(const Vertex& start_vertex)
{

	Vertex vstart = start_vertex;
	vfixed[vstart] = true;
	vfar[vstart] = false;
	Distance[vstart] = 0;

	for (auto const& v : mesh.vertices(vstart))
	{
		vclose[v] = true;
		vfar[v] = false;
		float dk= (mesh.position(v) - mesh.position(vstart)).norm();
		bool inserted=Queue.insert_or_update(v, dk);
		if (inserted)
			Distance[v] = dk;
	}

	while (!Queue.is_empty())
	{
		Vertex v = Queue.pop();
		vfixed[v] = true;
		vclose[v] = false;

		Halfedge e2,e3;
		Vertex vj, vk,vm;
		for (auto const& e1 : mesh.halfedges(v))
		{
			vj = mesh.to_vertex(e1);
			if (vfixed[vj])
			{
				e2 = mesh.next_halfedge(e1);
				e3 = mesh.next_halfedge(mesh.opposite_halfedge(e1));
				vk = mesh.to_vertex(e2);
				vm = mesh.to_vertex(e3);
				//only compute the non-fixed points.
				if (!vfixed[vk])
				{
					vclose[vk] = true;
					vfar[vk] = false;
					float dk = compute_distance(v, vj, vk);
					bool inserted = Queue.insert_or_update(vk, dk);
					//if inserted, means the value is smaller than previous that computed in another path, so update it in the Distance.
					if (inserted)
						Distance[vk] = dk;
				}
				if (!vfixed[vm])
				{
					vclose[vm] = true;
					vfar[vm] = false;
					float dm = compute_distance(v, vj, vm);
					bool inserted = Queue.insert_or_update(vm, dm);
					if (inserted)
						Distance[vm] = dm;
				}
			}
		}
	}
	//find out the Max G, Min G and the Max geodesic distance span along edge.
	for (auto const& vertex : mesh.vertices())
	{
		float d1 = Distance[vertex];
		Display[vertex] = d1;
		float d2 = 0.0;
		for (auto const& vj : mesh.vertices(vertex))
		{
			float d = std::abs(Distance[vertex] - Distance[vj]);
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
	
}


std::vector<std::pair<Vec3, Vec3>> FMM::draw_isolines()
{
	//make sure this is no more than one line cross the triangle (step>MaxEdgeSpan)
	int n_iso = MaxGeodesic / MaxEdgeSpan;
	float step = MaxGeodesic / float(n_iso);
	SurfaceMesh::Vertex v1,v2;
	std::vector<std::pair<Vec3, Vec3>> Lines;
	for (auto const& face : mesh.faces())
	{
		std::vector<Vec3> points;
		for (auto const& edge : mesh.halfedges(face))
		{
			v1 = mesh.from_vertex(edge);
			v2 = mesh.to_vertex(edge);
			int n1 = Distance[v1] / step;
			int n2 = Distance[v2] / step;
			if (n1 != n2)
			{
				int n = n1 < n2 ? n1 : n2;
				++n;
				float lamda = (n*step - Distance[v1]) / (Distance[v2] - Distance[v1]);
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


std::vector<std::pair<Vec3, Vec3>> FMM::draw_path(const Vertex& start, const Vertex& end)
{
	Vertex v_local = end, v_next = end;
	std::vector<std::pair<Vec3, Vec3>> Lines;
	Lines.push_back(std::pair<Vec3, Vec3>(mesh.position(v_local), mesh.position(v_next)));
	while (v_next != start)
	{
		Scalar step = -1;
		for (auto const& e : mesh.halfedges(v_local))
		{
			Vertex v = mesh.to_vertex(e);
			float d = Distance[v_local] - Distance[v];
			if (d > step)
			{
				v_next = v;
				step = d;
			}
		}
		Lines.push_back(std::pair<Vec3, Vec3>(mesh.position(v_local), mesh.position(v_next)));
		v_local = v_next;
	}
	return Lines;
}


