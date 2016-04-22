#pragma once
#include<OpenGP/SurfaceMesh/SurfaceMesh.h>
#include"FMMPriorityQueue.h"

using namespace OpenGP;

class FMM
{
	typedef SurfaceMesh::Face Face;
	typedef SurfaceMesh::Vertex Vertex;
	typedef SurfaceMesh::Halfedge Halfedge;
private:SurfaceMesh& mesh;
		FMMPriorityQueue Queue;
		SurfaceMesh::Vertex_property<bool> vfixed;
		SurfaceMesh::Vertex_property<bool> vclose;
		SurfaceMesh::Vertex_property<bool> vfar;
		SurfaceMesh::Vertex_property<float> Distance;
		SurfaceMesh::Vertex_property<float> Display;
		const Mat3x3 O = Mat3x3::Identity(3,3);
public:	float MaxGeodesic;
		float MinGeodesic;
		float MaxEdgeSpan;
public:
	FMM(SurfaceMesh& mesh) :mesh(mesh), Queue(mesh)
	{
		vfixed=mesh.add_vertex_property<bool>("FMM:v:fixed");
		vclose=mesh.add_vertex_property<bool>("FMM:v:close");
		vfar = mesh.add_vertex_property<bool>("FMM:v:far");
		Distance=mesh.add_vertex_property<float>("FMM:Geodesic");
		Display=mesh.vertex_property<float>("v:quality");
		MaxGeodesic = 0.0;
		MaxEdgeSpan = 0.0;
		MinGeodesic = 999;
	}

	~FMM()
	{
		mesh.remove_vertex_property(vfixed);
		mesh.remove_vertex_property(vclose);
		mesh.remove_vertex_property(Distance);
		mesh.remove_vertex_property(vfar);
	}

	void clean_up();

	void compute_geodesic(const Vertex& start_vertex);

	std::vector<std::pair<Vec3, Vec3>> draw_isolines();

	std::vector<std::pair<Vec3, Vec3>> draw_path(const Vertex& start, const Vertex& end);

	float Geodesic(const Vertex& v) { return Distance[v]; };

private:float compute_distance2(Vertex vx, Vertex vy, Vertex vz);

		float compute_distance_dijkstra(Vertex vx, Vertex vy, Vertex vz);

		double compute_distance(Vertex vi, Vertex vj, Vertex vk);
};