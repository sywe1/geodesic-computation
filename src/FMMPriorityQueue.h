#pragma once
#include<set>
#include<OpenGP/SurfaceMesh/SurfaceMesh.h>

using namespace OpenGP;

class VertexCompare
{
	typedef SurfaceMesh::Vertex Vertex;
	typedef SurfaceMesh::Vertex_property<float> PerVertexFloat;
private:const PerVertexFloat& vprio;
public:
	VertexCompare(PerVertexFloat& vprio_) :vprio(vprio_) {};
	bool operator()(Vertex v1, Vertex v2)
	{
		float d1 = vprio[v1];
		float d2 = vprio[v2];
		return ( (d1 == d2) ? (v1.idx() < v2.idx()) : (d1 < d2));
	}
};

class FMMPriorityQueue
{
	typedef SurfaceMesh::Vertex Vertex;
	typedef std::set<SurfaceMesh::Vertex, VertexCompare> VertexQueue;
private:
		SurfaceMesh& mesh;
		VertexQueue queue;
		SurfaceMesh::Vertex_property<float> Dist;
public:
	FMMPriorityQueue(SurfaceMesh& mesh);
	~FMMPriorityQueue();

	bool insert_or_update(Vertex v, float d);

	Vertex pop();

	void clear();

	bool is_empty()
	{
		return queue.empty();
	}

	int size()
	{
		return queue.size();
	}
};