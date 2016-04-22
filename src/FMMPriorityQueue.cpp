#include "FMMPriorityQueue.h"

FMMPriorityQueue::FMMPriorityQueue(SurfaceMesh& mesh):mesh(mesh),queue(Dist)
{
	Dist = mesh.add_vertex_property<float>("v:update distances compare");
}

FMMPriorityQueue::~FMMPriorityQueue()
{
	mesh.remove_vertex_property(Dist);
}

bool FMMPriorityQueue::insert_or_update(Vertex v, float d)
{
	//if f is not added then insert
	if (Dist[v] == -1)
	{
		Dist[v] = d;
		queue.insert(v);
		return true;
	}

	//if previous distance is minimal then return
	float prev_d = Dist[v];
	if (prev_d <= d)
		return false;

	//else update the distance
	queue.erase(v);
	Dist[v] = d;
	queue.insert(v);
	return true;
}

FMMPriorityQueue::Vertex FMMPriorityQueue::pop()
{
	Vertex v = *queue.begin();
	queue.erase(queue.begin());
	return v;
}

void FMMPriorityQueue::clear()
{
	for (auto const& vertex : mesh.vertices())
		Dist[vertex] = -1;

	queue.clear();
}