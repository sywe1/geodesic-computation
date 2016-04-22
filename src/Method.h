#pragma once

#include<OpenGP/SurfaceMesh/SurfaceMesh.h>
#include"SurfaceMeshRenderLines.h"
#include "FMM.h"
#include "GeodesicInHeat.h"
#include<iomanip>

using namespace OpenGP;
class Method
{
private:SurfaceMesh& mesh;
		FMM FastMarching;
		GeodesicInHeat Heat;
		int selectedmethod=0;
		SurfaceMesh::Vertex start_v;
		SurfaceMesh::Vertex end_v;
		bool start = false;
		bool end = false;
		bool Heat_computed = false;
		bool FMM_computed = false;
public:	float Color_Max=1;
		float Color_Min=0;
		Segmentslines Iso;
		Segmentslines Path;

public:Method(SurfaceMesh& mesh) :mesh(mesh),FastMarching(mesh),Heat(mesh) {
		};

	void selectmethod(int index)
	{
		std::vector<std::pair<Vec3, Vec3>> Lines;
		selectedmethod = index;
		if ((selectedmethod == 1)&&start&&end)
		{
			//display FMM result
			FastMarching.clean_up();
			FastMarching.compute_geodesic(start_v);
			//set the color map range
			Color_Max = FastMarching.MaxGeodesic;
			Color_Min = FastMarching.MinGeodesic;
			Lines = removelines();
			//clear lines 
			Path.update(Lines);
			Iso.update(Lines);
			FMM_computed = true;
			mLogger() << "FMM  Distance: " << FastMarching.Geodesic(end_v);
		}
		if ((selectedmethod == 2)&&start&&end)
		{
			//display Heat result
			Lines = removelines();
			Path.update(Lines);
			Iso.update(Lines);
			Heat.clean_up();
			//clear lines 
			Heat.compute_distance(start_v);
			Color_Max = Heat.MaxGeodesic;
			Color_Min = Heat.MinGeodesic;
			mLogger() << "Heat Distance: " << Heat.Geodesic(end_v);
			Heat_computed = true;
		}
	}

	void setHeatParameter(float m)
	{
		Heat.set_factor(m);
	}

	void MeanBoudary()
	{
		Heat.Dirichlet = false;
		Heat.Neumann = false;
	}

	void DirichletBoundary()
	{
		Heat.Dirichlet = true;
		Heat.Neumann = false;
	}

	void NeumannBounday()
	{
		Heat.Neumann = true;
		Heat.Dirichlet = false;
	}

	void selectvertex(Vec3 p)
	{
		SurfaceMesh::Vertex v;
		if (!start || !end)
		{
			float d = 999,d0;
			for (auto const& vertex : mesh.vertices())
			{
				d0 = (mesh.position(vertex) - p).norm();
				if (d0 < d)
				{
					v = vertex;
					d = d0;
				}
			}
		}
		if (!start)
		{
			start = true;
			start_v = v;
			mLogger() << "Starting point selected (" << v.idx() << ")";
			return;
		}
		if (!end)
		{
			end = true;
			end_v = v;
			mLogger() << "Ending point selected (" << v.idx() << ")";
			return;
		}
	}


	void removecolor()
	{
		//remove all color, lines, selected points
		if (selectedmethod == 0)
			return;
		if (selectedmethod == 1)
			FastMarching.clean_up();
		if (selectedmethod == 2)
			Heat.clean_up();
		selectedmethod = 0;
		start = false;
		end = false;
		Heat_computed = false;
		FMM_computed = false;
		std::vector<std::pair<Vec3, Vec3>> Lines=removelines();
		Iso.update(Lines);
		Path.update(Lines);
	}


	std::vector<std::pair<Vec3, Vec3>> removelines()
	{
		return std::vector<std::pair<Vec3, Vec3>>();
	}


	void drawpath()
	{
		std::vector<std::pair<Vec3, Vec3>> Lines;
		if (start&&end)
		{
			assert(start_v.is_valid());
			assert(end_v.is_valid());
			if (selectedmethod == 1)
			{
				Lines=FastMarching.draw_path(start_v,end_v);
				Path.update(Lines);
			}
			if (selectedmethod == 2)
			{
				Lines=Heat.draw_path(start_v,end_v);
				Path.update(Lines);
			}
		}
		else
		{
			if (!start)
				mLogger() << "Please select start and end point";
			if (!end)
				mLogger() << "Please select end point";
		}
	}

	void drawisolines()
	{
		std::vector<std::pair<Vec3, Vec3>> Lines;
		if (start)
		{
			if (selectedmethod == 1)
			{
				Lines = FastMarching.draw_isolines();
				Iso.update(Lines);
			}
			if (selectedmethod == 2)
			{
				Lines = Heat.draw_isolines();
				Iso.update(Lines);
			}
		}
		else
		{
			mLogger() << "Please select start point";
		}
	}

	void error_compare()
	{
		if (start&&Heat_computed&&FMM_computed)
		{
			float Max_Heat=-999, Max_FMM=-999, Mean_Heat=0, Mean_FMM=0, G_Heat, G_FMM, Euclidean, Er_Heat, Er_FMM;
			int n = 0;
			for (auto const& vertex : mesh.vertices())
			{
				if (vertex != start_v) {
					Euclidean = (mesh.position(vertex) - mesh.position(start_v)).norm();
					G_Heat = Heat.Geodesic(vertex);
					G_FMM = FastMarching.Geodesic(vertex);
					Er_Heat = std::abs(G_Heat - Euclidean) / Euclidean;
					Er_FMM = std::abs(G_FMM - Euclidean) / Euclidean;
					if (Er_Heat > Max_Heat)
						Max_Heat = Er_Heat;
					if (Er_FMM > Max_FMM)
						Max_FMM = Er_FMM;
					Mean_Heat += Er_Heat;
					Mean_FMM += Er_FMM;
				}
				++n;
			}
			
			Mean_FMM /= float(n);
			Mean_Heat /= float(n);
			mLogger() << "Test on Plane:" << n / 1000 << "k";
			mLogger() <<std::setprecision(2)<< std::setiosflags(std::ios::fixed | std::ios::showpoint) 
				<<"Heat Method----Max Error: " << Max_Heat*100 << "%        Mean Error:" << Mean_Heat*100<<"%";
			mLogger() << std::setprecision(2)<< std::setiosflags(std::ios::fixed | std::ios::showpoint)
				<< "FMM Method----Max Error: " << Max_FMM*100 << "%        Mean Error:" << Mean_FMM*100<<"%";
		}
	}

	void error_compare_sphere()
	{
		if (start&&Heat_computed&&FMM_computed)
		{
			float Max_Heat = -999, Max_FMM = -999, Mean_Heat = 0, Mean_FMM = 0, G_Heat, G_FMM, Radian, Er_Heat, Er_FMM;
			int n = 0;
			for (auto const& vertex : mesh.vertices())
			{
				if (vertex != start_v) {
					Vec3 P1 = mesh.position(vertex).normalized();
					Vec3 P2 = mesh.position(start_v).normalized();
					Radian = std::acos(P1.dot(P2));
					if (Radian == 0)
						Radian = P1.cross(P2).norm();
					G_Heat = Heat.Geodesic(vertex);
					G_FMM = FastMarching.Geodesic(vertex);
					Er_Heat = std::abs(G_Heat - Radian) / Radian;
					Er_FMM = std::abs(G_FMM - Radian) / Radian;
					if (Er_Heat > Max_Heat)
						Max_Heat = Er_Heat;
					if (Er_FMM > Max_FMM)
						Max_FMM = Er_FMM;
					Mean_Heat += Er_Heat;
					Mean_FMM += Er_FMM;
				}
				++n;
			}
			Mean_FMM /= float(n);
			Mean_Heat /= float(n);
			mLogger() << "Test on Sphere (Radius 1,Center Origin):" << n / 1000 << "k   ";
			mLogger() << std::setprecision(2) << std::setiosflags(std::ios::fixed | std::ios::showpoint)
				<< "Heat Method----Max Error: " << Max_Heat * 100 << "%        Mean Error:" << Mean_Heat * 100 << "%";
			mLogger() << std::setprecision(2) << std::setiosflags(std::ios::fixed | std::ios::showpoint)
				<< "FMM Method----Max Error: " << Max_FMM * 100 << "%        Mean Error:" << Mean_FMM * 100 << "%";
		}
	}


};