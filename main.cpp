
#include<Eigen/Sparse>
#include<OpenGP/GL/GlfwWindow.h>
#include"SurfaceMeshRenderGeodesic.h"
#include "GeodesicInHeat.h"
#include "FMM.h"
#include "Method.h"
#include<OpenGP/MLogger.h>
using namespace OpenGP;

struct CustomizeWindow : public GlfwWindow
{
	SurfaceMesh mesh;
	Method UI;
	SurfaceMeshRenderGeodesic renderer= SurfaceMeshRenderGeodesic(mesh);
	bool mouse_cliked = false;
	bool start = false;
	bool end = false;
	bool input = false;
	int W;
	int H;
	CustomizeWindow(char** argv,int W,int H) :UI(mesh),GlfwWindow("Final Project", W, H),W(W),H(H)
	{
		
		mesh.read(argv[1]);

		mesh.update_face_normals();
		mesh.update_vertex_normals();

		this->scene.add(renderer);
		this->scene.add(UI.Iso);
		this->scene.add(UI.Path);
		mLogger() << "Final Project-----Shengyang Wei\n\n\nMouse Left: Drag\n\nMouse Right: Select Point";
		mLogger()<<"Key 1: FMM Method\nKey 2: Heat Method(Mean)\n\nKey 3: Heat Method(Neumann)\n\nKey 4: Heat Method(Dirichlet)\n\nKey F: Set Heat Time Factor\n\nKey I : Isolines\n\nKey P : ShowPath\n\nKey R : Remove";
	}

	void key_callback(int key, int scancode, int action, int mods) override
	{
		GlfwWindow::key_callback(key, scancode, action, mods);

		if (action == GLFW_PRESS)
		{
			switch (key)
			{
			case GLFW_KEY_1:
				if (start) {
					UI.selectmethod(1);
					renderer.colormap_enabled(true);
					renderer.colormap_set_range(UI.Color_Min, UI.Color_Max);
				}
				else
					std::cout << "Please select starting point first\n";
				break;

			case GLFW_KEY_2:
				if (start) {
					UI.MeanBoudary();
					UI.selectmethod(2);
					renderer.colormap_enabled(true);
					renderer.colormap_set_range(UI.Color_Min, UI.Color_Max);
				}
				else
					std::cout << "Please select starting point first\n";
				break;

			case GLFW_KEY_3:
				if (start) {
					UI.NeumannBounday();
					UI.selectmethod(2);
					renderer.colormap_enabled(true);
					renderer.colormap_set_range(UI.Color_Min, UI.Color_Max);
				}
				else
					std::cout << "Please select starting point first\n";
				break;

			case GLFW_KEY_4:
				if (start) {
					UI.DirichletBoundary();
					UI.selectmethod(2);
					renderer.colormap_enabled(true);
					renderer.colormap_set_range(UI.Color_Min, UI.Color_Max);
				}
				else
					std::cout << "Please select starting point first\n";
				break;

			case GLFW_KEY_R:
				UI.removecolor();
				renderer.colormap_enabled(true);
				renderer.colormap_set_range(0,1);
				start = false;
				end = false;
				break;

			case GLFW_KEY_P:
				UI.drawpath();
				break;

			case GLFW_KEY_T:
				UI.error_compare();
				break;

			case GLFW_KEY_S:
				UI.error_compare_sphere();
				break;

			case GLFW_KEY_I:
				UI.drawisolines();
				break;

			case GLFW_KEY_F:
				input = true;
				break;

			default:
				break;
			}
		}
		else
		{
			if (input)
			{
				//after input the value, please press 2 or 3 or  4 to display the new result
				mLogger() << "Please input the parameter M:";
				float M;
				input = false;
				if (std::cin >> M&&M > 0)
					UI.setHeatParameter(M);
				else
					mLogger() << "Invalid parameter";
				input = false;
			}
		}

		renderer.init_data();
		
	}

	void mouse_press_callback(int button, int action, int mods) override {
		using namespace OpenGP;

		if (button == GLFW_MOUSE_BUTTON_LEFT) {
			if (action == GLFW_PRESS) {
				double x, y;
				glfwGetCursorPos(_window, &x, &y);
				scene.camera.mouse_down_tumble(Vec2(x, y));
				mouse_cliked = true;
			}
			else
				mouse_cliked= false;
		}

		if (button == GLFW_MOUSE_BUTTON_RIGHT) {
			if (action == GLFW_RELEASE&&(!start||!end)) {
				double Xpos, Ypos;
				glfwGetCursorPos(_window, &Xpos, &Ypos);
				if (!start) {
					UI.selectvertex(unproject_mouse(Xpos, Ypos));
					start = true;
					return;
				}
				if (!end) {
					UI.selectvertex(unproject_mouse(Xpos, Ypos));
					end = true;
					return;
				}
			}
		}
	}

	OpenGP::Vec3 unproject_mouse(OpenGP::Scalar xPos, OpenGP::Scalar yPos)
	{
		typedef Eigen::Matrix<Scalar, 4, 1> Vec4;
		float zPos;
		glReadBuffer(GL_FRONT);
		int Scale = _height / H;
		xPos = Scale*xPos;
		yPos = Scale*yPos;
		glReadPixels(int(xPos), _height - int(yPos), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &zPos);
		float Objectx, Objecty, Objectz, Objectw;
		Objectx = 2.0f*xPos / _width - 1.0f;
		Objecty = 1.0f - 2.0f*yPos / _height;
		Objectz = 2.0f*zPos - 1.0f;
		Objectw = 1.0f;
		Vec4 near(Objectx, Objecty, Objectz, Objectw);
		Mat4x4 U = (scene._projection*scene._view).inverse();
		Vec4 h_point = (U*near);
		Vec3 point(h_point[0]/h_point[3], h_point[1]/ h_point[3], h_point[2]/ h_point[3]);
		return point;
	}



	void mouse_move_callback(double xPos, double yPos) override {
		if (mouse_cliked)
			scene.camera.mouse_drag_tumble(OpenGP::Vec2(xPos, yPos));
	}

	void scroll_callback(double xOffset, double yOffset) override {
		scene.camera.mouse_scroll((float)yOffset);
	}
};



int main(int argc, char** argv){

	CustomizeWindow window(argv,500,500);
	window.run();

	return 0;
}
