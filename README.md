## Description
This program implements Fast Marching algorithm and Heat algorithm to compute geodesic distance on triangle meshes. Geodesic variation on a surface is represented by isolines and various colors. A numeric value of the distance is output to standard output.


## External Dependencies
- Operating System: Linux, Mac OS, Windows
- Libraries: [GLEW](http://glew.sourceforge.net), [Eigen](http://eigen.tuxfamily.org), [GLFW3](http://www.glfw.org), [OpenGP]( https://github.com/OpenGP/OpenGP) 
- Input file format: OBJ. The input mesh should be a triangle mesh.

## Build
cd ${PROJECT_SOURCE_DIR}

mkdir build && cd build

cmake ..

make 

$./demo ${model.obj}

## UI Controls
- Mouse
  * Left drag: Rotate the model
  * Right click: Select start and end points to compute geodesic distance
  * Middle scroll: Scale the model

- Key
  * 1: Apply Fast Marching Method
  * 2: Apply Head Method with mean boundary condition
  * 3: Apply Heat Method with Neumann boundary condition
  * 4: Apply Heat Method with Dirichlet boundary condition
  * F: Set the time factor of Heat method (standard input)
  * I: Display geodesic ioslines
  * P: Show a path between start point and end point, which is path with the minimum geodesic distance between the two points on surface.
  * R: Clear color, ioslines, and selected points. Reset to the initial state for next computation.

- Standard Output
  * Display the geodesic distance between two selected points.

## Result

![alt tag](https://github.com/shengyangwei/geodesic-computation/blob/master/images/graph.png)

![alt tag](https://github.com/shengyangwei/geodesic-computation/blob/master/images/numeric.png)

## Acknowledgement

This program is build on [OpenGP]( https://github.com/OpenGP/OpenGP), acknowledge to Dr. Andrea Tagliasacchi


