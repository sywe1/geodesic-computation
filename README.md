## Description
This program implements Fast Marching algorithm and Heat algorithm to compute geodesic distance on triangle mesh model. Geodesic variation on surface is represented by isolines and various colors. Numeric value of distance is output to standard output.


## External Dependencies
- Operating System: Linux, Mac OS, Windows
- Libraries: [GLEW](http://glew.sourceforge.net), [Eigen](http://eigen.tuxfamily.org), [GLFW3](http://www.glfw.org), [OpenGP]( https://github.com/OpenGP/OpenGP) 
- Input file format: .obj, all faces should be triangle and set as first parameter of application.

## Build
cd ${PROJECT_SOURCE_DIR}

mkdir build && cd build

cmake ..

make 

$./demo ${model.obj}

## Rough UI
- Mouse
  * Left drag: Rotate model
  * Right press: Select start and end points to compute geodesic distance
  * Middle scroll: Scale model

- Key
  * 1: Apply Fast Marching Method
  * 2: Apply Head Method with mean boundary condition
  * 3: Apply Heat Method with Neumann boundary condition
  * 4: Apply Heat Method with Dirichlet boundary condition
  * F: Set Heat Time Factor (to standard input)
  * I: Display geodesic ioslines
  * P: Show path between start point and end point, namely, the minimum path between the two points on surface.
  * R: Clear color, ioslines, selected points. Set to initial condition for next computation.

- Standard Output
  * Display the geodesic distance between two selected points.

## Result

![alt tag](https://github.com/shengyangwei/geodesic-computation/blob/master/images/graph.png)

![alt tag](https://github.com/shengyangwei/geodesic-computation/blob/master/images/numeric.png)

## Acknowledgement

This program is build on [OpenGP]( https://github.com/OpenGP/OpenGP), acknowledge to Dr. Andrea Tagliasacchi


