#ifndef MISC_LINALG_H
#define MISC_LINALG_H

#include <blaze/Math.h>
using namespace blaze;

/* Generates rotation matrix
   Params : 
    - unit_vec(3D StaticVector) -- unit vector to apply rotation around. 
    - theta(double) -- rotation angle (radians)
   Return value : 3x3 rotation StaticMatrix. */
StaticMatrix<double, 3UL, 3UL> rotation_matrix(const StaticVector<double, 3UL> &, double);

/* Calculates plane from 3 points
    - General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0
   Params : 
    - points (3x3 StaticMatrix) -- 3 points, each a column vector in 3D space
   Return value : 6D StaticVector -- parameters defining plane
    - in form [a, b, c, x_0, y_0, z_0] */
StaticVector<double, 6UL> plane(const StaticMatrix<double, 3UL, 3UL> &);

/* Calculates point on a plane given two independent variables
    - General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0
   Params : 
    - points(3x3 StaticMatrix) -- 3 points, each a column vector in 3D space 
    - x(double) -- x coordinate of point
    - y(double) -- y coordinate of point
    Return value : 3D StaticVector -- point on plane in form [x, y, x] */
StaticVector<double, 3UL> plane_eval(const StaticMatrix<double, 3UL, 3UL> &, double, double);


#endif //MISC_LINALG