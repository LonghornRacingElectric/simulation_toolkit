#include "misc_linalg.h"
#include <cmath>
#include <cassert>
using namespace blaze;
/* Generates rotation matrix
   Params : 
    - unit_vec(3D StaticVector) -- unit vector to apply rotation around. 
    - theta(double) -- rotation angle (radians)
   Return value : 3x3 rotation StaticMatrix. */
StaticMatrix<double, 3UL, 3UL> rotation_matrix(const StaticVector<double, 3UL> &unit_vec, double theta) {
    /* Sanitize inputs to 3D space */
    assert (size(unit_vec) == 3);
    /* Extract coordinates */
    double ux = unit_vec[0], uy = unit_vec[1], uz = unit_vec[2];
    double cos_t = cos(theta), sin_t = sin(theta);

    StaticMatrix<double, 3UL, 3UL> rot_mat{
        {   pow(ux, 2) * (1 - cos_t) + cos_t, 
            ux * uy * (1 - cos_t) - uz * sin_t,
            ux * ux * (1 - cos_t) + uy * sin_t  },

        {   ux * uy * (1 - cos_t) + uz * sin_t,
            pow(uy, 2) * (1 - cos_t) + cos_t,
            uy * uz * (1 - cos_t) - ux * sin_t  },

        {   ux * uz * (1 - cos_t) - uy * sin_t,
            uy * uz * (1 - cos_t) + ux * sin_t,
            pow(uz, 2) * (1 - cos_t) + cos_t    }
    };

    return rot_mat;
}

/* Calculates plane from 3 points
    - General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0
   Params : 
    - points (3x3 StaticMatrix) -- 3 points, each a column vector in 3D space
   Return value : 6D StaticVector -- parameters defining plane
    - in form [a, b, c, x_0, y_0, z_0] */
StaticVector<double, 6UL> plane(StaticMatrix<double, 3UL, 3UL> &points) {
    /* Sanitize inputs to 3D space */
    assert (columns(points) == 3);
    /* Get 2 vectors that define a plane*/
    StaticVector<double, 3UL> PQ = column(points, 1) - column(points, 0);
    StaticVector<double, 3UL> PR = column(points, 2) - column(points, 0);

    /* Get vector normal to plane */
    StaticVector<double, 3UL> normal_vec = cross(PQ, PR);

    /* Fill return value */
    StaticVector<double, 3UL> col_0 = column(points, 0);
    StaticVector<double, 6UL> plane_vec = {
        normal_vec[0],
        normal_vec[1],
        normal_vec[2], 
        col_0[0], 
        col_0[1], 
        col_0[2]
    };

    return plane_vec;
}

/* Calculates point on a plane given two independent variables
    - General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0
   Params : 
    - points(3x3 StaticMatrix) -- 3 points, each a column vector in 3D space 
    - x(double) -- x coordinate of point
    - y(double) -- y coordinate of point
    Return value : 3D StaticVector -- point on plane in form [x, y, x] */
StaticVector<double, 3UL> plane_eval(StaticMatrix<double, 3UL, 3UL> &points, double x, double y) {
    /* Sanitize inputs to 3D space */
    assert (columns(points) == 3);

    /* Get parameters defining plane and unpack them */
    StaticVector<double, 6> _plane = plane(points);
    double a = _plane[0], b = _plane[1], c = _plane[2];
    double x_0 = _plane[3], y_0 = _plane[4], z_0 = _plane[5];

    /* Compute missing coordinate */
    double z = a / c *(x_0 - x) + b / c * (y_0 - y) + z_0;

    /* Package point coordinates together */
    StaticVector<double, 3> point{x, y, z};
    return point;
}