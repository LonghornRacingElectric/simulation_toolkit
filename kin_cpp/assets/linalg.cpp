#include "../primary_elements/node.h"
#include <blaze/Math.h>
#include <iostream>
#include <cmath>

// Retur type = array 
blaze::DynamicVector<double> unit_vec(Node *p1, Node *p2){
    double dx = pi.position[0] - p2.position[0];
    double dy = pi.position[1] - p2.position[1];
    double dz = pi.position[2] - p2.position[2];
    blaze:: DynamicVector<double> vector_AB{dx, dy, dz};

    double AB_magntude = norm{vector_AB};

    return vector_AB / AB_magnitude;
}

blaze::staticMatrix<double, 3, 3> rotation_matrix(blaze::DynamicVector<double> *unit_vec, double theta){
    double ux = unit_vec[0];
    double uy = unit_vec[1];
    double uz = unit_vec[2];
    double cos_t = cos(theta);
    double sin_t = sin(theta);
    
    blaze::staticMatrix<double, 3, 3> matrix;
    matrix(0,0) = pow(ux, 2) * (1 - cos_t) + cos_t;
    matrix(0,1) = ux * uy * (1 - cos_t) - uz * sin_t;
    matrix(0,2) = ux * uz * (1 - cos_t) + uy * sin_t;

    matrix(1,0) = ux * uy * (1 - cos_t) + uz * sin_t;
    matrix(1,1) = pow(uy, 2) * (1 - cos_t) + cos_t;
    matrix(1,2) = uy * uz * (1 - cos_t) - ux * sin_t;

    matrix(2,0) = ux * uz * (1 - cos_t) - uy * sin_t;
    matrix(2,1) = uy * uz * (1 - cos_t) + ux * sin_t;
    matrix(2,2) = pow(uz, 2) * (1 - cos_t) + cos_t;
    
    return matrix;
};