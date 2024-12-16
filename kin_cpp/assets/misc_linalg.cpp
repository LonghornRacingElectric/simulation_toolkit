#include "misc_linalg.h"
#include <cmath>
#include <cassert>
#include <iostream>
#include <gsl/gsl_multimin.h>
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

/* Fsolve for one variable
 (The driver function to minimize any given function)
    
    To call fsolve1:

    // Initial guess for x (e.g., x = 0)
    gsl_vector *initial_guess = gsl_vector_alloc(1);
    gsl_vector_set(initial_guess, 0, 0.0);

    // Call the driver function to minimize the function
    double min_value = fsolve1(function_to_minimize, initial_guess);
    
    std::cout << "The minimum value of the function is at x = " << min_value << std::endl;

    // Free the memory
    gsl_vector_free(initial_guess);
    */

double fsolve1(double (*func)(const gsl_vector *, void *), gsl_vector *initial_guess) {
    // Define the GSL minimization context
    gsl_multimin_function min_func;
    min_func.n = 1;  // Number of parameters (we minimize over one variable here)
    min_func.f = func;  // The function to minimize
    min_func.params = nullptr;  // No extra parameters for the function

    // Create a minimizer (use Nelder-Mead simplex method)
    gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc(gsl_multimin_fminimizer_nmsimplex, 1);

    // Set the initial guess and step size (e.g., step size = 0.1)
    gsl_vector *step_size = gsl_vector_alloc(1);
    gsl_vector_set(step_size, 0, 0.1);  // Step size of 0.1 for the first (and only) parameter

    // Set the minimizer with the function, initial guess, and step size
    gsl_multimin_fminimizer_set(s, &min_func, initial_guess, step_size);

    // Perform the minimization (find the value where the function is minimized)
    int status;
    int iteration = 0;
    do {
        iteration++;

        // Take a step
        status = gsl_multimin_fminimizer_iterate(s);

        if (status) {
            std::cout << "Error during iteration " << iteration << std::endl;
            break;
        }

        // Check for convergence
        double size = gsl_multimin_fminimizer_size(s);
        if (size < 1e-5) {  // Convergence threshold
            break;
        }
    } while (true);

    // Return the minimized value (x value where the function is minimized)
    double min_value = gsl_vector_get(s->x, 0);
    
    // Free the memory allocated for the minimizer
    gsl_multimin_fminimizer_free(s);
    gsl_vector_free(step_size);

    return min_value;
}

//

// Fsolve for two variables (can be modified to inlcude more variables if needed)
/*(The driver function to minimize any given function)
    To call fsolve2:

    // Number of variables (e.g., 2 variables x1, x2)
    size_t n = 2;

    // Initial guess for x1, x2 (e.g., x1 = 0, x2 = 0)
    gsl_vector *initial_guess = gsl_vector_alloc(n);
    gsl_vector_set(initial_guess, 0, 0.0);
    gsl_vector_set(initial_guess, 1, 0.0);

    // Call the driver function to minimize the system of equations
    gsl_vector *solution = fsolve2(function_to_minimize, initial_guess, n);

    // Print the solution (values of x1, x2, ...)
    std::cout << "The solution to the system is:" << std::endl;
    for (size_t i = 0; i < n; i++) {
        std::cout << "x" << i + 1 << " = " << gsl_vector_get(solution, i) << std::endl;
    }

    // Free the memory
    gsl_vector_free(initial_guess);
    gsl_vector_free(solution);

*/
// The driver function to minimize the system of equations
gsl_vector* fsolve2(double (*func)(const gsl_vector *, void *), gsl_vector *initial_guess, size_t n) {
    // Define the GSL minimization context
    gsl_multimin_function min_func;
    min_func.n = n;  // Number of parameters
    min_func.f = func;  // The function to minimize
    min_func.params = nullptr;  // No extra parameters for the function

    // Create a minimizer (use Nelder-Mead simplex method)
    gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc(gsl_multimin_fminimizer_nmsimplex, n);

    // Set the initial guess and step size (e.g., step size = 0.1 for each parameter)
    gsl_vector *step_size = gsl_vector_alloc(n);
    for (size_t i = 0; i < n; i++) {
        gsl_vector_set(step_size, i, 0.1);  // Set step size for each parameter
    }

    // Set the minimizer with the function, initial guess, and step size
    gsl_multimin_fminimizer_set(s, &min_func, initial_guess, step_size);

    // Perform the minimization (find the value where the function is minimized)
    int status;
    int iteration = 0;
    do {
        iteration++;

        // Take a step
        status = gsl_multimin_fminimizer_iterate(s);

        if (status) {
            std::cout << "Error during iteration " << iteration << std::endl;
            break;
        }

        // Check for convergence
        double size = gsl_multimin_fminimizer_size(s);
        if (size < 1e-5) {  // Convergence threshold
            break;
        }
    } while (true);

    // Return the vector containing the minimized values (solutions to the system)
    gsl_vector *solution = gsl_vector_alloc(n);
    gsl_vector_memcpy(solution, s->x);

    // Free the memory allocated for the minimizer and step size
    gsl_multimin_fminimizer_free(s);
    gsl_vector_free(step_size);

    return solution;
}

/* Return cubic function mapping double->double
   Params : 
    - in -- independent variable value
    - out -- dependent variable value 
   Note : i'th data point is (in[i], out[i]) */
std::shared_ptr<std::function<double(double)>> cubic_spline (double in[], double out[], double length) {
    // Degree of polynomial is 3
    const int N = 3;
    /* Values of sum(in^n) for 0 <= n <= 2N. Highest n at lowest index */
    double cum_x [2 * N + 1];
    /* Values of sum(out * in^n) for 0 <= n <= N. Highest n at lowest index */
    double cum_xy [N + 1];

    /* Populate cum_x, cum_xy */
    for (int n1 = 0; n1 <= 2 * N; n1++) {
        /* I don't fucking remember if stack pages are zeroed out on being
           faulted in...covering my ass -- Arnav */
        /* Later matrices have highest pows at lowest indices, so reverse indices here */
        int n = 2 * N - n1;
        cum_x[n] = 0;
        cum_xy[n] = 0;
        for (int i = 0; i < length; i++) {
            cum_x[n] += pow (in[i], n);

            /* Account for bounds difference for cum_xy */
            if (n <= N) {
                cum_xy[n] += out[i] * pow (in[i], n);
            }
        }
    }

    StaticMatrix<double, N + 1, N + 1> x_pows;
    StaticVector<double, N + 1> xy_pows (cum_xy);

    /* Populate x_pows */
    for (int i = 0; i <= 2 * N; i++) {
        for (int j = 0; j <= 2 * N; j++) {
            x_pows(i, j) = cum_x[i + j];
        }
    }

    /* Solve x_pows * weights = xy_pows */
    StaticVector<double, N + 1> coeffs = solve (x_pows, xy_pows);

    auto lambda = [coeffs](double in) -> double {
        /* Extract coefficients */
        double a3 = coeffs[0], a2 = coeffs[1], a1 = coeffs[2], a0 = coeffs[3];
        /* Compute output from spline */
        return a3 * pow (in, 3) + a2 * pow (in, 2) + a1 * in + a0;
    };

    auto spline_func = std::make_shared<std::function<double(double)>> (lambda);

    return spline_func;
}
