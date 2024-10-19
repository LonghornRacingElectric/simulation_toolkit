#include <blaze/Math.h>
#include <iostream>

int main() {
    // Create a dynamic vector of integers
    blaze::StaticVector<int, 3> vecA{1, 2, 3};
    blaze::StaticVector<int, 3> vecB{4, 5, 6};

    // Perform vector addition
    blaze::StaticVector<int, 3> vecC = vecA + vecB;

    // Print the result of vector addition
    std::cout << "Vector A: " << vecA << std::endl;
    std::cout << "Vector B: " << vecB << std::endl;
    std::cout << "Vector C (A + B): " << vecC << std::endl;

    // Create a dynamic matrix
    blaze::DynamicMatrix<double> matA{{1.0, 2.0}, {3.0, 4.0}};
    blaze::DynamicMatrix<double> matB{{5.0, 6.0}, {7.0, 8.0}};

    // Perform matrix multiplication
    blaze::DynamicMatrix<double> matC = matA * matB;

    // Print the result of matrix multiplication
    std::cout << "Matrix A:\n" << matA << std::endl;
    std::cout << "Matrix B:\n" << matB << std::endl;
    std::cout << "Matrix C (A * B):\n" << matC << std::endl;

    return 0;
}
