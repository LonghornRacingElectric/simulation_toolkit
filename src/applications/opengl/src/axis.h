#include <array>

struct Axis
{
    // Three lines representing the X, Y, and Z axes
    static constexpr auto xyzArray = std::array{
        // X-axis (Red)
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // Origin
        1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  // X axis point

        // Y-axis (Green)
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // Origin
        0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,  // Y axis point

        // Z-axis (Blue)
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,  // Origin
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f   // Z axis point
    };

    // The indices for drawing the lines
    static constexpr auto lineIndices = std::array{
        0, 1, // X axis
        2, 3, // Y axis
        4, 5  // Z axis
    };
};
