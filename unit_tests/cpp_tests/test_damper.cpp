#include "test_damper.h"
#include <cmath>
#include <vector>

using namespace blaze;

// Test: Damper initialization
TEST_F(TestDamper, test_damper_init) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    std::vector<std::pair<double, double>> damping_curve = createStandardDampingCurve();
    
    Damper damper(&inboard, &outboard, damping_curve);
    
    // Check that velocity and force references are stored correctly
    // Python: self.damping_curve = list(zip(*damping_curve))
    // This creates: [(0, 1, 2, 3, 4), (0, 1, sqrt(2), sqrt(3), 2)]
    std::vector<double> expected_velocities = {0, 1, 2, 3, 4};
    std::vector<double> expected_forces = {0, 1, std::sqrt(2), std::sqrt(3), 2};
    
    expectVectorEqual(damper.getVelocityReference(), expected_velocities);
    expectVectorEqual(damper.getForceReference(), expected_forces);
}

// Test: Velocity reference
TEST_F(TestDamper, test_velocity_reference) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    std::vector<std::pair<double, double>> damping_curve = createStandardDampingCurve();
    
    Damper damper(&inboard, &outboard, damping_curve);
    
    std::vector<double> expected = {0, 1, 2, 3, 4};
    expectVectorEqual(damper.getVelocityReference(), expected);
}

// Test: Force reference
TEST_F(TestDamper, test_force_reference) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    std::vector<std::pair<double, double>> damping_curve = createStandardDampingCurve();
    
    Damper damper(&inboard, &outboard, damping_curve);
    
    std::vector<double> expected = {0, 1, std::sqrt(2), std::sqrt(3), 2};
    expectVectorEqual(damper.getForceReference(), expected);
}

// Test: Force calculation when velocity is 0 (default)
TEST_F(TestDamper, test_force_one) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    std::vector<std::pair<double, double>> damping_curve = createStandardDampingCurve();
    
    Damper damper(&inboard, &outboard, damping_curve);
    
    // Default velocity is 0, so force should be 0
    EXPECT_DOUBLE_EQ(damper.force(), 0.0);
}

// Test: Force calculation when velocity is 1 (exact match)
TEST_F(TestDamper, test_force_two) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    std::vector<std::pair<double, double>> damping_curve = createStandardDampingCurve();
    
    Damper damper(&inboard, &outboard, damping_curve);
    damper.setVelocity(1.0);
    
    // Velocity is 1, which exactly matches a point in the curve, so force should be 1
    EXPECT_DOUBLE_EQ(damper.force(), 1.0);
}

// Test: Force calculation when velocity is 2.5 (interpolation)
TEST_F(TestDamper, test_force_three) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    std::vector<std::pair<double, double>> damping_curve = createStandardDampingCurve();
    
    Damper damper(&inboard, &outboard, damping_curve);
    damper.setVelocity(2.5);
    
    // Velocity is 2.5, which is between 2 and 3
    // Force at 2 is sqrt(2), force at 3 is sqrt(3)
    // Linear interpolation: force = sqrt(2) + (sqrt(3) - sqrt(2)) * (2.5 - 2) / (3 - 2)
    //                     = sqrt(2) + (sqrt(3) - sqrt(2)) * 0.5
    //                     = (sqrt(2) + sqrt(3)) / 2
    double expected_force = (std::sqrt(2) + std::sqrt(3)) / 2.0;
    
    EXPECT_NEAR(damper.force(), expected_force, 1e-10);
}

