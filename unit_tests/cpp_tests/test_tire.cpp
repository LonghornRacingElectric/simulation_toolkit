#include "test_tire.h"
#include <cmath>

using namespace blaze;

// Test: Tire initialization
// Python equivalent: Basic tire creation
TEST_F(TestTire, test_tire_init) {
    Node contact_patch(StaticVector<double, 3UL>{0, 0, 0});
    Kingpin* kingpin = createVerticalKingpin(0.3);
    
    double static_gamma = 0.0;  // No camber
    double static_toe = 0.0;    // No toe
    double radius = 0.3;         // 300mm radius
    double width = 0.2;          // 200mm width
    
    Tire tire(&contact_patch, kingpin, static_gamma, static_toe, radius, width);
    
    // Check that properties are stored correctly
    EXPECT_DOUBLE_EQ(tire.getRadius(), 0.3);
    EXPECT_DOUBLE_EQ(tire.getWidth(), 0.2);
    EXPECT_DOUBLE_EQ(tire.getGamma(), 0.0);
    EXPECT_DOUBLE_EQ(tire.getStaticToe(), 0.0);
    EXPECT_EQ(tire.getContactPatch(), &contact_patch);
    
    // Cleanup
    delete kingpin->getBeam()->getInboardNode();
    delete kingpin->getBeam()->getOutboardNode();
    delete kingpin->getBeam();
    delete kingpin;
}

// Test: Tire center calculation (no camber, no toe)
// Python equivalent: tire.center property
TEST_F(TestTire, test_tire_center_no_camber) {
    Node contact_patch(StaticVector<double, 3UL>{0, 0, 0});
    Kingpin* kingpin = createVerticalKingpin(0.3);
    
    double static_gamma = 0.0;  // No camber
    double static_toe = 0.0;    // No toe
    double radius = 0.3;
    double width = 0.2;
    
    Tire tire(&contact_patch, kingpin, static_gamma, static_toe, radius, width);
    
    // Center should be directly above contact patch by radius distance
    // With no camber and vertical kingpin, center is at [0, 0, radius]
    StaticVector<double, 3UL> center = tire.center();
    StaticVector<double, 3UL> expected_center{0, 0, 0.3};
    
    expectVectorNear(center, expected_center, 1e-5);
    
    // Cleanup
    delete kingpin->getBeam()->getInboardNode();
    delete kingpin->getBeam()->getOutboardNode();
    delete kingpin->getBeam();
    delete kingpin;
}

// Test: Tire center calculation with camber
// Python equivalent: tire.center with static_gamma
TEST_F(TestTire, test_tire_center_with_camber) {
    Node contact_patch(StaticVector<double, 3UL>{0, 0, 0});
    Kingpin* kingpin = createVerticalKingpin(0.3);
    
    double static_gamma = 5.0 * M_PI / 180.0;  // 5 degrees camber
    double static_toe = 0.0;
    double radius = 0.3;
    double width = 0.2;
    
    Tire tire(&contact_patch, kingpin, static_gamma, static_toe, radius, width);
    
    // Center should be rotated by camber angle
    StaticVector<double, 3UL> center = tire.center();
    
    // With 5 degree camber, center should be rotated about x-axis
    // Expected: [0, radius*sin(gamma), radius*cos(gamma)]
    double expected_y = radius * std::sin(static_gamma);
    double expected_z = radius * std::cos(static_gamma);
    
    EXPECT_NEAR(center[0], 0.0, 1e-5);
    EXPECT_NEAR(center[1], expected_y, 1e-4);
    EXPECT_NEAR(center[2], expected_z, 1e-4);
    
    // Cleanup
    delete kingpin->getBeam()->getInboardNode();
    delete kingpin->getBeam()->getOutboardNode();
    delete kingpin->getBeam();
    delete kingpin;
}

// Test: Tire direction calculation
// Python equivalent: tire.direction property
TEST_F(TestTire, test_tire_direction) {
    Node contact_patch(StaticVector<double, 3UL>{0, 0, 0});
    Kingpin* kingpin = createVerticalKingpin(0.3);
    
    double static_gamma = 0.0;
    double static_toe = 0.0;
    double radius = 0.3;
    double width = 0.2;
    
    Tire tire(&contact_patch, kingpin, static_gamma, static_toe, radius, width);
    
    // Direction should be a unit vector
    StaticVector<double, 3UL> direction = tire.direction();
    double dir_magnitude = std::sqrt(direction[0]*direction[0] + 
                                     direction[1]*direction[1] + 
                                     direction[2]*direction[2]);
    
    // Should be approximately unit length (allowing for numerical precision)
    EXPECT_NEAR(dir_magnitude, 1.0, 1e-5);
    
    // Cleanup
    delete kingpin->getBeam()->getInboardNode();
    delete kingpin->getBeam()->getOutboardNode();
    delete kingpin->getBeam();
    delete kingpin;
}

// Test: Tire height calculation
// Python equivalent: outer_diameter
TEST_F(TestTire, test_tire_height) {
    Node contact_patch(StaticVector<double, 3UL>{0, 0, 0});
    Kingpin* kingpin = createVerticalKingpin(0.3);
    
    double static_gamma = 0.0;
    double static_toe = 0.0;
    double radius = 0.3;
    double width = 0.2;
    
    Tire tire(&contact_patch, kingpin, static_gamma, static_toe, radius, width);
    
    // Height should be radius * 2 (outer diameter)
    double expected_height = radius * 2.0;
    EXPECT_DOUBLE_EQ(tire.height(), expected_height);
    
    // Cleanup
    delete kingpin->getBeam()->getInboardNode();
    delete kingpin->getBeam()->getOutboardNode();
    delete kingpin->getBeam();
    delete kingpin;
}

// Test: Tire induced steer (initial value)
// Python equivalent: tire.delta (steered angle)
TEST_F(TestTire, test_tire_induced_steer_initial) {
    Node contact_patch(StaticVector<double, 3UL>{0, 0, 0});
    Kingpin* kingpin = createVerticalKingpin(0.3);
    
    double static_gamma = 0.0;
    double static_toe = 0.1;  // 0.1 radians toe
    double radius = 0.3;
    double width = 0.2;
    
    Tire tire(&contact_patch, kingpin, static_gamma, static_toe, radius, width);
    
    // Induced steer should initially equal static_toe
    // Python: delta = steered_angle + static_toe, where steered_angle starts at 0
    EXPECT_DOUBLE_EQ(tire.induced_steer(), static_toe);
    
    // Cleanup
    delete kingpin->getBeam()->getInboardNode();
    delete kingpin->getBeam()->getOutboardNode();
    delete kingpin->getBeam();
    delete kingpin;
}

// Test: Tire set_induced_steer
// Python equivalent: Setting steered_angle
TEST_F(TestTire, test_tire_set_induced_steer) {
    Node contact_patch(StaticVector<double, 3UL>{0, 0, 0});
    Kingpin* kingpin = createVerticalKingpin(0.3);
    
    double static_gamma = 0.0;
    double static_toe = 0.05;  // 0.05 radians static toe
    double radius = 0.3;
    double width = 0.2;
    
    Tire tire(&contact_patch, kingpin, static_gamma, static_toe, radius, width);
    
    // Set additional steer angle
    double additional_steer = 0.2;  // 0.2 radians additional steer
    tire.set_induced_steer(additional_steer);
    
    // Induced steer should be static_toe + additional_steer
    // Python: delta = steered_angle + static_toe
    double expected_steer = static_toe + additional_steer;
    EXPECT_DOUBLE_EQ(tire.induced_steer(), expected_steer);
    
    // Cleanup
    delete kingpin->getBeam()->getInboardNode();
    delete kingpin->getBeam()->getOutboardNode();
    delete kingpin->getBeam();
    delete kingpin;
}

// Test: Tire translate
// Python equivalent: Translating tire contact patch
TEST_F(TestTire, test_tire_translate) {
    Node contact_patch(StaticVector<double, 3UL>{0, 0, 0});
    Kingpin* kingpin = createVerticalKingpin(0.3);
    
    double static_gamma = 0.0;
    double static_toe = 0.0;
    double radius = 0.3;
    double width = 0.2;
    
    Tire tire(&contact_patch, kingpin, static_gamma, static_toe, radius, width);
    
    // Get initial center position
    StaticVector<double, 3UL> initial_center = tire.center();
    
    // Translate tire
    StaticVector<double, 3UL> translation{1.0, 2.0, 3.0};
    tire.translate(translation);
    
    // Center should be translated
    StaticVector<double, 3UL> new_center = tire.center();
    StaticVector<double, 3UL> expected_center = initial_center + translation;
    
    expectVectorNear(new_center, expected_center, 1e-5);
    
    // Cleanup
    delete kingpin->getBeam()->getInboardNode();
    delete kingpin->getBeam()->getOutboardNode();
    delete kingpin->getBeam();
    delete kingpin;
}

// Test: Tire with non-vertical kingpin
// Tests tire with angled kingpin (more realistic)
TEST_F(TestTire, test_tire_angled_kingpin) {
    Node contact_patch(StaticVector<double, 3UL>{0, 0, 0});
    
    // Create angled kingpin (typical in real suspension)
    Node* lower = new Node(StaticVector<double, 3UL>{0, 0, 0});
    Node* upper = new Node(StaticVector<double, 3UL>{0.05, 0, 0.3});  // 50mm offset, 300mm height
    Beam* kingpin_beam = new Beam(lower, upper);
    Kingpin* kingpin = new Kingpin(kingpin_beam);
    
    double static_gamma = 0.0;
    double static_toe = 0.0;
    double radius = 0.3;
    double width = 0.2;
    
    Tire tire(&contact_patch, kingpin, static_gamma, static_toe, radius, width);
    
    // Center should still be calculated correctly
    StaticVector<double, 3UL> center = tire.center();
    
    // Center should be a valid 3D point
    EXPECT_TRUE(std::isfinite(center[0]));
    EXPECT_TRUE(std::isfinite(center[1]));
    EXPECT_TRUE(std::isfinite(center[2]));
    
    // Cleanup
    delete lower;
    delete upper;
    delete kingpin_beam;
    delete kingpin;
}

