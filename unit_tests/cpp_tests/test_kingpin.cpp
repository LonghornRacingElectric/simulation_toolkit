#include "test_kingpin.h"
#include <cmath>

using namespace blaze;

// Test: Kingpin initialization with length (typical use: lower to upper outboard)
TEST_F(TestKingpin, test_kingpin_init_length) {
    // Typical kingpin setup: connects lower outboard to upper outboard
    Node lower_outboard(StaticVector<double, 3UL>{0, 0, 0});
    Node upper_outboard(StaticVector<double, 3UL>{0, 0, 5});
    Beam kingpin_beam(&lower_outboard, &upper_outboard);
    
    Kingpin kingpin(&kingpin_beam);
    
    expectNear(kingpin.length(), 5.0, 1e-6);
}

// Test: Kingpin getBeam returns usable beam (as used in Tire and Tie classes)
TEST_F(TestKingpin, test_kingpin_get_beam_usable) {
    Node lower_outboard(StaticVector<double, 3UL>{0, 0, 0});
    Node upper_outboard(StaticVector<double, 3UL>{0, 0, 5});
    Beam kingpin_beam(&lower_outboard, &upper_outboard);
    
    Kingpin kingpin(&kingpin_beam);
    
    // Test that getBeam() returns the correct beam
    EXPECT_EQ(kingpin.getBeam(), &kingpin_beam);
    
    // Test that we can access beam methods through getBeam() (as done in tire.cpp and tie.cpp)
    EXPECT_EQ(kingpin.getBeam()->getInboardNode(), &lower_outboard);
    EXPECT_EQ(kingpin.getBeam()->getOutboardNode(), &upper_outboard);
    
    // Test direction vector (used in tie.cpp)
    StaticVector<double, 3UL> expected_dir = {0, 0, 1};
    StaticVector<double, 3UL> actual_dir = kingpin.getBeam()->direction();
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual_dir[i], expected_dir[i], 1e-6) << "Direction mismatch at index " << i;
    }
}

// Test: Kingpin beam provides getInboardNode (used extensively in tire.cpp and tie.cpp)
TEST_F(TestKingpin, test_kingpin_beam_inboard_node) {
    Node lower_outboard(StaticVector<double, 3UL>{1, 2, 3});
    Node upper_outboard(StaticVector<double, 3UL>{4, 5, 6});
    Beam kingpin_beam(&lower_outboard, &upper_outboard);
    
    Kingpin kingpin(&kingpin_beam);
    
    // Access inboard node through getBeam() (as done in tire.cpp line 20, 26, etc.)
    Node* inboard = kingpin.getBeam()->getInboardNode();
    StaticVector<double, 3UL> expected = {1, 2, 3};
    StaticVector<double, 3UL> actual = inboard->position;
    
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], 1e-6) << "Inboard node position mismatch at index " << i;
    }
}

// Test: Kingpin length calculation (typical vertical kingpin)
TEST_F(TestKingpin, test_kingpin_length_vertical) {
    // Typical kingpin: vertical connection between lower and upper outboard
    Node lower_outboard(StaticVector<double, 3UL>{0, 0, 0});
    Node upper_outboard(StaticVector<double, 3UL>{0, 0, 0.3});  // 300mm typical kingpin length
    Beam kingpin_beam(&lower_outboard, &upper_outboard);
    
    Kingpin kingpin(&kingpin_beam);
    
    expectNear(kingpin.length(), 0.3, 1e-6);
}

// Test: Kingpin with angled beam (non-vertical, as in real suspension)
TEST_F(TestKingpin, test_kingpin_angled_beam) {
    // Kingpin with some angle (typical in real suspensions)
    Node lower_outboard(StaticVector<double, 3UL>{0, 0, 0});
    Node upper_outboard(StaticVector<double, 3UL>{0.05, 0, 0.3});  // 50mm offset, 300mm height
    Beam kingpin_beam(&lower_outboard, &upper_outboard);
    
    Kingpin kingpin(&kingpin_beam);
    
    // Length = sqrt(0.05^2 + 0.3^2) = sqrt(0.0025 + 0.09) = sqrt(0.0925)
    double expected_length = std::sqrt(0.05 * 0.05 + 0.3 * 0.3);
    expectNear(kingpin.length(), expected_length, 1e-6);
}

