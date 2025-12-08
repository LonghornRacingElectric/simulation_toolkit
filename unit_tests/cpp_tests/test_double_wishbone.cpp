#include "test_double_wishbone.h"
#include <cmath>

using namespace blaze;

// Test: DoubleWishbone initialization
// Python equivalent: Basic QuarterCar creation
TEST_F(TestDoubleWishbone, test_double_wishbone_init) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    // Check that contact patch is accessible
    Node* cp = dw->getContactPatch();
    EXPECT_NE(cp, nullptr);
    
    // Contact patch should be at the specified position
    StaticVector<double, 3UL> expected_cp{1, 2.5, 0};
    expectVectorNear(cp->position, expected_cp, 1e-5);
    
    delete dw;
}

// Test: DoubleWishbone contact patch getter
// Python equivalent: quarter_car.tire.contact_patch
TEST_F(TestDoubleWishbone, test_double_wishbone_contact_patch) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    Node* cp = dw->getContactPatch();
    EXPECT_NE(cp, nullptr);
    
    // Verify contact patch position
    EXPECT_NEAR(cp->position[0], 1.0, 1e-5);
    EXPECT_NEAR(cp->position[1], 2.5, 1e-5);
    EXPECT_NEAR(cp->position[2], 0.0, 1e-5);
    
    delete dw;
}

// Test: DoubleWishbone FVIC position calculation
// Python equivalent: FVIC calculation
TEST_F(TestDoubleWishbone, test_double_wishbone_fvic_position) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    StaticVector<double, 3UL> fvic = dw->FVIC_position();
    
    // FVIC should be a valid 3D point
    EXPECT_TRUE(std::isfinite(fvic[0]));
    EXPECT_TRUE(std::isfinite(fvic[1]));
    EXPECT_TRUE(std::isfinite(fvic[2]));
    
    // FVIC x-coordinate should match contact patch x-coordinate
    Node* cp = dw->getContactPatch();
    EXPECT_NEAR(fvic[0], cp->position[0], 1e-5);
    
    delete dw;
}

// Test: DoubleWishbone SVIC position calculation
// Python equivalent: SVIC calculation
TEST_F(TestDoubleWishbone, test_double_wishbone_svic_position) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    StaticVector<double, 3UL> svic = dw->SVIC_position();
    
    // SVIC should be a valid 3D point
    EXPECT_TRUE(std::isfinite(svic[0]));
    EXPECT_TRUE(std::isfinite(svic[1]));
    EXPECT_TRUE(std::isfinite(svic[2]));
    
    // SVIC y-coordinate should match contact patch y-coordinate
    Node* cp = dw->getContactPatch();
    EXPECT_NEAR(svic[1], cp->position[1], 1e-5);
    
    delete dw;
}

// Test: DoubleWishbone caster calculation
// Python equivalent: caster property
TEST_F(TestDoubleWishbone, test_double_wishbone_caster) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    double caster = dw->caster();
    
    // Caster should be a finite value (angle in radians)
    EXPECT_TRUE(std::isfinite(caster));
    
    delete dw;
}

// Test: DoubleWishbone KPI calculation
// Python equivalent: kpi property
TEST_F(TestDoubleWishbone, test_double_wishbone_kpi) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    double kpi = dw->kpi();
    
    // KPI should be a finite value (angle in radians)
    EXPECT_TRUE(std::isfinite(kpi));
    
    delete dw;
}

// Test: DoubleWishbone toe calculation
// Python equivalent: tire.delta
TEST_F(TestDoubleWishbone, test_double_wishbone_toe) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    double toe = dw->toe();
    
    // Toe should be a finite value (angle in radians)
    EXPECT_TRUE(std::isfinite(toe));
    
    // Initial toe should be close to zero (no static toe in test setup)
    EXPECT_NEAR(toe, 0.0, 1e-3);
    
    delete dw;
}

// Test: DoubleWishbone inclination angle calculation
// Python equivalent: tire.gamma
TEST_F(TestDoubleWishbone, test_double_wishbone_inclination_angle) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    double gamma = dw->inclination_angle();
    
    // Inclination angle should be a finite value (angle in radians)
    EXPECT_TRUE(std::isfinite(gamma));
    
    delete dw;
}

// Test: DoubleWishbone lateral arm calculation
// Python equivalent: Lateral distance from CG
TEST_F(TestDoubleWishbone, test_double_wishbone_lateral_arm) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    double lateral_arm = dw->lateral_arm();
    
    // Lateral arm should be positive and finite
    EXPECT_GE(lateral_arm, 0.0);
    EXPECT_TRUE(std::isfinite(lateral_arm));
    
    delete dw;
}

// Test: DoubleWishbone longitudinal arm calculation
// Python equivalent: Longitudinal distance from CG
TEST_F(TestDoubleWishbone, test_double_wishbone_longitudinal_arm) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    double longitudinal_arm = dw->longitudinal_arm();
    
    // Longitudinal arm should be positive and finite
    EXPECT_GE(longitudinal_arm, 0.0);
    EXPECT_TRUE(std::isfinite(longitudinal_arm));
    
    delete dw;
}

// Test: DoubleWishbone jounce (zero jounce)
// Python equivalent: quarter_car.jounce(0)
TEST_F(TestDoubleWishbone, test_double_wishbone_jounce_zero) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    // Get initial contact patch position
    Node* cp_before = dw->getContactPatch();
    StaticVector<double, 3UL> cp_pos_before = cp_before->position;
    
    // Apply zero jounce
    dw->jounce(0.0, 0.0, 0.0, 0.0);
    
    // Contact patch should remain at same position (within tolerance)
    StaticVector<double, 3UL> cp_pos_after = cp_before->position;
    expectVectorNear(cp_pos_after, cp_pos_before, 1e-4);
    
    delete dw;
}

// Test: DoubleWishbone translate
// Python equivalent: Translating the suspension
TEST_F(TestDoubleWishbone, test_double_wishbone_translate) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    // Get initial contact patch position
    Node* cp = dw->getContactPatch();
    StaticVector<double, 3UL> initial_cp = cp->position;
    
    // Translate
    StaticVector<double, 3UL> translation{1.0, 2.0, 3.0};
    dw->translate(translation);
    
    // Contact patch should be translated
    StaticVector<double, 3UL> expected_cp = initial_cp + translation;
    expectVectorNear(cp->position, expected_cp, 1e-5);
    
    delete dw;
}

// Test: DoubleWishbone motion ratio (basic check)
// Python equivalent: motion_ratio property
TEST_F(TestDoubleWishbone, test_double_wishbone_motion_ratio) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    double mr = dw->motion_ratio();
    
    // Motion ratio should be positive and finite
    // Note: Current implementation returns placeholder value
    EXPECT_GT(mr, 0.0);
    EXPECT_TRUE(std::isfinite(mr));
    
    delete dw;
}

// Test: DoubleWishbone wheelrate (basic check)
// Python equivalent: wheelrate property
TEST_F(TestDoubleWishbone, test_double_wishbone_wheelrate) {
    DoubleWishbone* dw = createSimpleDoubleWishbone();
    
    double wr = dw->wheelrate();
    
    // Wheelrate should be positive and finite
    EXPECT_GT(wr, 0.0);
    EXPECT_TRUE(std::isfinite(wr));
    
    delete dw;
}

