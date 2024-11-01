#include "double_wishbone.h"
#include "../assets/misc_linalg.h"
using namespace blaze;
/* NEW CHANGE : outboard points is of form [[upper outboard], [lower outboard], [tie rod], [push/pull rod]]*/
DoubleWishbone::DoubleWishbone(StaticMatrix<double, 3UL, 6UL> &inboard_points, StaticMatrix<double, 3UL, 4UL> &outboard_points, 
                   StaticMatrix<double, 3UL, 4UL> &bellcrank_params, double _spring_rate, double _weight, CG *cent_grav, 
                   bool _upper, StaticVector<double, 3UL> &_contact_patch, double inclination_angle, double toe, double tire_radius, 
                   double tire_width, bool show_ICs) {
    /* Initialize travel limits */
    max_jounce = NAN;
    max_rebound = NAN;
    max_steer = NAN;

    /* Initialize load parameters */
    spring_rate = _spring_rate;
    weight = _weight;

    /* Initialize state */
    total_jounce = 0.0;

    heave_jounce = 0.0;
    roll_jounce = 0.0;
    pitch_jounce = 0.0;

    steered_angle = 0.0;
    induced_steer = 0.0;

    /* Define all points */
    Node *upper_fore_inboard = new Node (column(inboard_points, 0));
    Node *upper_aft_inboard = new Node (column(inboard_points, 1));
    Node *lower_fore_inboard = new Node (column(inboard_points, 2));
    Node *lower_aft_inboard = new Node (column(inboard_points, 3));
    Node *tie_inboard = new Node (column(inboard_points, 4));

    Node *_upper_outboard = new Node (column(outboard_points, 0));
    Node *_lower_outboard = new Node (column(outboard_points, 1));
    Node *_tie_outboard = new Node (column(outboard_points, 2));

    cg = cent_grav;

    /* Define all links */
    upper_fore_link = new Beam (upper_fore_inboard, _upper_outboard);
    upper_aft_link = new Beam (upper_aft_inboard, _upper_outboard);
    lower_fore_link = new Beam (lower_fore_inboard, _lower_outboard);
    lower_aft_link = new Beam (lower_aft_inboard, _lower_outboard);

    /* Define high-level components */
    upper_wishbone = new Wishbone (upper_fore_link, upper_aft_link);
    lower_wishbone = new Wishbone (lower_fore_link, lower_aft_link);
    kingpin = new Kingpin (new Beam (_lower_outboard, _upper_outboard));
    steering_link = new Tie (new Beam (tie_inboard, tie_outboard), kingpin);

    /* Define unsprung parameters */
    upper_outboard = _upper_outboard;
    lower_outboard = _lower_outboard;
    tie_outboard = _tie_outboard;
    contact_patch = new Node (_contact_patch);
    tire = new Tire (contact_patch, kingpin, inclination_angle, toe, tire_radius, tire_width);

    /* Define instant centers */
    FVIC = new Node (FVIC_position ());
    FVIC_link = new Beam (FVIC, contact_patch);
    SVIC = new Node (SVIC_position ());
    SVIC_link = new Beam (SVIC, contact_patch);

    /* Define force application points */
    FV_FAP = new Node (FV_FAP_position ());
    SV_FAP = new Node (SV_FAP_position ());

    /* Define push / pull rod */
    rod_inboard = new Node (column (inboard_points, 5));
    rod_outboard = new Node (column (outboard_points, 3));
    bellcrank_pivot = new Node (column (bellcrank_params, 0));
    bellcrank_direction = column (bellcrank_params, 1);
    shock_outboard = new Node (column (bellcrank_params, 2));
    shock_inboard = new Node (column (bellcrank_params, 3));

    rod = new PushPullRod (rod_inboard, rod_outboard, upper, true,
                           bellcrank_pivot, bellcrank_direction, shock_outboard,
                           shock_inboard);
    /* Track pushrod location for transformations later */
    upper = _upper;

    /* Cache unsprung geometry */
    _fixed_unsprung_geom ();

    /* Create function for motion ratio */
    double jounce_sweep[101];
    double jounce_interval = 0.0254 * 2 * 5 / 100;

    for (int i = 0; i < 101; i++) {
        jounce_sweep[i] = (jounce_interval * i) + (-0.0254 * 5);
    }

    double motion_ratio_list[101];
    for (int i = 0; i < 101; i++) {
        double _jounce = jounce_sweep[i];
        double upper = _jounce + jounce_interval;
        jounce (_jounce, 0, 0, 0);
        double spring_pos_0 = rod->spring_damper_length ();
        jounce (upper, 0, 0, 0);
        double spring_pos_1 = rod->spring_damper_length ();

        double motion_ratio = jounce_interval / (spring_pos_0 - spring_pos_1);
        motion_ratio_list[i] = motion_ratio;
    }

    /* TODO : 
       self.motion_ratio_function = CubicSpline(x=jounce_sweep, y=motion_ratio_lst) */
    
    /* Reset jounce */
    jounce (0, 0, 0, 0);

    /* Create function for wheelrate */
    double wheelrate_list[101];
    for (int i = 0; i < 101; i++) {
        wheelrate_list[i] = spring_rate / pow (motion_ratio_list[i], 2);
    }

    /* TODO : 
        self.wheelrate_function = CubicSpline(x=jounce_sweep, y=wheelrate_lst) */

    bool pos_less_than_fifty = false;

    for (int i = 0; i < 3; i++) {
        if (SVIC->position[i] > -50 && SVIC->position[i] < 50) {
            pos_less_than_fifty = true;
            break;
        }
    }

    /* Plotting */
    if (!show_ICs) {
        elements[0] = (Node *) upper_wishbone;
        elements[1] = (Node *) lower_wishbone;
        elements[2] = (Node *) rod;
        elements[3] = (Node *) kingpin;
        elements[4] = (Node *) steering_link;
        elements[5] = (Node *) tire;
    } else if (pos_less_than_fifty) {
        elements[0] = (Node *) kingpin;
        elements[1] = (Node *) steering_link;
        elements[2] = (Node *) rod;
        elements[3] = (Node *) tire;
        elements[4] = (Node *) FVIC;
        elements[5] = (Node *) SVIC;
        elements[6] = (Node *) FVIC_link;
        elements[7] = (Node *) SVIC_link;
        elements[8] = (Node *) FV_FAP;
        elements[9] = (Node *) SV_FAP;
    } else {
        elements[0] = (Node *) kingpin;
        elements[1] = (Node *) steering_link;
        elements[3] = (Node *) tire;
        elements[6] = (Node *) FVIC_link;
        elements[8] = (Node *) FV_FAP;
        elements[9] = (Node *) SV_FAP;
    }

    all_elements[0] = (Node *) upper_wishbone;
    all_elements[1] = (Node *) lower_wishbone;
    all_elements[2] = (Node *) rod;
    all_elements[3] = (Node *) steering_link;
    all_elements[4] = (Node *) tire;
    all_elements[5] = (Node *) FVIC;
    
    if (pos_less_than_fifty) {
        all_elements[6] = (Node *) SVIC;
        all_elements[7] = (Node *) FV_FAP;
        all_elements[8] = (Node *) SV_FAP;
    } else {
        all_elements[6] = (Node *) FV_FAP;
        all_elements[7] = (Node *) SV_FAP;
    }
}
void DoubleWishbone::_fixed_unsprung_geom () {
    cp_to_lower = norm (contact_patch->position - lower_outboard->position);
    cp_to_upper = norm (contact_patch->position - upper_outboard->position);
    cp_to_tie = norm (contact_patch->position - tie_outboard->position);

    StaticVector<double, 2UL> xy_ang = kingpin->getBeam ()->normalized_transform ();
    StaticVector<double, 3UL> cp_pos_shifted = contact_patch->position - lower_outboard->position;
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix ({1, 0, 0}, xy_ang[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix ({0, 1, 0}, -1 * xy_ang[1]);

    cp_to_kingpin = y_rot * (x_rot * cp_pos_shifted);
}
StaticVector<double, 2UL>DoubleWishbone::_jounce_resid_func (StaticVector<double, 2UL> &x, double jounce) {
    double upper_wishbone_rot = x[0];
    double lower_wishbone_rot = x[1];

    /* Apply wishbone rotations */
    upper_wishbone->rotate (upper_wishbone_rot);
    lower_wishbone->rotate (lower_wishbone_rot);

    /* Calculate contact patch under jounce condition */
    StaticVector<double, 2UL> xy_ang = kingpin->getBeam ()->normalized_transform ();
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix ({1, 0, 0}, -1 * xy_ang[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix ({0, 1, 0}, xy_ang[1]);
    StaticVector<double, 3UL> cp_pos = (y_rot * (x_rot * cp_to_kingpin)) + lower_outboard->position;

    /* Geometry constraints */
    double _cp_to_lower = norm (cp_pos - lower_outboard->position);
    double _cp_to_upper = norm (cp_pos - upper_outboard->position);
    double offset = cp_pos[2] - jounce;\

    contact_patch->position = cp_pos;
    return {(_cp_to_lower - cp_to_lower) + offset, (_cp_to_upper - cp_to_upper) + offset};
}

double DoubleWishbone::_jounce_induced_steer_resid_func (StaticVector<double, 1UL> &x) {
    double induced_steer = x[0];
    steering_link->rotate (induced_steer);

    double residual_length = steering_link->getLength () - steering_link->getInitialLength ();
    return residual_length;
}
void DoubleWishbone::jounce (double jounce, double heave_jounce, double roll_jounce, double pitch_jounce) {

}
double DoubleWishbone::_steer_resid_func (StaticVector<double, 3UL> &x) {

}
void DoubleWishbone::steer (double steer) {

}
double DoubleWishbone::motion_ratio () const {

}
double DoubleWishbone::wheelrate () const {

}
void DoubleWishbone::translate (StaticVector<double, 3UL> translation) {

}
void DoubleWishbone::flatten_rotate (StaticVector<double, 3UL> angle) {

}
double DoubleWishbone::lateral_arm () const {

}
double DoubleWishbone::longitudinal_arm () const {

}
StaticVector<double, 3UL> DoubleWishbone::FVIC_position () const {

}
StaticVector<double, 3UL> DoubleWishbone::SVIC_position () const {

}
StaticVector<double, 3UL> DoubleWishbone::FV_FAP_position () const {

}
StaticVector<double, 3UL> DoubleWishbone::SV_FAP_position () const {

}
double DoubleWishbone::caster () const {

}
double DoubleWishbone::kpi () const {

}
double DoubleWishbone::toe () const {

}
double DoubleWishbone::inclination_angle () const {

}