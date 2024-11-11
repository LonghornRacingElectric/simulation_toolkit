#include "double_wishbone.h"
#include "../assets/misc_linalg.h"
using namespace blaze;
/* NEW CHANGE : outboard points is of form [[upper outboard], [lower outboard], [tie rod], [push/pull rod]]*/
DoubleWishbone::DoubleWishbone(StaticMatrix<double, 3UL, 6UL> &inboard_points, StaticMatrix<double, 3UL, 6UL> &outboard_points, 
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
    Node *_lower_outboard = new Node (column(outboard_points, 2));
    Node *_tie_outboard = new Node (column(outboard_points, 4));

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

    pos_less_than_fifty = false;

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
void DoubleWishbone::jounce (double _jounce, double _heave_jounce, double _roll_jounce, double _pitch_jounce) {
    if (_jounce) {
        heave_jounce = roll_jounce = pitch_jounce = 0;
    }

    if (_heave_jounce) {
        heave_jounce = _heave_jounce;
    }

    if (_roll_jounce) {
        roll_jounce = _roll_jounce;
    } 

    if (_pitch_jounce) {
        pitch_jounce = _pitch_jounce;
    }

    total_jounce = _jounce + heave_jounce + roll_jounce + pitch_jounce;

    StaticVector<double, 2UL> wishbone_angles;
    if (total_jounce) {
        /* NEED FSOLVE FOR THE FOLLOWING 
            wishbone_angles = fsolve(self._jounce_resid_func, [0, 0], args=(self.total_jounce))   
        */
    } else {
        wishbone_angles = {0, 0};
    }

    upper_wishbone->rotate (wishbone_angles[0]);
    lower_wishbone->rotate (wishbone_angles[1]);

    StaticVector<double, 1UL> _induced_steer;

    /* NEED FSOLVE FOR THE FOLLOWING : 
        induced_steer = fsolve(self._jounce_induced_steer_resid_func, [0]) 
    */

   /* Set jounce induced steer in tire */
   tire->set_induced_steer (_induced_steer[0]);

   /* Apply transformation to push/pull rod */
   if (upper) {
        rod->rotate_rod (upper_wishbone->getDirection (), upper_wishbone->getForeBeam ()->getInboardNode (), wishbone_angles[0]);
   } else {
        rod->rotate_rod (lower_wishbone->getDirection (), lower_wishbone->getForeBeam ()->getInboardNode (), wishbone_angles[1]);
   }

   rod->update ();

   induced_steer = _induced_steer[0];
   FVIC->position = FVIC_position ();
   SVIC->position = SVIC_position ();

   FV_FAP->position = FV_FAP_position ();
   SV_FAP->position = SV_FAP_position ();
}

double DoubleWishbone::_steer_resid_func (StaticVector<double, 3UL> &x) {
    double steer_angle = x[0];

    steering_link->rotate (steer_angle);
    double residual_length = steering_link->calculateLength () - steering_link->getInitialLength ();

    return residual_length;
}
void DoubleWishbone::steer (double steer) {
    Node *steering_inboard = steering_link->getTieBeam ()->getInboardNode ();
    steering_inboard->position[1] = steering_inboard->position[1] + steer;

    StaticVector<double, 1UL> angle;

    /* NEED FSOLVE FOR THE FOLLOWING : 
        angle = fsolve(self._steer_resid_func, [0]) */
    
    tire->set_induced_steer (angle[0] + induced_steer);
    steered_angle = steer;
}

double DoubleWishbone::motion_ratio () const {
    motion_ratio_function (total_jounce);
}

double DoubleWishbone::wheelrate () const {
    return spring_rate / pow (motion_ratio (), 2);
}

void DoubleWishbone::translate (StaticVector<double, 3UL> translation) {
    upper_wishbone->translate (translation);
    lower_wishbone->translate (translation);
    steering_link->getTieBeam ()->translate (translation);
    tire->translate (translation);
    FVIC->translate (translation);

    if (pos_less_than_fifty) {
        SVIC->translate (translation);
    }

    FV_FAP->translate (translation);
    SV_FAP->translate (translation);
    rod->translate (translation);
}

void DoubleWishbone::flatten_rotate (StaticVector<double, 3UL> angle) {
    upper_wishbone->flatten_rotate (angle);
    lower_wishbone->flatten_rotate (angle);
    steering_link->getTieBeam ()->flatten_rotate (angle);
    tire->flatten_rotate (angle);
    FVIC->flatten_rotate (angle);

    if (pos_less_than_fifty) {
        SVIC->flatten_rotate (angle);
    }

    FV_FAP->flatten_rotate (angle);
    SV_FAP->flatten_rotate (angle);
    rod->flatten_rotate (angle);

    /* Adjust fore application points for rotation */
    FV_FAP->position = FV_FAP_position ();
    SV_FAP->position = SV_FAP_position ();
}

double DoubleWishbone::lateral_arm () const {
    double lateral_arm = abs (contact_patch->position[1] - cg->getPosition ()->position[1]);

    return lateral_arm;
}

double DoubleWishbone::longitudinal_arm () const {
    double longitudinal_arm = abs (contact_patch->position[0] - cg->getPosition ()->position[0]);

    return longitudinal_arm;
}

StaticVector<double, 3UL> DoubleWishbone::FVIC_position () const {
    StaticVector<double, 6UL> upper_plane = upper_wishbone->plane ();
    StaticVector<double, 6UL> lower_plane = lower_wishbone->plane ();

    double x = contact_patch->position[0];

    if ((upper_plane[1] / upper_plane[2]) == (lower_plane[1] / lower_plane[2])) {
        double cp_y = contact_patch->position[1];
        /* get the sign of cp_y : -1 if less than 0, 0 if 0, 1 if > 0 */
        double sign = cp_y == 0 ? 0 : (cp_y < 0) ? -1 : 1; 
        return {x, -1 * 1e9 * sign, 0};
    }

    StaticMatrix<double, 2UL, 2UL> a {{upper_plane[1], upper_plane[2]}, {lower_plane[1], lower_plane[2]}};
    StaticVector<double, 2UL> b {upper_plane[0] * (upper_plane[3] - x) + upper_plane[1] * upper_plane[4] + upper_plane[2] * upper_plane[5], lower_plane[0] * (lower_plane[3] - x) + lower_plane[1] * lower_plane[4] + lower_plane[2] * lower_plane[5]};
    StaticVector<double, 2UL> soln = solve (a, b);

    double y = soln[0];
    double z = soln[1];

    return {x, y, z};
}

StaticVector<double, 3UL> DoubleWishbone::SVIC_position () const {
    StaticVector<double, 6UL> upper_plane = upper_wishbone->plane ();
    StaticVector<double, 6UL> lower_plane = lower_wishbone->plane ();

    double y = contact_patch->position[1];

    if ((upper_plane[0] / upper_plane[2]) == (lower_plane[0] / lower_plane[2])) {
        if (contact_patch->position[0] == 0) {
            return {-1 * 1e9, y, 0};
        } else {
            double cp_y = contact_patch->position[0];
            /* get the sign of cp_y : -1 if less than 0, 0 if 0, 1 if > 0 */
            double sign = cp_y == 0 ? 0 : (cp_y < 0) ? -1 : 1; 
            return {-1 * 1e9 * sign, y, 0};
        }
    }

    StaticMatrix<double, 2UL, 2UL> a = {{upper_plane[0], upper_plane[2]},
                                        {lower_plane[0], lower_plane[2]}};
    StaticVector<double, 2UL> b = {upper_plane[1] * (upper_plane[4] - y) + upper_plane[0] * upper_plane[3] + upper_plane[2] * upper_plane[5],
                                   lower_plane[1] * (lower_plane[4] - y) + lower_plane[0] * lower_plane[3] + lower_plane[2] * lower_plane[5]};

    StaticVector<double, 2UL> soln = solve (a, b);

    double x = soln[0];
    double z = soln[1];

    return {x, y, z};
}

StaticVector<double, 3UL> DoubleWishbone::FV_FAP_position () const {
    StaticVector<double, 3UL> dir_yz = FVIC_link->getInboardNode ()->position - FVIC_link->getOutboardNode ()->position;
    double z = (dir_yz[2] / dir_yz[1]) * (cg->getPosition()->position[1] - FVIC_link->getOutboardNode ()->position[1] + FVIC_link->getOutboardNode ()->position[2]);

    double x = FVIC_link->getOutboardNode ()->position[0];
    double y = cg->getPosition()->position[1];

    return {x, y, z};
}

StaticVector<double, 3UL> DoubleWishbone::SV_FAP_position () const {
    StaticVector<double, 3UL> dir_yz = SVIC_link->getInboardNode ()->position - SVIC_link->getOutboardNode ()->position;
    double z = (dir_yz[2] / dir_yz[0]) * (cg->getPosition()->position[0] - FVIC_link->getOutboardNode ()->position[0] + FVIC_link->getOutboardNode ()->position[2]);

    double x = cg->getPosition ()->position[0];
    double y = SVIC_link->getOutboardNode ()->position[1];

    return {x, y, z};
}

double DoubleWishbone::caster () const {
    return kingpin->getBeam ()->normalized_transform ()[1];
}

double DoubleWishbone::kpi () const {
    return kingpin->getBeam ()->normalized_transform ()[0];
}

double DoubleWishbone::toe () const {
    return tire->induced_steer ();
}

double DoubleWishbone::inclination_angle () const {
    StaticVector<double, 3UL> vec_a = tire->direction ();
    double gamma = atan (vec_a[2] / (sqrt (pow (vec_a[0], 2) + pow (vec_a[1], 2))));
    return gamma;
}

Node *DoubleWishbone::getContactPatch () const {
    return contact_patch;
}