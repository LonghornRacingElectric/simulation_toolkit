#ifndef DOUBLE_WISHBONE_H
#define DOUBLE_WISHBONE_H
#include "../secondary_elements/cg.h"
#include "../secondary_elements/tie.h"
#include "../secondary_elements/wishbone.h"
#include "push_pull_rod.h"
#include "tire.h"
class DoubleWishbone
{
private:
    double max_jounce;
    double max_rebound;
    double max_steer;
    double spring_rate;
    double weight;
    double total_jounce;
    double heave_jounce;
    double roll_jounce;
    double pitch_jounce;
    double steered_angle;
    double induced_steer;

    CG *cg;

    Beam *upper_fore_link;
    Beam *upper_aft_link;
    Beam *lower_fore_link;
    Beam *lower_aft_link;

    Wishbone *upper_wishbone;
    Wishbone *lower_wishbone;

    Kingpin *kingpin;
    Tie *steering_link;

    Node *upper_outboard;
    Node *lower_outboard;
    Node *tie_outboard;
    Node *contact_patch;
    
    Tire *tire;

    Node *FVIC;
    Beam *FVIC_link;
    Node *SVIC;
    Beam *SVIC_link;

    Node *FV_FAP;
    Node *SV_FAP;

    Node *rod_inboard;
    Node *rod_outboard;
    Node *bellcrank_pivot;
    StaticVector<double, 3UL> bellcrank_direction;
    Node *shock_outboard;
    Node *shock_inboard;

    PushPullRod *rod;
    bool upper;

    Node *elements[10];
    Node *all_elements[9];

    /* TODO -- NEED TO FIND SUITABLE FUNCTION FOR scipy.interpolate.CubicSpline:
       motion_ratio_function = CubicSpline (x=jounce_sweep, y=motion_ratio_lst)
       wheelrate_function = CubicSpline (x=jounce_sweep, y=wheelrate_lst) */
public:
    DoubleWishbone(StaticMatrix<double, 3UL, 6UL> &inboard_points, StaticMatrix<double, 3UL, 4UL> &outboard_points, 
                   StaticMatrix<double, 3UL, 4UL> &bellcrank_params, double spring_rate, double weight, CG *cent_grav, 
                   bool upper, StaticVector<double, 3UL> &contact_patch, double inclination_angle, double toe, double tire_radius, 
                   double tire_width, bool show_ICs);
    void _fixed_unsprung_geom ();
    StaticVector<double, 2UL>_jounce_resid_func (StaticVector<double, 3UL> &x, double jounce);
    double _jounce_induced_steer_resid_func (StaticVector<double, 3UL> &x);
    void jounce (double jounce, double heave_jounce, double roll_jounce, double pitch_jounce);
    double _steer_resid_func (StaticVector<double, 3UL> &x);
    void steer (double steer);
    double motion_ratio () const;
    double wheelrate () const;
    void translate (StaticVector<double, 3UL> translation);
    void flatten_rotate (StaticVector<double, 3UL> angle);
    double lateral_arm () const;
    double longitudinal_arm () const;
    StaticVector<double, 3UL> FVIC_position () const;
    StaticVector<double, 3UL> SVIC_position () const;
    StaticVector<double, 3UL> FV_FAP_position () const;
    StaticVector<double, 3UL> SV_FAP_position () const;
    double caster () const;
    double kpi () const;
    double toe () const;
    double inclination_angle () const;
};

#endif