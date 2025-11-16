#ifndef SUSPENSION_MODEL_H
#define SUSPENSION_MODEL_H

#include "../quinary_elements/full_suspension.h"

class SuspensionModel {
public:
    SuspensionModel (StaticMatrix<double, 3UL, 6UL> &FL_inboard_points, 
                    StaticMatrix<double, 3UL, 6UL> &FL_outboard_points,
                    StaticMatrix<double, 3UL, 4UL> &FL_bellcrank_params,
                    bool FL_upper, StaticVector<double, 3UL> &FL_contact_patch, 
                    double FL_inclination_angle, double FL_toe, double FL_rate, double FL_weight,
                    StaticMatrix<double, 3UL, 6UL> &FR_inboard_points, 
                    StaticMatrix<double, 3UL, 6UL> &FR_outboard_points,
                    StaticMatrix<double, 3UL, 4UL> &FR_bellcrank_params,
                    bool FR_upper, StaticVector<double, 3UL> &FR_contact_patch, 
                    double FR_inclination_angle, double FR_toe, double FR_rate, double FR_weight,
                    StaticMatrix<double, 3UL, 6UL> &RL_inboard_points, 
                    StaticMatrix<double, 3UL, 6UL> &RL_outboard_points,
                    StaticMatrix<double, 3UL, 4UL> &RL_bellcrank_params,
                    bool RL_upper, StaticVector<double, 3UL> &RL_contact_patch, 
                    double RL_inclination_angle, double RL_toe, double RL_rate, double RL_weight,
                    StaticMatrix<double, 3UL, 6UL> &RR_inboard_points, 
                    StaticMatrix<double, 3UL, 6UL> &RR_outboard_points,
                    StaticMatrix<double, 3UL, 4UL> &RR_bellcrank_params,
                    bool RR_upper, StaticVector<double, 3UL> &RR_contact_patch, 
                    double RR_inclination_angle, double RR_toe, double RR_rate, double RR_weight,
                    double tire_radius, double tire_width, StaticVector<double, 3UL> &cg_location, bool show_ICs);
    void generate_kin ();
    void steer (double rack_displacement);
    void heave (double heave);
    void pitch (double pitch);
    void roll (double roll);
private:
    bool verbose;
    double current_heave, current_pitch, current_roll;
    CG *cg;
    DoubleWishbone *FL_double_wishbone;
    DoubleWishbone *FR_double_wishbone;
    DoubleWishbone *RL_double_wishbone;
    DoubleWishbone *RR_double_wishbone;
    Axle *Fr_axle;
    Axle *Rr_axle;
    FullSuspension *full_suspension;
    FullSuspension *elements[1];
    DoubleWishbone *FL_dw;
    DoubleWishbone *FR_dw;
    DoubleWishbone *RL_dw;
    DoubleWishbone *RR_dw;
};
#endif //SUSPENSION_MODEL_H