#include "suspension_model.h"
#include <cmath>
#include <filesystem>
#include <vector>

SuspensionModel::SuspensionModel (StaticMatrix<double, 3UL, 6UL> &FL_inboard_points, 
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
                    double tire_radius, double tire_width, StaticVector<double, 3UL> &cg_location, bool show_ICs) {

    verbose = false;

    current_heave = 0;
    current_pitch = 0;
    current_roll = 0;

    cg = new CG (new Node (cg_location));
    FL_double_wishbone = new DoubleWishbone (FL_inboard_points, FL_outboard_points,
                                            FL_bellcrank_params, FL_rate, FL_weight,
                                            cg, FL_upper, FL_contact_patch, 
                                            FL_inclination_angle * M_PI / 180,
                                            FL_toe * M_PI / 180, 
                                            tire_radius, tire_width, show_ICs);
    FR_double_wishbone = new DoubleWishbone (FR_inboard_points, FR_outboard_points,
                                            FR_bellcrank_params, FR_rate, FR_weight,
                                            cg, FR_upper, FR_contact_patch, 
                                            FR_inclination_angle * M_PI / 180,
                                            FR_toe * M_PI / 180, 
                                            tire_radius, tire_width, show_ICs);
    RL_double_wishbone = new DoubleWishbone (RL_inboard_points, RL_outboard_points,
                                            RL_bellcrank_params, RL_rate, RL_weight,
                                            cg, RL_upper, RL_contact_patch, 
                                            RL_inclination_angle * M_PI / 180,
                                            RL_toe * M_PI / 180, 
                                            tire_radius, tire_width, show_ICs);
    RR_double_wishbone = new DoubleWishbone (RR_inboard_points, RR_outboard_points,
                                            RR_bellcrank_params, RR_rate, RR_weight,
                                            cg, RR_upper, RR_contact_patch, 
                                            RR_inclination_angle * M_PI / 180,
                                            RR_toe * M_PI / 180, 
                                            tire_radius, tire_width, show_ICs);
    Fr_axle = new Axle (FL_double_wishbone, FR_double_wishbone, cg);
    Rr_axle = new Axle (RL_double_wishbone, RR_double_wishbone, cg);

    full_suspension = new FullSuspension (Fr_axle, Rr_axle, cg);
    elements[0] = full_suspension;
}

/* Induce steer into system */
void SuspensionModel::steer (double rack_displacement) {
    full_suspension->steer (rack_displacement);
}

/* Vertically translate entire car */
void SuspensionModel::heave (double heave) {
    current_heave = heave;
    full_suspension->heave (heave);
}

/* Vertically translate one half of car by {pitch} degrees about cg */
void SuspensionModel::pitch (double pitch) {
    current_pitch = pitch;
    full_suspension->pitch (pitch * M_PI / 180);
}

/* Laterally rotate car by {roll} degrees about cg */
void SuspensionModel::roll (double roll) {
    current_roll = roll;
    full_suspension->roll (roll * M_PI / 180);
}