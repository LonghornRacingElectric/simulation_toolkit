#ifndef FULL_SUSPENSION_H
#define FULL_SUSPENSION_H

#include "../quaternary_elements/axle.h";
#include "../secondary_elements/cg.h"
#include "../secondary_elements/kin_pc.h"

class FullSuspension {
private:
    Axle *Fr_axle;
    Axle *Rr_axle;
    CG *cg;
    bool transform_origin;
    StaticVector<double, 3UL> current_average_cp, prac_average_cp;
    double ang_x, ang_y;
    Axle *elements[3];
    Axle *all_elements[2];

    KinPC *left_kin_pc, right_kin_PC; 
public:
    FullSuspension (Axle *Fr_axle, Axle *Rr_axle, CG *cg);
    void steer (double rack_displacement);
    void heave (double heave);
    void pitch (double angle);
    void roll (double angle);
    double left_wheelbase () const;
    double right_wheelbase () const;
    double _pitch_resid_func (double x, StaticVector<double, 2UL> args);
    void _update_FAP ();
    double heave_stiffness () const;
    double roll_stiffness () const;
    double pitch_stiffness () const;
    void reset_position ();
    void hard_reset (); /* For when shit hits the fan */
    void flatten ();
    void translate (StaticVector<double, 3UL> translation);
    void flatten_rotate (StaticVector<double, 3UL> angle);

    Axle *getFR () { return Fr_axle; }
    Axle *getRR () { return Rr_axle; }
};

#endif