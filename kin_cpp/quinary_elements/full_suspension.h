#ifndef FULL_SUSPENSION_H
#define FULL_SUSPENSION_H

#include "../quaternary_elements/axle.h";
#include "../secondary_elements/cg.h"

class FullSuspension {
private:
    Axle *Fr_axle;
    Axle *Rr_axle;
    CG *cg;
    bool transform_origin;
    StaticVector<double, 3UL> current_average_cp, prac_average_cp;
    double ang_x, ang_y;
    Node *elements[3];
    Node *all_elements[2];
public:
    FullSuspension (Axle *Fr_axle, Axle *Rr_axle, CG *cg);
    void steer (double rack_displacement);
    void heave (double heave);
    void pitch (double angle);
    void roll (double angle);
    double left_wheelbase () const;
    double right_wheelbase (const);
    StaticVector<double, 2UL> _pitch_resid_func (double x, StaticVector<double, 2UL> args);
    void _update_FAP ();
    double heave_stiffness () const;
    double roll_stiffness () const;
    double pitch_stiffness () const;
    void reset_position ();
    void hard_reset (); /* For when shit hits the fan */
    void flatten ();
    StaticVector<double, 6UL> plane (StaticMatrix<double, 3UL, 3UL> points);
    void translate (StaticVector<double, 3UL> translation);
    void flatten_rotate (StaticVector<double, 3UL> angle);
}
#endif