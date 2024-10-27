#ifndef STEERING_LINK_H
#define STEERING_LINK_H

#include "../primary_elements/beam.h"
#include "kingpin.h"

class SteeringLink {
private:
    Beam *steering_beam;
    Kingpin *kingpin;
    double initial_length;
    double steering_pickup_to_kp;
    double angle;
public:
    SteeringLink (Beam *, Kingpin *);
    double length () const;
    StaticVector<double, 3UL> _steering_pickup_to_kingpin () const;
    void update ();
    void rotate (double angle);
    void _set_initial_position ();
};
#endif