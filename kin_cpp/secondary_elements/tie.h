#ifndef TIE_H
#define TIE_H

#include <iostream>
#include "../primary_elements/beam.h"
#include "kingpin.h"
class Tie {
public: 
    Tie(Beam *, Kingpin *);
    // Functions for getting length and angle
    double getLength() const;
    double getAngle() const;
    StaticVector<double, 3UL> _steering_pickup_to_kingpin() const;
    void update();
    void rotate(double angle);
    void set_initial_position();
    double calculateLength();
private:
    Beam *tie_beam;
    Kingpin *kingpin;
    double length;
    double initial_length;
    double angle;
    StaticVector<double, 3UL> steering_pickup_to_kingpin;
};

#endif