#ifndef TIRE_H
#define TIRE_H

#include "../secondary_elements/kingpin.h"
#include "../primary_elements/node.h"

using namespace blaze;

class Tire {
public:
    Tire(Node *, Kingpin *, double, double, double, double);

    void translate(StaticVector<double, 3UL> &);
    void flatten_rotate(StaticVector<double, 3UL> &);

    void set_induced_steer(double);
    
    StaticVector<double, 3UL> direction() const;
    StaticVector<double, 3UL> center() const;
    double height() const;
    double induced_steer() const;

private:
    Node *cp;
    Kingpin *kingpin;
    double gamma;
    double static_toe;
    double radius;
    double width;
    double _induced_steer;
    StaticVector<double, 3UL> initial_center;
    StaticVector<double, 3UL> cp_to_kingpin;
    double initial_kpi;
    double initial_caster;
    StaticVector<double, 3UL> initial_direction;
    StaticVector<double, 3UL> tire_direction;
    StaticVector<double, 3UL> center_to_kingpin;
    Node *elements[1];
    Node *all_elements[1];
};

#endif //TIRE_H