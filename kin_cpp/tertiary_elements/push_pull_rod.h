#ifndef PUSHPULLROD_H
#define PUSHPULLROD_H
#include "../primary_elements/beam.h"
#include "../secondary_elements/bellcrank.h"
#include "../secondary_elements/damper.h"

using namespace blaze;

class PushPullRod
{
private:
    Beam *rod;
    Damper *damper;
    Bellcrank *Bellcrank;
    double bellcrank_angle;
    double wishbone_angle;
    double initial_rod_length;
    double initial_spring_damper_length;
    Node *bellcrank_pivot;
    StaticVector<double, 3UL> bellcrank_direction;

    bool bc_exists;
    Node *elements[3];
    Node *all_elements[3];
public:
    PushPullRod (Node *inboard, Node *outboard, bool upper, bool bellcrank, Node *bellcrank_pivot, StaticVector<double, 3UL> &bc_direction, Node *shock_outboard, Node *shock_inboard);
    void rotate_rod (StaticVector<double, 3UL> &axis, Node *origin, double angle);
    void rotate_bellcrank (double angle);
    double rod_length () const;
    double spring_damper_length () const;
    void update ();
    double _bellcrank_resid_func (double x);
    void _reset_bellcrank_position ();
    void _set_initial_position ();
    void translate (StaticVector<double, 3UL> &translation);
    void flatten_rotate(StaticVector<double, 3UL>& angle);
};

#endif

