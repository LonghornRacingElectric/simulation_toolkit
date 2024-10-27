#ifndef PUSHPULLROD_H
#define PUSHPULLROD_H
#include <iostream>
#include "../secondary_elements/bellcrank.h"
#include "../secondary_elements/pushrod.h"
#include "../secondary_elements/damper.h"

class PushPullRod {
public:
    PushPullRod(Node *inboard, Node *outboard, Node *bellcrank_pivot, 
                double bellcrank_direction, Node *shock_inboard, Node *shock_outboard);
    
    Beam *getInboardBeam();
    Beam *getOutboardbeam();
    Beam *getShockInboard();
    Beam *getShockOutboard();

    Node *getBellcrankPivot();
    double *getBellcrankDirection();

private:
    // Class members
    Beam::rod;  // Link object for rod
    double initial_rod_length;

    Beam::spring_damper_rod;  // Link for spring/damper rod (if bellcrank is used)
    double initial_spring_damper_length;

    Node* bellcrank_pivot;  // Pivot point of the bellcrank (optional)
    StaticVector<double> bellcrank_direction;  // Direction vector for bellcrank rotation

    double bellcrank_angle;  // Bellcrank angle
    double wishbone_angle;   // Wishbone angle

    Beam::elements;  // Elements in the PushPullRod
    Beam::all_elements;  // All elements (rod, bellcrank, etc.)
};

#endif

