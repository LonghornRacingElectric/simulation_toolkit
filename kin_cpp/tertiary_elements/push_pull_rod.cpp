#include "pushpullrod.h"
#include <iostream>
#include <blaze/Math.h>
#include "../secondary_elements/bellcrank.h"
#include "../secondary_elements/pushrod.h"
#include "../secondary_elements/damper.h"
using namespace std;

PushPullRod::PushPullRod (Node *in, Node *out, Node *BC_Pivot, double BC_Direction, Node *shock_inboard, Node *shock_outboard){
    inboard = in;
    outboard = out;
    bellcrank_pivot = BC_Pivot;
    bellcrank_direction = BC_Direction;
    shock_inboard = shock_inboard;
    shock_outboard = shock_outboard;

    rod = Beam::(inboard, outboard);
    initial_rod-length = rod->height()

    double bellcrank_angle = 0.0;
    double wishbone_angle = 0.0;
    bellcrank_pivot = bellcrank_pivot;
    bellcrank_direction = bellcrank_direction;
    damper_rod = Link::Link(shock_inboard, shock_outboard);
    initial_damper_length = damper_rod->getLength();
        
    elements[0] = all_elements[0] = rod;
    elements[1] = all_elements[1] = bellcrank_pivot;
    elements[2] = all_elements[2] = spring_damper_rod;

}

