#include <array>
#include "../primary_elements/beam.h"
#include "aframe.h"

//Aframe constructor
Aframe::Aframe (Beam *fore_beam, Beam *aft_beam) {
    fore = fore_beam;
    aft = aft_beam;

    elements[0] = all_elements[0] = fore;
    elements[1] = all_elements[1] = aft;
    //TODO : Initialize direction
    //TODO : Initialize angle
}
