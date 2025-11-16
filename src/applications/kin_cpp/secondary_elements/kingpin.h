#ifndef KINGPIN_H
#define KINGPIN_H

#include "../primary_elements/beam.h"

class Kingpin {
public:
    Kingpin (Beam *);
    double length() const;
    Beam *getBeam() const;
private:
    Beam *kingpin_beam;
    double initial_length;
};
#endif //KINGPIN_H