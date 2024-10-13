#include "beam.h"
using namespace std;

Beam(Node *inboard, Node *outboard);
Node *getInboardNode ();
Node *getOutboardNode ();
array<double, 2> normalized_transform ();
//calculate intersection points
array<double, 3> yz_intersection (Beam *);
array<double, 3> xz_intersection (Beam *);
void translate (array<double, 3>);
void flatten_rotate (array<double, 3>); 
//coordinates relating to the beam
array<double, 3> direction () const;
array<double, 3> center () const;
array<double, 3> radius () const;

double height ();
/* Beam constructor : 
   Params : in (Beam *), out (Beam *) : inboard and outboard ends of linkage*/
Beam::Beam (Node *in, Node *out) {
    inboard_node = in;
    outboard_node = out;
    plotted = false;

    elements[0] = all_elements[0] = in;
    elements[1] = all_elements[1] = out;
}

/* GETTER : gets inboard Node*/
Node *Beam::getInboardNode () {
    return inboard_node;
}

/* GETTER : gets outboard Node */
Node *Beam::getOutboardNode() {
    return outboard_node;
}

/* Calculates the rotations about x and y which result in a vector pointing strictly in z 
   - Gives kpi and caster, respectively, for kingpin
   Returns : 2-element array : [x_rotation, y_rotation] in radians */
array<double, 2> Beam::normalized_transform () {
    array<double, 3> origin_transform = {
                                            elements[1]->position[0] - elements[0]->position[0],
                                            elements[1]->position[1] - elements[0]->position[1],
                                            elements[1]->position[2] - elements[0]->position[2]                                        
                                        }
}

/* Calculates intersection point between two links in the y-z plane 
   Params : second_beam (Beam *) -- beam intersecting this beam in yz
   Returns 3-element array -- coordinates of intersection (averages x between the links) */
array<double, 3> Beam::yz_intersection (Beam *second_beam) {
    array<double, 3> l_1i = inboard_node->position;
    array<double, 3> l_1o = outboard_node->position;
    double m_1 = (l_1o[2] - l_1i[2]) / (l_1o[1] - l_1i[1])
    double x_1, z_1 = l_1o[1], l_1o[2]

    array<double, 3> l_2i = second_beam->inboard_node->position;
    array<double, 3> l_2o = second_beam->outboard_node->position;
    double m_2 = (l_2o[2] - l_2i[2]) / (l_2o[1] - l_2i[1])
    double x_2, z_2 = l_2o[1], l_2o[2]

    array<array<int, 1>, 2> a = {{-1 * m_1, 1}, {-1 * m_2, 1}};
    array<array<int, 1>, 2> b = {{-1 * m_1 * x_1 + z_1}, {-1 * m_2 * x_2 + z_1}};

    /* NEED LINALG FOR THE FOLLOWING : 
        a = np.array([
            [-1 * m_1, 1],
            [-1 * m_2, 1]
        ])

        b = np.array([
            [-1 * m_1 * y_1 + z_1],
            [-1 * m_2 * y_2 + z_2]
        ])

        y, z = np.linalg.solve(a=a, b=b)

        # Calculate x-value
        # I'll average between left and right halves for KinRC
        x = np.average([l_1o[0], l_2o[0]])

        return np.array([x, y[0], z[0]])
    */

   return nullptr;
}

/* Calculates intersection point between two links in the x-z plane 
   Params : second_beam (Beam *) -- beam intersecting this beam in xz
   Returns 3-element array -- coordinates of intersection (averages x between the links) */
array<double, 3> Beam::xz_intersection (Beam *second_beam) {
    array<double, 3> l_1i = inboard_node->position;
    array<double, 3> l_1o = outboard_node->position;
    
}

void Beam::translate (array<double, 3>); //parameter : [x_shift, y_shift, z_shift]
void Beam::flatten_rotate (array<double, 3>); //parameter : [x_rot, y_rot, z_rot]

//coordinates relating to the beam
array<double, 3> Beam::direction ();
array<double, 3> Beam::center ();
array<double, 3> Beam::radius ();

double Beam::height ();
