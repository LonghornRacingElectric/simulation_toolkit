#include "beam.h"
#include <blaze/Math.h>
using namespace std;

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
    double m_1 = (l_1o[2] - l_1i[2]) / (l_1o[1] - l_1i[1]);
    double x_1, z_1 = l_1o[1], l_1o[2];

    array<double, 3> l_2i = second_beam->inboard_node->position;
    array<double, 3> l_2o = second_beam->outboard_node->position;
    double m_2 = (l_2o[2] - l_2i[2]) / (l_2o[1] - l_2i[1]);
    double x_2, z_2 = l_2o[1], l_2o[2];

    array<array<double, 2>, 2> a = {{-1 * m_1, 1}, {-1 * m_2, 1}};
    array<array<double, 1>, 2> b = {{-1 * m_1 * x_1 + z_1}, {-1 * m_2 * x_2 + z_1}};

    /* NEED LINALG FOR THE FOLLOWING : 
        y, z = np.linalg.solve(a=a, b=b)

        # Calculate x-value
        # I'll average between left and right halves for KinRC
        x = np.average([l_1o[0], l_2o[0]])

        return np.array([x, y[0], z[0]])
    */

   return {0, 0, 0};
}

/* Calculates intersection point between two links in the x-z plane 
   Params : second_beam (Beam *) -- beam intersecting this beam in xz
   Returns 3-element array -- coordinates of intersection (averages x between the links) */
array<double, 3> Beam::xz_intersection (Beam *second_beam) {
    array<double, 3> l_1i = inboard_node->position;
    array<double, 3> l_1o = outboard_node->position;
    double m_1 = (l_1o[2] - l_1i[2]) / (l_1o[0] - l_1i[0]);
    double x_1, z_1 = l_1o[0], l_1o[2];

    array<double, 3> l_2i = inboard_node->position;
    array<double, 3> l_2o = outboard_node->position;
    double m_2 = (l_2o[2] - l_2i[2]) / (l_2o[0] - l_2i[0]);
    double x_2, z_2 = l_2o[0], l_2o[2];

    array<array<double, 2>, 2> a = {{-1 * m_1, 1}, {-1 * m_2, 1}};
    array<array<double, 1>, 2> b = {{-1 * m_1 * x_1 + z_1}, {-1 * m_2 * x_2 + z_2}};

    /* NEED LINALG FOR THE FOLLOWING : 
        y = np.average([l_1o[1], l_2o[1]])

        try:
            x, z = np.linalg.solve(a=a, b=b)
        except:
            y = np.average([l_1o[1], l_2o[1]])
            return [1e9, y, 0]

        return np.array([x[0], y, z[0]])
    */ 
   return {0, 0, 0};
}

/* Translates all children (inboard and outboard nodes)
   Params : translation (3-element array) -- translation to apply [x_shift, y_shift, z_shift]*/
void Beam::translate (array<double, 3> translation) {
    for (Node *curr : all_elements) {
        curr->translate (translation);
    }
}

/* Rotates all children (inboard and outboard nodes)
    - Used to re-orient vehicle so that contact patches intersect xy plane
    Params : angle (3-element array) -- angle in radians of rotation [x_rot, y_rot, z_rot] */
void Beam::flatten_rotate (array<double, 3> angle) {
    for (Node *curr : all_elements) {
        curr->flatten_rotate (angle);
    }
}

/* Direction attribute of Link
   Returns 3-element array -- direction of link */
array<double, 3> Beam::direction () const {
    array<double, 3> vec = {
                            outboard_node->position[0] - inboard_node->position[0],
                            outboard_node->position[1] - inboard_node->position[1],
                            outboard_node->position[2] - inboard_node->position[2],
                           };
    blaze::DynamicVector<double> dyn_vec(vec);
    double magnitude = blaze::norm (dyn_vec);
    blaze::DynamicVector<double> unit_vec = dyn_vec / magnitude;
    return {unit_vec[0], unit_vec[1], unit_vec[2]};
}

/* Center attribute of link 
   Returns 3-element array -- center of beam */
array<double, 3> Beam::center () const {
    array<double, 3> in_pos = inboard_node->position;
    array<double, 3> out_pos = outboard_node->position;
    
    for (double e : inboard_node->position) {
        if (e == 1e9) {
            return {
                    (in_pos[0] + out_pos[0]) / 2,
                    (in_pos[1] + out_pos[1]) / 2,
                    (in_pos[2] + out_pos[2]) / 2
                   };
        }
    }

    return out_pos;
}

/* Radius attribute of Link
   Returns : double --  radius of link */
double Beam::radius () const {
    const double DIAMETER = 0.015875;
    return  DIAMETER / 2
}

/* Height (length) attribute of link
   Returns : float -- length of link */
double Beam::height () const {
   array<double, 3> vec = {
                            outboard_node->position[0] - inboard_node->position[0],
                            outboard_node->position[1] - inboard_node->position[1],
                            outboard_node->position[2] - inboard_node->position[2],
                           };
    blaze::DynamicVector<double> dyn_vec(vec);
    return blaze::norm(dyn_vec);
}
