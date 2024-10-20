#include "beam.h"
#include <blaze/Math.h>
using namespace blaze;

// void Beam(Node *inboard, Node *outboard);
// Node *getInboardNode ();
// Node *getOutboardNode ();
// StaticVector<double, 2UL> normalized_transform ();
// //calculate intersection points
// StaticVector<double, 3UL> yz_intersection (Beam *);
// StaticVector<double, 3UL> xz_intersection (Beam *);
// void translate (StaticVector<double, 3UL>);
// void flatten_rotate (StaticVector<double, 3UL>); 
// //coordinates relating to the beam
// StaticVector<double, 3UL> direction ();
// StaticVector<double, 3UL> center ();
// StaticVector<double, 3UL> radius ();

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
StaticVector<double, 2UL> Beam::normalized_transform () {
    StaticVector<double, 3UL> origin_transform = elements[1]->position - elements[0]->position;
    return origin_transform;
}

/* Calculates intersection point between two links in the y-z plane 
   Params : second_beam (Beam *) -- beam intersecting this beam in yz
   Returns 3-element array -- coordinates of intersection (averages x between the links) */
StaticVector<double, 3> Beam::yz_intersection (Beam *second_beam) {
    StaticVector<double, 3UL> &l_1i = inboard_node->position;
    StaticVector<double, 3UL> &l_1o = outboard_node->position;
    double m_1 = (l_1o[2] - l_1i[2]) / (l_1o[1] - l_1i[1]);
    double x_1 = l_1o[1], z_1 = l_1o[2];

    StaticVector<double, 3UL> &l_2i = second_beam->inboard_node->position;
    StaticVector<double, 3UL> &l_2o = second_beam->outboard_node->position;
    double m_2 = (l_2o[2] - l_2i[2]) / (l_2o[1] - l_2i[1]);
    double x_2 = l_2o[1], z_2 = l_2o[2];

    StaticMatrix<double, 2, 2> a {{-1 * m_1, 1}, {-1 * m_2, 1}};
    StaticVector<double, 2> b {-1 * m_1 * x_1 + z_1, -1 * m_2 * x_2 + z_2};

    // Computing solution to x
    StaticVector<double, 2> yz;   
    solve(a, yz, b);
    double x = (l_1o[0] + l_2o[0]) / 2.0;

    // return intersecting coordinates {x, y, z} 
    return StaticVector<double, 3UL>{x, yz[0], yz[1]}; 
}

/* Calculates intersection point between two links in the x-z plane 
   Params : second_beam (Beam *) -- beam intersecting this beam in xz
   Returns 3-element array -- coordinates of intersection (averages x between the links) */
StaticVector<double, 3> Beam::xz_intersection (Beam *second_beam) {
    StaticVector<double, 3UL> &l_1i = inboard_node->position;
    StaticVector<double, 3UL> &l_1o = outboard_node->position;
    double m_1 = (l_1o[2] - l_1i[2]) / (l_1o[0] - l_1i[0]);
    double x_1 = l_1o[0], z_1 = l_1o[2];

    StaticVector<double, 3UL> &l_2i = second_beam->inboard_node->position;
    StaticVector<double, 3UL> &l_2o = second_beam->outboard_node->position;
    double m_2 = (l_2o[2] - l_2i[2]) / (l_2o[0] - l_2i[0]);
    double x_2 = l_2o[0], z_2 = l_2o[2];

    StaticMatrix<double, 2, 2> A {{-1 * m_1, 1}, {-1 * m_2, 1}};
    StaticVector<double, 2> B = {-1 * m_1 * x_1 + z_1, -1 * m_2 * x_2 + z_2};

    /* NEED LINALG FOR THE FOLLOWING : 
        y = np.average([l_1o[1], l_2o[1]])

        try:
            x, z = np.linalg.solve(a=a, b=b)
        except:
            y = np.average([l_1o[1], l_2o[1]])
            return [1e9, y, 0]

        return np.array([x[0], y, z[0]])
    */ 
   double y = (l_1o[1] + l_2o[1]) / 2;
   StaticVector<double, 2> X;
   try {    
        solve(A, X, B);
   } catch (...) {
        return StaticVector<double, 3UL> {1e9, y, 0};
   }

    return StaticVector<double, 3UL>{X[0], y, X[1]};
}

/* Translates all children (inboard and outboard nodes)
   Params : translation (3-element array) -- translation to apply [x_shift, y_shift, z_shift]*/
void Beam::translate (const StaticVector<double, 3UL> &translation) {
    for (Node *curr : all_elements) {
        curr->translate (translation);
    }
}

/* Rotates all children (inboard and outboard nodes)
    - Used to re-orient vehicle so that contact patches intersect xy plane
    Params : angle (3-element array) -- angle in radians of rotation [x_rot, y_rot, z_rot] */
void Beam::flatten_rotate (const StaticVector<double, 3UL> &angle) {
    for (Node *curr : all_elements) {
        curr->flatten_rotate (angle);
    }
}

/* Direction attribute of Link
   Returns 3-element array -- direction of link */
StaticVector<double, 3> Beam::direction () const {
    double norm = norm(outboard_node->position - inboard_node->position);
    return (outboard_node->position - inboard_node->position) / norm;
    //returns unit vector
}

/* Center attribute of link 
   Returns 3-element array -- center of beam */
StaticVector<double, 3UL> Beam::center () const {
    return StaticVector<double, 3> ();
}

/* Radius attribute of Link
   Returns : double --  radius of link */
double Beam::radius () const {
    const double DIAMETER = 0.015875;
    return DIAMETER / 2;
}

/* Height (length) attribute of link
   Returns : float -- length of link */
double Beam::height () const {
    return (norm(inboard_node->position - outboard_node->position));
}
