#include <math.h>

#ifndef QUATERNION_C
#define QUATERNION_C

typedef struct {
	double x;
	double y;
	double z;
} xyz_t;

const xyz_t AXIS_X = {1, 0, 0};
const xyz_t AXIS_Y = {0, 1, 0};
const xyz_t AXIS_Z = {0, 0, 1};

typedef struct {
	double w;
	double x;
	double y;
	double z;
} quaternion_t;

const quaternion_t QUATERNION_UNITY = {1, 0, 0, 0};

// VECTOR OPERATIONS
// Convenience initialization {{{

xyz_t xyz(double x, double y, double z) {
	xyz_t p = {x, y, z};
	return p;
}

// }}}
// Vector addition {{{

xyz_t xyz_add (xyz_t a, xyz_t b) {
	xyz_t sum = {a.x + b.x, a.y + b.y, a.z + b.z};
	return sum;
}

// }}}
// Scalar multiplication {{{

xyz_t xyz_scale (xyz_t vec, double alpha) {
	xyz_t scaled = {vec.x * alpha, vec.y * alpha, vec.z * alpha};
	return scaled;
}

// }}}
// Vector dot product {{{

double xyz_dot (xyz_t a, xyz_t b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

// }}}
// Vector projection {{{

// Return the projection of vec onto base
xyz_t xyz_project (xyz_t vec, xyz_t base) {
	return xyz_scale(base, xyz_dot(vec, base) / xyz_dot(base, base));
}

// }}}

// QUATERNION ACCESSORS / MUTATORS
// Get hyper-imaginary part {{{

xyz_t quaternion_get_hyperimag (quaternion_t q) {
	xyz_t hyperimag = {q.x, q.y, q.z};
	return hyperimag;
}

// }}}
// Set hyper-imaginary part {{{

void quaternion_set_hyperimag (quaternion_t q, xyz_t hyperimag) {
	q.x = hyperimag.x;
	q.y = hyperimag.y;
	q.z = hyperimag.z;
}

// }}}

// BASIC QUATERNION MATH
// Magnitude {{{

double quaternion_mag (quaternion_t q) {
	return sqrt(pow(q.w, 2) + pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2));
}

// }}}
// Multiplication {{{

quaternion_t quaternion_mul (quaternion_t q, quaternion_t r) {
	quaternion_t qr; // product q times r

	qr.w = r.w * q.w - r.x * q.x - r.y * q.y - r.z * q.z;
	qr.x = r.w * q.x + r.x * q.w - r.y * q.z + r.z * q.y;
	qr.y = r.w * q.y + r.x * q.z + r.y * q.w - r.z * q.x;
	qr.z = r.w * q.z - r.x * q.y + r.y * q.x + r.z * q.w;

	return qr;
}

// }}}
// Scalar multiplication {{{

quaternion_t quaternion_scale (quaternion_t q, double alpha) {
	quaternion_t scaled = {q.w * alpha, q.x * alpha, q.y * alpha, q.z * alpha};
	return scaled;
}

// }}}
// Inverse {{{

quaternion_t quaternion_inv (quaternion_t q) {
	quaternion_t p; // inverse

	double magsq = pow(q.w, 2) + pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2);

	p.w =   q.w / magsq;
	p.x = - q.x / magsq;
	p.y = - q.y / magsq;
	p.z = - q.z / magsq;

	return p;
}

// }}}
// Division {{{

// Does this function compute the attitude difference from unit quaternion a
// to unit quaternion b ???
// NO!  It does not.  It merely computes the Hamiltonian quotient of a and b.
quaternion_t quaternion_div (quaternion_t b, quaternion_t a) {
	quaternion_t dq;

	float asquared = pow(a.w, 2) + pow(a.x, 2) + pow(a.y, 2) + pow(a.z, 2);

	dq.w = (a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z) / asquared;
	dq.x = (a.w * b.x - a.x * b.w - a.y * b.z + a.z * b.y) / asquared;
	dq.y = (a.w * b.y + a.x * b.z - a.y * b.w - a.z * b.x) / asquared;
	dq.z = (a.w * b.z + a.x * b.y - a.y * b.x - a.z * b.w) / asquared;

	return dq;
}

// }}}
// Local basis relation {{{

// rephrases the attitude quaternion ``absol'' with respect to the attitude
// quaternion ``tare''
quaternion_t quaternion_relate (quaternion_t absol, quaternion_t tare) {
	return quaternion_mul(absol, quaternion_inv(tare));
}

// }}}

// QUATERNION ROTATIONS
// Construct a quaternion from an axis and an angle {{{

// Returns a quaternion to represent a rotation of angle theta
// (in radians) around the given axis
quaternion_t quaternion_from_axis_angle (xyz_t axis, double theta) {
	quaternion_t q;

	q.w = cos(theta / 2);
	double coeff = sin(theta / 2);
	q.x = coeff * axis.x;
	q.y = coeff * axis.y;
	q.z = coeff * axis.z;

	return q;
}

// }}}
// Apply a quaternion rotation to a vector {{{

// Returns the vector v that is the image of u under the rotation represented by q
xyz_t quaternion_rotate (quaternion_t q, xyz_t u) {
	quaternion_t u_quat = {0, u.x, u.y, u.z};
	quaternion_t v_quat = quaternion_mul(q, quaternion_mul(u_quat, quaternion_inv(q)));

	xyz_t v = {v_quat.x, v_quat.y, v_quat.z};
	return v;
}

// }}}
// Compute the angle of a quaternion's rotation {{{

double quaternion_angle (quaternion_t q) {
	return acos(q.w) * 2.0;
}

// }}}
// Compute the axis of a quaternion's rotation {{{

xyz_t quaternion_axis (quaternion_t q) {
	xyz_t hyperimag = quaternion_get_hyperimag(q);
	return xyz_scale(hyperimag, 1.0 / sin(quaternion_angle(q) / 2.0));
}

// }}}

// QUATERNION PROJECTION
// Find component of a quaternion rotation that lies around a given axis {{{

quaternion_t quaternion_project (quaternion_t q, xyz_t axis) {
	quaternion_t tprime;
	tprime.w = q.w;
	quaternion_set_hyperimag(tprime, xyz_project(quaternion_get_hyperimag(q), axis));
	quaternion_t twist = quaternion_scale(tprime, 1.0 / quaternion_mag(tprime));

	return twist;
}

// }}}

// QUATERNION CHANGE OF BASIS
// Change a quaternion's basis {{{

// Change a quaternion q's basis from one to another.  The specified bases themselves
// are expected to be with respect to the unity basis {1, 0, 0, 0}.

quaternion_t quaternion_chbasis (quaternion_t q, quaternion_t basis_from, quaternion_t basis_to) {
	quaternion_t xform = quaternion_mul(basis_to, quaternion_inv(basis_from));
	return quaternion_mul(xform, quaternion_mul(q, quaternion_inv(xform)));
}

// }}}

// RELATIONS
// Serialize the relationship between two quaternions as a quaternion {{{

// This function records the rotation from attitude a to attitude b with
// respect to a as a frame of reference, so that the relationship may be
// reapplied to a different quaternion later. a and b must be defined on the same basis.

quaternion_t relation_capture (quaternion_t a, quaternion_t b) {
	return quaternion_mul(quaternion_inv(a), b);
}

// }}}
// Apply a serialized relationship to a quaternion {{{

// This function applies a serialized relation delta to a quaternion q. 
// It will return a result expressed in the same basis that q is defined upon.

quaternion_t relation_apply (quaternion_t q, quaternion_t delta) {
	return quaternion_mul(q, delta);
}

// }}}
// Unapply a serialized relationship to a quaternion {{{

// This function unapplies a serialized relation delta to a quaternion p. 
// It will return a result expressed in the same basis that q is defined upon.

quaternion_t relation_unapply (quaternion_t p, quaternion_t delta) {
	return quaternion_mul(p, quaternion_inv(delta));
}

// }}}

#endif
