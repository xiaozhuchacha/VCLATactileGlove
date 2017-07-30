// Includes {{{

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Quaternion.h"
//#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
//#include "tf2_msgs/TFMessage.h"
//#include "tf/tfMessage.h"
//#include "tf/transform_datatypes.h"
//#include "sensor_msgs/Imu.h"
//#include "sensor_msgs/MagneticField.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "tcabno.c"
#include "quaternion.c"

#include "imutracker/glove.h"

#include <fstream>

// }}}
// Main code definitions {{{

#define DT 0.02		 // [s/loop] loop period.

#define RAD_TO_DEG 57.29577951308232
#define DEG_TO_RAD 0.017453292519943295

#define ROS_REFFRAME "/glove_link"

//#define M_PI 3.14159265358989323846

// }}}
// Convenience functions {{{

// Conversion functions {{{

float deg2rad(float deg)
{
	return deg / 180.0 * M_PI;
}

geometry_msgs::PoseStamped quat2posest(quaternion_t q)
{
	geometry_msgs::Quaternion geoqtmsg;
	geoqtmsg.x = q.x;
	geoqtmsg.y = q.y;
	geoqtmsg.z = q.z;
	geoqtmsg.w = q.w;

	geometry_msgs::Pose posemsg;
	posemsg.orientation = geoqtmsg;
	geometry_msgs::PoseStamped posestmsg;
	posestmsg.pose = posemsg;
	posestmsg.header.frame_id = ROS_REFFRAME;

	return posestmsg;
}

quaternion_t pose2quat (geometry_msgs::Pose p) {
	quaternion_t q;

	q.w = p.orientation.w;
	q.x = p.orientation.x;
	q.y = p.orientation.y;
	q.z = p.orientation.z;

	return q;
}

// }}}
// Point functions {{{

xyz_t compute_endpt(xyz_t startpt, xyz_t base_vector, quaternion_t rotation) {
	return xyz_add(startpt, quaternion_rotate(rotation, base_vector));
}

xyz_t compute_midpt(xyz_t startpt, xyz_t base_vector, quaternion_t rotation) {
	xyz_t half_base = xyz_scale(base_vector, 0.5);
	return xyz_add(startpt, quaternion_rotate(rotation, half_base));
}

// }}}
// Drawing function {{{

const float CHROMA_MAX = 1.5f;

visualization_msgs::Marker genmark(xyz_t startpt, quaternion_t orientation, double length, double radius, float chroma, xyz_t * endpt) {
	visualization_msgs::Marker cylinder;
	cylinder.header.frame_id = ROS_REFFRAME;
	cylinder.header.stamp = ros::Time::now();
	cylinder.ns = "imutracker_finger";
	cylinder.type = visualization_msgs::Marker::CYLINDER;

	// cylinders in ros align along the Z axis by default
	// we'll convert that to our desired orientation
	xyz_t midpt = compute_midpt(startpt, xyz(0.0, 0.0, length), orientation);
	cylinder.pose.position.x = midpt.x;
	cylinder.pose.position.y = midpt.y;
	cylinder.pose.position.z = midpt.z;

	cylinder.pose.orientation = quat2posest(orientation).pose.orientation;

	cylinder.scale.x = radius * 2;
	cylinder.scale.y = radius * 2;
	cylinder.scale.z = length;

	if (chroma > CHROMA_MAX) {
		chroma = CHROMA_MAX;
	}
	if (chroma < 0.0f) {
		chroma = 0.0f;
	}
	cylinder.color.r = chroma / CHROMA_MAX;
	cylinder.color.g = 1.0f - (chroma / CHROMA_MAX);
	cylinder.color.b = 0.0f;
	cylinder.color.a = 1.0f;

	*endpt = compute_endpt(startpt, xyz(0.0, 0.0, length), orientation);
	return cylinder;
}

// }}}

// }}}
// Pose computation defs {{{

const quaternion_t Q_GLOVE2CALPOSE[15] = {
	// The glove reference frame is defined as the current
	// attitude of the palm, so the relation between them
	// is unity
	QUATERNION_UNITY,
	// The following two represent the thumb splayed out and slightly downward
	quaternion_mul(quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * 25), quaternion_from_axis_angle(AXIS_Y, DEG_TO_RAD * 15.0)), 
	quaternion_mul(quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * 25), quaternion_from_axis_angle(AXIS_Y, DEG_TO_RAD * 15.0)),
	// All other phalanges shuold be straight out forward,
	// aligned with the glove reference frame
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD *   5),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD *   5),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD *   5),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD *   0),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD *   0),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD *   0),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * - 5),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * - 5),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * - 5),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * -12),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * -12),
	quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * -12),
};


// The sensor sample readings
quaternion_t q_global2sensors[16];

// Remember that the glove reference frame is defined
// as the attitude of the palm
// The palm is the "0th phalange"
quaternion_t q_phalanges2sensors[16];

// The actual hand pose
quaternion_t q_global2phalanges[16];

// }}}
// Visualization defs {{{

double vizcoeffw = 1.0;
double vizcoeffx = 0.0;
double vizcoeffy = 0.0;
double vizcoeffz = 0.0;

float forces[8];

// definitions for how to visualize a phalange
struct phmarkdef {
	quaternion_t q_ph2mark;
		// relation describing the orientation of the marker
		// cylinder to draw wrt the phalange it represents
		// (remember cylinders by default are colinear with Z)
	double length;
	double radius;
	float * force;
};

quaternion_t y90 = quaternion_from_axis_angle(AXIS_Y, DEG_TO_RAD * 90.0);

float empty = 0.0f;

struct phmarkdef phmarkdefs[15] = {
	{QUATERNION_UNITY, 0.4, 1.0, &forces[3]},
	{             y90, 0.8, 0.2, &forces[4]},
	{             y90, 0.6, 0.2, &forces[6]},
	{             y90, 0.8, 0.2,     &empty},
	{             y90, 0.6, 0.2, &forces[7]},
	{             y90, 0.3, 0.2, &forces[0]},
	{             y90, 0.8, 0.2,     &empty},
	{             y90, 0.6, 0.2, &forces[5]},
	{             y90, 0.3, 0.2, &forces[1]},
	{             y90, 0.8, 0.2,     &empty},
	{             y90, 0.6, 0.2,     &empty},
	{             y90, 0.3, 0.2,     &empty},
	{             y90, 0.6, 0.2,     &empty},
	{             y90, 0.3, 0.2,     &empty},
	{             y90, 0.3, 0.2,     &empty}
};

// }}}
// ROS defs {{{

	ros::Publisher ros_pub_imutracker;
	ros::Publisher ros_pub_imutracker_rel;
	ros::Subscriber ros_sub_imuctrl;
	ros::Subscriber ros_sub_forcedata;
	ros::Subscriber ros_sub_vizctrlw;
	ros::Subscriber ros_sub_vizctrlx;
	ros::Subscriber ros_sub_vizctrly;
	ros::Subscriber ros_sub_vizctrlz;
	ros::Publisher ros_pub_vizcoeff;
	ros::Publisher ros_pub_testa;
	ros::Publisher ros_pub_testb;
	ros::Publisher ros_pub_testc;
	ros::Publisher ros_pub_testd;

	ros::Publisher ros_pub_imutracker_raw;
	ros::Subscriber ros_sub_imutracker_raw;

// }}}
// Forward kinematics {{{

void fwdkin () {
	// Assume a sample has just been taken and q_global2sensors
	// has just been freshly populated

	for (int idx = 0; idx <= 0xE; idx++) {
		q_global2phalanges[idx] = relation_unapply(q_global2sensors[idx], q_phalanges2sensors[idx]);
	}

	xyz_t lastendpt;
	unsigned int phidx = 0;

	// Palm {{{

	phidx = 0;
	visualization_msgs::Marker palm = genmark(
		xyz(0.0, 0.0, 0.0),
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	// }}}
	// Thumb {{{

	phidx++;
	visualization_msgs::Marker thumbbase = genmark(
		quaternion_rotate(q_global2phalanges[0], quaternion_rotate(quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * 20.0), AXIS_Y)), 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	phidx++;
	visualization_msgs::Marker thumbtip = genmark(
		lastendpt, 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	// }}}
	// Index {{{

	phidx++;
	visualization_msgs::Marker indexbase = genmark(
		quaternion_rotate(q_global2phalanges[0], quaternion_rotate(quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * 25.0), AXIS_X)), 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	phidx++;
	visualization_msgs::Marker indexmed = genmark(
		lastendpt, 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	phidx++;
	visualization_msgs::Marker indextip = genmark(
		lastendpt, 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	// }}}
	// Middle {{{

	phidx++;
	visualization_msgs::Marker middlebase = genmark(
		quaternion_rotate(q_global2phalanges[0], quaternion_rotate(quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * 0.0), AXIS_X)), 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	phidx++;
	visualization_msgs::Marker middlemed = genmark(
		lastendpt, 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	phidx++;
	visualization_msgs::Marker middletip = genmark(
		lastendpt, 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	// }}}
	// Ring {{{

	phidx++;
	visualization_msgs::Marker ringbase = genmark(
		quaternion_rotate(q_global2phalanges[0], quaternion_rotate(quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * -25.0), AXIS_X)), 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	phidx++;
	visualization_msgs::Marker ringmed = genmark(
		lastendpt, 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	phidx++;
	visualization_msgs::Marker ringtip = genmark(
		lastendpt, 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	// }}}
	// Pinkie {{{

	phidx++;
	visualization_msgs::Marker pinkiebase = genmark(
		quaternion_rotate(q_global2phalanges[0], quaternion_rotate(quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * -55.0), AXIS_X)), 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	phidx++;
	visualization_msgs::Marker pinkiemed = genmark(
		lastendpt, 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	phidx++;
	visualization_msgs::Marker pinkietip = genmark(
		lastendpt, 
		relation_apply(q_global2phalanges[phidx], phmarkdefs[phidx].q_ph2mark),
		phmarkdefs[phidx].length,
		phmarkdefs[phidx].radius,
		*(phmarkdefs[phidx].force),
		&lastendpt
	);

	// }}}

	// Bundle the markers up nicely and publish them to ROS {{{

	palm.id = 0x0;
	thumbbase.id = 0x1;
	thumbtip.id = 0x2;
	indexbase.id = 0x3;
	indexmed.id = 0x4;
	indextip.id = 0x5;
	middlebase.id = 0x6;
	middlemed.id = 0x7;
	middletip.id = 0x8;
	ringbase.id = 0x9;
	ringmed.id = 0xA;
	ringtip.id = 0xB;
	pinkiebase.id = 0xC;
	pinkiemed.id = 0xD;
	pinkietip.id = 0xE;

	visualization_msgs::MarkerArray markarray;
	markarray.markers.resize(15);
	markarray.markers[0x0] = palm;
	markarray.markers[0x1] = thumbbase;
	markarray.markers[0x2] = thumbtip;
	markarray.markers[0x3] = indexbase;
	markarray.markers[0x4] = indexmed;
	markarray.markers[0x5] = indextip;
	markarray.markers[0x6] = middlebase;
	markarray.markers[0x7] = middlemed;
	markarray.markers[0x8] = middletip;
	markarray.markers[0x9] = ringbase;
	markarray.markers[0xA] = ringmed;
	markarray.markers[0xB] = ringtip;
	markarray.markers[0xC] = pinkiebase;
	markarray.markers[0xD] = pinkiemed;
	markarray.markers[0xE] = pinkietip;
	ros_pub_imutracker.publish(markarray);

	ros_pub_testa.publish(quat2posest(q_global2sensors[0]));
	ros_pub_testb.publish(quat2posest(q_global2phalanges[0]));
	ros_pub_testc.publish(quat2posest(QUATERNION_UNITY));

	// }}}
}

// }}}
// ROS CB {{{

void ros_cb_imuctrl (const std_msgs::String::ConstPtr& msg) {
	printf("\n OK %s", msg->data.c_str());

	//quaternion_t q_palm2sensor0 = {vizcoeffw, vizcoeffx, vizcoeffy, vizcoeffz};
	quaternion_t q_palm2sensor0 = quaternion_from_axis_angle(AXIS_Z, DEG_TO_RAD * 135.0);
	ros_pub_testd.publish(quat2posest(q_palm2sensor0));

	// Assume the glove is in the calibration pose
	// First find the glove frame, which is defined with respect to the palm
	quaternion_t q_global2glove = quaternion_inv(relation_unapply(q_palm2sensor0, q_global2sensors[0]));

	// compute the phalanges->sensors relations
	// Note: 
	//   q_phalanges2sensors[0] = q_palm2sensor0;
	//   the above line should be defined to be true
	//   implicitly based on the following code
	for (int idx = 0; idx <= 0xE; idx++) {
		q_phalanges2sensors[idx] = quaternion_inv(
			relation_capture(
				q_global2sensors[idx],
				relation_apply(q_global2glove, Q_GLOVE2CALPOSE[idx])
			)
		);
	}
}

void ros_cb_forcedata (const imutracker::glove msg) {

	// SAVE THIS:
	// It's the mapping used in the tac_glove code.
	// forces[0] = msg.segments[0].force[0];
	// forces[1] = msg.segments[0].force[1];
	// forces[2] = msg.segments[1].force[1];
	// forces[3] = msg.segments[2].force[0];
	// forces[4] = msg.segments[2].force[1];
	// forces[5] = msg.segments[3].force[0];
	// forces[6] = msg.segments[3].force[1];

	forces[0] = msg.segments[0].force[0];
	forces[1] = msg.segments[0].force[1];
	forces[2] = msg.segments[1].force[0];
	forces[3] = msg.segments[1].force[1];
	forces[4] = msg.segments[2].force[0];
	forces[5] = msg.segments[2].force[1];
	forces[6] = msg.segments[3].force[0];
	forces[7] = msg.segments[3].force[1];
}

void ros_cb_vizctrlw (const std_msgs::Float64 msg) {
	vizcoeffw = (double) msg.data;
}

void ros_cb_vizctrlx (const std_msgs::Float64 msg) {
	vizcoeffx = (double) msg.data;
}

void ros_cb_vizctrly (const std_msgs::Float64 msg) {
	vizcoeffy = (double) msg.data;
}

void ros_cb_vizctrlz (const std_msgs::Float64 msg) {
	vizcoeffz = (double) msg.data;
}

void ros_cb_imutracker_raw (const geometry_msgs::PoseArray::ConstPtr& recposearray) {
	for (int idx = 0; idx <= 0xE; idx++) {
		q_global2sensors[idx] = pose2quat(recposearray->poses[idx]);
	}

	fwdkin();
}

// }}}
// Timing handler {{{

void INThandler(int sig)
{
		signal(sig, SIG_IGN);
		exit(0);
}

int mymillis()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
	long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
	result->tv_sec = diff / 1000000;
	result->tv_usec = diff % 1000000;
	return (diff<0);
}

// }}}
// Main {{{

void prepBus() {
	usleep(300000);
}

typedef enum mode {MODE_NORMAL, MODE_LISTEN, MODE_SINGLE} trackermode_t;

int main(int argc, char * argv[])
{
	trackermode_t MODE = MODE_NORMAL;

	if (strcmp(argv[argc-1], "--listen") == 0) {
		MODE = MODE_LISTEN;
		printf("Mode: Listen\n");
	} else if (strcmp(argv[argc-1], "-1") == 0) {
		MODE = MODE_SINGLE;
		printf("Mode: Single-sensor hardware test\n");
	} else {
		printf("Mode: Normal\n");
	}

	// Set up ROS and timing stuff {{{

	ros::init(argc, argv, "imutracker");
	ros::NodeHandle ros_n;

	if (MODE == MODE_NORMAL) {
		ros_pub_imutracker = ros_n.advertise<visualization_msgs::MarkerArray>("tac_glove_imutracker", 1000);
	} else if (MODE == MODE_LISTEN) {
		ros_pub_imutracker = ros_n.advertise<visualization_msgs::MarkerArray>("tac_glove_genmarks", 1000);
	}
	ros_pub_imutracker_rel = ros_n.advertise<geometry_msgs::PoseArray>("tac_glove_imutracker_rel", 1000);
	ros_sub_imuctrl = ros_n.subscribe("tac_glove_imutracker_imuctrl", 1000, ros_cb_imuctrl);
	ros_sub_forcedata = ros_n.subscribe("/glove_sensors", 1000, ros_cb_forcedata);
	ros_sub_vizctrlw = ros_n.subscribe("tac_glove_imutracker_vizctrlw", 1000, ros_cb_vizctrlw);
	ros_sub_vizctrlx = ros_n.subscribe("tac_glove_imutracker_vizctrlx", 1000, ros_cb_vizctrlx);
	ros_sub_vizctrly = ros_n.subscribe("tac_glove_imutracker_vizctrly", 1000, ros_cb_vizctrly);
	ros_sub_vizctrlz = ros_n.subscribe("tac_glove_imutracker_vizctrlz", 1000, ros_cb_vizctrlz);
	ros_pub_vizcoeff = ros_n.advertise<geometry_msgs::PoseStamped>("tac_glove_imutracker_vizcoeff", 1000);
	ros_pub_testa = ros_n.advertise<geometry_msgs::PoseStamped>("tac_glove_imutracker_testa", 1000);
	ros_pub_testb = ros_n.advertise<geometry_msgs::PoseStamped>("tac_glove_imutracker_testb", 1000);
	ros_pub_testc = ros_n.advertise<geometry_msgs::PoseStamped>("tac_glove_imutracker_testc", 1000);
	ros_pub_testd = ros_n.advertise<geometry_msgs::PoseStamped>("tac_glove_imutracker_testd", 1000);

	if (MODE == MODE_NORMAL) {
		ros_pub_imutracker_raw = ros_n.advertise<geometry_msgs::PoseArray>("tac_glove_imutracker_raw", 1000);
	} else if (MODE == MODE_LISTEN) {
		ros_sub_imutracker_raw = ros_n.subscribe("tac_glove_imutracker_raw", 1000, ros_cb_imutracker_raw);
	}

	signal(SIGINT, INThandler);

	int startInt  = mymillis();
	struct  timeval tvBegin, tvEnd,tvDiff;

	// }}}
	// Set up IMUs {{{
	tcabno_t imus[16];
	if (MODE == MODE_NORMAL) {
		imus[0x0] = tcabno_init(1, 0x70, 0, 0x29);
		imus[0x1] = tcabno_init(1, 0x70, 1, 0x29);
		imus[0x2] = tcabno_init(1, 0x70, 2, 0x29);
		imus[0x3] = tcabno_init(1, 0x70, 3, 0x29);
		imus[0x4] = tcabno_init(1, 0x70, 4, 0x29);
		imus[0x5] = tcabno_init(1, 0x70, 5, 0x29);
		imus[0x6] = tcabno_init(1, 0x70, 6, 0x29);
		imus[0x7] = tcabno_init(1, 0x70, 7, 0x29);
		imus[0x8] = tcabno_init(0, 0x70, 0, 0x29);
		imus[0x9] = tcabno_init(0, 0x70, 1, 0x29);
		imus[0xA] = tcabno_init(0, 0x70, 2, 0x29);
		imus[0xB] = tcabno_init(0, 0x70, 3, 0x29);
		imus[0xC] = tcabno_init(0, 0x70, 4, 0x29);
		imus[0xD] = tcabno_init(0, 0x70, 5, 0x29);
		imus[0xE] = tcabno_init(0, 0x70, 6, 0x29);

		printf("Collecting data as of... ");
		fflush(stdout);
		prepBus();
		printf("now.");
		fflush(stdout);
	}
	bno_t imu = bno_init(1, 0x29);
	std::ofstream ofs;
	if (MODE == MODE_SINGLE) {
		// imus[0] = tcabno_init(1, 0x70, 0, 0x29);
		// quaternion_t sample = tcabno_safesample(imus[0]);
		// ros_pub_testa.publish(quat2posest(sample));
		ofs.open("imu_RPY_360.txt");
		quaternion_t sample = bno_sample(imu);
		ros_pub_testa.publish(quat2posest(sample));
	}

	// }}}


	gettimeofday(&tvBegin, NULL);
	int count = 0;

	while(ros::ok())
	{
		if (count < 150) {
			count++;
		}
		startInt = mymillis();



		if (MODE == MODE_NORMAL) {
			for (int idx = 0; idx <= 0xE; idx++) {
				q_global2sensors[idx] = tcabno_safesample(imus[idx]);
			};

			fwdkin();

			// Save some additional information for archival purposes {{{

			geometry_msgs::PoseArray rawposearray;
			rawposearray.poses.resize(15);
			for (int idx = 0; idx <= 0xE; idx++) {
				rawposearray.poses[idx] = quat2posest(q_global2sensors[idx]).pose;
			}
			ros_pub_imutracker_raw.publish(rawposearray);

			geometry_msgs::PoseArray relposearray;
			relposearray.poses.resize(15);
			for (int idx = 0; idx <= 0xE; idx++) {
				relposearray.poses[idx] = quat2posest(q_global2phalanges[idx]).pose;
			}
			ros_pub_imutracker_rel.publish(relposearray);

			// }}}
		}
		else if(MODE == MODE_SINGLE){
			quaternion_t sample = bno_sample(imu);
			printf("%f %f %f %f\n",sample.x,sample.y,sample.z,sample.w);
			//ofs<<sample.x<<","<<sample.y<<","<<sample.z<<","<<sample.w<<"\n";
                	ros_pub_testa.publish(quat2posest(sample));
		}


		//Each loop should be at least DT seconds
		while(mymillis() - startInt < (DT*1000))
		{
			usleep(100);
		}

		ros::spinOnce();
	}
	ofs.close();
}

// }}}
