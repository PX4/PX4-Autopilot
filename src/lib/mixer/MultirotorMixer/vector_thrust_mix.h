/**
 * @file vector_thrust_mix.h
 *
 * File for defining vector thrust mixer.
 */

#include <cstdio>
#include <float.h>

struct VectorThrustMix {
	float	vtx_scale;	/**< scales vector thrust in x (north) direction for this rotor */
	float	vty_scale;	/**< scales vector thrust in y (east) direction for this rotor */
};

//define columns of mixing matrix here
//      { SCALE X,  SCALE Y}
constexpr VectorThrustMix multirotor_vt_config[] = {   //add a line for each motor indicating scaling for each direction
	{ -0.49636,  0.50000},	//Motor 1
	{ -0.49636, -0.50000},	//Motor 2
	{  0.49636, -0.50000},	//Motor 3
	{  0.49636,  0.50000},	//Motor 4
	{  0.50000,  0.49636},	//Motor 5
	{  0.50000, -0.49636},	//Motor 6
	{ -0.50000, -0.49636},	//Motor 7
	{ -0.50000,  0.49636},	//Motor 8
}; //See Airframes Reference for motor numbering -> Octorotor Coaxial

// EXAMPLE FOR COAXIAL OCTO WITH TILTED PROPELLERS
// { -0.1767,  0.1767},	//Motor 1
// { -0.1767, -0.1767},	//Motor 2
// {  0.1767, -0.1767},	//Motor 3
// {  0.1767,  0.1767},	//Motor 4
// {  0.1767,  0.1767},	//Motor 5
// {  0.1767, -0.1767},	//Motor 6
// { -0.1767, -0.1767},	//Motor 7
// { -0.1767,  0.1767},	//Motor 8
