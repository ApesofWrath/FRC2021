#pragma once

#include "DriveBase.h"

class DriveController : public DriveBase {
public:

	//24, 6, 7, 0, 21, 5, 8, 0, 0, 0, 0, false - comp bot //TODO: ifdef for comp/practice bot

	//l1, l2, l3, l4, r1, r2, r3, r4, pcm, f_channel, r_channel, two_speed
	//left right from robot perspective
	DriveController() : DriveBase(10,12,11,13,0,0,0,false) {

 
	}

};
