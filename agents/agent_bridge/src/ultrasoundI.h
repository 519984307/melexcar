/*
 *    Copyright (C) 2022 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef ULTRASOUND_H
#define ULTRASOUND_H

// Ice includes
#include <Ice/Ice.h>
#include <Ultrasound.h>

#include <config.h>
#include "genericworker.h"


class UltrasoundI : public virtual RoboCompUltrasound::Ultrasound
{
public:
	UltrasoundI(GenericWorker *_worker);
	~UltrasoundI();

	RoboCompUltrasound::SensorsState getAllSensorDistances(const Ice::Current&);
	RoboCompUltrasound::SensorParamsList getAllSensorParams(const Ice::Current&);
	RoboCompUltrasound::SonarPoseList getAllSonarPose(const Ice::Current&);
	RoboCompUltrasound::BusParams getBusParams(const Ice::Current&);
	int getSensorDistance(std::string sensor, const Ice::Current&);
	RoboCompUltrasound::SensorParams getSensorParams(std::string sensor, const Ice::Current&);
	int getSonarsNumber(const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
