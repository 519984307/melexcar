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
#include "ultrasoundI.h"

UltrasoundI::UltrasoundI(GenericWorker *_worker)
{
	worker = _worker;
}


UltrasoundI::~UltrasoundI()
{
}


RoboCompUltrasound::SensorsState UltrasoundI::getAllSensorDistances(const Ice::Current&)
{
	return worker->Ultrasound_getAllSensorDistances();
}

RoboCompUltrasound::SensorParamsList UltrasoundI::getAllSensorParams(const Ice::Current&)
{
	return worker->Ultrasound_getAllSensorParams();
}

RoboCompUltrasound::SonarPoseList UltrasoundI::getAllSonarPose(const Ice::Current&)
{
	return worker->Ultrasound_getAllSonarPose();
}

RoboCompUltrasound::BusParams UltrasoundI::getBusParams(const Ice::Current&)
{
	return worker->Ultrasound_getBusParams();
}

int UltrasoundI::getSensorDistance(std::string sensor, const Ice::Current&)
{
	return worker->Ultrasound_getSensorDistance(sensor);
}

RoboCompUltrasound::SensorParams UltrasoundI::getSensorParams(std::string sensor, const Ice::Current&)
{
	return worker->Ultrasound_getSensorParams(sensor);
}

int UltrasoundI::getSonarsNumber(const Ice::Current&)
{
	return worker->Ultrasound_getSonarsNumber();
}

