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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
#include <CommonBehavior.h>

#include <BatteryStatus.h>
#include <CameraRGBDSimple.h>
#include <CameraSimple.h>
#include <FullPoseEstimation.h>
#include <FullPoseEstimationPub.h>
#include <GenericBase.h>
#include <GpsUblox.h>
#include <HumanCameraBody.h>
#include <Laser.h>
#include <OdometryMelex.h>
#include <Radar.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100


using TuplePrx = std::tuple<RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr,RoboCompHumanCameraBody::HumanCameraBodyPrxPtr,RoboCompLaser::LaserPrxPtr>;


class GenericWorker : public QMainWindow, public Ui_guiDlg
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	RoboCompCameraRGBDSimple::CameraRGBDSimplePrxPtr camerargbdsimple_proxy;
	RoboCompHumanCameraBody::HumanCameraBodyPrxPtr humancamerabody_proxy;
	RoboCompLaser::LaserPrxPtr laser_proxy;

	virtual RoboCompBatteryStatus::TBattery BatteryStatus_getBatteryState() = 0;
	virtual RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera) = 0;
	virtual RoboCompCameraSimple::TImage CameraSimple_getImage() = 0;
	virtual void FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose) = 0;
	virtual RoboCompGpsUblox::DatosGPS GpsUblox_getData() = 0;
	virtual void GpsUblox_setInitialPose(float x, float y) = 0;
	virtual RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState) = 0;
	virtual RoboCompLaser::LaserConfData Laser_getLaserConfData() = 0;
	virtual RoboCompLaser::TLaserData Laser_getLaserData() = 0;
	virtual RoboCompOdometryMelex::TOdometry OdometryMelex_getOdometryState() = 0;
	virtual RoboCompRadar::RadarData Radar_getData() = 0;

protected:

	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
};

#endif
