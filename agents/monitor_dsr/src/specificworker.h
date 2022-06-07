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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include "/home/pioneernuc/robocomp/components/melexcar/etc/pioneer_world_names.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void DifferentialRobot_correctOdometer(int x, int z, float alpha);
	void DifferentialRobot_getBasePose(int &x, int &z, float &alpha);
	void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void DifferentialRobot_resetOdometer();
	void DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void DifferentialRobot_setOdometerPose(int x, int z, float alpha);
	void DifferentialRobot_setSpeedBase(float adv, float rot);
	void DifferentialRobot_stopBase();

	void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

    RoboCompJoystickAdapter::TData joydata;
    float adv_speed , rot_speed, adv_speed_filter , rot_speed_filter ;
    int brake_speed = 1500;
    void bumper_robot();
    bool active_laser_front;

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
    void set_car_movement(float adv, float rot, int brake);
    void check_gps_connection();

private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
    bool gps_check = false;
    float last_mapx = 0.0;
    float last_mapy = 0.0;
    float max_lat = 39.4810;
    float min_lat = 39.4779;
    float max_long = -6.3385;
    float min_long = -6.3453;
    bool bumper;
    int stop_threshold;
    int first_threshold;
    int second_threshold;
    int trim;
    float first_threshold_velocity;
    float second_threshold_velocity;
	//DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};

	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};     
	bool startup_check_flag;

};

#endif
