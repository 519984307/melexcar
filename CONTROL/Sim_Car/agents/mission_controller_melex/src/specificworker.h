/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include <custom_widget.h>
#include  "../../../etc/pioneer_world_names.h"
#include <opencv2/opencv.hpp>
#include "/home/robocomp/robocomp/components/robocomp-pioneer/etc/plan.h"
#include <QListWidget>
#include <QSpinBox>
#include "ui_mission_pointUI.h"
#include "ui_mission_pathfollowUI.h"
#include "/home/robocomp/robocomp/classes/abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <string>
#include <fstream>
#include <vector>
#include <utility>
#include <stdexcept>
#include <sstream>

using namespace std::chrono_literals;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
    void slot_start_mission();
    void slot_stop_mission();
    void slot_cancel_mission();
    void slot_change_destination_selector(int);
    void slot_change_pickup_selector(int);

private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
    //std::shared_ptr<DSR::CameraAPI> cam_api;
    std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
    //std::shared_ptr<DSR::RT_API> rt_api;

	//DSR params
	std::string agent_name;
    std::string route_filename;

	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;

	void add_or_assign_node_slot(std::uint64_t, const std::string &type);
	bool startup_check_flag;

    // local widgets
    DSR::QScene2dViewer* widget_2d;
    Custom_widget custom_widget;
    Ui_Goto_UI point_dialog;
    Ui_PathFollow_UI pathfollow_dialog;

    // Laser
    using LaserData = std::tuple<std::vector<float>, std::vector<float>>;  //<angles, dists>
    DoubleBuffer<LaserData, std::tuple<std::vector<float>, std::vector<float>, QPolygonF, std::vector<QPointF>>> laser_buffer;

    // Robot and shape
    QPolygonF robot_polygon;
    void send_command_to_robot(const std::tuple<float, float, float> &speeds);   //adv, rot, side
    Eigen::Vector3d robot_pos;
    // Camera
    DoubleBuffer<std::vector<std::uint8_t>, std::vector<std::uint8_t>> virtual_camera_buffer;

    // Missions
    DoubleBuffer<Plan, Plan> plan_buffer;
    Plan temporary_plan;
    Plan current_plan;
    int stop_1, stop_2, stop_3, stop_4;
    vector<float> xpts, ypts;
    std::vector<std::pair<float, float>> task_pickup_point, task_destination_point;
    QGraphicsEllipseItem *target_pickup_scene;
    void insert_intention_node(const Plan &plan);
    void insert_pickup_node(const Plan &plan, std::vector<std::pair<float, float>> task_pickup_point );
    void insert_destination_coords(const Plan &plan, std::vector<std::pair<float, float>> task_destination_point);
    void check_task_completed();

    //Path
    std::vector<Eigen::Vector2f> path;  // check if can be made local
    //QPointF last_point;
    std::vector<QGraphicsLineItem *> lines;
    //DoubleBuffer<std::vector<Eigen::Vector3d>,std::vector<Eigen::Vector3d>> path_buffer;
    void draw_path(std::vector<Eigen::Vector2f> &path, QGraphicsScene* viewer_2d, bool remove = false);
    void follow_path_copy_path_to_graph(const std::vector<float> &x_values, const std::vector<float> &y_values);
    void  load_path(string filename);
    //std::string selected_route = "";
};

#endif
