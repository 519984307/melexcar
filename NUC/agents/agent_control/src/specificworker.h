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
#include "ui_localUI.h"
#include <custom_widget.h>
#include </opt/robocomp/classes/abstract_graphic_viewer/abstract_graphic_viewer.h>
#include  "../../../etc/pioneer_world_names.h"
#include <QGraphicsPolygonItem>
#include <ranges>
#include <QPixmap>
#include <QTransform>
#include <LeafletViewer.h>


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
    private:
        // DSR graph
        std::shared_ptr<DSR::DSRGraph> G;
        std::unique_ptr<DSR::RT_API> rt;

        //DSR params
        std::string agent_name;
        int agent_id;

        bool tree_view;
        bool graph_view;
        bool qscene_2d_view;
        bool osg_3d_view;
        QSize size;
        float last_mapx = 0.0;
        float last_mapy = 0.0;
        std::vector<std::string> distances_name = {"Ultra_left_side", "Ultra_left_front", "Ultra_right_front", "Ultra_right_side", "Lidar_front", "Lidar_right_front", "Lidar_left_front"};
        LeafLetGPSViewer *map;
        std::uint64_t front_laser_id, right_laser_id, back_laser_id, left_laser_id;


        // laser
        using LaserData = std::tuple<std::vector<float>, std::vector<float>>;  //<angles, dists>
        DoubleBuffer<LaserData, QPolygonF> laser_buffer;

        //custom
        Custom_widget custom_widget;
        AbstractGraphicViewer *local_view;

        // DSR graph viewer
        std::unique_ptr<DSR::DSRViewer> graph_viewer;

//        void modify_node_slot(std::uint64_t, const std::string &type);
//        void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
//        void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
//        void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
//        void del_node_slot(std::uint64_t from){};

        void read_laser(std::uint64_t id, const std::string laser_name, QPolygonF &poly);
        void read_ultrasound(std::string laser_name, QPolygonF &us_poly);
        void read_ultrasounds(std::vector<std::string> sensors_names, std::vector<QPolygonF> &us_polys);
        inline QPointF e2q(const Eigen::Vector3d &p)const {
            return QPointF(p.x(), p.y());
        }
        void update_robot_localization_gps();
        void draw_laser(QPolygonF &poly);
        void draw_us(std::vector<QPolygonF> poly);

        void read_battery();
        void read_cords();
        void read_odometry();
        void laser_integrator(QPolygonF poly, QPolygonF &poly_complete);
        bool are_different(const vector<float> &a, const vector<float> &b, const vector<float> &epsilon);

    std::map<double, double> laser_map;





    bool startup_check_flag;

};

#endif
