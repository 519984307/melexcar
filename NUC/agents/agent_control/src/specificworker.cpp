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
#include "specificworker.h"
#include "cppitertools/zip.hpp"
#include <cppitertools/range.hpp>
#include "cppitertools/sliding_window.hpp"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);

	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
        rt = G->get_rt_api();
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
//		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
//		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
//		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);


		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    //main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}

        main = opts::graph;

        graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        // custom widget
        graph_viewer->add_custom_widget_to_dock("Melex's control", &custom_widget); //una pestaña
        local_view = new AbstractGraphicViewer(custom_widget.laser, QRectF(-5000, -5000, 10000, 10000));

        //axis
        QLineF x_axis(0, 0, 0, 2000);
        QLineF y_axis(0, 0, 1000, 0);
        local_view->scene.addLine(x_axis, QPen(QColor("Red"), 30));
        local_view->scene.addLine(y_axis, QPen(QColor("Green"), 30));
        // Laser integrator polygon
        QPolygonF laser_integrator;
        static QGraphicsItem *laser_polygon = nullptr;


        double ra = 2000.0;
        double rad_pas = (M_PI/180.0);
        cout << "RAD_PAS " << rad_pas << endl;

        for (int i=-180; i<=180; i++)
        {
            double x = ra * sin(rad_pas*i);
            double y = ra * cos(rad_pas*i);
            laser_map.insert(pair<double, double>(i, 10000.0));
        }

        //laser_polygon = local_view->scene.addPolygon(laser_integrator, QPen(QColor("DarkGreen"), 30));

        //robot
        float ROBOT_LENGTH =  2000;
        float ROBOT_WIDTH = 1000;

        //melex's image
        QImage icon("/home/pioneernuc/robocomp/components/melexcar/images/iconmelex.png");
        auto icon2 = icon.scaled(ROBOT_WIDTH *2.0,ROBOT_LENGTH*2.0, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        auto icon3 = QPixmap::fromImage(icon2, Qt::AutoColor);
        QTransform myTransform;
        myTransform.rotate(180);
        auto icon4 = icon3.transformed(myTransform);
        local_view->scene.addPixmap(icon4)->setPos(-icon2.size().width()/2.0,-icon2.size().height()/2.0);

        //points around melex's center
        float r = 1000.0;
        for (int i=1; i<30; i++)
        {
            //auto factor = exp(0.1*i)*i;
            auto dist = i * r;
            std::string str1 = std::to_string(dist);
            std::string str = "Radius: ";
            std::string com = str +str1;
            QString comp = QString::fromUtf8(com.c_str());
            auto cir = new QGraphicsEllipseItem(-dist, -dist, 2 * dist, 2 * dist);
            cir->setPen(QPen(QColor("DarkBlue"), 10));
            local_view->scene.addItem(cir);
            //QGraphicsTextItem *t = cir->scene()->addText("Radius:",QFont("Times", 200, QFont::Bold));
            QFont pr = QFont("Monospace", 100, 100);

            QGraphicsSimpleTextItem *t = local_view->scene.addSimpleText(comp,pr);

            t->setX(dist);


        }

        //webengine
        map= new LeafLetGPSViewer(G.get());
        //auto h = custom_widget.webengine->height();
        //auto w = custom_widget.webengine->width();
        //map->resize(w,h);
        map->adjustSize();
        //map->sizeIncrement();
        map->setParent(custom_widget.webengine);
        size = custom_widget.webengine->size();
        //cout << "SIZE1" << size.width() << "height"<< size.height();
        map->setFixedSize(size);



        this->Period = period;
		timer.start(Period);
        try {

            if (auto laser_front_node = G->get_node(laser_front_name); laser_front_node.has_value())
                front_laser_id = laser_front_node->id();
            if (auto laser_back_node = G->get_node(laser_back_name); laser_back_node.has_value())
                back_laser_id = laser_back_node->id();
            if (auto laser_right_node = G->get_node(laser_right_name); laser_right_node.has_value())
                right_laser_id = laser_right_node->id();
            if (auto laser_left_node = G->get_node(laser_left_name); laser_left_node.has_value())
                left_laser_id = laser_left_node->id();
        }
        catch(const std::exception &e) { qFatal("Laser nodes Missing at initialize"); }


}}

void SpecificWorker::compute()
{
    QPolygonF laserpoly;
    QPolygonF lasercomplete;
    QPolygonF ultrasound_left_front;
    QPolygonF ultrasound_right_front;
    QPolygonF ultrasound_right_side;
    QPolygonF ultrasound_left_side;
    std::vector<QPolygonF> us_polys;
    read_ultrasounds(distances_name, us_polys);

    //read_ultrasound("Ultra_left_front", ultrasound_left_front);
    //read_ultrasound("Ultra_right_front", ultrasound_right_front);
    //read_ultrasound("Ultra_left_side", ultrasound_left_side);
    //read_ultrasound("Ultra_right_side", ultrasound_right_side);
    read_laser(front_laser_id, laser_front_name, laserpoly); // 0-179
    read_laser(right_laser_id, laser_right_name, laserpoly); // 0-86
    read_laser(back_laser_id, laser_back_name, laserpoly); // 0 - 86
    read_laser(left_laser_id, laser_left_name, laserpoly); // 0 - 86
    //us_poly = {ultrasound_right_side, ultrasound_left_side, ultrasound_right_front, ultrasound_left_front};
    laser_integrator(laserpoly, lasercomplete);
    draw_laser(lasercomplete);
    draw_us(us_polys);
    //draw_us(ultrasound_left_front);
    //draw_us(ultrasound_left_side);
    //draw_us(ultrasound_right_front);

    read_battery();
    read_cords();
    read_odometry();
    update_robot_localization_gps();
    //cout << "SIZE1" << size.width() << "height"<< size.height();
    if (size != custom_widget.webengine->size()){
        map->setFixedSize(custom_widget.webengine->size());
        size = custom_widget.webengine->size();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::update_robot_localization_gps()
{
    //static RoboCompFullPoseEstimation::FullPoseEuler last_state;
    //RoboCompFullPoseEstimation::FullPoseEuler pose;
    float rot,mapx,mapy;
    //RoboCompGpsUblox::DatosGPS map;
    try
    {
        //pose = fullposeestimation_proxy->getFullPoseEuler();
        //map = gpsublox_proxy->getData();
        if( auto robot = G->get_node(robot_name); robot.has_value())
        {
            if (auto gps = G->get_node(gps_name); gps.has_value())
            {
                mapx = G->get_attrib_by_name<gps_map_x_att>(gps.value()).value();
                mapy = G->get_attrib_by_name<gps_map_y_att>(gps.value()).value();
                rot = G->get_attrib_by_name<gps_rot_att>(gps.value()).value();
                cout<< " rot_nodo: "<< rot <<endl;
                //        qInfo() << __FUNCTION__ << " mapx" << map.mapx;
                //        qInfo() << __FUNCTION__ << " mapY" << map.mapy;
                //qInfo() << "X:" << pose.x  << "// Y:" << pose.y << "// Z:" << pose.z << "// RX:" << pose.rx << "// RY:" << pose.ry << "// RZ:" << pose.rz;
            }
        }

    }
    catch(const Ice::Exception &e){ std::cout << e.what() <<  __FUNCTION__ << std::endl;};

    if( auto robot = G->get_node(robot_name); robot.has_value())
    {
        if( auto parent = G->get_parent_node(robot.value()); parent.has_value())
        {
            if (are_different(std::vector < float > {mapx, mapy},
                              std::vector < float > {last_mapx, last_mapy},
                              std::vector < float > {1, 1, 0.05}))
            {
                auto edge = rt->get_edge_RT(parent.value(), robot->id()).value();
                G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector < float > {0.0, 0.0, rot});
                G->modify_attrib_local<rt_translation_att>(edge, std::vector < float > {mapx, mapy, 1800});
                //G->modify_attrib_local<rt_translation_velocity_att>(edge, std::vector<float>{pose.vx, pose.vy, pose.vz});
                //G->modify_attrib_local<rt_rotation_euler_xyz_velocity_att>(edge, std::vector<float>{pose.vrx, pose.vry, pose.vrz});
                // linear velocities are WRT world axes, so local speed has to be computed WRT to the robot's moving frame
                //float side_velocity = -sin(pose.rz) * pose.vx + cos(pose.rz) * pose.vy;
                //float adv_velocity = -cos(pose.rz) * pose.vx + sin(pose.rz) * pose.vy;
                G->insert_or_assign_edge(edge);
                //G->add_or_modify_attrib_local<robot_local_linear_velocity_att>(robot.value(), std::vector<float>{adv_velocity, side_velocity, pose.rz});
                G->update_node(robot.value());
                //last_state = pose;
                last_mapx = mapx;
                last_mapy = mapy;
            }
        }
        else  qWarning() << __FUNCTION__ << " No parent found for node " << QString::fromStdString(robot_name);
    }
    else    qWarning() << __FUNCTION__ << " No node " << QString::fromStdString(robot_name);
}
void SpecificWorker::read_laser(std::uint64_t id, const std::string laser_name, QPolygonF &laserpoly)
{
    if(auto laser_node = G->get_node(id); laser_node.has_value())
    {
        auto angles = G->get_attrib_by_name<laser_angles_att>(laser_node.value());
        auto dists = G->get_attrib_by_name<laser_dists_att>(laser_node.value());
        if (dists.has_value() and angles.has_value())
        {
            const auto &d = dists.value().get();
            const auto &a = angles.value().get();
            if (d.empty() or a.empty()) return;
                 QPolygonF laser_poly;
                 auto inner = G->get_inner_eigen_api();
                 for (const auto &[a, d]: iter::zip(a, d)) {

                     //convert laser polar coordinates to cartesian
                     if (d == 0) continue;
                     float x = d * sin(a);
                     float y = d * cos(a);
                     if (auto aux = inner->transform(robot_name, Eigen::Vector3d(x, y, 0.f),
                                                     laser_name); aux.has_value()){
                         laserpoly << QPointF(aux.value().x(), aux.value().y());
//                         cout << "ANGULO" << ang << endl;
                 }
                 }

                 //laserpoly << QPointF(0, 0);
        }
    }
}
void SpecificWorker::read_ultrasound(const std::string ultrasound_name, QPolygonF &us_poly)
{
    if(auto us_node = G->get_node(ultrasound_name); us_node.has_value())
    {
        auto dist = G->get_attrib_by_name<ultrasound_distance_att>(us_node.value());
        if (dist.has_value())
        {
            const auto &d = dist.value();

            QPolygonF laser_poly;
            auto inner = G->get_inner_eigen_api();
            auto aux3 = inner -> transform(robot_name, Eigen::Vector3d(0.f, 0.f, 0.f), ultrasound_name);
            us_poly << QPointF(aux3.value().x(), aux3.value().y());

            float x_1 = d * sin((-5) * (M_PI/180)), y_1 = d * cos((-5)* (M_PI/180));
            auto aux1 = inner->transform(robot_name, Eigen::Vector3d(x_1, y_1, 0.f), ultrasound_name);
            us_poly << QPointF(aux1.value().x(), aux1.value().y());
            //us_poly << QPointF(aux1.value().x(), aux1.value().y());

            float x_2 = d * sin((5) * (M_PI/180)), y_2 = d * cos((5) * (M_PI/180) );
            auto aux2 = inner->transform(robot_name, Eigen::Vector3d(x_2, y_2, 0.f), ultrasound_name);
            us_poly << QPointF(aux2.value().x(), aux2.value().y());


            //us_poly << QPointF(0, 0);
        }
    }
}

void SpecificWorker::read_ultrasounds(const std::vector<std::string> distances_name, std::vector<QPolygonF> &sensor_poly)
{
    for (std::string sensor_name : distances_name ){
        QPolygonF us_poly;
        if(auto us_node = G->get_node(sensor_name); us_node.has_value())
        {
            auto dist = G->get_attrib_by_name<ultrasound_distance_att>(us_node.value());
            if (dist.has_value())
            {
                const auto &d = dist.value();

                auto inner = G->get_inner_eigen_api();
                auto aux3 = inner -> transform(robot_name, Eigen::Vector3d(0.f, 0.f, 0.f), sensor_name);
                us_poly << QPointF(aux3.value().x(), aux3.value().y());

                float x_1 = d * sin((-5) * (M_PI/180)), y_1 = d * cos((-5)* (M_PI/180));
                auto aux1 = inner->transform(robot_name, Eigen::Vector3d(x_1, y_1, 0.f), sensor_name);
                us_poly << QPointF(aux1.value().x(), aux1.value().y());
                //us_poly << QPointF(aux1.value().x(), aux1.value().y());

                float x_2 = d * sin((5) * (M_PI/180)), y_2 = d * cos((5) * (M_PI/180) );
                auto aux2 = inner->transform(robot_name, Eigen::Vector3d(x_2, y_2, 0.f), sensor_name);
                us_poly << QPointF(aux2.value().x(), aux2.value().y());

                sensor_poly.push_back(us_poly);
                //us_poly << QPointF(0, 0);
            }
        }
    }

}

void SpecificWorker::draw_laser(QPolygonF &laserpoly)
{
    static QGraphicsItem *laser_polygon = nullptr;

    if (laser_polygon != nullptr)
        local_view->scene.removeItem(laser_polygon);

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = local_view->scene.addPolygon(laserpoly, QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

void SpecificWorker::draw_us(std::vector<QPolygonF> us_poly)
{
    //static QGraphicsItem *us_polygon = nullptr;
    static std::vector<QGraphicsItem *> us_polygons;
    for (QGraphicsItem* item : us_polygons)
    {
        local_view->scene.removeItem(item);
        delete item;
    }
    us_polygons.clear();

    for (QPolygonF ultra : us_poly ){
        QColor color("red");
        color.setAlpha(40);
        us_polygons.push_back(local_view->scene.addPolygon(ultra, QPen(QColor("red"), 30), QBrush(color)));
        us_polygons.back() ->setZValue(3);
    }

    //if (us_polygon != nullptr)
      //  cout << "BORRA" << endl;
        //local_view->scene.removeItem(us_polygon);

    //QColor color("red");
    //color.setAlpha(40);
    //us_polygon = local_view->scene.addPolygon(us_poly, QPen(QColor("red"), 30), QBrush(color));
    //us_polygon->setZValue(3);
}

void SpecificWorker::laser_integrator(QPolygonF laserpoly, QPolygonF &lasercomplete)
{
    std::set<std::pair<double,double>> hor_ang;   // lambda to sort elements on insertion
    for (int i = 0; i<laserpoly.size(); i++) {
        double ang = atan2(laserpoly[i].x(), laserpoly[i].y());
        double dist = sqrt((pow(laserpoly[i].x(),2 ) + pow(laserpoly[i].y(),2 )));
        hor_ang.insert(pair<double, double>  (ang, dist));
    }
    int last_ang = 1000;
    double last_distance = 100000.0;
    for (std::set<std::pair<double, double>>::iterator it=hor_ang.begin(); it!=hor_ang.end(); ++it) {
        auto angulo = (int)(round)((it->first *180) /M_PI);
        try{
            if (angulo == last_ang) last_distance = min(it->second, last_distance);
            else last_distance = it->second;
            laser_map.at(angulo) = last_distance;
            last_ang = angulo;
        }catch(const std::exception &e) { cout << "bad insert"<< endl; }

    }
    for (auto &bin : laser_map) {
        auto angulo_norm = (bin.first );
        float x = bin.second * sin((angulo_norm*M_PI)/180 );
        float y = bin.second * cos((angulo_norm*M_PI)/180 );
        lasercomplete << QPointF(x, y);
    }



}

void SpecificWorker::read_battery()
{
    if(auto battery_node = G->get_node(battery_name); battery_node.has_value())
    {
        auto load = G->get_attrib_by_name<battery_load_att>(battery_node.value());
        custom_widget.lcdNumber->display(load.value());
    }
}

void SpecificWorker::read_odometry()
{
    if(auto odometry_node = G->get_node(odometry_name); odometry_node.has_value())
    {
        auto vel = G->get_attrib_by_name<odometry_vel_att>(odometry_node.value());
        auto ang = G->get_attrib_by_name<odometry_steer_att>(odometry_node.value());
        custom_widget.speed->display(vel.value());
        custom_widget.wheel_angle->display(ang.value());
    }
}


void SpecificWorker::read_cords()
{
    if(auto gps_node = G->get_node(gps_name); gps_node.has_value())
    {
        auto lat = G->get_attrib_by_name<gps_latitude_att>(gps_node.value());
        auto lon = G->get_attrib_by_name<gps_longitude_att>(gps_node.value());

        custom_widget.latitud_n->display(lat.value());
        custom_widget.longitud_n->display(lon.value());
    }
}
/////////////// AUX //////////////
bool SpecificWorker::are_different(const std::vector<float> &a, const std::vector<float> &b, const std::vector<float> &epsilon)
{
    for(auto &&[aa, bb, e] : iter::zip(a, b, epsilon))
        if (fabs(aa - bb) > e)
            return true;
    return false;
};
///////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////




