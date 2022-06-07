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
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		//connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
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

            if (auto laser_node = G->get_node("laser_nivel_0"); laser_node.has_value())
                laser1_id = laser_node->id();
            if (auto laser_node = G->get_node("laser_nivel_1"); laser_node.has_value())
                laser2_id = laser_node->id();
            if (auto laser_node = G->get_node("laser_nivel_2"); laser_node.has_value())
                laser3_id = laser_node->id();
            if (auto laser_node = G->get_node("laser_nivel_3"); laser_node.has_value())
                laser4_id = laser_node->id();
        }
        catch(const std::exception &e) { qFatal("Laser nodes Missing at initialize"); }


}}

void SpecificWorker::compute()
{
    QPolygonF laserpoly,laserpoly1,laserpoly2,laserpoly3;
    QPolygonF lasercomplete;

    read_laser(laser1_id, laser_front_name, laserpoly); // 0-179
    read_laser(laser2_id, laser_right_name, laserpoly1); // 0-86
    read_laser(laser3_id, laser_back_name, laserpoly2); // 0 - 86
    read_laser(laser4_id, laser_left_name, laserpoly3); // 0 - 86
    QStringView Green = QStringView(QString("LightGreen"));
    QStringView Red = QStringView(QString("Red"));
    QStringView Blue = QStringView(QString("Blue"));
    QStringView Yellow = QStringView(QString("Yellow"));
    //laser_integrator(laserpoly, lasercomplete);
    draw_laser(laserpoly,Green);
    draw_laser(laserpoly1,Red);
    draw_laser(laserpoly2,Blue);
    draw_laser(laserpoly3,Yellow);

    //read_battery();
    //read_cords();
    //read_odometry();
    //cout << "SIZE1" << size.width() << "height"<< size.height();
    if (size != custom_widget.webengine->size()){
        map->setFixedSize(custom_widget.webengine->size());
        size = custom_widget.webengine->size();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////

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

void SpecificWorker::draw_laser(QPolygonF &laserpoly, QStringView colour)
{
    static QGraphicsItem *laser_polygon = nullptr;
    QColor color(colour);
    color.setAlpha(40);
    laser_polygon = local_view->scene.addPolygon(laserpoly, QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
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

///////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////




