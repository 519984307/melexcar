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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }





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
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    check_gps_connection();
    try{
        set_car_movement();
//        std::cout << "SetSpeed_OK" << "adv="<<adv_speed<<" rot="<<rot_speed<<std::endl;
    }catch(const Ice::Exception &e) { std::cout << "SetSpeed_OFF" << std::endl;}


}

void SpecificWorker::set_car_movement()
{
    if(auto robot_node = G->get_node(robot_name); robot_node.has_value())
    {
        //int x = system("ping -c1 -s1 192.168.50.22  > /dev/null 2>&1");
        //if ((x!=0) or (gps_check == false )){
        if ((gps_check == false )){
            cout<<"ping failed or wrong gps dignal"<<endl;
            adv_speed = 0;
            rot_speed = 0;
            G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(), (float) 0);
            G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), (float) 0);
            G->update_node(robot_node.value());
        }else {
            cout << "avanceeeeee" << adv_speed << endl;
            G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(), (float) adv_speed);
            G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), (float) rot_speed);
            G->update_node(robot_node.value());
        }
    }
    else qWarning() << __FUNCTION__ << "No robot node found";

}

void SpecificWorker::check_gps_connection()
{
    if (auto robot_node = G->get_node(robot_name); robot_node.has_value())
    {
        if (auto gps_node = G->get_node(gps_name); gps_node.has_value())
        {
            gps_check = G->get_attrib_by_name<gps_connected_att>(gps_node.value()).value();
            if (auto odo_node = G->get_node(odometry_name); odo_node.has_value())
                auto vel = G->get_attrib_by_name<odometry_vel_att>(odo_node.value()).value();
            if (gps_check)
            {
                auto latitude = G->get_attrib_by_name<gps_latitude_att>(gps_node.value()).value();
                auto longitude = G->get_attrib_by_name<gps_longitude_att>(gps_node.value()).value();
                auto mapx = G->get_attrib_by_name<gps_map_x_att>(gps_node.value()).value();
                auto mapy = G->get_attrib_by_name<gps_map_y_att>(gps_node.value()).value();
                auto diff_mapx = abs(mapx - last_mapx);
                auto diff_mapy = abs(mapy - last_mapy);
                if (latitude < max_lat and latitude > min_lat and longitude > min_long and longitude < max_long and abs(diff_mapx) < 35 and abs(diff_mapy) < 35)
                    gps_check = true;
                else
                    gps_check = false;
                last_mapx = mapx;
                last_mapy = mapy;
            }
        }
    }
    else qWarning() << __FUNCTION__ << "No robot node found";
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


//SUBSCRIPTION to sendData method from JoystickAdapter interface
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    joydata = data;
    for(auto  &a: joydata.axes)
    {
        if(a.name == "advance"){
            //adv_speed = clamp(a.value, -1.0, 1.0);
            if (a.value > 1.0){
                adv_speed = 1.0;
            }
            else if (a.value < -1.0){
                adv_speed = -1.0;
            }
            else{
                adv_speed = a.value;
            }
        }
        if(a.name == "turn"){
            //rot_speed = clamp(a.value, -1.0, 1.0);
            //std::cout << "Rotation: << std::endl;
            if (a.value > 1.0){
                rot_speed = 1.0;
            }
            else if (a.value < -1.0){
                rot_speed = -1.0;
            }
            else{
                rot_speed = a.value;
            }
        }

    }

    if(fabs(rot_speed) < 0.05) rot_speed = 0;
    if(fabs(adv_speed) < 0.05) adv_speed = 0;




}





/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

