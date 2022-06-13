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
update_three_camera();
auto laser = read_laser_front_robot();
update_laser(laser, laser_front_name);
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


RoboCompBatteryStatus::TBattery SpecificWorker::BatteryStatus_getBatteryState()
{
    RoboCompBatteryStatus::TBattery res;

    if (auto battery_node = G->get_node(battery_name); battery_node.has_value())
    {
        res.current =  float(G->get_attrib_by_name<battery_A_att>(battery_node.value()).value());
        res.voltage = float(G->get_attrib_by_name<battery_V_att >(battery_node.value()).value());
        res.percentage = float(G->get_attrib_by_name<battery_load_att >(battery_node.value()).value());
        res.timetogo = float(G->get_attrib_by_name<battery_TTG_att>(battery_node.value()).value());
        res.consumptionperhour = G->get_attrib_by_name<battery_CE_att>(battery_node.value()).value();
        res.power =  G->get_attrib_by_name<battery_P_att>(battery_node.value()).value();

    }else
        qWarning() << __FUNCTION__ << "Node AAAA not found";
    return res;

}

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
//    RoboCompCameraRGBDSimple::TImage res;
//
//    if (auto camrgbd_node = G->get_node(three_frontal_camera_name); camrgbd_node.has_value()) {
//        res.depth =  G->get_attrib_by_name<cam_rgb_depth_att>(camrgbd_node.value()).value();
//        res.height = G->get_attrib_by_name<cam_rgb_height_att >(camrgbd_node.value()).value();
//        res.width = G->get_attrib_by_name<cam_rgb_width_att >(camrgbd_node.value()).value();
//        res.compressed = true;
//        res.image =  G->get_attrib_by_name<compressed_data_att>(camrgbd_node.value()).value();
//
//    }else
//        qWarning() << __FUNCTION__ << "Node CamSimpleCompressed not found";
//    return res;

}

RoboCompCameraSimple::TImage SpecificWorker::CameraSimple_getImage()
{
//    RoboCompCameraSimple::TImage res;
//
//    if (auto camsimple_node = G->get_node(wide_angle_camera_name); camsimple_node.has_value()) {
//        res.depth =  G->get_attrib_by_name<cam_rgb_depth_att>(camsimple_node.value()).value();
//        res.height = G->get_attrib_by_name<cam_rgb_height_att >(camsimple_node.value()).value();
//        res.width = G->get_attrib_by_name<cam_rgb_width_att >(camsimple_node.value()).value();
//        res.compressed = true;
//        res.image =  G->get_attrib_by_name<compressed_data_att>(camsimple_node.value()).value();
//
//    }else
//    qWarning() << __FUNCTION__ << "Node CamSimpleCompressed not found";
//    return res;
}

void SpecificWorker::FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)
{
//implementCODE

}
std::vector<SpecificWorker::LaserPoint> SpecificWorker::read_laser_front_robot()
{
    std::vector<LaserPoint> laser_data;

    try
    {
        auto laser = laser_proxy->getLaserData();
        std::transform(laser.begin(), laser.end(), std::back_inserter(laser_data), [](const auto &l) {return LaserPoint{l.dist, l.angle}; });
    }catch (const Ice::Exception &e){ std::cout << e.what() << " No laser_pioneer_data front" << std::endl; return {};}

    return laser_data;
}
RoboCompGpsUblox::DatosGPS SpecificWorker::GpsUblox_getData()
{
    RoboCompGpsUblox::DatosGPS ret;
    if(auto node_gps = G->get_node(gps_name); node_gps.has_value()){
        ret.UTMx = G->get_attrib_by_name<gps_UTMx_att>(node_gps.value()).value();
        ret.UTMy = G->get_attrib_by_name<gps_UTMy_att>(node_gps.value()).value();
        ret.mapx = G->get_attrib_by_name<gps_map_x_att >(node_gps.value()).value();
        ret.mapy = G->get_attrib_by_name<gps_map_y_att >(node_gps.value()).value();
        ret.altitude = G->get_attrib_by_name<gps_altitude_att >(node_gps.value()).value();
        ret.longitude = G->get_attrib_by_name<gps_longitude_att >(node_gps.value()).value();
        ret.latitude = G->get_attrib_by_name<gps_latitude_att >(node_gps.value()).value();
        ret.rot = G->get_attrib_by_name<gps_rot_att >(node_gps.value()).value();
        ret.azimut = G->get_attrib_by_name<gps_azimut_att >(node_gps.value()).value();
    }else
        qWarning() << __FUNCTION__ << "Node Laser not found";
    return ret;



}

void SpecificWorker::GpsUblox_setInitialPose(float x, float y)
{
//implementCODE

}

void SpecificWorker::update_three_camera()
{
    RoboCompCameraRGBDSimple::TImage rgb;

    rgb = camerargbdsimple_proxy->getImage(three_frontal_camera_compressed_name);
    qDebug() << 'IMAGEN' << rgb.width << endl;
    cv::Mat frame_uncompressed = cv::imdecode(rgb.image, -1);
    //cv::cvtColor(frame_uncompressed, frame_uncompressed, cv::COLOR_BGR2RGB);
    //cv::imshow("RGB image", frame_uncompressed);

    if( auto three_camera = G->get_node(three_frontal_camera_name); three_camera.has_value())
    {
        cout << "PASA" << endl;
        std::vector<uint8_t> rgb; rgb.assign(frame_uncompressed.data, frame_uncompressed.data + frame_uncompressed.total()*frame_uncompressed.channels());
        G->add_or_modify_attrib_local<cam_rgb_att>(three_camera.value(), rgb);
        G->add_or_modify_attrib_local<cam_rgb_width_att>(three_camera.value(), frame_uncompressed.cols);
        G->add_or_modify_attrib_local<cam_rgb_height_att>(three_camera.value(), frame_uncompressed.rows);
        G->add_or_modify_attrib_local<cam_rgb_depth_att>(three_camera.value(), frame_uncompressed.depth());
        G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(three_camera.value(),1);
        G->add_or_modify_attrib_local<cam_rgb_alivetime_att>(three_camera.value(), (int)std::chrono::time_point_cast<std::chrono::milliseconds>(MyClock::now()).time_since_epoch().count());
        G->update_node(three_camera.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node three_frontal_camera not found";
}

void SpecificWorker::update_laser(const std::vector<LaserPoint> &laser_data, const std::string laser_name)
{
    if( auto node = G->get_node(laser_name); node.has_value())
    {

        // Transform laserData into two std::vector<float>
        std::vector<float> dists;
        std::transform(laser_data.begin(), laser_data.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
        std::vector<float> angles;
        std::transform(laser_data.begin(), laser_data.end(), std::back_inserter(angles), [](const auto &l) { return l.angle; });

        // update laser in DSR
        G->add_or_modify_attrib_local<laser_dists_att>(node.value(), dists);
        G->add_or_modify_attrib_local<laser_angles_att>(node.value(), angles);
        G->update_node(node.value());

    }
    else
        qWarning() << __FUNCTION__ << "No laser node found";
}
RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
//implementCODE

}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
//implementCODE

}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
//    RoboCompLaser::TLaserData ldata(MAX_LASER_BINS);
//    if(auto node_laser = G->get_node(laser_name); node_laser.has_value()){
//        auto angles = G->get_attrib_by_name<laser_angles_att>(node_laser.value()).value();
//        auto dist = G->get_attrib_by_name<laser_dists_att>(node_laser.value()).value();
//        std::vector<float> d = dist;
//        std::vector<float> a = angles;
//        try{
//        for (int i=1; i <= MAX_LASER_BINS; i++){
//            ldata[i].dist = d[i];
//            ldata[i].angle = a[i];
//        }}
//        catch(const std::exception &e)
//        { std::cout << e.what()<< " FAIL LASER" << std::endl;}
//
//
//    }else
//        qWarning() << __FUNCTION__ << "Node Laser not found";
//    return ldata;
}

RoboCompRadar::RadarData SpecificWorker::Radar_getData()
{
//implementCODE

}

RoboCompOdometryMelex::TOdometry SpecificWorker::OdometryMelex_getOdometryState()
{
    RoboCompOdometryMelex::TOdometry res;

    if (auto odometry_node = G->get_node(odometry_name); odometry_node.has_value()) {
        res.vel =  G->get_attrib_by_name<odometry_vel_att>(odometry_node.value()).value();
        res.steer = G->get_attrib_by_name<odometry_steer_att >(odometry_node.value()).value();

    }else
        qWarning() << __FUNCTION__ << "Node CamSimpleCompressed not found";
    return res;
}



/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompHumanCameraBody you can call this methods:
// this->humancamerabody_proxy->newPeopleData(...)

/**************************************/
// From the RoboCompHumanCameraBody you can use this types:
// RoboCompHumanCameraBody::TImage
// RoboCompHumanCameraBody::TGroundTruth
// RoboCompHumanCameraBody::KeyPoint
// RoboCompHumanCameraBody::Person
// RoboCompHumanCameraBody::PeopleData

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompBatteryStatus you can use this types:
// RoboCompBatteryStatus::TBattery

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompGpsUblox you can use this types:
// RoboCompGpsUblox::DatosGPS

/**************************************/

// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompOdometryMelex you can use this types:
// RoboCompOdometryMelex::TOdometry

/**************************************/
// From the RoboCompRadar you can use this types:
// RoboCompRadar::RadarData

