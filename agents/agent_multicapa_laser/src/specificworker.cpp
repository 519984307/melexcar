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
    //std::setlocale(LC_NUMERIC, "C");
    serial_left = params["serial_left"].value;
    serial_right = params["serial_right"].value;
    serial_center = params["serial_center"].value;
    display_depth = (params["display_depth"].value == "true") or (params["display_depth"].value == "True");
    display_rgb = (params["display_rgb"].value == "true") or (params["display_rgb"].value == "True");
    display_laser = (params["display_laser"].value == "true") or (params["display_laser"].value == "True");
    compressed = (params["compressed"].value == "true") or (params["compressed"].value == "True");
    view = (params["view"].value == "true") or (params["view"].value == "True");
    consts.max_up_height = std::stof(params.at("max_up_height").value);
    consts.max_down_height = std::stof(params.at("max_down_height").value);

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

    compression_params_image.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params_image.push_back(15);
    compression_params_depth.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params_depth.push_back(15);

    int fps_depth = 15;
    int fps_color = 15;

    segmento = std::vector<double>(_CALFAR+1);
    array_segment_local=std::vector<SEGMENTO>(100);

    // center camera
    try
    {
        cfg_center.enable_device(serial_center);
        cfg_center.enable_stream(RS2_STREAM_DEPTH, consts.width, consts.height, RS2_FORMAT_Z16, fps_depth);
        cfg_center.enable_stream(RS2_STREAM_COLOR, consts.width, consts.height, RS2_FORMAT_BGR8, fps_color);
        rs2::pipeline center_pipe;
        rs2::pipeline_profile profile_center = center_pipe.start(cfg_center);
        center_depth_intr = center_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        Eigen::Translation<float, 3> center_tr(0., 0.,  -0.020);
        Eigen::Matrix3f center_m;
        center_m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
                   * Eigen::AngleAxisf(0.244346, Eigen::Vector3f::UnitY())  //60 degrees
                   * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Transform<float, 3, Eigen::Affine> center_depth_extrinsics;;
        center_depth_extrinsics = center_tr;
        center_depth_extrinsics.rotate(center_m);
        cam_map[serial_center] = std::make_tuple(center_pipe, center_depth_intr, center_depth_extrinsics, rs2::frame(), rs2::points(), rs2::frame());
        print_camera_params(serial_center, profile_center);
        qInfo() << __FUNCTION__ << " center-camera started";
    }
    catch(const std::exception &e)
    { std::cout << e.what()<< " Serial number:" << serial_center << std::endl; std::terminate();}

    // right camera
    try
    {
        cfg_right.enable_device(serial_right);
        cfg_right.enable_stream(RS2_STREAM_DEPTH, consts.width, consts.height, RS2_FORMAT_Z16, fps_depth);
        cfg_right.enable_stream(RS2_STREAM_COLOR, consts.width, consts.height, RS2_FORMAT_BGR8, fps_color);
        rs2::pipeline right_pipe;
        rs2::pipeline_profile profile_right = right_pipe.start(cfg_right);
        right_depth_intr = right_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        Eigen::Translation<float, 3> right_tr(0.0, -0.020, 0.0);
        // Eigen::Translation<float, 3> right_tr(0.0963, 0., 0.0578);
        Eigen::Matrix3f right_m;
        right_m = Eigen::AngleAxisf(M_PI/3, Eigen::Vector3f::UnitX())
                  * Eigen::AngleAxisf(0.244346, Eigen::Vector3f::UnitY())  //60 degrees
                  * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Transform<float, 3, Eigen::Affine> right_depth_extrinsics;;
        right_depth_extrinsics = right_tr;
        right_depth_extrinsics.rotate(right_m);
        std::cout << "right transform " << right_depth_extrinsics.matrix() << std::endl;
        cam_map[serial_right] = std::make_tuple(right_pipe, right_depth_intr, right_depth_extrinsics,  rs2::frame(), rs2::points(), rs2::frame());
        print_camera_params(serial_right, profile_right);
        qInfo() << __FUNCTION__ << " right-camera started";
    }
    catch(const std::exception &e)
    { std::cout << e.what()<< " Serial number:" << serial_right << std::endl; std::terminate();}

    // left camera
    try
    {
        cfg_left.enable_device(serial_left);
        cfg_left.enable_stream(RS2_STREAM_DEPTH, consts.width, consts.height, RS2_FORMAT_Z16, fps_depth);
        cfg_left.enable_stream(RS2_STREAM_COLOR, consts.width, consts.height, RS2_FORMAT_BGR8, fps_color);
        rs2::pipeline left_pipe;
        rs2::pipeline_profile profile_left = left_pipe.start(cfg_left);
        left_depth_intr = left_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        Eigen::Translation<float, 3> left_tr(0, 0.020, 0.0);
        // Eigen::Translation<float, 3> left_tr(-0.0963, 0., 0.0578);
        Eigen::Matrix3f left_m;
        left_m = Eigen::AngleAxisf(-M_PI/3., Eigen::Vector3f::UnitX())
                 * Eigen::AngleAxisf(0.244346, Eigen::Vector3f::UnitY())  //60 degrees
                 * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Transform<float, 3, Eigen::Affine> left_depth_extrinsics;;
        left_depth_extrinsics = left_tr;
        left_depth_extrinsics.rotate(left_m);
        cam_map[serial_left] = std::make_tuple(left_pipe, left_depth_intr, left_depth_extrinsics, rs2::frame(), rs2::points(), rs2::frame());
        print_camera_params(serial_left, profile_left);
        qInfo() << __FUNCTION__ << " left-camera started";
    }
    catch(const std::exception &e)
    { std::cout << e.what()<< " Serial number:" << serial_left << std::endl; std::terminate();}

    // Filters
    rs2_set_option(dec_filter, RS2_OPTION_FILTER_MAGNITUDE, 2, error);
    filters.emplace_back("Decimate", dec_filter);
    filters.emplace_back("Spatial", spat_filter);
    filters.emplace_back("Temporal", temp_filter);
    filters.emplace_back("HFilling", holef_filter);

    //compression params
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(50);

    this->Period = 50;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);
}

void SpecificWorker::compute()
{
    string nombre_laser ="laser_nivel_";
    int i=0;
    auto &cam_map_extended = read_and_filter(cam_map);   // USE OPTIONAL
    std::cout << "EXTENDED SIZE: " << cam_map_extended.size() << std::endl;
    auto ldata_local = compute_laser(cam_map_extended);
    //auto &&[virtual_frame] = mosaic(cam_map_extended);
    auto &&[v_f] = mosaic(cam_map_extended);
    std::cout<<"LLEGOOOOOOOOOOOO"<<endl;
    my_mutex.lock();
        virtual_frame=v_f.clone();
    my_mutex.unlock();
    std::cout<<"LLEGOOOOOOOOOOOO"<<endl;
    if(compressed)
    {
        cv::imencode(".jpg", virtual_frame, buffer, compression_params);
        insert_camera_node();
        cv::cvtColor(virtual_frame, virtual_frame, cv::COLOR_BGR2RGB);
        update_three_camera_node(three_frontal_camera_name, virtual_frame, buffer);
    }

    if(display_depth)
        show_depth_images(cam_map_extended);
    if(display_rgb)
    {
        cv::imshow("Virtual", virtual_frame);
        cv::waitKey(1);
    }
    std::cout<<"LLEGOOOOOOOOOOOO antes"<<endl;
    for(auto laser:ldata_local){
        std::cout<<"LLEGOOOOOOOOOOOOfor1"<<endl;
        auto nombre=nombre_laser+std::to_string(i);
        std::cout<<"LLEGOOOOOOOOOOOOfor2"<<endl;
        //draw_laser(laser);feature
        features( laser, &scan_mapa, &scan_maparef);
        std::cout<<"LLEGOOOOOOOOOOOOfor3"<<endl;
        update_laser_node(nombre,laser);
        std::cout<<"LLEGOOOOOOOOOOOOfor4"<<endl;
        i++;
    }



    //ldata_return = ldata_local;
    fps.print("FPS: ");

//    if (view) //si quiero visualizar la imagen no comprimida
//    {
//        insert_camera_node();
//        cv::cvtColor(virtual_frame, virtual_frame, cv::COLOR_BGR2RGB);
//        update_three_camera_node(three_frontal_camera_name, virtual_frame);
//    }

//    insert_laser_node();
    //update_laser_node(laser_front_name,ldata_local);
}

SpecificWorker::Camera_Map& SpecificWorker::read_and_filter(Camera_Map &cam_map)
{
    for (auto &[key, value] : cam_map)
    {
        cout << "READ AND FILTER " << key << endl;
        //if(key != serial_center and key != serial_right) continue;

        auto &[my_pipe, intr, extr, depth_frame, points, color_frame] = value;
        rs2::frameset data = my_pipe.wait_for_frames();
        depth_frame = data.get_depth_frame(); // Find and colorize the depth dat
        color_frame = data.get_color_frame(); // Find and colorize the depth dat

        for (auto &&filter : filters)
            depth_frame = filter.filter.process(depth_frame);

        // rgb_list[i] = data.get_color_frame(); // Find the color data
        rs2::pointcloud pointcloud;
        pointcloud.map_to(color_frame);
        points = pointcloud.calculate(depth_frame);
    }
    return cam_map;
}


////////////////// MOSAIC  WITHOUT PROCESSING //////////////////////////////////////////
std::tuple<cv::Mat> SpecificWorker::mosaic(const Camera_Map &cam_map)
{
    // virtual frame
    cv::Mat frame_virtual = cv::Mat::zeros(consts.width, consts.height*3, CV_8UC3);
    cv::Mat winLeftR = frame_virtual(cv::Rect(0,0,consts.height, consts.width));
    cv::Mat winCenterR = frame_virtual(cv::Rect(consts.height,0,consts.height, consts.width));
    cv::Mat winRightR = frame_virtual(cv::Rect(2*consts.height,0,consts.height,consts.width));

    const auto &[pipeL, intrL, extrL, depth_frameL, pointsL, color_frameL] = cam_map.at(serial_left);
    cv::Mat imgLeft(consts.height, consts.width, CV_8UC3, (char *) color_frameL.get_data());
    cv::Mat hsv;
    cv::cvtColor(imgLeft, hsv, cv::COLOR_RGB2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::equalizeHist(channels[2], channels[2]);
    cv::merge(channels, hsv);
    cv::cvtColor(hsv, imgLeft, cv::COLOR_HSV2RGB);
    cv::rotate(imgLeft, winLeftR, cv::ROTATE_90_CLOCKWISE);

    const auto &[pipeC, intrC, extrC, depth_frameC, pointsC, color_frameC] = cam_map.at(serial_center);
    cv::Mat imgCenter(consts.height, consts.width, CV_8UC3, (char *) color_frameC.get_data());
    cv::Mat hsv2;
    cv::cvtColor(imgCenter, hsv2, cv::COLOR_RGB2HSV);
    cv::split(hsv2, channels);
    cv::equalizeHist(channels[2], channels[2]);
    cv::merge(channels, hsv2);
    cv::cvtColor(hsv2, imgCenter, cv::COLOR_HSV2RGB);
    cv::rotate(imgCenter, winCenterR, cv::ROTATE_90_CLOCKWISE);

    const auto &[pipeR, intrR, extrR, depth_frameR, pointsR, color_frameR] = cam_map.at(serial_right);
    cv::Mat imgRight(consts.height, consts.width, CV_8UC3, (char *) color_frameR.get_data());
    cv::Mat hsv3;
    cv::cvtColor(imgRight, hsv3, cv::COLOR_RGB2HSV);
    cv::split(hsv3, channels);
    cv::equalizeHist(channels[2], channels[2]);
    cv::merge(channels, hsv3);
    cv::cvtColor(hsv3, imgRight, cv::COLOR_HSV2RGB);
    cv::rotate(imgRight, winRightR, cv::ROTATE_90_CLOCKWISE);

    return std::make_tuple(frame_virtual);
}

///////////////////// DISPLAY ////////////////////////////////////77
void SpecificWorker::show_depth_images(Camera_Map &cam_map)
{
    std::map<std::string,cv::Mat> image_stack;
    int w=0, h=0;
    for (auto &[key, value] : cam_map)
    {
        auto &[pipe, intr, extr, depth_frame, points, color_frame] = value;
        if(depth_frame)
        {
            rs2::frame depth_color = depth_frame.apply_filter(color_map);
            w = depth_frame.as<rs2::video_frame>().get_width();
            h = depth_frame.as<rs2::video_frame>().get_height();
            cv::Mat frame_depth(cv::Size(w,h), CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP);
            //image_stack.emplace_back(cv::Mat(cv::Size(w,h), CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP));
            image_stack[key] = frame_depth.clone();
        }
    }
    if(not cam_map.empty())
    {
        cv::Mat frame_final(cv::Size(w * image_stack.size(), h), CV_8UC3);
        image_stack[serial_left].copyTo(frame_final(cv::Rect(0,0,w,h)));
        image_stack[serial_center].copyTo(frame_final(cv::Rect(w,0,w,h)));
        image_stack[serial_right].copyTo(frame_final(cv::Rect(2*w,0,w,h)));
        //        for(auto &&[i, img] : iter::enumerate(image_stack))
        //            img.copyTo(frame_final(cv::Rect(i*w,0,w,h)));
        cv::imshow("Depth mosaic", frame_final);
        cv::waitKey(1);
    }
}

void SpecificWorker::print_camera_params(const std::string &serial, const rs2::pipeline_profile &profile)
{
    float center_fov[2]; // X, Y fov
    const auto &[pipe, intr, extr, depth_frame, points, color_frame] = cam_map.at(serial);
    rs2_fov(&intr, center_fov);
    std::cout << "Camera " << serial << " started" << std::endl;
    std::cout << "  width: " << center_depth_intr.width << std::endl;
    std::cout << "  height: " << center_depth_intr.height << std::endl;
    std::cout << "  image center x: " << center_depth_intr.ppx << std::endl;
    std::cout << "  image center y: " << center_depth_intr.ppy << std::endl;
    std::cout << "  focal x: " << center_depth_intr.fx << std::endl;
    std::cout << "  focal y: " << center_depth_intr.fy << std::endl;
    std::cout << "  horizontal angle: " << center_fov[0] << std::endl;
    std::cout << "  vertical angle: " << center_fov[1] << std::endl;
    std::cout << "  extrinsics: " << extr.matrix() << std::endl;
    for (auto p : profile.get_streams())
        std::cout << "  stream ID: " << p.unique_id() << " - Stream name: " << p.stream_name() << std::endl;
}
std::vector<RoboCompLaser::TLaserData> SpecificWorker::compute_laser(const Camera_Map &cam_map_extended)
{
    std::vector<RoboCompLaser::TLaserData> ldatavector;
    using Point = std::tuple<float, float, float>;
    auto cmp = [](Point a, Point b)
    {
        auto &[ax, ay, az] = a;
        auto &[bx, by, bz] = b;
        return (ax * ax + ay * ay + az * az) < (bx * bx + by * by + bz * bz);
    };
    std::vector<std::set<Point, decltype(cmp) >> hor_bins(consts.MAX_LASER_BINS);
    std::vector<std::set<Point, decltype(cmp) >> hor_bins1(consts.MAX_LASER_BINS);
    std::vector<std::set<Point, decltype(cmp) >> hor_bins2(consts.MAX_LASER_BINS);
    std::vector<std::set<Point, decltype(cmp) >> hor_bins3(consts.MAX_LASER_BINS);// lambda to sort elements on insertion

    for( const auto &[key, value] : cam_map_extended)
    {
        cout << key << " CAAAAAAAAAAAAM "<< cam_map_extended.size()<< endl;
        const auto &[pipe, intrin, extrin, depth_frame, points, color_frame] = value;
        if(points.size() == 0) continue;
        const rs2::vertex *vertices = points.get_vertices();

        float FLOOR_DISTANCE_MINUS_OFFSET =  extrin.translation().x() * 0.9;  // X+ axis points downwards since the camera is rotated
        for (size_t i = 0; i < points.size(); i++)
        {
            if((vertices[i].z >= 0.99 ) and (vertices[i].z <= 15) ) //and vertices[i].z <=3
            {
                auto diferencia= (consts.max_down_height-(consts.max_up_height))/num_planos;
                auto to_point = extrin * Eigen::Vector3f{vertices[i].x, vertices[i].y, vertices[i].z};
                const float &xv = to_point[0]; const float &yv = to_point[1]; const float &zv = to_point[2];
                //if (xv < RIG_ELEVATION_FROM_FLOOR * 0.9)
                if( xv < (consts.max_down_height+diferencia)  and xv > consts.max_up_height)  // central band - positive x downwards
                {
                    float hor_angle = atan2(yv, zv);
                    // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                    int angle_index = (int) ((consts.MAX_LASER_BINS / consts.TOTAL_HOR_ANGLE) * hor_angle + (consts.MAX_LASER_BINS / 2.f));
                    if (angle_index >= consts.MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins[angle_index].emplace(std::make_tuple(xv, yv, zv));
                }
                if( xv < (consts.max_down_height + (diferencia*2)) and xv > (consts.max_up_height+diferencia))  // central band - positive x downwards
                {
                    float hor_angle = atan2(yv, zv);
                    // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                    int angle_index = (int) ((consts.MAX_LASER_BINS / consts.TOTAL_HOR_ANGLE) * hor_angle + (consts.MAX_LASER_BINS / 2.f));
                    if (angle_index >= consts.MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins1[angle_index].emplace(std::make_tuple(xv, yv, zv));
                }
                if( xv < (consts.max_down_height + (diferencia*3) )and xv > (consts.max_up_height+(diferencia*2))) // central band - positive x downwards
                {
                    float hor_angle = atan2(yv, zv);
                    // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                    int angle_index = (int) ((consts.MAX_LASER_BINS / consts.TOTAL_HOR_ANGLE) * hor_angle + (consts.MAX_LASER_BINS / 2.f));
                    if (angle_index >= consts.MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins2[angle_index].emplace(std::make_tuple(xv, yv, zv));
                }
                if( xv < (consts.max_down_height + (diferencia*4) and xv > (consts.max_up_height+(diferencia*3))))  // central band - positive x downwards
                {
                    float hor_angle = atan2(yv, zv);
                    // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                    int angle_index = (int) ((consts.MAX_LASER_BINS / consts.TOTAL_HOR_ANGLE) * hor_angle + (consts.MAX_LASER_BINS / 2.f));
                    if (angle_index >= consts.MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins3[angle_index].emplace(std::make_tuple(xv, yv, zv));
                }


        }
    }}
    RoboCompLaser::TLaserData ldata(consts.MAX_LASER_BINS);
    uint i = 0;
    for (auto &bin : hor_bins)
    {
        ldata[(hor_bins.size()-1)-i].angle = (i - consts.MAX_LASER_BINS / 2.f) * ( consts.TOTAL_HOR_ANGLE / consts.MAX_LASER_BINS );
        if (bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            ldata[i].dist = sqrt(X * X + Y * Y + Z * Z)*1000;
        }
        else
        {
            if(i>0) {

                ldata[i].dist = ldata[i - 1].dist;  // link to the adjacent
                //cout<<"ADYACENTEEEEEEEE"<< endl;
            }
            else ldata[i].dist = 10000;
        }
        i++;
    }
    ldatavector.push_back(ldata);
    RoboCompLaser::TLaserData ldata1(consts.MAX_LASER_BINS);
    i = 0;
    for (auto &bin : hor_bins1)
    {
        ldata1[(hor_bins1.size()-1)-i].angle = (i - consts.MAX_LASER_BINS / 2.f) * ( consts.TOTAL_HOR_ANGLE / consts.MAX_LASER_BINS );
        if (bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            ldata1[i].dist = sqrt(X * X + Y * Y + Z * Z)*1000;
        }
        else
        {
            if(i>0) ldata1[i].dist = ldata1[i - 1].dist;  // link to the adjacent
            else ldata1[i].dist = 10000;
        }
        i++;
    }
    ldatavector.push_back(ldata1);

    RoboCompLaser::TLaserData ldata2(consts.MAX_LASER_BINS);
    i = 0;
    for (auto &bin : hor_bins2)
    {
        ldata2[(hor_bins2.size()-1)-i].angle = (i - consts.MAX_LASER_BINS / 2.f) * ( consts.TOTAL_HOR_ANGLE / consts.MAX_LASER_BINS );
        if (bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            ldata2[i].dist = sqrt(X * X + Y * Y + Z * Z)*1000;
        }
        else
        {
            if(i>0) ldata2[i].dist = ldata2[i - 1].dist;  // link to the adjacent
            else ldata2[i].dist = 10000;
        }
        i++;
    }
    ldatavector.push_back(ldata2);

    RoboCompLaser::TLaserData ldata3(consts.MAX_LASER_BINS);
    i = 0;
    for (auto &bin : hor_bins3)
    {
        ldata3[(hor_bins3.size()-1)-i].angle = (i - consts.MAX_LASER_BINS / 2.f) * ( consts.TOTAL_HOR_ANGLE / consts.MAX_LASER_BINS );
        if (bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            ldata3[i].dist = sqrt(X * X + Y * Y + Z * Z)*1000;
        }
        else
        {
            if(i>0)
                ldata3[i].dist = ldata3[i - 1].dist;  // link to the adjacent
            else
                ldata3[i].dist = 10000;
        }
        i++;
    }
    ldatavector.push_back(ldata3);
    return ldatavector;
}



//void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
//{
//    if(ldata.empty()) return;
//
//    const int lado = 800;
//    const int semilado = lado/2;
//    const int yoffset = 600;
//    cv::Mat laser_img(cv::Size(lado, lado), CV_8UC3);
//    laser_img = cv::Scalar(255,255,255);
//    float scale = 0.1;
//    cv::circle(laser_img, cv::Point(semilado,yoffset), 10, cv::Scalar(200,100,0), cv::FILLED, 8,0);
//    float x = ldata.front().dist * sin(ldata.front().angle) * scale + lado/2;
//    float y = yoffset - ldata.front().dist * cos(ldata.front().angle) * scale;
//    cv::line(laser_img, cv::Point{semilado,yoffset}, cv::Point(x,y), cv::Scalar(0,200,0));
//    for(auto &&l : iter::sliding_window(ldata, 2))
//    {
//        int x1 = l[0].dist * sin(l[0].angle) * scale + semilado;
//        int y1 = yoffset - l[0].dist * cos(l[0].angle) * scale;
//        int x2 = l[1].dist * sin(l[1].angle) * scale + semilado;
//        int y2 = yoffset - l[1].dist * cos(l[1].angle) * scale;
//        cv::line(laser_img, cv::Point{x1,y1}, cv::Point(x2,y2), cv::Scalar(0,200,0));
//    }
//    x = ldata.back().dist * sin(ldata.back().angle) * scale + semilado;
//    y = yoffset - ldata.back().dist * cos(ldata.back().angle) * scale;
//    cv::line(laser_img, cv::Point(x,y), cv::Point(semilado,yoffset), cv::Scalar(0,200,0));
//
//    cv::imshow("Laser", laser_img);
//    cv::waitKey(2);
//}
///////////////nodes graph///////////////////


void SpecificWorker::insert_camera_node()
{
    if (auto cam_node = G->get_node(three_frontal_camera_name); cam_node.has_value())
        cam_api = G->get_camera_api(cam_node.value());
    else {
        std::cout << "Controller-DSR terminate: could not find a camera node named "
                  << three_frontal_camera_name << std::endl;
        std::terminate();
    }
}

void SpecificWorker::update_three_camera_node(std::string camera_name, const cv::Mat &v_image, const vector<uchar> compressed_data)
{
    if(auto node = G->get_node(camera_name); node.has_value())
    {
        std::vector<uint8_t> rgb; rgb.assign(v_image.data, v_image.data + v_image.total()*v_image.channels());
        G->add_or_modify_attrib_local<cam_rgb_att>(node.value(), rgb);
        G->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), v_image.cols);
        G->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), v_image.rows);
        G->add_or_modify_attrib_local<cam_rgb_depth_att>(node.value(), v_image.depth());
        G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(node.value(),1);
        G->add_or_modify_attrib_local<compressed_data_att>(node.value(), compressed_data);
        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node three_frontal_camera not found";
}

void SpecificWorker::insert_laser_node()
{
//    if (auto laser_node = G->get_node(laser_name); laser_node.has_value())
//        auto laser_data = RoboCompLaser::TLaserData();
//    else {
//        std::cout << "Controller-DSR terminate: could not find a camera node named "
//                  << laser_name << std::endl;
//        std::terminate();
//    }
}

void SpecificWorker::update_laser_node(std::string laser_name, const RoboCompLaser::TLaserData &ldata)
{
    if( auto three_cam_node =G->get_node(three_frontal_camera_name); three_cam_node.has_value()){
        std::vector<float> dists;
        std::transform(ldata.begin(), ldata.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
        std::vector<float> angles;
        std::transform(ldata.begin(), ldata.end(), std::back_inserter(angles), [](const auto &l) { return l.angle; });
        if( auto laser_node = G->get_node(laser_name); laser_node.has_value())
        {
            //auto laser_data = RoboCompLaser::TLaserData();
            G->add_or_modify_attrib_local<laser_dists_att>(laser_node.value(), dists);
            G->add_or_modify_attrib_local<laser_angles_att>(laser_node.value(), angles);
            cout<<dists[80]<<endl;
            G->update_node(laser_node.value());
            if (auto breakpoints_node = G->get_node(breakpoints_laser_name); breakpoints_node.has_value())
            {
                //auto laser_data = RoboCompLaser::TLaserData();
                G->add_or_modify_attrib_local<laser_dists_att>(breakpoints_node.value(), distanbreak);
                G->add_or_modify_attrib_local<laser_angles_att>(breakpoints_node.value(), angulobreak);
                G->update_node(breakpoints_node.value());
            }
            else{
                DSR::Node breakpoints = DSR::Node::create<laser_node_type>(breakpoints_laser_name);
                G->add_or_modify_attrib_local<breakpoints_dists_att>(breakpoints, distanbreak );
                G->add_or_modify_attrib_local<breakpoints_angles_att>(breakpoints, angulobreak);
                G->insert_node(breakpoints);
                DSR::Edge edge = DSR::Edge::create<RT_edge_type>(breakpoints.id(), laser_node.value().id());
                G->insert_or_assign_edge(edge);
            }
        }
        else{
            DSR::Node new_laser_node = DSR::Node::create<laser_node_type>(laser_name);
            G->add_or_modify_attrib_local<laser_dists_att>(new_laser_node, dists);
            G->add_or_modify_attrib_local<laser_angles_att>(new_laser_node, angles);
            G->insert_node(new_laser_node);
            DSR::Edge edge = DSR::Edge::create<RT_edge_type>(three_cam_node.value().id(), new_laser_node.id());
            G->insert_or_assign_edge(edge);
        }

    }
        }


void SpecificWorker::insert_camera_node_compressed()
{
    if (auto cam_node = G->get_node(three_frontal_camera_compressed_name); cam_node.has_value())
        cam_api = G->get_camera_api(cam_node.value());
    else {
        std::cout << "Controller-DSR terminate: could not find a camera node named "
                  << three_frontal_camera_compressed_name << std::endl;
        std::terminate();
    }
}

//void SpecificWorker::update_three_camera_compressed(std::string camera_name, const vector<uchar> compressed_data)
//{
//    if(auto node = G->get_node(three_frontal_camera_compressed_name); node.has_value())
//    {
//        G->add_or_modify_attrib_local<compressed_data_att>(node.value(), compressed_data);
//
//        G->update_node(node.value());
//    }
//    else
//        qWarning() << __FUNCTION__ << "Node wide_angle_camera_compressed not found";
//}

//////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
//////////////////////////////////

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
    RoboCompCameraRGBDSimple::TImage im;
    my_mutex.lock();
    im.height = virtual_frame.rows;
    im.width = virtual_frame.cols;
    if(im.height != 0 and im.width != 0){
        im.compressed = compressed;
        if(true){

            //im.focalx = cam_map[camera].rs2_intrinsics.fx;
            //im.focaly = cam_map[camera].rs2_intrinsics.fy;
            //if(im.compressed)
            if ( im.compressed){
                vector<uchar> buffer;
                cv::Mat frame(cv::Size(im.width, im.height), CV_8UC3, &virtual_frame.data[0], cv::Mat::AUTO_STEP);
                cv::imencode(".jpg", frame, buffer, compression_params_image);
                std::cout << "raw: " << frame.total() * frame.elemSize() << " compressed: " << buffer.size() << " Ratio:"
                          << frame.total() * frame.elemSize() / buffer.size() << std::endl;
                im.image.assign(buffer.begin(), buffer.end());
            }
            else
                im.image.assign(virtual_frame.data, virtual_frame.data + (virtual_frame.total() * virtual_frame.elemSize()));


        }
    }
    else
        qInfo() << "I dont have image";

    my_mutex.unlock();
    return im;
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
    std::lock_guard<std::mutex> lg(my_mutex);
    return ldata_return;
}

void SpecificWorker::iniciar_mapa_local()
{
    mapa.n_esquinas=0;
    mapa.esquinas=(double*)malloc(N_CARACT*offset_corners*sizeof(double));
    mapa.R_esquinas=(double*)malloc(N_CARACT*offset_Rp*sizeof(double));

    mapa.n_segmentos=0;
    mapa.segmentos=(double*)malloc(N_CARACT*offset_segments*sizeof(double));
    mapa.R_segmentos=(double*)malloc(N_CARACT*offset_Rs*sizeof(double));

    mapa.n_circulos=0;
    mapa.circulos=(double*)malloc((N_CARACT*offset_circles+1)*sizeof(double));
    mapa.R_circulos=(double*)malloc(N_CARACT*offset_Rc*sizeof(double));
}

void SpecificWorker::features ( const std::vector<RoboCompLaser::TData>  & lData, MAPA_LASER *scan_mapa, MAPA_LASER *scan_maparef )
{

    virtual_corners.push_back(0);
    indice_grupo = 0;
    for(auto &laser: lData) // procesaiento de lData
    {
        x_coords.push_back(laser.dist * cos(laser.angle));
        y_coords.push_back(laser.dist * sin(laser.angle));
        angles.push_back(laser.angle);
        dists.push_back(laser.dist);
    }
    int i=0;
    while(i<angles.size()) {
        array_points.push_back(dists[i]);
        array_points.push_back(angles[i]);
        i++;
    }
    // [breakpoints]

    std::cout<<"LLEGOOOOOOOOOOOOfeature2"<<endl;
    std::vector<bool> breakPoints = MapBreakPoints();
    std::cout<<"LLEGOOOOOOOOOOOOfeature3"<<endl;

     for (int i = 0; i < breakPoints.size(); i++)
     {
         std::cout<<"LLEGOOOOOOOOOOOOfeature4"<<endl;
         if(breakPoints[i] == true)
         {
             distanbreak.push_back(dists[i]);
             angulobreak.push_back(angles[i]);
         }
     }

    // Comment: "[LE] Inicializo curvatureLaser --> breakpoints[contseg+3]: almacena la longitud del scan"
    indice_grupo=0; cont_segment=0; cont_circle=0;
    scan_mapa->n_esquinas = 0;

    // Comment: "[LE] BUCLE ESTUDIO TROZOS DEL SCAN: contseg - contador del subsegmento o trozo"
    contseg=0;

    int inicio, fin;
    for(int i = 0; i < x_coords.size(); i++)
    {
        if(breakPoints[i] == true)
        {
            CURVATURALASER curvatureLaser = CURVATURALASER(distanbreak.size(), -1, 15, 7, 1, 0.2);
            curvatureLaser.DataInPixels(x_coords, y_coords, i);
            curvatureLaser.CalculoCurvatura();

            indice_grupo++;
            int n_esquinas= curvatureLaser.DetectarEsquinas();
            array_corners.push_back(n_esquinas);
            std::cout<<"LLEGOOOOOOOOOOOOfeature6"<<endl;

            std::vector<float> x_, y_, theta_, type_, index_;
            for (int m = 0; m < n_esquinas; m++ )
            {
                x_.push_back(curvatureLaser.Esquinas[m*offset_corners+_X]);
                y_.push_back(curvatureLaser.Esquinas[m*offset_corners+_Y]);
                theta_.push_back(curvatureLaser.Esquinas[m*offset_corners+_THETA]);
                type_.push_back(0); // Comment: Esquina Real
                index_.push_back(curvatureLaser.Esquinas[m*offset_corners+_INDEX]);
            }
            std::cout<<"LLEGOOOOOOOOOOOOfeature61"<<endl;

            if(n_esquinas != -1)
            {
                inicio = i;

                for (int k = 0; k <= n_esquinas; k++)
                {
                    std::cout<<"LLEGOOOOOOOOOOOOfeature62"<<endl;

                    if ( k==n_esquinas )
                    {
                        std::cout<<"LLEGOOOOOOOOOOOOfeature63"<<endl;

                        fin = y_coords[i] - 3;

                    }
                    else
                    {
                        std::cout<<"LLEGOOOOOOOOOOOOfeature64"<<endl;
                        fin = (int) (array_corners[k] + x_coords[i]);
                        std::cout<<"LLEGOOOOOOOOOOOOfeature65"<<endl;

                    }
                }
                std::cout<<"LLEGOOOOOOOOOOOOfeature66"<<endl;
                if (fin-inicio >= UMBRAL_PIXEL)
                {
                    std::cout<<"LLEGOOOOOOOOOOOOfeature67"<<endl;

                    segment_new(inicio, fin);

                    if (segmento[_DIST] >= LONG_SEGMENTO)
                    {
                        incluir_segmento();
                        std::cout<<"LLEGOOOOOOOOOOOOfeature68"<<endl;
                        cont_segment++;
                        virtual_corners[0]=0;
                        detectar_EsquinaVirtual();

                        incluir_esquina_virtual(scan_mapa);
                    }
                }

                inicio = fin;
            }
            else std::cout << "No se detectarn esquinas" << std::endl;

        }
        
    }
    
    scan_mapa->circulos[0]=cont_circle;
    obtener_mapa (scan_mapa);
    map_copy(scan_mapa, scan_maparef);
}

bool SpecificWorker::find_breakPoints(int contador)
{
    double alfa = 0.8; // TODO: Pasar alfa a radianes

    double distBetweenPoints = distance(x_coords[contador], y_coords[contador], x_coords[contador+1], y_coords[contador+1]);
    double dist2robot = distance(robot_x, robot_y, x_coords[contador-1], y_coords[contador-1]);
    dist2robot *= sin (SIGMA_THETA);
    dist2robot /= sin (alfa - SIGMA_THETA);
    dist2robot += 3 * SIGMA_R;

    return distBetweenPoints > dist2robot;
}

std::vector<bool> SpecificWorker::MapBreakPoints ()
{
    std::vector<bool> breakpoints;
    for(int i = 0; i < x_coords.size(); i++)
        breakpoints.push_back(find_breakPoints(i));
    return breakpoints;
}

double SpecificWorker::distance(double x1, double y1, double x2, double y2)
{
    //double x1 = x_coords[cont], x2 = x_coords[cont+1], y1 = y_coords[cont], y2 = y_coords[cont+1];

    double x = x2 - x1,  y = y2 - y1;

    return sqrt(x*x + y*y);
}

double SpecificWorker::rad2angle ( double angle )
{
    return angle*180/M_PI;
}
double SpecificWorker::angle2rad ( double angle )
{
    return angle*M_PI/180;
}
void SpecificWorker::incluir_circulo(MAPA_LASER *mapa, CIRCULO *array_circle_local, CIRCULO circle, int cont_circle, double *matriz)
{
    int i; // Comment: [LE] added by Ricardo Vazquez

    array_circle_local[cont_circle].x = circle.x;
    array_circle_local[cont_circle].y = circle.y;
    array_circle_local[cont_circle].r = circle.r;

    for(i=0; i<6; i++) {
        array_circle_local[cont_circle].vector[i] = circle.vector[i];
        array_circle_local[cont_circle].puntos[i] = circle.puntos[i];
    }

    matriz_covarianza_circulo(circle, matriz, cont_circle);

    mapa->circulos[cont_circle*offset_circles+_X]=circle.x;
    mapa->circulos[cont_circle*offset_circles+_Y]=circle.y;
    mapa->circulos[cont_circle*offset_circles+_RADIUS]=circle.r;

    mapa->R_circulos[cont_circle*offset_Rc]  =matriz[0];
    mapa->R_circulos[cont_circle*offset_Rc+1]=matriz[1];
    mapa->R_circulos[cont_circle*offset_Rc+2]=matriz[2];
    mapa->R_circulos[cont_circle*offset_Rc+3]=matriz[3];
    mapa->R_circulos[cont_circle*offset_Rc+4]=matriz[4];
    mapa->R_circulos[cont_circle*offset_Rc+5]=matriz[5];

    mapa->n_circulos++;
}
void SpecificWorker::segment_new(int inicio, int fin)
{
    segmento_Kai(inicio, fin);
}
void SpecificWorker::incluir_segmento()
{
    // Comment: [LE] Conversion de datos al stma. de referencia utilizado
    array_segment_local[cont_segment].x_ini = segmento[0];
    array_segment_local[cont_segment].y_ini = segmento[1];
    array_segment_local[cont_segment].x_fin = segmento[2];
    array_segment_local[cont_segment].y_fin = segmento[3];

    // Comment: [LE] Caracterizacion de array_segment_local
    array_segment_local[cont_segment].a = segmento[5];
    array_segment_local[cont_segment].b = segmento[6];

    array_segment_local[cont_segment].longitud = segmento[4];

    array_segment_local[cont_segment].lalfa = segmento[8];
    array_segment_local[cont_segment].lr  = segmento[9];


    array_segment_local[cont_segment].lcalfa  = segmento[10];
    array_segment_local[cont_segment].lcr     = segmento[11];
    array_segment_local[cont_segment].lcalfar = 0; //segmento[12];
}
void SpecificWorker::detectar_EsquinaVirtual()
{

    int i;
    int n_esquina;					// Numero de esquinas virtuales encontradas

    double pc_x,pc_y;				// Punto de corte entre dos rectas
    int array_c;					// vble. auxiliar

    double inc_angle;

    n_esquina=(int)(virtual_corners[0]);

    array_c=0;

    pc_x=0; pc_y=0;

    if (cont_segment!=1)
    { // Comment: [LE] si no existen segmentos todavia"
        for (i=0;i<cont_segment;i++)
        {

            inc_angle=(array_segment_local[cont_segment-1].lalfa-array_segment_local[i].lalfa)*180/PI;

            if (inc_angle<0)
                inc_angle=inc_angle+180;

            if (inc_angle>=180)
                inc_angle=inc_angle-180;

            // Comment: [LE] Se detecta si son rectas casi-paralelas: si es asi, se cancela la busqueda"
            if ((fabs(inc_angle)<30) ||
                (fabs(inc_angle)>150))
            {

            }
            else
            {
                // Comment: [LE] El punto de corte define la esquina virtual

                pc_y=(array_segment_local[i].lr*cos(array_segment_local[(cont_segment-1)].lalfa) - array_segment_local[(cont_segment-1)].lr*cos(array_segment_local[i].lalfa));
                pc_y=pc_y/(sin(array_segment_local[i].lalfa)*cos(array_segment_local[cont_segment-1].lalfa)- sin(array_segment_local[(cont_segment-1)].lalfa)*cos(array_segment_local[i].lalfa));

                pc_x= (array_segment_local[(cont_segment-1)].lr - pc_y*sin(array_segment_local[(cont_segment-1)].lalfa))/
                      (cos(array_segment_local[(cont_segment-1)].lalfa));

#ifdef DEBUGVC
                printf("(Xv,Yv) (%f,%f) - punto de corte entre: \n", pc_x,pc_y);
				  printf("A(rho,theta): (%f,%f) B(rho,theta): (%f,%f) \n",array_segment_local[i].lr,array_segment_local[i].lalfa*180/PI,
					  array_segment_local[(_cont-1)].lr,array_segment_local[(_cont-1)].lalfa*180/PI);
#endif
                virtual_corners[n_esquina*offset_corners + _X]= pc_x;
                virtual_corners[n_esquina*offset_corners + _Y]= pc_y;
                virtual_corners[n_esquina*offset_corners + _THETA]= (array_segment_local[(cont_segment-1)].lalfa + array_segment_local[i].lalfa)/2;

                virtual_corners[n_esquina*offset_corners + _ALFA1]= array_segment_local[(cont_segment-1)].lalfa;
                virtual_corners[n_esquina*offset_corners + _ALFA2]= array_segment_local[i].lalfa;
                virtual_corners[n_esquina*offset_corners + _RHO1] = array_segment_local[(cont_segment-1)].lr;
                virtual_corners[n_esquina*offset_corners + _RHO2] = array_segment_local[i].lr;

                // Comment: [LE] para la inclusion, incertidumbre
                matriz_covarianza_virtual(array_segment_local[(cont_segment-1)],array_segment_local[i],matriz_virtual,n_esquina);

#ifdef DEBUGVC
                printf("matriz_R[0]: %f ", matriz_R[n_esquina*offset_Rp]);
				  printf("matriz_R[1]: %f \n", matriz_R[n_esquina*offset_Rp+1]);
				  printf("matriz_R[2]: %f ", matriz_R[n_esquina*offset_Rp+2]);
				  printf("matriz_R[3]: %f \n", matriz_R[n_esquina*offset_Rp+3]);
#endif

                n_esquina++;

            }

        }
    }
    virtual_corners[0]=(double)n_esquina;
}

void SpecificWorker::incluir_esquina_virtual(MAPA_LASER *mapa)
{
    int i,j, cont,indice;
    bool exit;

    cont=0;
    indice=(int)mapa->n_esquinas;

    // Comment: [LE] Comprobamos si hay esquinas reales que coinciden con las virtuales

    for (i=0;i<virtual_corners[0];i++)
    {
        exit =false;


        // PMNT Dic. 08
        for (j =0; j< mapa->n_esquinas; j++) {
            if (dist_euclidean2D(virtual_corners[i*offset_corners   +_X], virtual_corners[i*offset_corners   +_Y],
                                 mapa->esquinas[j*offset_corners + _X], mapa->esquinas[j*offset_corners + _Y]) < 100) {
                exit = true;

            }
        }


        // Comment: [LE] Adaptamos al stma. de referencia del robot
        if (!exit)
        {


            mapa->esquinas[(indice+i)*offset_corners + _X]= virtual_corners[i*offset_corners   +_X];
            mapa->esquinas[(indice+i)*offset_corners +_Y]= virtual_corners[i*offset_corners +_Y];
            mapa->esquinas[(indice+i)*offset_corners +_THETA] = virtual_corners[i*offset_corners +_THETA];

            mapa->esquinas[(indice+i)*offset_corners +_ALFA1] = virtual_corners[i*offset_corners +_ALFA1];
            mapa->esquinas[(indice+i)*offset_corners +_ALFA2] = virtual_corners[i*offset_corners +_ALFA2];
            mapa->esquinas[(indice+i)*offset_corners +_RHO1] = virtual_corners[i*offset_corners +_RHO1];
            mapa->esquinas[(indice+i)*offset_corners +_RHO2] = virtual_corners[i*offset_corners +_RHO2];

            mapa->esquinas[(indice+i)*offset_corners + _TYPEc]= 0; // Comment: Esquina virtual

            mapa->R_esquinas[(indice+i)*offset_Rp]=matriz_virtual[i*offset_Rp];
            mapa->R_esquinas[(indice+i)*offset_Rp+1]=matriz_virtual[i*offset_Rp+1];
            mapa->R_esquinas[(indice+i)*offset_Rp+2]=matriz_virtual[i*offset_Rp+2];
            mapa->R_esquinas[(indice+i)*offset_Rp+3]=matriz_virtual[i*offset_Rp+3];
            mapa->R_esquinas[(indice+i)*offset_Rp+4]=matriz_virtual[i*offset_Rp+4];
            mapa->R_esquinas[(indice+i)*offset_Rp+5]=matriz_virtual[i*offset_Rp+5];

#ifdef DEBUGVC
            std::cout << "coord esq.: " << mapa->esquinas[mapa->n_esquinas*offset_corners + _X] << " " << mapa->esquinas[mapa->n_esquinas*offset_corners+_Y]
			      << " " << mapa->esquinas[mapa->n_esquinas*4+_THETA] << " (" << mapa->esquinas[j*4+2] << ")" << std::endl;
		    std::cout << "matriz_R[0]: " << mapa->R_esquinas[(indice+i)*offset_Rp] << "\nmatriz_R[1]: "
			      <<  mapa->R_esquinas[(indice+i)*offset_Rp+1] << "\nmatriz_R[2]: "
			      << mapa->R_esquinas[(indice+i)*offset_Rp+2] << "\nmatriz_R[3]: "
			      << mapa->R_esquinas[(indice+i)*offset_Rp+3] << std::endl;
#endif

            if (mapa->R_esquinas[(indice+i)*offset_Rp]<100000) {

#ifdef DEBUGVC
                printf("VC=[%f,%f]; P=[%f %f;%f %f; %f %f]\n",-mapa->esquinas[mapa->n_esquinas*4+1],mapa->esquinas[mapa->n_esquinas*4],mapa->R_esquinas[(indice+i)*offset_Rp],mapa->R_esquinas[(indice+i)*offset_Rp+2],mapa->R_esquinas[(indice+i)*offset_Rp+1],mapa->R_esquinas[(indice+i)*offset_Rp+3],mapa->R_esquinas[(indice+i)*offset_Rp+4],mapa->R_esquinas[(indice+i)*offset_Rp+5]);
#endif
            }



            mapa->n_esquinas++;
        }
    }

}
void  SpecificWorker::obtener_mapa(MAPA_LASER *mapa)
{
    int i;
    int X_INI,Y_INI,X_FIN,Y_FIN;
    int RO, THETA, ALFA, R;
    int n_segm;

    // Comment: "[LE] Segmentos"
    n_segm=cont_segment;
    X_INI=1; Y_INI=2; X_FIN=3; Y_FIN=4;
    RO=1; THETA=2; ALFA=5; R=6;

    mapa->n_segmentos=(int)n_segm;

    for (i=0;i<n_segm;i++)
    {
        mapa->segmentos[i*offset_segments+X_INI]=array_segment_local[i].x_ini;
        mapa->segmentos[i*offset_segments+Y_INI]=array_segment_local[i].y_ini;
        mapa->segmentos[i*offset_segments+X_FIN]=array_segment_local[i].x_fin;
        mapa->segmentos[i*offset_segments+Y_FIN]=array_segment_local[i].y_fin;

        mapa->segmentos[i*offset_segments+ALFA]=array_segment_local[i].lalfa;
        mapa->segmentos[i*offset_segments+R]=array_segment_local[i].lr;

        mapa->R_segmentos[i*offset_Rs]=array_segment_local[i].lcalfa;    // sigmaalfaalfa
        mapa->R_segmentos[i*offset_Rs+1]=array_segment_local[i].lcr;     // sigmaphopho
        mapa->R_segmentos[i*offset_Rs+2]=array_segment_local[i].lcalfar; // sigmaalfapho
        mapa->R_segmentos[i*offset_Rs+3]=array_segment_local[i].longitud;

// 		std::cout << "----obtener_mapa----" << std::endl;
// 		std::cout << "segmento: (alfa, d, length)" << mapa->segmentos[i*offset_segments+ALFA] *180/M_PI << ", " << mapa->segmentos[i*offset_segments+R] << ", " << mapa->R_segmentos[i*offset_Rs+3] << std::endl;
// 		std::cout << "segmento: (calfa, crho, calfarho)" << mapa->R_segmentos[i*offset_Rs] << ", " << mapa->R_segmentos[i*offset_Rs + 1] << ", " << mapa->R_segmentos[i*offset_Rs + 2] << std::endl;
// 		std::cout << "det2R: " << mapa->R_segmentos[i*offset_Rs]*mapa->R_segmentos[i*offset_Rs + 1] - mapa->R_segmentos[i*offset_Rs + 2] * mapa->R_segmentos[i*offset_Rs + 2] << std::endl;
// 		std::cout << "-----------" << std::endl;

    }
}
void  SpecificWorker::map_copy(MAPA_LASER *mapa, MAPA_LASER *ref)
{
    ref->n_segmentos = mapa->n_segmentos;
    ref->n_esquinas  = mapa->n_esquinas;
    ref->n_circulos  = mapa->n_circulos;

    ref->segmentos   = mapa->segmentos;
    ref->R_segmentos = mapa->R_segmentos;
    ref->esquinas    = mapa->esquinas;
    ref->R_esquinas  = mapa->R_esquinas;
    ref->circulos    = mapa->circulos;
    ref->R_circulos  = mapa->R_circulos;

}
void  SpecificWorker::matriz_covarianza_circulo(CIRCULO circle, double *matriz, int cont)
{
    double N,D;

    double a, b, c, d, e, f;

    a = circle.vector[0]; b = circle.vector[1]; c = circle.vector[2]; d = circle.vector[3]; e = circle.vector[4]; f = circle.vector[5];

    double dadx1, dbdx1, dcdx1, dddx1, dedx1, dfdx1;
    double dadx2, dbdx2, dcdx2, dddx2, dedx2, dfdx2;
    double dadx3, dbdx3, dcdx3, dddx3, dedx3, dfdx3;

    double dady1, dbdy1, dcdy1, dddy1, dedy1, dfdy1;
    double dady2, dbdy2, dcdy2, dddy2, dedy2, dfdy2;
    double dady3, dbdy3, dcdy3, dddy3, dedy3, dfdy3;

    double dRdx1, dRdx2, dRdx3, dRdy1, dRdy2, dRdy3;

    dadx1 = -2; dbdx1 =  0; dcdx1 =  2*circle.puntos[0]; dddx1 = -2; dedx1 = 0; dfdx1 = 2*circle.puntos[0];
    dadx2 =  2; dbdx2 =  0; dcdx2 = -2*circle.puntos[2]; dddx2 =  0; dedx2 = 0; dfdx2 = 0;
    dadx3 =  0; dbdx3 =  0; dcdx3 =  0; dddx3 = 2; dedx3 = 0; dfdx3 = -2*circle.puntos[4];
    dady1 =  0; dbdy1 = -2; dcdy1 =  2*circle.puntos[1]; dddy1 = 0; dedy1 = -2; dfdy1 = 2*circle.puntos[1];
    dady2 =  0; dbdy2 =  2; dcdy2 = -2*circle.puntos[3]; dddy2 = 0; dedy2 = 0;  dfdy2 = 0;
    dady3 =  0; dbdy3 =  0; dcdy3 =  0; dddy3 = 0; dedy3 = 2; dfdy3 = -2*circle.puntos[5];

    double dxdx1, dxdx2, dxdx3, dxdy1, dxdy2, dxdy3, dydx1, dydx2, dydx3, dydy1, dydy2, dydy3, drdx1, drdx2, drdx3, drdy1, drdy2, drdy3;

    double SIGMA_X1,SIGMA_Y1,SIGMA_XY1;
    double SIGMA_X2,SIGMA_Y2,SIGMA_XY2;
    double SIGMA_X3,SIGMA_Y3,SIGMA_XY3;

    double theta, rho;

    double R;

    double M11, M12, M13, M14, M15, M16, M21, M22, M23, M24, M25, M26, M31, M32, M33, M34, M35, M36;

#ifdef DEBUG_CR
    printf("(x,y): (%f,%f)|(%f,%f)|(%f,%f)\n",circle.puntos[0],circle.puntos[1],
		circle.puntos[2],circle.puntos[3],circle.puntos[4],circle.puntos[5]);
#endif

    N = a*f - c*d;
    D = b*d - a*e;
    R = (circle.puntos[0] - circle.x)*(circle.puntos[0] - circle.x)
        + ((circle.puntos[1] - circle.y)*(circle.puntos[1] - circle.y));

#ifdef DEBUG_CR
    printf("(xc,yc,r): (%f,%f,%f)\n",(N*circle.vector[1]/(D*circle.vector[0]) +circle.vector[2]/circle.vector[0]),N/D,sqrt(R));
	printf("f: %f", circle.puntos[0]*circle.puntos[0] + circle.puntos[1]*circle.puntos[1] - circle.puntos[4]*circle.puntos[4] - circle.puntos[5]*circle.puntos[5]);
#endif


    dydx1  = (((dadx1*f + dfdx1*a) - (dcdx1*d + dddx1*c))*D - ((dbdx1*d + dddx1*b) - (dadx1*e + dedx1*a))*N)/(D*D);
    dydx2  = (((dadx2*f + dfdx2*a) - (dcdx2*d + dddx2*c))*D - ((dbdx2*d + dddx2*b) - (dadx2*e + dedx2*a))*N)/(D*D);
    dydx3  = (((dadx3*f + dfdx3*a) - (dcdx3*d + dddx3*c))*D - ((dbdx3*d + dddx3*b) - (dadx3*e + dedx3*a))*N)/(D*D);

    dydy1  = (((dady1*f + dfdy1*a) - (dcdy1*d + dddy1*c))*D - ((dbdy1*d + dddy1*b) - (dady1*e + dedy1*a))*N)/(D*D);
    dydy2  = (((dady2*f + dfdy2*a) - (dcdy2*d + dddy2*c))*D - ((dbdy2*d + dddy2*b) - (dady2*e + dedy2*a))*N)/(D*D);
    dydy3  = (((dady3*f + dfdy3*a) - (dcdy3*d + dddy3*c))*D - ((dbdy3*d + dddy3*b) - (dady3*e + dedy3*a))*N)/(D*D);

    dxdx1  = -(dydx1*b/a + N/D*(dbdx1*a - dadx1*b)/(a*a) + (dcdx1*a - dadx1*c)/(a*a));
    dxdx2  = -(dydx2*b/a + N/D*(dbdx2*a - dadx2*b)/(a*a) + (dcdx2*a - dadx2*c)/(a*a));
    dxdx3  = -(dydx3*b/a + N/D*(dbdx3*a - dadx3*b)/(a*a) + (dcdx3*a - dadx3*c)/(a*a));

    dxdy1  = -(dydx1*b/a + N/D*(dbdy1*a - dady1*b)/(a*a) + (dcdy1*a - dady1*c)/(a*a));
    dxdy2  = -(dydx2*b/a + N/D*(dbdy2*a - dady2*b)/(a*a) + (dcdy2*a - dady2*c)/(a*a));
    dxdy3  = -(dydx3*b/a + N/D*(dbdy3*a - dady3*b)/(a*a) + (dcdy3*a - dady3*c)/(a*a));

    dRdx1 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdx1) + 2*(circle.puntos[1] - circle.y)*(0 - dydx1));
    dRdx2 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdx2) + 2*(circle.puntos[1] - circle.y)*(0 - dydx2));
    dRdx3 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdx3) + 2*(circle.puntos[1] - circle.y)*(0 - dydx3));
    dRdy1 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdy1) + 2*(circle.puntos[1] - circle.y)*(0 - dydy1));
    dRdy2 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdy2) + 2*(circle.puntos[1] - circle.y)*(0 - dydy2));
    dRdy3 =   (2*(circle.puntos[0] - circle.x)*(0 - dxdy3) + 2*(circle.puntos[1] - circle.y)*(0 - dydy3));

    drdx1 = (1/(2*sqrt(R)))*dRdx1;
    drdx2 = (1/(2*sqrt(R)))*dRdx2;
    drdx3 = (1/(2*sqrt(R)))*dRdx3;
    drdy1 = (1/(2*sqrt(R)))*dRdy1;
    drdy2 = (1/(2*sqrt(R)))*dRdy2;
    drdy3 = (1/(2*sqrt(R)))*dRdy3;

    // Comment: "[LE] parametros para la matriz de covarianza"
    rho   = sqrt( circle.puntos[0]*circle.puntos[0] + circle.puntos[1]*circle.puntos[1]);
    theta = atan2(circle.puntos[0],circle.puntos[1])-PI/2;

#ifdef DEBUG_CR
    printf("(r,theta): (%f,%f)\n",rho,theta);
#endif
    if (theta < 0) theta+=PI;

#ifdef DEBUG_CR
    printf("(r,theta): (%f,%f)\n",rho,theta*180/PI);
#endif


    SIGMA_X1   = SIGMA_R*SIGMA_R*sin(theta)*sin(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*cos(theta)*cos(theta);
    SIGMA_XY1  = SIGMA_R*SIGMA_R*sin(theta)*cos(theta) - rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*cos(theta);
    SIGMA_Y1   = SIGMA_R*SIGMA_R*cos(theta)*cos(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*sin(theta);

    rho = sqrt(circle.puntos[2]*circle.puntos[2] + circle.puntos[3]*circle.puntos[3]);
    theta = atan2(circle.puntos[2],circle.puntos[3])-PI/2;

    if (theta < 0) theta+=PI;

#ifdef DEBUG_CR
    printf("(r,theta): (%f,%f)\n",rho,theta*180/PI);
#endif

    SIGMA_X2   = SIGMA_R*SIGMA_R*sin(theta)*sin(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*cos(theta)*cos(theta);
    SIGMA_XY2  = SIGMA_R*SIGMA_R*sin(theta)*cos(theta) - rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*cos(theta);
    SIGMA_Y2   = SIGMA_R*SIGMA_R*cos(theta)*cos(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*sin(theta);

    rho=sqrt(circle.puntos[4]*circle.puntos[4]+circle.puntos[5]*circle.puntos[5]);
    theta= atan2(circle.puntos[4],circle.puntos[5])-PI/2;
    if (theta < 0) theta+=PI;

#ifdef DEBUG_CR
    printf("(r,theta): (%f,%f)\n",rho,theta*180/PI);
#endif
    SIGMA_X3   = SIGMA_R*SIGMA_R*sin(theta)*sin(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*cos(theta)*cos(theta);
    SIGMA_XY3   = SIGMA_R*SIGMA_R*sin(theta)*cos(theta) - rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*cos(theta);
    SIGMA_Y3  = SIGMA_R*SIGMA_R*cos(theta)*cos(theta) + rho*rho*SIGMA_THETA*SIGMA_THETA*sin(theta)*sin(theta);

#ifdef DEBUG_CR
    printf("sigmas: (%f,%f,%f) (%f,%f,%f) (%f,%f,%f) ", SIGMA_X1,SIGMA_Y1,SIGMA_XY1,
		SIGMA_X2,SIGMA_Y2,SIGMA_XY2,SIGMA_X3,SIGMA_Y3,SIGMA_XY3);
#endif

    M11= dxdx1 * SIGMA_X1  + dxdy1 * SIGMA_XY1;
    M12= dxdx1 * SIGMA_XY1 + dxdy1 * SIGMA_Y1;
    M13= dxdx2 * SIGMA_X2  + dxdy2 * SIGMA_XY2;
    M14= dxdx2 * SIGMA_XY2 + dxdy2 * SIGMA_Y2;
    M15= dxdx3 * SIGMA_X3  + dxdy3 * SIGMA_XY3;
    M16= dxdx3 * SIGMA_XY3 + dxdy3 * SIGMA_Y3;

    M21= dydx1 * SIGMA_X1  + dydy1 * SIGMA_XY1;
    M22= dydx1 * SIGMA_XY1 + dydy1 * SIGMA_Y1;
    M23= dydx2 * SIGMA_X2  + dydy2 * SIGMA_XY2;
    M24= dydx2 * SIGMA_XY2 + dydy2 * SIGMA_Y2;
    M25= dydx3 * SIGMA_X3  + dydy3 * SIGMA_XY3;
    M26= dydx3 * SIGMA_XY3 + dydy3 * SIGMA_Y3;

    M31= drdx1 * SIGMA_X1  + drdy1 * SIGMA_XY1;
    M32= drdx1 * SIGMA_XY1 + drdy1 * SIGMA_Y1;
    M33= drdx2 * SIGMA_X2  + drdy2 * SIGMA_XY2;
    M34= drdx2 * SIGMA_XY2 + drdy2 * SIGMA_Y2;
    M35= drdx3 * SIGMA_X3  + drdy3 * SIGMA_XY3;
    M36= drdx3 * SIGMA_XY3 + drdy3 * SIGMA_Y3;


    matriz[offset_Rc*cont]  = M11*dxdx1 + M12*dxdy1 + M13*dxdx2 + M14*dxdy2 + M15*dxdx3 + M16*dxdy3; // sigmaxx
    matriz[offset_Rc*cont+1]= M21*dydx1 + M22*dydy1 + M23*dydx2 + M24*dydy2 + M25*dydx3 + M26*dydy3; // sigmayy

    matriz[offset_Rc*cont+2]= M11*dydx1 + M12*dydy1 + M13*dydx2 + M14*dydy2 + M15*dydx3 + M16*dydy3; // sigmaxy
    matriz[offset_Rc*cont+3]= M31*drdx1 + M32*drdy1 + M33*drdx2 + M34*drdy2 + M35*drdx3 + M36*drdy3; // sigmaphopho

    matriz[offset_Rc*cont+4]= M11*drdx1 + M12*drdy1 + M13*drdx2 + M14*drdy2 + M15*drdx3 + M16*drdy3; // sigmaxpho
    matriz[offset_Rc*cont+5]= M21*drdx1 + M22*drdy1 + M23*drdx2 + M24*drdy2 + M25*drdx3 + M26*drdy3; // sigmaypho
}

void SpecificWorker::segmento_Kai(int inicio, int fin)
{
    std::vector<double> puntos = array_points;
    //double xi,xf,yi,yf;
    double sum_x;double sum_y;
    double w; double w_i;

    double c_alfa; double c_r; double c_ralfa;
    double r,alfa;
    double N,D; //double N_,D_;

    int i; //int j;

    double dalfa=0, drho=0;
    int num_puntos;

    w_i=1;

    sum_x=sum_y=w=0;
    num_puntos=(fin-inicio);


    for (i=inicio;i<=fin;i++)
    {

        w+= w_i;
        sum_x = sum_x+w_i*puntos[offset_points*i+_R]*cos(puntos[offset_points*i+_Q]);
        sum_y = sum_y+w_i*puntos[offset_points*i+_R]*sin(puntos[offset_points*i+_Q]);

    }

    sum_x = (1/w)*sum_x;
    sum_y = (1/w)*sum_y;

    N=0;D=0;

    for (i=inicio;i<=fin;i++)
    {
        N+= w_i*(sum_y-puntos[offset_points*i+_R]*sin(puntos[offset_points*i+_Q]))*(sum_x-puntos[offset_points*i+_R]*cos(puntos[offset_points*i+_Q]));
        D+= w_i*((sum_y-puntos[offset_points*i+_R]*sin(puntos[offset_points*i+_Q]))*(sum_y-puntos[offset_points*i+_R]*sin(puntos[offset_points*i+_Q])) -
                 (sum_x-puntos[offset_points*i+_R]*cos(puntos[offset_points*i+_Q]))*(sum_x-puntos[offset_points*i+_R]*cos(puntos[offset_points*i+_Q])));
    }

    // Comment: "usamos la arcontangente cuarto-cuadrante"

    N=-2*N;
    alfa=(double)0.5*(double)(atan2(N,D));
    r=sum_x*cos(alfa) + sum_y*sin(alfa);


    if (r<0) {
        r=-r;
        alfa+=(double)PI;
        if (alfa>PI) alfa-=2*(double)PI;
    }

    segmento[_ALFA]=alfa;
    segmento[_RO]=r;


//   printf("\n[SEGM]: (alfa,r) (%f,%f) inicio(r,th) (%f,%f) (x,y) (%f,%f)\n",alfa*180/PI,r,puntos[3*inicio+1],puntos[3*inicio+2]*180/PI,
//     puntos[3*inicio+1]*cos(puntos[3*inicio+2]),puntos[3*inicio+1]*sin(puntos[3*inicio+2]));


    extremo_kai(puntos[offset_points*inicio+_R],puntos[offset_points*inicio+_Q],alfa,r,&segmento[_XINI],&segmento[_YINI]);


//  printf("\n[SEGM]: (alfa,r)  (%f,%f) fin(r,th) (%f,%f) (x,y) (%f,%f)\n",alfa*180/PI,r,puntos[3*fin+1],puntos[3*fin+2]*180/PI,
//	  puntos[3*fin+1]*cos(puntos[3*fin+2]),puntos[3*fin+1]*sin(puntos[3*fin+2]));

    extremo_kai(puntos[offset_points*fin+_R],puntos[offset_points*fin+_Q],alfa,r,&segmento[_XFIN],&segmento[_YFIN]);


    segmento[_DIST]=sqrt((segmento[_XINI]-segmento[_XFIN])*(segmento[_XINI]-segmento[_XFIN]) +
                        (segmento[_YINI]-segmento[_YFIN])*(segmento[_YINI]-segmento[_YFIN]));


    c_alfa=0; c_r=0; c_ralfa=0;


    for (i=inicio;i<=fin;i++)
    {

        dalfa=N*(sum_x*cos(puntos[offset_points*i+_Q]) - sum_y*sin(puntos[offset_points*i+_Q]) - puntos[offset_points*i+_R]*cos(2*puntos[offset_points*i+_Q])) -
              D*(sum_x*sin(puntos[offset_points*i+_Q]) + sum_y*cos(puntos[offset_points*i+_Q]) - puntos[offset_points*i+_R]*sin(2*puntos[offset_points*i+_Q]));

        dalfa=dalfa/(D*D + N*N);

        drho=(1/w)*cos(puntos[offset_points*i+_Q] -alfa) + dalfa*(sum_y*cos(alfa) - sum_x*sin(alfa));

        c_alfa+=w_i*w_i*pow(N*(sum_x*cos(puntos[offset_points*i+_Q]) - sum_y*sin(puntos[offset_points*i+_Q]) - puntos[offset_points*i+_R]*cos(2*puntos[offset_points*i+_Q]))
                            -D*(sum_x*sin(puntos[offset_points*i+_Q]) + sum_y*cos(puntos[offset_points*i+_Q]) - puntos[offset_points*i+_R]*sin(2*puntos[offset_points*i+_Q])),2)*
                SIGMA_R*SIGMA_R;

        c_r+=pow((w_i/w)*cos((puntos[offset_points*i+_Q])-alfa) + (sum_y*cos(alfa) - sum_x*sin(alfa))*dalfa,2)*SIGMA_R*SIGMA_R;

        c_ralfa+=dalfa*drho*SIGMA_R*SIGMA_R;

    }

    c_alfa=c_alfa/((N*N + D*D)*(N*N + D*D));

    if (c_alfa<0) {printf("[DEBUG line segments: error in covariance matrix. c_alfa is a Negativa value.\n");
    }
    segmento[_CALFA]=c_alfa*SN_Rth;
    segmento[_CR]=c_r*SN_Rxy;
    if (c_r<0) {printf("[DEBUG line segments: error in covariance matrix. c_r is a Negativa value.\n");
    }
    segmento[_CALFAR]=c_ralfa*SN_Rxy;


    if ((segmento[_CALFA]*segmento[_CR] - segmento[_CALFAR]*segmento[_CALFAR]) >  (MAX_DET_Rs*1e6)) {
        segmento[_DIST]=0;
    }
}

void  SpecificWorker::matriz_covarianza_virtual(SEGMENTO segmento_2,SEGMENTO segmento_1, std::vector<double> matriz, int cont)
{

    // definicion de parametros tmps
    double A,B,C,D,E,F,G,H, I, J;
    double A_,B_,C_,D_,E_,F_,G_,H_;

    double senodif_alfa, seno_alfa1, seno_alfa2;
    double cosenodif_alfa, coseno_alfa1, coseno_alfa2;

    // calculo de parametros

    senodif_alfa   = sin(segmento_2.lalfa - segmento_1.lalfa);
    seno_alfa1     = sin(segmento_1.lalfa);
    seno_alfa2     = sin(segmento_2.lalfa);
    cosenodif_alfa = sin(segmento_2.lalfa - segmento_1.lalfa);
    coseno_alfa1   = cos(segmento_1.lalfa);
    coseno_alfa2   = cos(segmento_2.lalfa);

    A = (seno_alfa2)/(senodif_alfa);
    B = (-segmento_2.lr*coseno_alfa1*senodif_alfa + cosenodif_alfa*(segmento_1.lr*seno_alfa2 - segmento_2.lr*seno_alfa1))/(senodif_alfa*senodif_alfa);
    C = -seno_alfa1/(senodif_alfa);
    D = (segmento_1.lr*coseno_alfa2*senodif_alfa -(segmento_1.lr*seno_alfa2 - segmento_2.lr*seno_alfa1)*cosenodif_alfa)/(senodif_alfa*senodif_alfa);
    E = -coseno_alfa2/senodif_alfa;
    F = (-segmento_2.lr*senodif_alfa*seno_alfa1 +cosenodif_alfa*(segmento_2.lr*coseno_alfa1 - segmento_1.lr*coseno_alfa2))/(senodif_alfa*senodif_alfa);
    G = coseno_alfa1/(senodif_alfa);
    H = (segmento_1.lr*seno_alfa2*senodif_alfa - cosenodif_alfa*(segmento_2.lr*coseno_alfa1 - segmento_1.lr*coseno_alfa2))/(senodif_alfa*senodif_alfa);
    I = -0.5;
    J = 0.5;

    A_ = A*segmento_1.lcr + B*segmento_1.lcalfar;
    B_ = A*segmento_1.lcalfar + B*segmento_1.lcalfa;

    C_ = C*segmento_2.lcr + D*segmento_2.lcalfar;
    D_ = C*segmento_2.lcalfar + D*segmento_2.lcalfa;

    E_ = E*segmento_1.lcr + F*segmento_1.lcalfar;
    F_ = E*segmento_1.lcalfar + F*segmento_1.lcalfa;

    G_ = G*segmento_2.lcr + H*segmento_2.lcalfar;
    H_ = G*segmento_2.lcalfar + H*segmento_2.lcalfa;

    matriz[offset_Rp*cont+0] = A*A_ + B*B_ + C*C_ + D*D_;
    matriz[offset_Rp*cont+1] = E*E_ + F*F_ + G*G_ + H*H_;

    matriz[offset_Rp*cont+2] = A*E_ + B*F_ + C*G_ + D*H_;


    // comprobaciones
#ifdef DEBUGVC
    std::cout<<"[VC] sigma_x: "<<matriz[offset_Rp*cont+0]<<" sigma_y: "<<matriz[offset_Rp*cont+1]<<std::endl;
  std::cout<<"[VC] sigma_xy:"<<matriz[offset_Rp*cont+2]<<" sigma_yx:"<<(A_*E + B_*F + C_*G + D_*H)<<std::endl;

  if (matriz[offset_Rp*cont+0]<0 || matriz[offset_Rp*cont +1]<0) {
    std::cout<<"[VC ERROR] error en el calculo de la matriz de incertidumbre (valores negativos)"<<std::endl;
  }

  if (matriz[offset_Rp*cont+2]!= (A_*E + B_*F + C_*G + D_*H)){
    std::cout<<"[VC ERROR] error en el calculo de la matriz de incertidumbre (valores distintos cruzados)"<<std::endl;
	std::cout<<"[VC ERROR] sigma_xy:"<<matriz[offset_Rp*cont+2]<<" sigma_yx:"<<(A_*E + B_*F + C_*G + D_*H)<<std::endl;
  }

#endif

    matriz[offset_Rp*cont+4] = B_*I+D_*J;
    matriz[offset_Rp*cont+5] = F_*I+H_*J;

    matriz[offset_Rp*cont+3] = I*segmento_1.lcalfa*I + J*segmento_2.lcalfa*J;

#ifdef DEBUGVC
    std::cout<<"[VC] sigma_alfa: "<<matriz[offset_Rp*cont+3]<<std::endl;
  std::cout<<"[VC] sigma_xalfa: "<<matriz[offset_Rp*cont+4]<<std::endl;
  std::cout<<"[VC] sigma_yalfa: "<<matriz[offset_Rp*cont+5]<<std::endl;

	if (matriz[offset_Rp*cont+4]!= (I*segmento_1.lcalfar*A+I*segmento_1.lcalfa*B+J*segmento_2.lcalfar*C+J*segmento_2.lcalfa*D)){
		std::cout<<"[VC ERROR] error en el calculo de la matriz de incertidumbre (valores distintos cruzados)"<<std::endl;
		std::cout<<"[VC ERROR] sigma_xalfa:"<<matriz[offset_Rp*cont+4]<<" sigma_alfax:"
			 << (I*segmento_1.lcalfar*A+I*segmento_1.lcalfa*B+J*segmento_2.lcalfar*C+J*segmento_2.lcalfa*D)<<std::endl;
	}

	if (matriz[offset_Rp*cont+5]!= (I*segmento_1.lcalfar*E+I*segmento_1.lcalfa*F+J*segmento_2.lcalfar*G+J*segmento_2.lcalfa*H)){
		std::cout<<"[VC ERROR] error en el calculo de la matriz de incertidumbre (valores distintos cruzados)"<<std::endl;
		std::cout<<"[VC ERROR] sigma_yalfa:"<<matriz[offset_Rp*cont+5]<<" sigma_alfay:"
			 <<(I*segmento_1.lcalfar*E+I*segmento_1.lcalfa*F+J*segmento_2.lcalfar*G+J*segmento_2.lcalfa*H)<<std::endl;
	}
#endif

}

double SpecificWorker::dist_euclidean2D(double x1, double y1, double x2, double y2){

#ifdef DEBUGCCDA
    // printf("[DistEuclidea] %f\n",  sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)));
#endif


    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));

}
void SpecificWorker::extremo_kai(double rho, double theta, double alfa, double r, double *x1, double *y1)
{

    double r2, alfa2; double x,y;

    // Comment: punto de corte entre el punto (su recta generada) y la recta pasado como parametro
    alfa2=alfa+(double)(PI/2);

    x=rho*cos(theta);
    y=rho*sin(theta);


    r2=x*cos(alfa2) + y*sin(alfa2);

    if (r2<0) {alfa2+=(double)PI;r2=-r2;}

#ifdef __DEBUG__
    printf("[EXT] alfa2,r2: %f,%f x,y: %f,%f \n",alfa2*180/PI,r2,x,y);
#endif

    *y1=(r2*cos(alfa) - r*cos(alfa2))/(sin(alfa2)*cos(alfa) - sin(alfa)*cos(alfa2));
    *x1=(r - *y1*sin(alfa))/cos(alfa);


}



/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

