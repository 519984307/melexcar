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
    viewer = new AbstractGraphicViewer(this, QRect(-360, -240, 720, 480));

    scene = new QGraphicsScene();
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
        graph_viewer->add_custom_widget_to_dock("Melex's control laser", &custom_widget); //una pestaña

        local_view = new AbstractGraphicViewer(custom_widget.laser, QRectF(-5000, -5000, 10000, 10000));

		this->Period = period;
		timer.start(Period);
	}

    compression_params_image.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params_image.push_back(15);
    compression_params_depth.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params_depth.push_back(15);

    int fps_depth = 15;
    int fps_color = 15;

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
    auto &cam_map_extended = read_and_filter(cam_map);   // USE OPTIONAL
    std::cout << "EXTENDED SIZE: " << cam_map_extended.size() << std::endl;
    auto  ldata_local= compute_laser(cam_map_extended);
    //auto &&[virtual_frame] = mosaic(cam_map_extended);
    auto &&[v_f] = mosaic(cam_map_extended);
    my_mutex.lock();
        virtual_frame=v_f.clone();
    my_mutex.unlock();

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

    std::vector<std::vector<std::tuple<float, float, int>>> lasers;
    int i=0;
    for(auto laser:ldata_local){
        auto nombre=nombre_laser+std::to_string(i++);
        //draw_laser(laser);feature
//        features( laser, &scan_mapa, &scan_maparef);

        LaserAnalyzer analizador = LaserAnalyzer(laser, 500, 10000, 300);
        analizador.calcularPendiente();
        analizador.extraerBreakPoints();
        auto data = analizador.extraerDatosParaPintar();

        analizador.filtrarTipos(data);
        analizador.filtrarbptobp(data);
        lasers.push_back(data);

        //update_laser_node(nombre,laser);
    }
    draw_points(lasers);



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
                cout << "MAX_DOWN"<< consts.max_down_height << endl;
                qDebug() << "MAX_DOWN"<< consts.max_down_height << endl;
                auto diferencia= (consts.max_down_height-(consts.max_up_height))/num_planos;
                auto to_point = extrin * Eigen::Vector3f{vertices[i].x, vertices[i].y, vertices[i].z};
                const float &xv = to_point[0]; const float &yv = to_point[1]; const float &zv = to_point[2];
                //if (xv < RIG_ELEVATION_FROM_FLOOR * 0.9)
                if( xv < (0.35)  and xv > (0.15))  // central band - positive x downwards
                {
                    float hor_angle = atan2(yv, zv);
                    // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                    int angle_index = (int) ((consts.MAX_LASER_BINS / consts.TOTAL_HOR_ANGLE) * hor_angle + (consts.MAX_LASER_BINS / 2.f));
                    if (angle_index >= consts.MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins[angle_index].emplace(std::make_tuple(xv, yv, zv));
                }
                if( xv < (1.1) and xv > (0.9))  // central band - positive x downwards
                {
                    float hor_angle = atan2(yv, zv);
                    // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                    int angle_index = (int) ((consts.MAX_LASER_BINS / consts.TOTAL_HOR_ANGLE) * hor_angle + (consts.MAX_LASER_BINS / 2.f));
                    if (angle_index >= consts.MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins1[angle_index].emplace(std::make_tuple(xv, yv, zv));
                }
                if( xv < (1.8 )and xv > (1.6)) // central band - positive x downwards
                {
                    float hor_angle = atan2(yv, zv);
                    // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                    int angle_index = (int) ((consts.MAX_LASER_BINS / consts.TOTAL_HOR_ANGLE) * hor_angle + (consts.MAX_LASER_BINS / 2.f));
                    if (angle_index >= consts.MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins2[angle_index].emplace(std::make_tuple(xv, yv, zv));
                }
                if( xv < (0.8) and xv > (0.6))  // central band - positive x downwards
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
//            if (auto breakpoints_node = G->get_node(breakpoints_camera_name); breakpoints_node.has_value())
//            {
//                //auto laser_data = RoboCompLaser::TLaserData();
//                G->add_or_modify_attrib_local<laser_dists_att>(breakpoints_node.value(), distanbreak);
//                G->add_or_modify_attrib_local<laser_angles_att>(breakpoints_node.value(), angulobreak);
//                G->update_node(breakpoints_node.value());
//            }
//            else{
//                DSR::Node breakpoints = DSR::Node::create<laser_node_type>(breakpoints_camera_name);
//                G->add_or_modify_attrib_local<breakpoints_dists_att>(breakpoints, distanbreak );
//                G->add_or_modify_attrib_local<breakpoints_angles_att>(breakpoints, angulobreak);
//                G->insert_node(breakpoints);
//                DSR::Edge edge = DSR::Edge::create<RT_edge_type>(breakpoints.id(), laser_node.value().id());
//                G->insert_or_assign_edge(edge);
//            }
        }
        else
        {
            DSR::Node new_laser_node = DSR::Node::create<laser_node_type>(laser_name);//Añadir ID
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
void SpecificWorker::draw_points(std::vector<std::vector<std::tuple<float, float, int>>> data)
{
    qDebug() << "HOLAAAAAAAAAAAAAAAA"<< endl;
    static std::vector<QGraphicsItem *> points;
    qDebug() << __FUNCTION__  << data.size();
    for (QGraphicsItem* item : points)
    {
        local_view->scene.removeItem(item);
        delete item;
    }
    points.clear();

    for (int i = 0; i < data.size(); i++)
    {
        auto laser = data[i];
        for(auto &p: laser)
        {
            auto &[x, y, type] = p;
            int factor = i < 2 ? 1 : 0;
            int x_coord = 2500 * (i%2);
            int y_coord = 2500 * factor;
            QRect dentro(x/10 +1 + x_coord, -y/10 +1 + y_coord, 1, 1);
            QRect fuera(x/10 + x_coord, -y/10 + y_coord, 3, 3);
            points.push_back(local_view->scene.addRect(fuera, QPen(colors[type])));
            points.push_back(local_view->scene.addRect(dentro, QPen(colors[type])));
        }
    }
    qDebug() << __FUNCTION__ << "Termina";
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

