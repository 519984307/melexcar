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

#include "specificworker.h"
#include <cppitertools/enumerate.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <unordered_map>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");
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
    route_filename = params["route_filename"].value;
    stop_1 = stoi(params["stop_1"].value);
    stop_2 = stoi(params["stop_2"].value);
    stop_3 = stoi(params["stop_3"].value);
    stop_4 = stoi(params["stop_4"].value);
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
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
        int current_opts = 0;
        opts main = opts::none;

        if(tree_view)
		    current_opts = current_opts | opts::tree;
		if(graph_view)
            current_opts = current_opts | opts::graph;
		if(qscene_2d_view)
		    current_opts = current_opts | opts::scene;
		if(osg_3d_view)
		    current_opts = current_opts | opts::osg;

        main = opts::graph;

        graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        // 2D widget
        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        widget_2d->set_draw_laser(true);

        // custom widget
        graph_viewer->add_custom_widget_to_dock("Melex's route", &custom_widget);

        connect(custom_widget.pushButton_start_mission, SIGNAL(clicked()), this, SLOT(slot_start_mission()));
        connect(custom_widget.pushButton_stop_mission, SIGNAL(clicked()), this, SLOT(slot_stop_mission()));
        connect(custom_widget.pushButton_cancel_mission, SIGNAL(clicked()), this, SLOT(slot_cancel_mission()));


        custom_widget.layout()->addWidget(widget_2d);
        custom_widget.raise();

        // stacked widgets
        point_dialog.setupUi(custom_widget.stacked_widget->widget(0));
        pathfollow_dialog.setupUi(custom_widget.stacked_widget->widget(1));
        pathfollow_dialog.list_pickuppoints->clear();
        connect(pathfollow_dialog.list_pickuppoints, SIGNAL(currentIndexChanged(int)), this , SLOT(slot_change_pickup_selector(int)));
        pathfollow_dialog.list_destinationpoints->clear();
        connect(pathfollow_dialog.list_destinationpoints, SIGNAL(currentIndexChanged(int)), this , SLOT(slot_change_destination_selector(int)));

        auto empty_widget = new QWidget();
        custom_widget.stacked_widget->addWidget(empty_widget);
        custom_widget.stacked_widget->setCurrentIndex(2);
        QRectF dimensions;
        auto world_node = G->get_node(world_name).value();
        dimensions.setLeft(G->get_attrib_by_name<OuterRegionLeft_att>(world_node).value());
        dimensions.setTop(G->get_attrib_by_name<OuterRegionTop_att>(world_node).value());
        dimensions.setRight(G->get_attrib_by_name<OuterRegionRight_att>(world_node).value());
        dimensions.setBottom(G->get_attrib_by_name<OuterRegionBottom_att>(world_node).value());

        // get camera_api
//        if(auto cam_node = G->get_node(pioneer_camera_virtual_name); cam_node.has_value())
//            cam_api = G->get_camera_api(cam_node.value());
//        else
//        {
//            std::cout << "Controller-DSR terminate: could not find a camera node named " << pioneer_head_camera_right_name << std::endl;
//            std::terminate();
//        }

        // Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // Robot polygon
        if(auto robot_body = G->get_node(robot_body_name); robot_body.has_value())
        {
            auto width = G->get_attrib_by_name<width_att>(robot_body.value());
            auto height = G->get_attrib_by_name<depth_att>(robot_body.value());
            cout << "Width " << width.value() << " height " << height.value() << endl;
            if (width.has_value() and height.has_value())
            {
                robot_polygon << QPointF(-width.value() / 2, -height.value() / 2)
                              << QPointF(-width.value() / 2, height.value() / 2)
                              << QPointF(width.value() / 2, height.value() / 2)
                              << QPointF(width.value() / 2, -height.value() / 2);
            } else
            {
                std::cout << __FUNCTION__ << " No robot body width or depth found. Terminating..." << std::endl;
                std::terminate();
            }
        }
        else
        {
            std::cout << __FUNCTION__ << " No robot body found. Terminating..." << std::endl;
            std::terminate();
        }

        // trail
//        auto robot_pos = inner_eigen->transform(world_name, robot_name).value();
//        last_point = QPointF(robot_pos.x(), robot_pos.y());
//        connect( custom_widget.path_trail_button, SIGNAL(toggled(bool)), this, SLOT(trace_button_slot(bool)));

        // Eigen format
//        OctaveFormat = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
//        CommaInitFmt = Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

        this->Period = 100;
		timer.start(Period);

        //carga ruta
        load_path(route_filename);
        xpts.erase(xpts.end()-1); ypts.erase(ypts.end()-1);
        temporary_plan.x_path.resize(xpts.size());
        temporary_plan.y_path.resize(ypts.size());
        std::copy( xpts.begin(), xpts.end(),temporary_plan.x_path.begin());
        std::copy( ypts.begin(), ypts.end(), temporary_plan.y_path.begin());
//        follow_path_copy_path_to_graph(temporary_plan.x_path, temporary_plan.y_path);
        std::vector<Eigen::Vector2f> local_path;  // for drawing
        for(auto &&[x,y] : iter::zip(xpts, ypts))
            local_path.emplace_back(Eigen::Vector2f(x,y));

        draw_path(local_path, &widget_2d->scene);

        //paint stop
        widget_2d->scene.removeItem(target_pickup_scene);
        target_pickup_scene = widget_2d->scene.addEllipse(-100, -100, 400, 400, QPen(QColor("Red")),
                                                          QBrush(QColor("Red")));
        target_pickup_scene->setPos(xpts[stop_1], ypts[stop_1]);
        target_pickup_scene->setZValue(100);

        target_pickup_scene = widget_2d->scene.addEllipse(-100, -100, 400, 400, QPen(QColor("Blue")),
                                                          QBrush(QColor("Green")));
        target_pickup_scene->setPos(xpts[stop_2], ypts[stop_2]);
        target_pickup_scene->setZValue(100);

        target_pickup_scene = widget_2d->scene.addEllipse(-100, -100, 400, 400, QPen(QColor("Orange")),
                                                          QBrush(QColor("Orange")));
        target_pickup_scene->setPos(xpts[stop_3], ypts[stop_3]);
        target_pickup_scene->setZValue(100);
        target_pickup_scene = widget_2d->scene.addEllipse(-100, -100, 400, 400, QPen(QColor("Yellow")),
                                                          QBrush(QColor("Yellow")));
        target_pickup_scene->setPos(xpts[stop_4], ypts[stop_4]);
        target_pickup_scene->setZValue(100);

    }
}

void SpecificWorker::compute()
{
    static std::chrono::steady_clock::time_point begin, lastPathStep;

    // check for existing missions
    if (auto plan_o = plan_buffer.try_get(); plan_o.has_value()) {
        current_plan = plan_o.value();
        std::cout << __FUNCTION__ << " New plan arrived: " << std::endl;
        std::cout << current_plan.pprint() << std::endl;
        custom_widget.textedit_current_plan->appendPlainText(
                "-> compute: initiating plan " + current_plan.get_action());
        current_plan.set_running();
    }

    if (auto robot_pos_o = inner_eigen->transform(world_name, robot_name); robot_pos_o.has_value())
        robot_pos = robot_pos_o.value();

    check_task_completed();


}

////////////////////////////////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    if(type == laser_type_name)
    {
        if( auto node = G->get_node(id); node.has_value())
        {
            auto angles = G->get_attrib_by_name<laser_angles_att>(node.value());
            auto dists = G->get_attrib_by_name<laser_dists_att>(node.value());
            if(dists.has_value() and angles.has_value())
            {
                if(dists.value().get().empty() or angles.value().get().empty()) return;
                laser_buffer.put(std::move(std::make_tuple(angles.value().get(), dists.value().get())),
                                 [this](const LaserData &in, std::tuple<std::vector<float>, std::vector<float>, QPolygonF,std::vector<QPointF>> &out) {
                                     QPolygonF laser_poly_local;
                                     std::vector<QPointF> laser_cart_world;
                                     const auto &[angles, dists] = in;
                                     for (const auto &[angle, dist] : iter::zip(angles, dists))
                                     {
                                         //convert laser polar coordinates to cartesian in the robot's coordinate frame
                                         float x = dist * sin(angle);
                                         float y = dist * cos(angle);
                                         Mat::Vector3d laserWorld = inner_eigen->transform(world_name,Mat::Vector3d(x, y, 0), laser_name).value();
                                         laser_poly_local << QPointF(x, y);
                                         laser_cart_world.emplace_back(QPointF(laserWorld.x(), laserWorld.y()));
                                     }
                                     out = std::make_tuple(angles, dists, laser_poly_local, laser_cart_world);
                                 });
            }
        }
    }
    else if (type == path_to_target_type_name)
    {
        if( auto path_to_target_node = G->get_node(id); path_to_target_node.has_value())
        {
            auto x_values_o = G->get_attrib_by_name<path_x_values_att>(path_to_target_node.value());
            auto y_values_o = G->get_attrib_by_name<path_y_values_att >(path_to_target_node.value());
            if(x_values_o.has_value() and y_values_o.has_value())
            {
                path.clear();
                auto &x_values = x_values_o.value().get();
                auto &y_values = y_values_o.value().get();
                path.reserve(x_values.size());
                for(auto &&[p, q] : iter::zip(x_values,y_values))
                    path.emplace_back(Eigen::Vector2f(p, q));
                draw_path(path, &widget_2d->scene);
            }
        }
    }
}

void SpecificWorker::insert_intention_node(const Plan &plan)
{
    // Check if there is not 'intention' node yet in G
    if(auto mind = G->get_node(robot_mind_name); mind.has_value())
    {
        if (auto intention = G->get_node(current_intention_name); not intention.has_value())
        {
            DSR::Node intention_node = DSR::Node::create<intention_node_type>(current_intention_name);
            G->add_or_modify_attrib_local<parent_att>(intention_node, mind.value().id());
            G->add_or_modify_attrib_local<parent_att>(intention_node, mind.value().id());
            G->add_or_modify_attrib_local<level_att>(intention_node, G->get_node_level(mind.value()).value() + 1);
            G->add_or_modify_attrib_local<pos_x_att>(intention_node, (float) -290);
            G->add_or_modify_attrib_local<pos_y_att>(intention_node, (float) -474);
            G->add_or_modify_attrib_local<current_intention_att>(intention_node, plan.to_json());
            if (std::optional<int> intention_node_id = G->insert_node(intention_node); intention_node_id.has_value())
            {
                DSR::Edge edge = DSR::Edge::create<has_edge_type>(mind.value().id(), intention_node.id());
                if (G->insert_or_assign_edge(edge))
                {
                    std::cout << __FUNCTION__ << " Edge successfully inserted: " << mind.value().id() << "->" << intention_node.id()
                              << " type: has" << std::endl;
                    G->add_or_modify_attrib_local<current_intention_att>(intention_node, plan.to_json());
                    G->update_node(intention_node);
                }
                else
                {
                    std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << mind.value().id() << "->" << intention_node_id.value()
                              << " type: has" << std::endl;
                    std::terminate();
                }
            } else
            {
                std::cout << __FUNCTION__ << ": Fatal error inserting_new 'intention' node" << std::endl;
                std::terminate();
            }
        }
        else // there is one intention node
        {
            std::cout << __FUNCTION__ << ": Updating existing intention node with Id: " << intention.value().id() << std::endl;
            G->add_or_modify_attrib_local<current_intention_att>(intention.value(), plan.to_json());
            G->update_node(intention.value());
            std::cout << "INSERT: " << plan.to_json() << std::endl;
        }
    }
    else
    {
        std::cout  << __FUNCTION__ << " No node " << robot_mind_name << " found in G" << std::endl;
        std::terminate();
    }
}

void SpecificWorker::insert_pickup_node(const Plan &plan, std::vector<std::pair<float, float>>task_pickup_point)
{
    std::vector<float> v;
    v.push_back(get<0>(task_pickup_point.back()));
    v.push_back(get<1>(task_pickup_point.back()));

    // Check if there is not 'intention' node yet in G
    if (auto mind = G->get_node(robot_mind_name); mind.has_value())
    {
        if (auto task = G->get_node(task_name); not task.has_value())
        {
            DSR::Node task_node = DSR::Node::create<task_node_type>(task_name);
            G->add_or_modify_attrib_local<task_assigned_att>(task_node, true);
            G->add_or_modify_attrib_local<task_car_att>(task_node, 1);
            G->add_or_modify_attrib_local<task_pickup_values_att>(task_node, v);
            G->add_or_modify_attrib_local<task_completed_att>(task_node, false);
            G->add_or_modify_attrib_local<task_movement_att>(task_node, true);
            if (std::optional<int> task_node_id = G->insert_node(task_node); task_node_id.has_value())
            {
                DSR::Edge edge = DSR::Edge::create<has_edge_type>(mind.value().id(), task_node.id());
                if (G->insert_or_assign_edge(edge))
                {
                    std::cout << __FUNCTION__ << " Edge successfully inserted: " << mind.value().id() << "->"
                              << task_node.id()
                              << " type: has" << std::endl;
                    G->update_node(task_node);
                }
                else
                {
                    std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << mind.value().id() << "->"
                              << task_node_id.value()
                              << " type: has" << std::endl;
                    std::terminate();
                }
            }
            else
            {
                std::cout << __FUNCTION__ << ": Fatal error inserting_new 'task' node" << std::endl;
                std::terminate();
            }
        }
        else
        {
            std::cout << __FUNCTION__ << " No node " << robot_mind_name << " found in G" << std::endl;
            std::terminate();
        }
    }
}

void SpecificWorker::insert_destination_coords(const Plan &plan, std::vector<std::pair<float, float>> task_destination_point)
{
    // Check if there is not 'intention' node yet in G
    std::vector<float> v;
    v.push_back(get<0>(task_destination_point.back()));
    v.push_back(get<1>(task_destination_point.back()));

    if (auto robot = G->get_node(robot_name); robot.has_value())
    {
        G->add_or_modify_attrib_local<robot_occupied_att>(robot.value(), true);
        G->update_node(robot.value());

        if (auto mind = G->get_node(robot_mind_name); mind.has_value())
        {
            if (auto task = G->get_node(task_name); task.has_value())
            {
                if (!(G->get_attrib_by_name<task_movement_att>(task.value())).value() and
                        !(G->get_attrib_by_name<task_completed_att>(task.value())).value()
                        and (G->get_attrib_by_name<robot_occupied_att>(robot.value())).value())
                {
                    G->add_or_modify_attrib_local<task_destination_values_att>(task.value(), v );
                    G->update_node(task.value());
                }
            }
        }
        else
        {
            std::cout << __FUNCTION__ << " No node " << robot_mind_name << " found in G" << std::endl;
            std::terminate();
        }
    }
}


void SpecificWorker::follow_path_copy_path_to_graph(const std::vector<float> &x_values, const std::vector<float> &y_values)
{
    if (auto path = G->get_node(current_path_name); path.has_value())
    {
        auto path_to_target_node = path.value();
        G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
        G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
        G->add_or_modify_attrib_local<path_is_cyclic_att>(path_to_target_node, true);
        G->update_node(path_to_target_node);
    }
    else // create path_to_target_node with the solution path
    {
        if(auto intention = G->get_node(current_intention_name); intention.has_value())
        {
            auto path_to_target_node = DSR::Node::create<path_to_target_node_type>(current_path_name);
            G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
            G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
            G->add_or_modify_attrib_local<path_is_cyclic_att>(path_to_target_node, true);
            G->add_or_modify_attrib_local<pos_x_att>(path_to_target_node, (float) -542);
            G->add_or_modify_attrib_local<pos_y_att>(path_to_target_node, (float) 106);
            G->add_or_modify_attrib_local<parent_att>(path_to_target_node, intention.value().id());
            G->add_or_modify_attrib_local<level_att>(path_to_target_node, 3);
            auto id = G->insert_node(path_to_target_node);
            DSR::Edge edge_to_intention = DSR::Edge::create<thinks_edge_type>(id.value(), intention.value().id());
            G->insert_or_assign_edge(edge_to_intention);
        }
        else qWarning() << __FUNCTION__ << "No intention node found. Can't create path node";
    }
}

void SpecificWorker::slot_start_mission()
{
    insert_intention_node(temporary_plan);
    custom_widget.stacked_widget->setCurrentIndex(1);
    temporary_plan.new_plan(Plan::Actions::FOLLOW_PATH);
    custom_widget.textedit_current_plan->appendPlainText("-> New temporary plan: FOLLOW_PATH");

    if(temporary_plan.is_action(Plan::Actions::FOLLOW_PATH))
            follow_path_copy_path_to_graph(temporary_plan.x_path, temporary_plan.y_path);

    auto temp_plan = temporary_plan;
    plan_buffer.put(std::move(temp_plan));
    custom_widget.textedit_current_plan->verticalScrollBar()->setValue(custom_widget.textedit_current_plan->verticalScrollBar()->maximum());
    custom_widget.textedit_current_plan->appendPlainText(QString::fromStdString(temporary_plan.pprint()));


    // List of pick up points
    pathfollow_dialog.list_pickuppoints->clear();
    pathfollow_dialog.list_pickuppoints->addItem("Select a pick up");
    pathfollow_dialog.list_pickuppoints->addItem("First stop");
    pathfollow_dialog.list_pickuppoints->addItem("Second stop");
    pathfollow_dialog.list_pickuppoints->addItem("Third stop");
    pathfollow_dialog.list_pickuppoints->addItem("Fourth stop");


    // List of destination points
    pathfollow_dialog.list_destinationpoints->clear();
    pathfollow_dialog.list_destinationpoints->addItem("Select a destination");
    pathfollow_dialog.list_destinationpoints->addItem("First stop");
    pathfollow_dialog.list_destinationpoints->addItem("Second stop");
    pathfollow_dialog.list_destinationpoints->addItem("Third stop");
    pathfollow_dialog.list_destinationpoints->addItem("Fourth stop");
}

void SpecificWorker::slot_stop_mission()
{
    send_command_to_robot(std::make_tuple(0.f,0.f,0.f));   //adv, rot, side

    if(auto task = G->get_node(task_name); task.has_value())
        G->delete_node(task.value().id());

    if(auto intention = G->get_node(current_intention_name); intention.has_value())
    {
        if(auto path = G->get_node(current_path_name); path.has_value())
            G->delete_node(path.value().id());
        G->delete_node(intention.value().id());
    }
    else
        qWarning() << __FUNCTION__ << "No intention node found";
    if(temporary_plan.is_valid())
        custom_widget.textedit_current_plan->appendPlainText("-> mission " + temporary_plan.get_action() + " cancelled");

    custom_widget.stacked_widget->setCurrentIndex(2); //se quita el seleccionador de pickup y destino
    temporary_plan.reset();
    current_plan.reset();

    // remove path form drawing
    std::vector<Eigen::Vector2f> fake_path;
    draw_path(fake_path, &widget_2d->scene, true); // just remove
}

void SpecificWorker::slot_cancel_mission()
{
    slot_stop_mission();
}

void SpecificWorker::slot_change_pickup_selector(int index)
{
    bool insert = true;

    switch(index)
    {
        case 0:
            insert = false;
            break;
        case 1:
            task_pickup_point.push_back(std::make_pair(xpts[stop_1]*1.0, ypts[stop_1]*1.0));
            break;
        case 2:
            task_pickup_point.push_back(std::make_pair(xpts[stop_2]*1.0, ypts[stop_2]*1.0));
            break;
        case 3:
            task_pickup_point.push_back(std::make_pair(xpts[stop_3]*1.0, ypts[stop_3]*1.0));
            break;
        case 4:
            task_pickup_point.push_back(std::make_pair(xpts[stop_4]*1.0, ypts[stop_4]*1.0));
            break;
        default:
            insert = false;
    }
    if (insert)
        insert_pickup_node(temporary_plan, task_pickup_point);

}

void SpecificWorker::slot_change_destination_selector(int index)
{
    bool insert = true;

    switch(index)
    {
        case 0:
            insert = false;
            break;
        case 1:
            task_destination_point.push_back(std::make_pair(xpts[stop_1]*1.0, ypts[stop_1]*1.0));
            break;
        case 2:
            task_destination_point.push_back(std::make_pair(xpts[stop_2]*1.0, ypts[stop_2]*1.0));
            break;
        case 3:
            task_destination_point.push_back(std::make_pair(xpts[stop_3]*1.0, ypts[stop_3]*1.0));
            break;
        case 4:
            task_destination_point.push_back(std::make_pair(xpts[stop_4]*1.0, ypts[stop_4]*1.0));
            break;
        default:
            insert = false;
    }

    if (insert)
        insert_destination_coords(temporary_plan, task_destination_point);
}

void SpecificWorker::check_task_completed()
{
    if (auto task = G->get_node(task_name); task.has_value())
    {
        if (auto task_completed = G->get_attrib_by_name<task_completed_att>(task.value());
                task_completed.has_value() and task_completed == true)
        {
            G->delete_node(task.value().id());
            custom_widget.stacked_widget->setCurrentIndex(2);
        }
    }
}

void SpecificWorker::load_path(string filename)
{
    ifstream fin;
    fin.open(filename);
    string x, y;


    while(!fin.eof())
    {
        getline(fin, x, ',');
        getline(fin, y);
        xpts.push_back(atof(x.c_str()));
        ypts.push_back(atof(y.c_str()));
    }
    fin.close();
}


void SpecificWorker::draw_path(std::vector<Eigen::Vector2f> &path, QGraphicsScene* viewer_2d, bool remove)
{
    static std::unordered_map<QGraphicsScene *, std::vector<QGraphicsLineItem *> *> scene_road_points_map;
    std::vector<QGraphicsLineItem *> *scene_road_points;

    if (scene_road_points_map.contains(viewer_2d))
        scene_road_points = scene_road_points_map[viewer_2d];
    else
        scene_road_points = new std::vector<QGraphicsLineItem *>();

    scene_road_points_map[viewer_2d] = scene_road_points;

    //clear previous points
    for (QGraphicsLineItem* item : *scene_road_points)
        viewer_2d->removeItem((QGraphicsItem *) item);
    scene_road_points->clear();

    if(remove) return;

    // Draw all points
    QGraphicsLineItem *line1;
    QPen pen(QColor("Green"), 100);
    for(auto &&p_pair : iter::sliding_window(path, 2))
    {
        if(p_pair.size() < 2)
            continue;
        Mat::Vector2d a_point(p_pair[0].x(), p_pair[0].y());
        Mat::Vector2d b_point(p_pair[1].x(), p_pair[1].y());
        QLineF qsegment(QPointF(a_point.x(), a_point.y()), QPointF(b_point.x(), b_point.y()));
        line1 = viewer_2d->addLine(qsegment, pen);
        line1->setZValue(2000);
        scene_road_points->push_back(line1);
    }
}

void SpecificWorker::send_command_to_robot(const std::tuple<float, float, float> &speeds)   //adv, rot, side
{
    auto &[adv_, rot_, side_] = speeds;
    auto robot_node = G->get_node(robot_name);
    G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(),  (float)adv_);
    G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), (float)rot_);
    G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot_node.value(), (float)side_);
    G->update_node(robot_node.value());
}

////////////////////////////////////

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

///////////////////////////////////

