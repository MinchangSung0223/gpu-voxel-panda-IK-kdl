// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann
 * \date    2018-01-07
 *
 */
//----------------------------------------------------------------------/*
#include <iostream>
using namespace std;
#include <gpu_voxels/logging/logging_gpu_voxels.h>

#define IC_PERFORMANCE_MONITOR
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <unistd.h> 
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/fmt/FMT.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

#include <ompl/geometric/PathSimplifier.h>

#include "gvl_ompl_planner_helper.h"
#include <stdlib.h>

#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

#define PI 3.141592


int main(int argc, char **argv)
{
    // Always initialize our logging framework as a first step, with the CLI arguments:
    icl_core::logging::initialize(argc, argv);

    PERF_MON_INITIALIZE(100, 1000);
    PERF_MON_ENABLE("planning");

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(7));
    //We then set the bounds for the R3 component of this state space:
    ob::RealVectorBounds bounds(7);
    bounds.setLow(-3.14159265);
    bounds.setHigh(3.14159265);

    bounds.setLow(0,-2.9671);
    bounds.setHigh(0,2.9671);

    bounds.setLow(1,-1.7628);
    bounds.setHigh(1,1.7628);

    bounds.setLow(2,-2.8973);
    bounds.setHigh(2,2.8973);

    bounds.setLow(3,-3.0718);
    bounds.setHigh(3,0.0175);

    bounds.setLow(4,-2.8973);
    bounds.setHigh(4,2.8973);

    bounds.setLow(5,-0.0175);
    bounds.setHigh(5,3.7525);

    bounds.setLow(6,-2.8973);
    bounds.setHigh(6,2.8973);





    space->setBounds(bounds);
    //Create an instance of ompl::base::SpaceInformation for the state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    //Set the state validity checker
    std::shared_ptr<GvlOmplPlannerHelper> my_class_ptr(std::make_shared<GvlOmplPlannerHelper>(si));
       std::cout << "-------------------------------------------------" << std::endl;
    og::PathSimplifier simp(si);

    si->setStateValidityChecker(my_class_ptr->getptr());
    si->setMotionValidator(my_class_ptr->getptr());
    si->setup();


    //Create a random start state:
    ob::ScopedState<> start(space);
    float start_value[7] = {-0.11303271230870877, -0.27671833759076736, -0.685219556677635, -2.5957449261657013, -0.25012459331129966, 2.363867067660963, 0.1865279646571519};//{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    start[0] = start_value[0];
    start[1] = start_value[1];
    start[2] = start_value[2];
    start[3] = start_value[3];
    start[4] = start_value[4];
    start[5] = start_value[5];
    start[6] = start_value[6];


    KDL::Tree my_tree;
    KDL::Chain my_chain;


   if (!kdl_parser::treeFromFile("panda_coarse/panda.urdf", my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
   }

    LOGGING_INFO(Gpu_voxels, "\n\nKDL Number of Joints : "<<my_tree.getNrOfJoints() <<"\n"<< endl);
    LOGGING_INFO(Gpu_voxels, "\n\nKDL Chain load : "<<my_tree.getChain("panda_link0","panda_rightfinger",my_chain) <<"\n"<< endl);

    KDL::JntArray q(my_chain.getNrOfJoints());
    KDL::JntArray q_init(my_chain.getNrOfJoints());
    q_init(0) = 0;
    q_init(1) = -0.7850857777;
    q_init(2) = 0;
    q_init(3) = -2.3555949;
    q_init(4) = 0;
    q_init(5) = 1.57091693296;
    q_init(6) = 0.785094180;

    KDL::Frame cart_pos;
    KDL::ChainFkSolverPos_recursive fk_solver(my_chain);
    fk_solver.JntToCart(q_init, cart_pos);
    std::cout<<"\nKDL hand pos : "<<cart_pos <<"\n"<< std::endl;
    KDL::ChainIkSolverVel_pinv iksolver1v(my_chain);
    KDL::ChainIkSolverPos_NR iksolver1(my_chain,fk_solver,iksolver1v,100,1e-6);

    std::cout<<"Jnt Array q_int : "<<q_init(0)<<", "<<q_init(1)<<", "<<q_init(2)<<", "<<q_init(3)<<", "<<q_init(4)<<", "<<q_init(5)<<", "<<q_init(6)<<std::endl;
    KDL::Frame F_dest(KDL::Rotation::RPY(0.0, PI/2, 0.0), KDL::Vector(0.309608, 0.409473, 0.607041));
    int ret = iksolver1.CartToJnt(q_init,F_dest,q);
    std::cout<<"iksolverpos ret : "<<ret<<std::endl;

    std::cout<<"IK Jnt Array q : "<<q(0)<<", "<<q(1)<<", "<<q(2)<<", "<<q(3)<<", "<<q(4)<<", "<<q(5)<<", "<<q(6)<<std::endl;


    //And a random goal state:
 
    ob::ScopedState<> goal(space);
    goal[0] = double(q(0));
    goal[1] =  double(q(1));
    goal[2] =  double(q(2));
    goal[3] = double(q(3));
    goal[4] =  double(q(4));
    goal[5] = double(q(5));
    goal[6] = double(q(6));

    my_class_ptr->insertStartAndGoal(start, goal);
    my_class_ptr->doVis();


    //Create an instance of ompl::base::ProblemDefinition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    //Set the start and goal states for the problem definition.
    pdef->setStartAndGoalStates(start, goal);
    //Create an instance of a planner
    auto planner(std::make_shared<og::LBKPIECE1>(si));
    //Tell the planner which problem we are interested in solving
    planner->setProblemDefinition(pdef);
    //Make sure all the settings for the space and planner are in order. This will also lead to the runtime computation of the state validity checking resolution.
    planner->setup();

    int succs = 0;

    //use this to stop planning, until Visualizer is connected
    std::cout << "Waiting for Viz. Press Key if ready!" << std::endl;
    std::cin.ignore();

    for(int i = 0;i<1;i++)
    {
        my_class_ptr->moveObstacle();

        //We can now try to solve the problem. This call returns a value from ompl::base::PlannerStatus which describes whether a solution has been found within the specified amount of time (in seconds). If this value can be cast to true, a solution was found.
        PERF_MON_START("planner");
        planner->clear(); // this clears all roadmaps
        ob::PlannerStatus solved = planner->ob::Planner::solve(20.0);
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("planner", "Planning time", "planning");

        //If a solution has been found, we simplify and display it.
        if (solved)
        {
            ++succs;
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            ob::PathPtr path = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;
            // print the path to screen
            path->print(std::cout);

            PERF_MON_START("simplify");
            simp.simplifyMax(*(path->as<og::PathGeometric>()));
            PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("simplify", "Simplification time", "planning");

            std::cout << "Simplified solution:" << std::endl;
            // print the path to screen
            path->print(std::cout);

           //my_class_ptr->visualizeSolution(path);
 

        }else{
            std::cout << "No solution could be found" << std::endl;
        }

        PERF_MON_SUMMARY_PREFIX_INFO("planning");
        std::cout << "END OMPL" << std::endl;
        my_class_ptr->doVis();


    }
    PERF_MON_ADD_STATIC_DATA_P("Number of Planning Successes", succs, "planning");

    PERF_MON_SUMMARY_PREFIX_INFO("planning");

    // keep the visualization running:
    ob::PathPtr path = pdef->getSolutionPath();
    og::PathGeometric* solution = path->as<og::PathGeometric>();
    solution->interpolate();
    int step_count = solution->getStateCount();
    for(int i=0;i<step_count;i++){
    
     const double *values = solution->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
    std::cout<<"step "<<i<<" joint :"<< values[0]<<","<<values[1]<<","<<values[2]<<","<<values[3]<<","<<values[4]<<","<<values[5]<<","<<values[6] <<std::endl;
    my_class_ptr->visualizeRobot(values);
     usleep(100000);
     }
    return 1;
}
