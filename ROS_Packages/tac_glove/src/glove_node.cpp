/**
 * Copyright 2015 by California Institute of Technology.  ALL RIGHTS RESERVED.
 * United  States  Government  sponsorship  acknowledged.   Any commercial use
 * must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the
 * California Institute of Technology.
 *
 * This software may be subject to  U.S. export control laws  and regulations.
 * By accepting this document,  the user agrees to comply  with all applicable
 * U.S. export laws and regulations.  User  has the responsibility  to  obtain
 * export  licenses,  or  other  export  authority  as may be required  before
 * exporting  such  information  to  foreign  countries or providing access to
 * foreign persons.
 *
 *
 * @file glove_node.cpp
 * @author Brandon Rothrock, brandon.rothrock@jpl.nasa.gov
 *
 * @brief ROS node entry point for tac_glove
 * @details
 *
 */


#include <ros/ros.h>
#include <nodelet/loader.h>
#include "glove.h"

int main(int argc, char **argv) {

  try {

    ros::init(argc, argv, "tac_glove", ros::init_options::AnonymousName);

    // nodelet::Loader manager(false);
    // nodelet::M_string remappings;
    // nodelet::V_string my_argv(argv + 1, argv + argc);

    // manager.load(ros::this_node::getName(), "tac_glove/tac_glove_nodelet", remappings, my_argv);
    if(argc!=4)
    {
	ROS_ERROR_STREAM("arguments not correct");
	exit(0);
    }
    bool replay,record;
    std::string dest_dir="";
    std::string task_name="";
    if(strcmp(argv[1],"--visual")==0)
    {
    	replay=false;
	record=false;
    }
    else if(strcmp(argv[1],"--replay")==0)
    {
    	replay=true;
	record=false;
    }
    else if(strcmp(argv[1],"--record")==0)
    {
	replay=false;
	record=true;
    }
    dest_dir.assign(argv[2]);
    task_name.assign(argv[3]);
    dest_dir+=task_name;
    Glove g(replay,record,dest_dir);

    //  print published/subscribed topics
    ros::V_string topics;
    ros::this_node::getSubscribedTopics(topics);
    std::string nodeName = ros::this_node::getName();
    std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
    for(unsigned int i=0; i<topics.size(); ++i) {
      topicsStr+=("\t\t" + topics.at(i) + "\n");
    }

    topicsStr += "\tadvertised topics:\n";
    ros::this_node::getAdvertisedTopics(topics);
    for(unsigned int i=0; i<topics.size(); ++i) {
      topicsStr+=("\t\t" + topics.at(i) + "\n");
    }

    ROS_INFO_STREAM(topicsStr);

    while (ros::ok()) {
      ros::spin();
    }
    
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Fatal error: " << e.what());
  }

  return EXIT_SUCCESS;
}
