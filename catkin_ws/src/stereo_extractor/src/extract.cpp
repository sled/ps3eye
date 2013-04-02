/*
 * stereo_bag_extractor.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: sled
 */

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <set>
#include <map>
#include <iostream>
#include <string>
#include <cstdlib>

int main(int argc, const char *argv[]) {
/*
	if(argc < 3) {
		std::cout<<"No bag file and/or timestamp specified"<<std::endl;
		return 1;
	}

	const std::string bag_filename = argv[1];
	float timestamp = atof(argv[2]);

	std::cout<<"Opening "<<bag_filename<<" at timestamp "<<timestamp<<std::endl;

	rosbag::Bag bag;

	try {
		bag.open(bag_filename, rosbag::bagmode::Read);
	} catch(rosbag::BagException &ex) {
		std::cerr<<ex.what()<<std::endl;
		return 1;
	}
	// create view on the bag
	rosbag::View view(bag);

	// connection info
	std::vector<const rosbag::ConnectionInfo*> connection_infos = view.getConnections();

	// hold topics
	std::set<std::string> topics;

	std::map<const std::string, const std::string> topic_map;
	topic_map.insert("left_img", "my_stereo_cam/")

	std::map<const std::string, const rosbag::ConnectionInfo*> topic_remap;

	/my_stereo_cam/points2
	// iterate through topics
	BOOST_FOREACH(const rosbag::ConnectionInfo* info, connection_infos) {
		// unique topic names
		if(topics.find(info->topic) == topics.end()) {
			topics.insert(info->topic);
			topic_remap[info->topic] = info;
		}
	}
*/



	return 0;
}



