//
// Created by felix on 26.07.22.
//

#include <ros/ros.h>

#include <bag_resample/BagResampler.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "bag_resample_node");
    ros::NodeHandle nh("bag_resample_node");

    const BagResamplerConfig config = initConfig(nh);
    BagResampler br(config);
    br.run();

    ros::shutdown();

}
