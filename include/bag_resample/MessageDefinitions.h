//
// Created by felix on 27.07.22.
//

#ifndef BAG_RESAMPLE_MESSAGEDEFINITIONS_H
#define BAG_RESAMPLE_MESSAGEDEFINITIONS_H

#include <unordered_map>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Transform.h>
#include <gps_common/GPSFix.h>
#include <irt_msgs/MICROSTRAIN.h>
#include <irt_msgs/PPS.h>
#include <irt_msgs/SFUSION.h>
#include <irt_msgs/SIMULATIONTIME.h>
#include <nav_msgs/Odometry.h>
#include <novatel_oem7_msgs/BESTPOS.h>
#include <novatel_oem7_msgs/BESTVEL.h>
#include <novatel_oem7_msgs/CLOCKMODEL.h>
#include <novatel_oem7_msgs/DUALANTENNAHEADING.h>
#include <novatel_oem7_msgs/GALCLOCK.h>
#include <novatel_oem7_msgs/GALFNAVEPHEMERIS.h>
#include <novatel_oem7_msgs/GALINAVEPHEMERIS.h>
#include <novatel_oem7_msgs/GALIONO.h>
#include <novatel_oem7_msgs/GPSEPHEM.h>
#include <novatel_oem7_msgs/RANGE.h>
#include <novatel_oem7_msgs/RXSTATUS.h>
#include <novatel_oem7_msgs/TIME.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Temperature.h>
#include <tf2_msgs/TFMessage.h>

template <typename Message>
bool writeMessageWithoutHeader(const rosbag::MessageInstance &instance, rosbag::Bag &bag, unsigned long stamp, bool doResequencing) {
    boost::shared_ptr<const Message> msgPtr = instance.instantiate<Message>();
    if (msgPtr)
        bag.write(instance.getTopic(), instance.getTime(), msgPtr);
    return (msgPtr != nullptr);
}

template <typename Message>
bool writeMessageWithHeader(const rosbag::MessageInstance &instance, rosbag::Bag &bag, unsigned long stamp, bool doResequencing) {
    // If no resequencing required, we can cast message to pointer to const (big speedup)
    if (!doResequencing)
        return writeMessageWithoutHeader<Message>(instance, bag, stamp, doResequencing);
    boost::shared_ptr<Message> msgPtr = instance.instantiate<Message>();
    msgPtr->header.seq = stamp;
    if (msgPtr)
        bag.write(instance.getTopic(), instance.getTime(), msgPtr);
    return (msgPtr != nullptr);
}

using FunctionPtrType = bool (*)(const rosbag::MessageInstance&, rosbag::Bag&, unsigned long, bool);

const std::unordered_map<std::string, FunctionPtrType> typeFunctionPtrs {
        {"geometry_msgs/PoseWithCovarianceStamped", &writeMessageWithHeader<geometry_msgs::PoseWithCovarianceStamped>},
        {"geometry_msgs/Transform", &writeMessageWithoutHeader<geometry_msgs::Transform>},
        {"gps_common/GPSFix", &writeMessageWithHeader<gps_common::GPSFix>},
        {"irt_msgs/MICROSTRAIN", &writeMessageWithHeader<irt_msgs::MICROSTRAIN>},
        {"irt_msgs/PPS", &writeMessageWithHeader<irt_msgs::PPS>},
        {"irt_msgs/SFUSION", &writeMessageWithHeader<irt_msgs::SFUSION>},
        {"irt_msgs/SIMULATIONTIME", &writeMessageWithHeader<irt_msgs::SIMULATIONTIME>},
        {"nav_msgs/Odometry", &writeMessageWithHeader<nav_msgs::Odometry>},
        {"novatel_oem7_msgs/BESTPOS", &writeMessageWithHeader<novatel_oem7_msgs::BESTPOS>},
        {"novatel_oem7_msgs/BESTVEL", &writeMessageWithHeader<novatel_oem7_msgs::BESTVEL>},
        {"novatel_oem7_msgs/CLOCKMODEL", &writeMessageWithHeader<novatel_oem7_msgs::CLOCKMODEL>},
        {"novatel_oem7_msgs/DUALANTENNAHEADING", &writeMessageWithHeader<novatel_oem7_msgs::DUALANTENNAHEADING>},
        {"novatel_oem7_msgs/GALCLOCK", &writeMessageWithHeader<novatel_oem7_msgs::GALCLOCK>},
        {"novatel_oem7_msgs/GALFNAVEPHEMERIS", &writeMessageWithHeader<novatel_oem7_msgs::GALFNAVEPHEMERIS>},
        {"novatel_oem7_msgs/GALINAVEPHEMERIS", &writeMessageWithHeader<novatel_oem7_msgs::GALINAVEPHEMERIS>},
        {"novatel_oem7_msgs/GALIONO", &writeMessageWithHeader<novatel_oem7_msgs::GALIONO>},
        {"novatel_oem7_msgs/GPSEPHEM", &writeMessageWithHeader<novatel_oem7_msgs::GPSEPHEM>},
        {"novatel_oem7_msgs/RANGE", &writeMessageWithHeader<novatel_oem7_msgs::RANGE>},
        {"novatel_oem7_msgs/RXSTATUS", &writeMessageWithHeader<novatel_oem7_msgs::RXSTATUS>},
        {"novatel_oem7_msgs/TIME", &writeMessageWithHeader<novatel_oem7_msgs::TIME>},
        {"sensor_msgs/CameraInfo", &writeMessageWithHeader<sensor_msgs::CameraInfo>},
        {"sensor_msgs/FluidPressure", &writeMessageWithHeader<sensor_msgs::FluidPressure>},
        {"sensor_msgs/Image", &writeMessageWithHeader<sensor_msgs::Image>},
        {"sensor_msgs/Imu", &writeMessageWithHeader<sensor_msgs::Imu>},
        {"sensor_msgs/LaserScan", &writeMessageWithHeader<sensor_msgs::LaserScan>},
        {"sensor_msgs/MagneticField", &writeMessageWithHeader<sensor_msgs::MagneticField>},
        {"sensor_msgs/NavSatFix", &writeMessageWithHeader<sensor_msgs::NavSatFix>},
        {"sensor_msgs/PointCloud2", &writeMessageWithHeader<sensor_msgs::PointCloud2>},
        {"sensor_msgs/Temperature", &writeMessageWithHeader<sensor_msgs::Temperature>},
        {"tf2_msgs/TFMessage", &writeMessageWithoutHeader<tf2_msgs::TFMessage>}
};


#endif //BAG_RESAMPLE_MESSAGEDEFINITIONS_H
