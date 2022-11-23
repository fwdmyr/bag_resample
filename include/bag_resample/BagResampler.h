//
// Created by felix on 26.07.22.
//

#ifndef BAG_RESAMPLE_BAGRESAMPLER_H
#define BAG_RESAMPLE_BAGRESAMPLER_H

#include <map>
#include <unordered_set>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <iostream>
#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <XmlRpcValue.h>

#include <bag_resample/MessageDefinitions.h>

enum class MODE {
    DEFAULT,
    USER,
    COMBINED
};

struct BagResamplerConfig {
    double freq = 1.0;
    MODE mode = MODE::DEFAULT;
    std::string dir;
    std::vector<std::string> bags;
    std::map<std::string, double> topicFreqs;
};

inline MODE str2MODE(std::string &str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    if (str == "default")
        return MODE::DEFAULT;
    else if (str == "user")
        return MODE::USER;
    else if (str == "combined")
        return MODE::COMBINED;
    else
        throw std::invalid_argument("Invalid mode string");
}

BagResamplerConfig initConfig(const ros::NodeHandle &nh) {
    ROS_INFO_STREAM("Loading parameters...");

    BagResamplerConfig config;
    nh.getParam("resampleFrequency", config.freq);
    nh.getParam("bagDirectory", config.dir);
    nh.getParam("bagFiles", config.bags);

    std::string mode;
    nh.getParam("resampleMode", mode);
    config.mode = str2MODE(mode);

    XmlRpc::XmlRpcValue topicFrequenciesList;
    nh.getParam("topicFrequencies", topicFrequenciesList);
    ROS_ASSERT(topicFrequenciesList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i=0; i<topicFrequenciesList.size(); ++i) {
        XmlRpc::XmlRpcValue topicFrequencyDict = topicFrequenciesList[i];
        ROS_ASSERT(topicFrequencyDict.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        XmlRpc::XmlRpcValue topic = topicFrequencyDict["topic"];
        ROS_ASSERT(topic.getType() == XmlRpc::XmlRpcValue::TypeString);
        XmlRpc::XmlRpcValue freq = topicFrequencyDict["freq"];
        ROS_ASSERT(freq.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        config.topicFreqs[static_cast<std::string>(topic)] = static_cast<double>(freq);
    }

    ROS_INFO_STREAM("[resampleFrequency]: " << config.freq);
    ROS_INFO_STREAM("[bagDirectory]: " << config.dir);
    ROS_INFO_STREAM("[resampleMode]: " << mode);

    std::stringstream ss;
    ss << "[bagFiles]: [";
    for (const auto & bag : config.bags)
        ss << bag << ", ";
    ss.seekp(-2,std::stringstream::cur);
    ss << ']';

    ROS_INFO_STREAM(ss.str());

    ss.str("");
    ss.clear();
    ss << "[topicFrequencies]: {";
    for (const auto &[key, value] : config.topicFreqs)
        ss << key << ": " << value << ", ";
    ss.seekp(-2,std::stringstream::cur);
    ss << "}";

    ROS_INFO_STREAM(ss.str());

    return config;
}


class BagResampler {

public:
    explicit BagResampler(BagResamplerConfig config) : config_(std::move(config)) {
        buildDurations();
        buildDirectory();
    }
    ~BagResampler() = default;

    void run() {
        std::for_each(bags_.begin(), bags_.end(), [this](const auto &bag) -> void {
            std::filesystem::path path(bag);
            ROS_INFO_STREAM("Resampling " << path.filename().string() << "...");
            resampleBag(bag);
            ROS_INFO_STREAM("Done");
        });
    }

private:
    BagResamplerConfig config_;
    std::vector<std::string> bags_;
    std::map<std::string, ros::Duration> topicDurations_;

    void buildDurations() {
        // Ignore all-user defined frequencies when in default mode
        if (config_.mode == MODE::DEFAULT)
            return;
        for (const auto &[key, value] : config_.topicFreqs) {
            topicDurations_.emplace(key, 1.0 / value);
        }
    }

    void buildDirectory() {
        const std::string ext(".bag");
        for (const auto &it : std::filesystem::recursive_directory_iterator(config_.dir))
        {
            const std::string bagFound = it.path().string();
            if (config_.bags.empty()) {
                if (it.path().extension() == ext)
                    bags_.push_back(bagFound);
            } else {
                std::for_each(config_.bags.begin(), config_.bags.end(), [&](const auto &bag) -> void {
                    if (it.path().extension() == ext && bagFound.find(bag) != std::string::npos)
                        bags_.push_back(bagFound);
                });
            }
        }
    }

    void resampleBag(const std::string &readBagName) {
        rosbag::Bag writeBag;
        std::filesystem::path writePath(readBagName);
        std::string writeName = writePath.stem().string();
        if (config_.topicFreqs.empty())
            writeName += "_" + std::to_string(config_.freq) + "Hz";
        else
            writeName += "_Resampled";
        const std::string writeBagName = config_.dir + writeName + ".bag";
        writeBag.open(writeBagName, rosbag::bagmode::Write);

        rosbag::Bag readBag;
        readBag.open(readBagName, rosbag::bagmode::Read);
        rosbag::View view(readBag);

        // Pre-allocated gathering of bag information
        const auto infos = view.getConnections();
        std::unordered_set<std::string> topicNames;
        for (const auto &info : infos) {

            const std::string topicName = info->topic;
            topicNames.insert(topicName);

            switch (config_.mode) {

                // In USER mode, dont touch the non user-defined topics (== assign infinite frequency)
                case MODE::USER:
                    if (topicDurations_.find(topicName) == topicDurations_.end())
                        topicDurations_.emplace(topicName, 0.0);
                    break;
                    // In COMBINED mode, all topics without user-defined frequencies get assigned the default
                case MODE::COMBINED:
                    // In DEFAULT mode, all topics get assigned the default
                case MODE::DEFAULT:
                    if (topicDurations_.find(topicName) == topicDurations_.end())
                        topicDurations_.emplace(topicName, 1.0 / config_.freq);
                    break;
            }
        }

        std::for_each(topicNames.begin(), topicNames.end(), [&](const std::string &topicName) -> void {
            // Creating a topic-specific view of the bag avoids map lookups during the write-read loop
            rosbag::View topicView(readBag, rosbag::TopicQuery(topicName));
            if (topicView.size() == 0)
                return;

            const auto &firstIt = topicView.begin();
            const std::string typeName = firstIt->getDataType();
            int seq = 0;
            ros::Time timestamp = firstIt->getTime();
            const ros::Duration &duration = topicDurations_.at(topicName);
            // If infinite resample frequency, we write all messages and do not required resequencing
            const bool resequencingRequired = (duration != ros::Duration(0.0));
            FunctionPtrType writeMessage = typeFunctionPtrs.at(typeName);

            for (const rosbag::MessageInstance &msg : topicView) {
                if (!resequencingRequired) {
                    writeMessage(msg, writeBag, seq, resequencingRequired);
                } else {
                    // Discard message when minimum time between writes not elapsed
                    if (msg.getTime() - timestamp < duration)
                        continue;

                    // Increment seq and update timestamp if msg write successful
                    if (writeMessage(msg, writeBag, seq, resequencingRequired)) {
                        timestamp = msg.getTime();
                        seq++;
                    }
                }
            }
        });

        writeBag.close();
        readBag.close();
    }

};

#endif //BAG_RESAMPLE_BAGRESAMPLER_H
