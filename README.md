# bag_resample

## Overview

This package takes one or many ROS1 bag files and resamples the topics with user-defined frequency while re-generating the sequence markers of the std_msgs/Header for the stamped message types

## Adding new message types

A list of supported message types can be found in **MessageDefinitions.h**. Adding support for new message type **NewType** requires the following steps:

- Include the header for **NewType** in **MessageDefinitions.h**
- Add key-value pair for **NewType** in the *typeFunctionPtrs* map in **MessageDefinitions.h** 

## Configuring the resampling process

Configuration of the node behavior can be controlled via the **config.yaml** file:

- *resampleFrequency*: double that controls the default resampling frequency
- *resampleMode*: string that selects the resampling mode. The following options are available:
    - "default": resample whole bag with resampleFrequency
    - "user": resample only based on topicFrequencies, leave remaining topics unchanged
    - "combined": combine "user" and "default" with user taking precedence
- *bagDirectory*: string that defines the base directory in which the rosbags are contained
- *bagFiles*: list of strings that define specific rosbags for resampling (use **[]** to indicate an empty list to the ROS parameter server when resampling of all ROS nodes in bagDirectory and its sub directories is desired)
- *topicFrequencies*: list of dicts with the following keys:
    - *topic*: string that specifies a topic
    - *freq*: double that controls the resampling frequency for this specific topic

