 #pragma once

 #include <string> 
 #include <vector>
 #include <numeric>
 #include <tuple>
 #include <cmath>
 #include <algorithm>
 #include "std_msgs/Float64.h"
 #include <ros/ros.h>


 
 
  class zScore{
    public:
      int lag = 500; // How many previous values are we talking into account for data smoothing
      float threshold = 5; // Number of std deviations needed to show a signal
      float influence = 0; // How much weight do we give to signaled values
      bool publish_values = true; // Do you want these values continuously published in ros?
      
      float max_threshold = 6.7;
      std::tuple<bool, float> getSignal(double new_value);
      double updateThreshold(double velocity);

      // Constructors
      zScore();
      zScore(ros::NodeHandle & node_handle, std::string topic_prefix);


    private:
      double current_stdDev;
      double current_mean;
      double current_signal;
      double current_raw_value;

      std::string topic_prefix = "not_set";
      double getStdDev(std::vector<double> data);
      double getMean(std::vector<double> data);
      void publishValues();
      std::vector<double> lag_values;

      // Create all publishers
      ros::Publisher pub_mean;
      ros::Publisher pub_positive_threshold;
      ros::Publisher pub_negative_threshold;
      ros::Publisher pub_signal;
      ros::Publisher pub_raw_value;
  };