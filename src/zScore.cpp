#include <zScore.h>


// Constructors
zScore::zScore(ros::NodeHandle & node_handle, std::string topic_prefix){
    
    topic_prefix = topic_prefix;
    
    //Create all topics we will publish values to
    pub_mean = node_handle.advertise<std_msgs::Float64>(topic_prefix + "_zScore_mean", 1);
    pub_positive_threshold = node_handle.advertise<std_msgs::Float64>(topic_prefix + "_zScore_positive_threshold", 1);
    pub_negative_threshold = node_handle.advertise<std_msgs::Float64>(topic_prefix + "_zScore_negative_threshold", 1);
    pub_signal = node_handle.advertise<std_msgs::Float64>(topic_prefix + "_zScore_signal", 1);
    pub_raw_value = node_handle.advertise<std_msgs::Float64>(topic_prefix + "_zScore_raw_value", 1);

}

zScore::zScore(){
    this->publish_values = false;
}


// Public Functions


double zScore::updateThreshold(double velocity)
{
    float max = max_threshold - 1;
    float a = 0.6;


    threshold = (max/(1 + std::exp((10 * std::abs(velocity))/a - max))) + 0.3;
    return threshold;
}


// Returns if the signal exceeds the threshold and the value if it did
std::tuple<bool, float> zScore::getSignal(double new_value){
  // Do we have enough points to start the calculation?
  current_raw_value = new_value;
  current_signal = 0;
  bool signal_found = false;
  if(lag_values.size() < lag){
  
    lag_values.push_back(new_value);
    return std::make_tuple(signal_found, current_signal);

  }else{

    current_mean = getMean(lag_values);
    current_stdDev = getStdDev(lag_values);
    

    if(std::abs(new_value - current_mean) > threshold * current_stdDev){
      // Note this signal so we can graph it!
      current_signal = new_value;
      signal_found = true;
      // How much weight do we want this new signal to have?
      new_value = influence * new_value + (1 - influence) * lag_values[lag-1];
    }

    publishValues();
    lag_values.erase(lag_values.begin());
    lag_values.push_back(new_value);
    return std::make_tuple(signal_found, current_signal);
    // At the end incorporate the new value into our list
  }
}

/*

    Private helper functions used above

*/

void zScore::publishValues(){

    if(this->publish_values){
        //Create all topics we will publish values to
        pub_mean.publish(current_mean);
        pub_positive_threshold.publish(current_mean + current_stdDev * threshold);
        pub_negative_threshold.publish(current_mean - current_stdDev * threshold);
        pub_signal.publish(current_signal);
        pub_raw_value.publish(current_raw_value);
    }
}

double zScore::getStdDev(std::vector<double> data){
    double mean = getMean(data);
    double accum = 0.0;
    std::for_each (data.begin(), data.end(), [&](const double d) {
      accum += (d - mean) * (d - mean);
    });

    return sqrt(accum / (lag - 1));
}

double zScore::getMean(std::vector<double> data){
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    return sum / data.size();
}