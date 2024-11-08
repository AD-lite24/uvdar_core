#include <ros/ros.h>

#include <mrs_msgs/Float64Srv.h>

extern "C" {
#include <wiringPi.h>
}

#define BLINK_GPIO_PIN 25
#define INIT_BITRATE 2

namespace uvdar {


  /**
   * @brief A processing class for controlling the blinking UV LEDs for UVDAR on Raspberry Pi-based platforms
   */
  class Raspi_UVDAR_Blinker {
    private:
      /* attributes //{ */

      bool _debug_ = false;
      std::string _uav_name_;

      //}
      //
      
      bool initialized_ = false;
      
      std::mutex sequence_mutex;
      std::vector<bool> _sequence_;
      int curr_index_ = -1;
      
      ros::ServiceServer serv_frequency;

      ros::Timer timer;
      
    public:
      /**
       * @brief Constructor - loads parameters and initializes necessary structures
       *
       * @param nh Private NodeHandle of this ROS node
       */
      /* Constructor //{ */
      Raspi_UVDAR_Blinker(ros::NodeHandle nh) {
        wiringPiSetupGpio();
        pinMode(BLINK_GPIO_PIN, OUTPUT);
        ROS_INFO_STREAM("[Raspi_UVDAR_blinker]: GPIO " << BLINK_GPIO_PIN << " has been set as OUTPUT.");

        _sequence_ = {true, false, false, true, true, false}; //TODO

        serv_frequency = nh.advertiseService("set_frequency", &Raspi_UVDAR_Blinker::callbackSetFrequency, this);

        timer = nh.createTimer(INIT_BITRATE, &Raspi_UVDAR_Blinker::spin, this);

        initialized_ = true;
      }
      //}

      /**
       * @brief Thread for blinking - each iteration represents transmission of a bit
       *
       * @param te TimerEvent for the timer spinning this thread
       */
      /* spin //{ */
      void spin([[ maybe_unused ]] const ros::TimerEvent& te){
        if (!initialized_)
          return;

        std::scoped_lock lock(sequence_mutex);

        curr_index_++;
        if (curr_index_ >= (int)(_sequence_.size())){
          curr_index_ = 0;
        }

        digitalWrite(BLINK_GPIO_PIN, _sequence_[curr_index_]?HIGH:LOW);
      }
      //}
      
      bool callbackSetFrequency(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res){
        if (!initialized_){
          ROS_ERROR("[Raspi_UVDAR_Blinker]: Blinker is NOT initialized!");
          res.success = false;
          res.message = "Blinker is NOT initialized!";
          return true;
        }
          

        unsigned short int_frequency = (unsigned short)(req.value); // Hz

        timer.setPeriod(ros::Duration(1.0/int_frequency));

        res.message = std::string("Setting the frequency to "+std::to_string((int)(int_frequency))+" Hz").c_str();
        res.success = true;

        ROS_INFO_STREAM("[Raspi_UVDAR_Blinker]: " << res.message);


        return true;
      }
  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "raspi_UVDAR_blinker");
  ros::NodeHandle nh("~");
  uvdar::Raspi_UVDAR_Blinker        rub(nh);

  ROS_INFO("[Raspi_UVDAR_blinker]: UWB-UVDAR fuser node initiated");

  ros::spin();

  return 0;
}
