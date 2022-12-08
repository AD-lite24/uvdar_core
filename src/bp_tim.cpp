#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <stdio.h>
#include <mrs_lib/param_loader.h>
#include <std_msgs/Float32.h>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include <fstream>




#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>


namespace uvdar
{

    class UVDAR_BP_Tim : public nodelet::Nodelet
    {
        public:
            UVDAR_BP_Tim()
            {}

        private:
            void onInit()
            {
                ros::NodeHandle& private_nh_ = getMTPrivateNodeHandle();
                NODELET_DEBUG("[UVDAR_BP_Tim]: Initializing Nodelet...");

                bool printParams = false; 

                loadParams(printParams, private_nh_);
                parseSequenceFile(_sequence_file);
                 
                if (!checkVectorSizeMatch(_blinkers_seen_topics,_estimated_framerate_topics, _points_seen_topics)) return;

                subscribeToCameras(private_nh_);


                // init data structure
                std::vector<mrs_msgs::ImagePointsWithFloatStampedConstPtr> b;
                small_buffer_.reserve(_points_seen_topics.size());
                for (const auto i : _points_seen_topics) {
                    small_buffer_.push_back(b);
                }

                // for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
                //     pub_blinkers_seen_.push_back(private_nh_.advertise<mrs_msgs::ImagePointsWithFloatStamped>(_blinkers_seen_topics[i], 1));
                //     pub_estimated_framerate_.push_back(private_nh_.advertise<std_msgs::Float32>(_estimated_framerate_topics[i], 1));
                // }
        
                current_visualization_done_ = false;
                // timer_visualization_ = private_nh_.createTimer(ros::Rate(_visualization_rate_), &UVDAR_BP_Tim::VisualizationThread, this, false);


                getResults(private_nh_);
  

                initialized_ = true;
                if (initialized_) ROS_INFO("[UVDAR_BP_Tim]: Nodelet sucessfully initialized");
            }

            // runs @ 60 Hz for each camera on which points are detected
            void processPoint(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &ptsMsg, size_t img_index) {
                ROS_INFO_STREAM("[UVDAR_BP_Tim] The Time Stamp: "<<  ptsMsg->stamp <<   " The width: " << ptsMsg->image_width <<" The image height" << ptsMsg->image_height);

                for (auto a : ptsMsg->points) 
                    ROS_INFO_STREAM("x " << a.x << " y " << a.y << " Value " << a.value);

                if (small_buffer_.size() < 3 ) {
                    small_buffer_.at(img_index).push_back(ptsMsg);
                } else if (small_buffer_.size() == 3) {
                    processBuffer();
                } else {
                    small_buffer_.clear();
                }
                ROS_INFO_STREAM("THE VECTOR SIZE IS" << small_buffer_.size());

            }
            
            
            void processBuffer(){
                
            }

            void processSunPoint(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &ptMsg, size_t img_index) {
                

            }

            void getResults(ros::NodeHandle & private_nh_){
                // for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
                
                //     timer_process_.push_back(private_nh_.createTimer(ros::Duration(1.0/(double)(_proces_rate)), boost::bind(&UVDAR_BP_Tim::ProcessThread, this, _1, i), false, true));
                // }

            }

            /**
             * @brief Callback for each camera
             * 
             * @param private_nh_ 
             */
            void subscribeToCameras( ros::NodeHandle & private_nh_){
                 
                 for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
                    // Subscribe to corresponding topics
                    points_seen_callback_t callback = [image_index=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& pointsMessage) { 
                        processPoint(pointsMessage, image_index);
                    };
                    cals_points_seen_.push_back(callback);
                    sub_points_seen_.push_back(private_nh_.subscribe(_points_seen_topics[i], 1, cals_points_seen_[i]));
    
                    points_seen_callback_t sun_callback = [image_index=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& sunPointsMessage) { 
                        processSunPoint(sunPointsMessage, image_index);
                    };
                    cals_sun_points_.push_back(callback);
                    sub_sun_points_.push_back(private_nh_.subscribe(_points_seen_topics[i]+"/sun", 1, cals_sun_points_[i]));
                }
            }


            /**
             * @brief Checks if the _blinkers_seen_topics and _estimated_framerate_topics equals the size of _points_seen_topics
             *
             * @param _blinkers_seen_topics
             * @param _points_seen_topics
             * @param _estimated_framerate_topics
             */
            bool checkVectorSizeMatch(const std::vector<std::string> & _blinkers_seen_topics, const std::vector<std::string> & _estimated_framerate_topics , const std::vector<std::string> & _points_seen_topics){

                if (_blinkers_seen_topics.size() != _points_seen_topics.size()) {
                  ROS_ERROR_STREAM("[UVDAR_BP_Tim] The number of poinsSeenTopics (" << _points_seen_topics.size() 
                      << ") is not matching the number of blinkers_seen_topics (" << _blinkers_seen_topics.size() << ")!");
                  return false;
                }
                if (_estimated_framerate_topics.size() != _points_seen_topics.size()) {
                  ROS_ERROR_STREAM("[UVDAR_BP_Tim] The number of poinsSeenTopics (" << _points_seen_topics.size() 
                      << ") is not matching the number of blinkers_seen_topics (" << _estimated_framerate_topics.size() << ")!");
                  return false;
                }
                return true;
            }

            /**
             * @brief Loads the file with lines describing useful blinking singals
             *
             * @param sequence_file The input file name
             *
             * @return Success status
             */
            bool parseSequenceFile(std::string sequence_file){
                ROS_WARN_STREAM("[UVDAR_BP_Tim]: Add sanitation - sequences must be of equal, non-zero length");
                ROS_INFO_STREAM("[UVDAR_BP_Tim]: Loading sequence from file: [ " + sequence_file + " ]");
                std::ifstream ifs;
                ifs.open(sequence_file);
                std::string word;
                std::string line;
                std::vector<std::vector<bool>> sequences;
                if (ifs.good()) {
                    ROS_INFO("[UVDAR_BP_Tim]: Loaded Sequences: [: ");
                    while (getline( ifs, line )){
                        if (line[0] == '#'){
                            continue;
                        }
                        std::string show_string = "";
                        std::vector<bool> sequence;
                        std::stringstream iss(line); 
                        std::string token;
                        while(std::getline(iss, token, ',')) {
                            if (!_enable_manchester_) {
                                sequence.push_back(token=="1");
                            } else {
                            // Manchester Coding - IEEE 802.3 Conversion: 1 = [0,1]; 0 = [1,0]
                                if (token=="1"){
                                    sequence.push_back(false);
                                    sequence.push_back(true); 
                                } else {
                                    sequence.push_back(true);
                                    sequence.push_back(false);
                                }
                            }
                        }
                        for (const auto boolVal : sequence ) {
                            if (boolVal) show_string += "1,";
                            else show_string += "0,";
                        }
                
                        sequences.push_back(sequence);
                        ROS_INFO_STREAM("[UVDAR_BP_Tim]:   [" << show_string << "]");
                    }
                    ROS_INFO("[UVDAR_BP_Tim]: ]");
                    ifs.close();
                    _sequences_ = sequences;
              
                } else {
                    ROS_ERROR_STREAM("[UVDAR_BP_Tim]: Failed to load sequence file " << sequence_file << "! Returning.");
                    ifs.close();
                    return false;
                }
            return true;
            }
  
            /**
             * @brief calls ParamLoad and loads parameters from launch file
             * 
             * @param printParams 
             * @param private_nh_ 
             */
            void loadParams(const bool & printParams, const ros::NodeHandle & private_nh_){

                mrs_lib::ParamLoader param_loader(private_nh_, printParams, "UVDAR_BP_Tim");

                param_loader.loadParam("uav_name", _uav_name_, std::string());
                param_loader.loadParam("debug", _debug_, bool(false));
                param_loader.loadParam("visual_debug", _visual_debug_, bool(false));
                if ( _visual_debug_) {
                    ROS_WARN_STREAM("[UVDAR_BP_Tim]: You are using visual debugging. This option is only meant for development. Activating it significantly increases load on the system and the user should do so with care.");
                }
                
                param_loader.loadParam("gui", _gui_, bool(true)); // currently not used!
                param_loader.loadParam("publish_visualization", _publish_visualization_, bool(false)); // currently not used!
                param_loader.loadParam("visualization_rate", _visualization_rate_, float(2.0)); // currently not used!

                param_loader.loadParam("points_seen_topics", _points_seen_topics, _points_seen_topics);
                
                param_loader.loadParam("enable_manchester", _enable_manchester_, bool(false));
                if (_enable_manchester_) ROS_WARN_STREAM("[UVDARBlinkProcessor]: Manchester Decoding is enabled. Make sure Transmitter has same coding enabled!");

                param_loader.loadParam("sequence_file", _sequence_file, std::string());
                
                private_nh_.param("blinkers_seen_topics", _blinkers_seen_topics, _blinkers_seen_topics);
                private_nh_.param("estimated_framerate_topics", _estimated_framerate_topics, _estimated_framerate_topics);

                private_nh_.param("use_camera_for_visualization", _use_camera_for_visualization_, bool(true));

            }

            std::atomic_bool initialized_ = false;  
            std::atomic_bool current_visualization_done_ = false;
            ros::Timer timer_visualization_;
            std::vector<std::vector<bool>> _sequences_;
            std::vector<ros::Publisher> pub_blinkers_seen_;
            std::vector<ros::Publisher> pub_estimated_framerate_;
            std::vector<ros::Timer> timer_process_;
            std::vector<std::shared_ptr<std::mutex>>  mutex_camera_image_;


            std::vector<std::string> _blinkers_seen_topics;
            std::vector<std::string> _estimated_framerate_topics;
            std::vector<std::string> _points_seen_topics;
            std::string _sequence_file;

            std::vector<std::vector<mrs_msgs::ImagePointsWithFloatStampedConstPtr>> small_buffer_;

               

            using image_callback_t = boost::function<void (const sensor_msgs::ImageConstPtr&)>;
            std::vector<image_callback_t> cals_image_;
            std::vector<ros::Subscriber> sub_image_;

            using points_seen_callback_t = boost::function<void (const mrs_msgs::ImagePointsWithFloatStampedConstPtr&)>;
            std::vector<points_seen_callback_t> cals_points_seen_;
            std::vector<points_seen_callback_t> cals_sun_points_;
            std::vector<ros::Subscriber> sub_points_seen_;
            std::vector<ros::Subscriber> sub_sun_points_;
            
            // Params
            std::string _uav_name_;   
            bool        _debug_;
            bool        _visual_debug_;
            bool        _gui_;
            bool        _publish_visualization_;
            float       _visualization_rate_;
            bool        _use_camera_for_visualization_;
            bool        _enable_manchester_;





    }; // UVDAR_BP_Tim class
} // uvdar namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDAR_BP_Tim, nodelet::Nodelet);

