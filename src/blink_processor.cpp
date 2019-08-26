#undef _DEBUG

#define camera_delay 0.50

#define min_frequency 3
#define max_frequency 36.0
#define boundary_ratio 0.7

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <ht3dbt/ht3d.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt32MultiArray.h>
#include <uvdar/Int32MultiArrayStamped.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>

namespace enc = sensor_msgs::image_encodings;

namespace uvdar {

class BlinkProcessor : public nodelet::Nodelet {
public:

  /* onInit() //{ */

  void onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    nh_.param("uav_name", uav_name, std::string());
    nh_.param("DEBUG", DEBUG, bool(false));
    nh_.param("VisDEBUG", VisDEBUG, bool(false));
    nh_.param("GUI", GUI, bool(false));
    if (GUI)
      ROS_INFO("[BlinkProcessor]: GUI is true");
    else
      ROS_INFO("[BlinkProcessor]: GUI is false");

    nh_.param("accumulatorLength", accumulatorLength, int(23));
    nh_.param("pitchSteps", pitchSteps, int(16));
    nh_.param("yawSteps", yawSteps, int(8));
    nh_.param("maxPixelShift", maxPixelShift, int(1));

    nh_.param("publishVisualization", publishVisualization, bool(false));
    nh_.param("visualizationRate", visualizatinRate, int(5));

    nh_.param("reasonableRadius", _reasonable_radius_, int(3));
    nh_.param("nullifyRadius", _nullify_radius_, int(5));
    ht3dbt = new HT3DBlinkerTracker(accumulatorLength, pitchSteps, yawSteps, maxPixelShift, cv::Size(752, 480), _nullify_radius_, _reasonable_radius_);
    nh_.param("processRate", processRate, int(10));
    nh_.param("returnFrequencies", returnFrequencies, bool(false));

    /* subscribe to cameras //{ */

    std::vector<std::string> cameraTopics;
    nh_.param("cameraTopics", cameraTopics, cameraTopics);
    if (cameraTopics.empty()) {
      ROS_WARN("[BlinkProcessor]: No topics of cameras were supplied");
    }
    currentImages.resize(cameraTopics.size());

    // Create callbacks for each camera
    for (size_t i = 0; i < cameraTopics.size(); ++i) {
      image_callback_t callback = [imageIndex=i,this] (const sensor_msgs::ImageConstPtr& image_msg) { 
        ProcessRaw(image_msg, imageIndex);
      };
      imageCallbacks.push_back(callback);
    }
    // Subscribe to corresponding topics
    for (size_t i = 0; i < cameraTopics.size(); ++i) {
      imageSubscribers.push_back(nh_.subscribe(cameraTopics[i], 1, &image_callback_t::operator(), &imageCallbacks[i]));
    }

    //}
    
    /* subscribe to pointsSeen //{ */

    nh_.param("legacy", _legacy, bool(false));

    if (_legacy){
      nh_.param("legacy_delay", _legacy_delay, double(0.2));
      ROS_INFO_STREAM("Legacy mode in effect. Set delay is " << _legacy_delay << "s");
    }

    /* if (_legacy){ */
    /*   pointsSubscriberLegacy = nh_.subscribe("pointsSeen", 1, &BlinkProcessor::insertPointsLegacy, this); */
    /* } */
    /* else{ */
    /*   pointsSubscriber = nh_.subscribe("pointsSeen", 1, &BlinkProcessor::insertPoints, this); */
    /* } */

    /* pointsPublisher  = nh_.advertise<uvdar::Int32MultiArrayStamped>("blinkersSeen", 1); */

    std::vector<std::string> pointsSeenTopics;
    nh_.param("pointsSeenTopics", pointsSeenTopics, pointsSeenTopics);
    if (pointsSeenTopics.empty()) {
      ROS_WARN("[BlinkProcessor]: No topics of pointsSeen were supplied");
    }
    blinkData.resize(pointsSeenTopics.size());

    // Create callbacks for each camera
    for (size_t i = 0; i < pointsSeenTopics.size(); ++i) {
      points_seen_callback_t callback = [imageIndex=i,this] (const uvdar::Int32MultiArrayStampedConstPtr& pointsMessage) { 
        InsertPoints(pointsMessage, imageIndex);
      };
      pointsSeenCallbacks.push_back(callback);
    }
    // Subscribe to corresponding topics
    for (size_t i = 0; i < pointsSeenTopics.size(); ++i) {
      if (_legacy)
        pointsSeenSubscribers.push_back(nh_.subscribe(pointsSeenTopics[i], 1, &points_seen_callback_t::operator(), &pointsSeenCallbacksLegacy[i]));
      else 
        pointsSeenSubscribers.push_back(nh_.subscribe(pointsSeenTopics[i], 1, &points_seen_callback_t::operator(), &pointsSeenCallbacks[i]));
    }

    //}
    
    for (size_t i = 0; i < pointsSeenTopics.size(); ++i) {
      ht3dbt_trackers.push_back(
          new HT3DBlinkerTracker(accumulatorLength, pitchSteps, yawSteps, maxPixelShift, cv::Size(752, 480)));
      ht3dbt_trackers.back()->setDebug(DEBUG, VisDEBUG);
      processSpinRates.push_back(new ros::Rate((double)processRate));
    }

    /* initialize the publishers //{ */

    /* currImage = cv::Mat(cv::Size(752, 480), CV_8UC3, cv::Scalar(0, 0, 0)); */
    /* viewImage = currImage.clone(); */
    //CHECK


    nh_.param("UseCameraForVisualization", use_camera_for_visualization_, bool(true));
    if (use_camera_for_visualization_)
      ImageSubscriber = nh_.subscribe("camera", 1, &BlinkProcessor::ProcessRaw, this);

    std::vector<std::string> blinkersSeenTopics;
    std::vector<std::string> estimatedFramerateTopics;
    nh_.param("blinkersSeenTopics", blinkersSeenTopics, blinkersSeenTopics);
    nh_.param("estimatedFramerateTopics", estimatedFramerateTopics, estimatedFramerateTopics);

    if (blinkersSeenTopics.size() != pointsSeenTopics.size()) {
      ROS_ERROR_STREAM("[BlinkProcessor] The number of poinsSeenTopics (" << pointsSeenTopics.size() 
          << ") is not matching the number of blinkersSeenTopics (" << blinkersSeenTopics.size() << ")!");
    }
    if (estimatedFramerateTopics.size() != pointsSeenTopics.size()) {
      ROS_ERROR_STREAM("[BlinkProcessor] The number of poinsSeenTopics (" << pointsSeenTopics.size() 
          << ") is not matching the number of blinkersSeenTopics (" << estimatedFramerateTopics.size() << ")!");
    }

    for (size_t i = 0; i < blinkersSeenTopics.size(); ++i) {
      blinkersSeenPublishers.push_back(nh_.advertise<uvdar::Int32MultiArrayStamped>(blinkersSeenTopics[i], 1));
    }
    for (size_t i = 0; i < estimatedFramerateTopics.size(); ++i) {
      estimatedFrameratePublishers.push_back(nh_.advertise<std_msgs::Float32>(estimatedFramerateTopics[i], 1));
    }

    //}

    nh_.param("InvertedPoints", InvertedPoints, bool(false));
    nh_.param("frequencyCount", frequencyCount, int(4));
    /* if (frequencyCount != 2){ */
    /*   ROS_ERROR("HEYYY"); */
    /*   return; */
    /* } */

    // load the frequencies
    frequencySet.resize(frequencyCount);
    std::vector<double> defaultFrequencySet{6, 10, 15, 30, 8, 12};
    for (int i = 0; i < frequencyCount; ++i) {
      nh_.param("frequency" + std::to_string(i + 1), frequencySet[i], defaultFrequencySet.at(i));
    }

    prepareFrequencyClassifiers();

    for (size_t i = 0; i < pointsSeenTopics.size(); ++i) {
      process_threads.emplace_back(&BlinkProcessor::ProcessThread, this, i);
    }
    if (GUI) {
      show_thread  = std::thread(&BlinkProcessor::ShowThread, this);
    }

    if (publishVisualization) {
      visualization_thread = std::thread(&BlinkProcessor::VisualizeThread, this);
      image_transport::ImageTransport it(nh_);
      imPub = it.advertise("visualization", 1);
    }

    initialized = true;
    ROS_INFO("[BlinkProcessor]: initialized");
  }

  //}

  /* prepareFrequencyClassifiers() //{ */

  void prepareFrequencyClassifiers() {
    for (int i = 0; i < frequencySet.size(); i++) {
      periodSet.push_back(1.0 / frequencySet[i]);
    }
    for (int i = 0; i < frequencySet.size() - 1; i++) {
      periodBoundsBottom.push_back((periodSet[i] * (1.0 - boundary_ratio) + periodSet[i + 1] * boundary_ratio));
    }
    periodBoundsBottom.push_back(1.0 / max_frequency);

    periodBoundsTop.push_back(1.0 / min_frequency);
    for (int i = 1; i < frequencySet.size(); i++) {
      periodBoundsTop.push_back((periodSet[i] * boundary_ratio + periodSet[i - 1] * (1.0 - boundary_ratio)));
    }


    /* periodBoundsTop.back()           = 0.5 * periodSet.back() + 0.5 * periodSet[secondToLast]; */
    /* periodBoundsBottom[secondToLast] = 0.5 * periodSet.back() + 0.5 * periodSet[secondToLast]; */
  }

  //}

private:

  void insertPointsLegacy(const std_msgs::UInt32MultiArrayConstPtr& msg){
    /* if (DEBUG) */
    /*   ROS_INFO_STREAM("Getting message: " << *msg); */
    uvdar::Int32MultiArrayStampedPtr msg_stamped(new uvdar::Int32MultiArrayStamped);
    msg_stamped->stamp = ros::Time::now()-ros::Duration(_legacy_delay);
    msg_stamped->layout= msg->layout;
    std::vector<int> intVec(msg->data.begin(), msg->data.end());
    msg_stamped->data = intVec;
    /* msg_stamped->data = msg->data; */
    insertPoints(msg_stamped);
    //CHECK
  }
  
  /* InsertPoints //{ */

  void InsertPoints(const uvdar::Int32MultiArrayStampedConstPtr& msg, size_t imageIndex) {
    if (!initialized) return;

    int                      countSeen;
    std::vector<cv::Point2i> points;
    countSeen = (int)((msg)->layout.dim[0].size);

    BlinkData& data = blinkData[imageIndex];
    auto* ht3dbt = ht3dbt_trackers[imageIndex];

    data.timeSamples++;

    if (data.timeSamples >= 10) {
      ros::Time nowTime = ros::Time::now();

      data.framerateEstim = 10000000000.0 / (double)((nowTime - data.lastSignal).toNSec());
      if (DEBUG)
        std::cout << "Updating frequency: " << data.framerateEstim << " Hz" << std::endl;
      data.lastSignal = nowTime;
      ht3dbt->updateFramerate(data.framerateEstim);
      data.timeSamples = 0;
    }

    if (DEBUG) {
      ROS_INFO("Received contours: %d", countSeen);
    }
    if (countSeen < 1) {
      data.foundTarget = false;
    } else {
      data.foundTarget = true;
      data.lastSeen    = ros::Time::now();
    }

    /* { */
    cv::Point currPoint;
    /* bool      hasTwin; */
    for (int i = 0; i < countSeen; i++) {
      if (msg->data[(i * 3) + 2] <= 200) {
        if (InvertedPoints)
          currPoint = cv::Point2d(currentImages[imageIndex].cols - msg->data[(i * 3)], currentImages[imageIndex].rows - msg->data[(i * 3) + 1]);
        else
          currPoint = cv::Point2d(msg->data[(i * 3)], msg->data[(i * 3) + 1]);

        /* hasTwin = false; */
        /* for (int n = 0; n < points.size(); n++) { */
        /*   if (cv::norm(currPoint - points[i]) < 5) { */
        /*     hasTwin = true; */
        /*     break; */
        /*   } */
        /* } */

        /* if (!hasTwin) */
        points.push_back(currPoint);
      }
    }
    /* } */

    lastPointsTime = msg->stamp;
    ht3dbt->insertFrame(points);
  }

  //}

  /* ProcessThread //{ */

  void ProcessThread(size_t imageIndex) {
    std::vector<int>  msgdata;
    uvdar::Int32MultiArrayStamped msg;
    clock_t                    begin, end;
    double                     elapsedTime;

    while (!initialized) 
      processSpinRates[imageIndex]->sleep();

    auto* ht3dbt = ht3dbt_trackers[imageIndex];
    auto& retrievedBlinkers = blinkData[imageIndex].retrievedBlinkers;

    processSpinRates[imageIndex]->reset();
    for (;;) {
      if (ht3dbt->isCurrentBatchProcessed()){
        /* if (DEBUG) */
        /*   ROS_INFO("Skipping batch, already processed."); */
        continue;
      }

      if (DEBUG)
        ROS_INFO("Processing accumulated points.");

      begin = std::clock();
      ros::Time local_lastPointsTime = lastPointsTime;
      blinkData[imageIndex].retrievedBlinkersMutex->lock();

      {
        retrievedBlinkers = ht3dbt->getResults();
      }
      end         = std::clock();
      elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
      if (DEBUG)
        std::cout << "Processing: " << elapsedTime << " s, " << 1.0 / elapsedTime << " Hz" << std::endl;

      msgdata.clear();
      msg.stamp = local_lastPointsTime;
      msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      msg.layout.dim[0].size   = retrievedBlinkers.size();
      msg.layout.dim[0].label  = "count";
      msg.layout.dim[0].stride = retrievedBlinkers.size() * 3;
      msg.layout.dim[1].size   = 3;
      msg.layout.dim[1].label  = "value";
      msg.layout.dim[1].stride = 3;
      for (size_t i = 0; i < retrievedBlinkers.size(); i++) {
        msgdata.push_back(retrievedBlinkers[i].x);
        msgdata.push_back(retrievedBlinkers[i].y);
        if (returnFrequencies)
          msgdata.push_back(retrievedBlinkers[i].z);
        else
          msgdata.push_back(findMatch(retrievedBlinkers[i].z));
      }

      blinkData[imageIndex].retrievedBlinkersMutex->unlock();

      msg.data = msgdata;
      blinkersSeenPublishers[imageIndex].publish(msg);

      std_msgs::Float32 msgFramerate;
      msgFramerate.data = blinkData[imageIndex].framerateEstim;
      estimatedFrameratePublishers[imageIndex].publish(msgFramerate);

      processSpinRates[imageIndex]->sleep();
    }
  }

  //}

  /* findMatch() //{ */

  int findMatch(double i_frequency) {
    double period = 1.0 / i_frequency;
    for (int i = 0; i < periodSet.size(); i++) {
      /* std::cout << period << " " <<  periodBoundsTop[i] << " " << periodBoundsBottom[i] << " " << periodSet[i] << std::endl; */
      if ((period > periodBoundsBottom[i]) && (period < periodBoundsTop[i])) {
        return i;
      }
    }
    return -1;
  }

  //}

  /* rainbow() //{ */

  cv::Scalar rainbow(double value, double max) {
    unsigned char r, g, b;
    
    /*
    r = 255 * (std::max(0.0, (1 - (value / (max / 2.0)))));
    if (value < (max / 2.0))
      g = 255 * (std::max(0.0, value / (max / 2.0)));
    else
      g = 255 * (std::max(0.0, 1 - (value - (max / 2.0)) / (max / 2.0)));
    b   = 255 * (std::max(0.0, (((value - (max / 2.0)) / (max / 2.0)))));
    */

    double fraction = value / max;
    r = 255 * (fraction < 0.25 ? 1 : fraction > 0.5 ? 0 : 2 - fraction * 4);
    g = 255 * (fraction < 0.25 ? fraction * 4 : fraction < 0.75 ? 1 : 4 - fraction * 4);
    b = 255 * (fraction < 0.5 ? 0 : fraction < 0.75 ? fraction * 4 - 2 : 1);

    return cv::Scalar(b, g, r);
  }

  //}

  /* VisualizeThread() //{ */

  void VisualizeThread() {
    ROS_ERROR("Visualize thread");

    ros::Rate r(visualizatinRate);
    sensor_msgs::ImagePtr msg;
    while (ros::ok()) {
      r.sleep();

      mutex_show.lock();
      viewImage = cv::Mat(
          (currentImages[0].cols + 2) * currentImages.size() - 2, 
          currentImages[0].rows, 
          currentImages[0].type(),
          cv::Scalar(255, 255, 255));
      mutex_show.unlock();

      /* loop through all trackers and update the data //{ */

      for (size_t imageIndex = 0; imageIndex < ht3dbt_trackers.size(); ++imageIndex) {
        auto* ht3dbt = ht3dbt_trackers[imageIndex];
        BlinkData& data = blinkData[imageIndex];

        mutex_show.lock();
        int differenceX = (currentImages[0].cols + 2) * imageIndex;
        auto& currentImage = currentImages[imageIndex];

        // copy the image
        // CHECK
        for (int y = 0; y < currentImage.rows; ++y) {
          for (int x = 0; x < currentImage.cols; ++x) {
            if (x + differenceX >= viewImage.cols || y >= viewImage.rows) continue;
            viewImage.data[y * viewImage.cols + differenceX + x] = 
              currentImage.data[y * currentImage.cols + x];
          }
        }
        mutex_show.unlock();

        data.currTrackerCount = ht3dbt->getTrackerCount();

        cv::circle(viewImage, cv::Point(10, 10), 5, cv::Scalar(255, 100, 0));
        cv::circle(viewImage, cv::Point(10, 25), 5, cv::Scalar(0, 50, 255));
        cv::circle(viewImage, cv::Point(10, 40), 5, cv::Scalar(0, 200, 255));
        cv::circle(viewImage, cv::Point(10, 55), 5, cv::Scalar(255, 0, 100));
        cv::putText(viewImage, cv::String(to_string_precision(frequencySet[0],0)), cv::Point(15, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        cv::putText(viewImage, cv::String(to_string_precision(frequencySet[1],0)), cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        cv::putText(viewImage, cv::String(to_string_precision(frequencySet[2],0)), cv::Point(15, 45), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        cv::putText(viewImage, cv::String(to_string_precision(frequencySet[3],0)), cv::Point(15, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        //CHECK


        data.retrievedBlinkersMutex->lock();
        auto& rbs = data.retrievedBlinkers;
        for (int i = 0; i < rbs.size(); i++) {
          cv::Point center = cv::Point(rbs[i].x + differenceX, rbs[i].y);

          int        freqIndex = findMatch(rbs[i].z);
          char       freqText[4];
          char       freqTextRefined[4];
          cv::Scalar markColor;
          sprintf(freqText, "%d", std::max((int)rbs[i].z, 0));
          cv::putText(viewImage, cv::String(freqText), center + cv::Point(-5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
          if (freqIndex >= 0) {
            switch (freqIndex) {
              case 0:
                markColor = cv::Scalar(255, 100, 0);
                break;
              case 1:
                markColor = cv::Scalar(0, 50, 255);
                break;
              case 2:
                markColor = cv::Scalar(0, 200, 255);
                break;
              case 3:
                markColor = cv::Scalar(255, 0, 100);
                break;
            }

            cv::circle(viewImage, center, 5, markColor);
          }
          double yaw, pitch, len;
          yaw              = ht3dbt->getYaw(i);
          pitch            = ht3dbt->getPitch(i);
          len              = cos(pitch);
          cv::Point target = center - (cv::Point(len * cos(yaw) * 20, len * sin(yaw) * 20.0));
          cv::line(viewImage, center, target, cv::Scalar(0, 0, 255), 2);
        }
        else {
          viewImage.at<cv::Vec3b>(center) = cv::Vec3b(255,255,255);
        }

        data.retrievedBlinkersMutex->unlock();
      }

      //}

      msg = cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, viewImage).toImageMsg();
      imPub.publish(msg);
    }
  }

  //}

  /* ShowThread() //{ */

  void ShowThread() {
    //CHECK - maybe just reuse the same viewImage
    for (;;) {
      // create the viewImage
      // assuming that all images are of the same size
      mutex_show.lock();
      if (!use_camera_for_visualization_)
        viewImage = cv::Mat(
            currentImages[0].rows, 
            (currentImages[0].cols + 2) * currentImages.size() - 2, 
            currentImages[0].type(),
            cv::Scalar(255, 255, 255));
      else
        viewImage = cv::Mat(
            currentImages[0].rows, 
            (currentImages[0].cols + 2) * currentImages.size() - 2, 
            currentImages[0].type(),
            cv::Scalar(255, 255, 255));
      mutex_show.unlock();

      if (viewImage.rows == 0 || viewImage.cols == 0) continue;

      /* loop through all trackers and update the data //{ */

      for (size_t imageIndex = 0; imageIndex < ht3dbt_trackers.size(); ++imageIndex) {
        auto* ht3dbt = ht3dbt_trackers[imageIndex];
        BlinkData& data = blinkData[imageIndex];

        mutex_show.lock();
        int differenceX = (currentImages[0].cols + 2) * imageIndex;
        auto& currentImage = currentImages[imageIndex];

        for (int x = 0; x < currentImage.cols; ++x) {
          for (int y = 0; y < currentImage.rows; ++y) {
            viewImage.at<cv::Vec3b>(y, x + differenceX) = currentImage.at<cv::Vec3b>(y, x);
          }
        }
        mutex_show.unlock();

        data.currTrackerCount = ht3dbt->getTrackerCount();

        data.retrievedBlinkersMutex->lock();
        auto& rbs = data.retrievedBlinkers;
        for (int i = 0; i < rbs.size(); i++) {
          cv::Point center = cv::Point(rbs[i].x + differenceX, rbs[i].y);

          /* std::cout << "f: " << retrievedBlinkers[i].z << std::endl; */
          int        freqIndex = findMatch(rbs[i].z);
          char       freqText[4];
          char       freqTextRefined[4];
          /* sprintf(freqText,"%d",std::max((int)retrievedBlinkers[i].z,0)); */
          sprintf(freqText, "%d", std::max((int)rbs[i].z, 0));
          cv::putText(viewImage, cv::String(freqText), center + cv::Point(-5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255));
          
          if (freqIndex >= 0) {
            cv::Scalar color = rainbow(freqIndex, frequencySet.size() - 1);
            cv::circle(viewImage, center, 5, color);
          }
          double yaw, pitch, len;
          yaw              = ht3dbt->getYaw(i);
          pitch            = ht3dbt->getPitch(i);
          len              = cos(pitch);
          cv::Point target = center - (cv::Point(len * cos(yaw) * 20, len * sin(yaw) * 20.0));
          cv::line(viewImage, center, target, cv::Scalar(0, 0, 255), 2);
        }
        else {
          viewImage.at<cv::Vec3b>(center) = cv::Vec3b(255,255,255);
        }

        data.retrievedBlinkersMutex->unlock();
      }

      //}
      
      // draw the legend
      for (size_t i = 0; i < frequencySet.size(); ++i) {
        cv::Scalar color = rainbow(i, frequencySet.size() - 1);
        cv::circle(viewImage, cv::Point(10, 10 + 15 * i), 5, color);
        cv::putText(viewImage, cv::String(std::to_string((int) frequencySet[i])), cv::Point(15, 15 + 15 * i), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
      }

      /* cv::Scalar currColor; */
      /* for (int j = 0; j<256;j++){ */
      /*   currColor = rainbow(double(j),255.0); */
      /*   viewImage.at<cv::Vec3b>(viewImage.rows-1, j) = cv::Vec3b(currColor[0],currColor[1],currColor[2]); */
      /* } */
      /* ROS_INFO("W:%d, H:%d", viewImage.size().width,viewImage.size().height); */

      if (!VisDEBUG && GUI)
        cv::imshow("ocv_blink_retrieval_" + uav_name, viewImage);

      if (!VisDEBUG)
        cv::waitKey(1000.0 / 25.0);
      /* prevTime = currTime; */
    }
  }

  //}

  /* ProcessCompressed() & ProcessRaw() //{ */

  void ProcessCompressed(const sensor_msgs::CompressedImageConstPtr& image_msg, size_t imageIndex) {
    cv_bridge::CvImagePtr image;
    if (image_msg != NULL) {
      image = cv_bridge::toCvCopy(image_msg, enc::RGB8);
      mutex_show.lock();
      { currentImages[imageIndex] = image->image; }
      mutex_show.unlock();
    }
  }

  void ProcessRaw(const sensor_msgs::ImageConstPtr& image_msg, size_t imageIndex) {
    cv_bridge::CvImagePtr image;
    image = cv_bridge::toCvCopy(image_msg, enc::RGB8);
    mutex_show.lock();
    { currentImages[imageIndex] = image->image; }
    mutex_show.unlock();
  }

  //}
  
  
  std::string to_string_precision(double input, unsigned int precision){
    std::string output = std::to_string(input);
    if (precision>=0){
      if (precision==0)
        return output.substr(0,output.find_first_of("."));
    }
    return "";
  }


  /* attributes //{ */

  bool initialized = false;

  std::string              uav_name;
  bool                     currBatchProcessed;
  bool                     DEBUG;
  bool                     VisDEBUG;
  bool                     GUI;
  bool                     InvertedPoints;
  std::vector<cv::Mat>     currentImages;
  cv::Mat                  viewImage;
  std::thread              show_thread;
  std::thread              visualization_thread;
  std::vector<std::thread> process_threads;
  std::mutex               mutex_show;

  bool                     use_camera_for_visualization_;

  using image_callback_t = std::function<void (const sensor_msgs::ImageConstPtr&)>;
  std::vector<image_callback_t> imageCallbacks;
  std::vector<ros::Subscriber> imageSubscribers;

  using points_seen_callback_t = std::function<void (const uvdar::Int32MultiArrayStampedConstPtr&)>;
  std::vector<points_seen_callback_t> pointsSeenCallbacks;
  std::vector<points_seen_callback_legacy_t> pointsSeenCallbacksLegacy;
  std::vector<ros::Subscriber> pointsSeenSubscribers;

  std::vector<ros::Publisher> blinkersSeenPublishers;
  std::vector<ros::Publisher> estimatedFrameratePublishers;
  //CHECK

  bool publishVisualization;
  int visualizatinRate;
  image_transport::Publisher imPub;

  struct BlinkData {
    bool                     foundTarget;
    int                      currTrackerCount;
    std::vector<cv::Point3d> retrievedBlinkers;
    std::mutex*              retrievedBlinkersMutex;
    ros::Time                lastSeen;
    ros::Time                lastSignal;
    int                      timeSamples = 0;
    double                   timeSum = 0;

    double framerateEstim = 72;

  bool _legacy;
  double _legacy_delay;
  //CHECK

    BlinkData(): retrievedBlinkersMutex(new std::mutex{}) {};
    ~BlinkData() { delete retrievedBlinkersMutex; };
  };

  std::vector<BlinkData> blinkData;
  std::vector<HT3DBlinkerTracker*> ht3dbt_trackers;

  bool returnFrequencies;

  std::vector<double> frequencySet;
  std::vector<double> periodSet;
  std::vector<double> periodBoundsTop;
  std::vector<double> periodBoundsBottom;

  std::vector<ros::Rate*> processSpinRates;
  int        processRate;

  int accumulatorLength;
  int pitchSteps;
  int yawSteps;
  int maxPixelShift;
  int _reasonable_radius_;
  int _nullify_radius_;

  int frequencyCount;

  ros::Time lastPointsTime;

  //}
};

/* int main(int argc, char** argv) { */
/*   ros::init(argc, argv, "blink_processor"); */
/*   ROS_INFO("Starting the Blink processor node"); */
/*   ros::NodeHandle nodeA; */
/*   BlinkProcessor  bp(nodeA); */

/*   ROS_INFO("Blink processor node initiated"); */

/*   ros::spin(); */
/*   /1* ros::MultiThreadedSpinner spinner(4); *1/ */
/*   /1* spinner.spin(); *1/ */
/*   /1* while (ros::ok()) *1/ */
/*   /1* { *1/ */
/*   /1*   ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001)); *1/ */
/*   /1* } *1/ */
/*   return 0; */
/* } */

} //namsepace uvdar

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::BlinkProcessor, nodelet::Nodelet)
