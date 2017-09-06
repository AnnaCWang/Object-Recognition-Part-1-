	#include <ros/ros.h>
	#include <image_transport/image_transport.h>
	#include <cv_bridge/cv_bridge.h>
	#include <sensor_msgs/image_encodings.h>
	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/opencv.hpp> 
	#include "std_msgs/Float32.h" 
	
	static const std::string OPENCV_WINDOW = "Image window";
	static const std::string OUT_WINDOW = "Output window";
	
	class ImageConverter
	{
	  ros::NodeHandle nh_;
	  image_transport::ImageTransport it_;
	  image_transport::Subscriber image_sub_;
	  image_transport::Publisher image_pub_;
	  ros::NodeHandle n;
	  ros::Publisher pub;
	  
	public:
	  ImageConverter()
	    : it_(nh_)
	  {
	    // Subscrive to input video feed and publish output video feed
	    image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, 
	      &ImageConverter::imageCb, this);
	    image_pub_ = it_.advertise("/image_converter/output_video", 1);
		pub = n.advertise<std_msgs::Float32>("xCoordinate", 100);
	    cv::namedWindow(OPENCV_WINDOW);
	  }
	
	  ~ImageConverter()
	  {
	    cv::destroyWindow(OPENCV_WINDOW);
	  }
	
	  void imageCb(const sensor_msgs::ImageConstPtr& msg)
	  {
	    cv_bridge::CvImagePtr cv_ptr;
	    try
	    {
	      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    }
	    catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	      return;
	    }
	    std_msgs::Float32 xMsg;
	    //declare output image
	    cv::Mat outImg;
	   
		cv::GaussianBlur( cv_ptr->image, outImg, cv::Size( 51, 51 ), 0, 0 );
		cv::cvtColor(outImg, outImg, CV_BGR2HSV); 
	    cv::Scalar  min(100,210,200);
		cv::Scalar  max(255,255,255);
		cv::inRange( outImg, min, max, outImg);
	
	
		double x = 0;
		double y = 0;
		int total = 0;
		for (unsigned int i = 0; i < outImg.rows; i ++){
			for (unsigned int j = 0; j < outImg.cols; j ++){
				int b_ij = (int)outImg.at<cv::Vec3b>(i,j)[0];
				int g_ij = (int)outImg.at<cv::Vec3b>(i,j)[1];
				int r_ij = (int)outImg.at<cv::Vec3b>(i,j)[2];
			
				if (b_ij + g_ij + r_ij >= 700) {
					x+=j;
					y+=i;
					total++;
				}
			}
		} 
		
		//compute avg xy floating points
		double xAvg = x/ (double)total;
		double yAvg = y/ (double)total;
		if (total > 40) {
			ROS_INFO("x: %f y: %f", xAvg,yAvg);
			cv::circle(outImg, cv::Point((int)xAvg, (int)yAvg), 30, CV_RGB(100,150,100));
		}
		
		xMsg.data = ((x - ((double)outImg.cols / 2)) / ((double)outImg.cols / 2)) / 2;
		
		
		
		//show input
	    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	    
		//show output
		cv::imshow(OUT_WINDOW, outImg);
	    
		//pause for 3 ms
	    cv::waitKey(3);
	    
	    //public x information to move base client
	    pub.publish(xMsg);
	    
	    // Output modified video stream
	    image_pub_.publish(cv_ptr->toImageMsg());
	  }
	};
	
	int main(int argc, char** argv)
	{
	  ros::init(argc, argv, "image_converter");
	  ImageConverter ic;
	  ros::spin();
	  return 0;
	}
	
