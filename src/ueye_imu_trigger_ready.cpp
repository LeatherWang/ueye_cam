#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include <std_srvs/Trigger.h>
#include <slam_car/OdomTriggerControl.h>

class TriggerReady
{

public:
	TriggerReady()
	{
		cam0_OK_ = false;
		cam1_OK_ = false;
		framerate_hz_ = 18.0; // default framerate TODO get this from the ueye node
        triggerClient_ = n_.serviceClient<slam_car::OdomTriggerControl>("/xsens/trigger_control");
		advertiseService();
	}

	bool servCam0(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
	{
		cam0_OK_ = true;
		resp.success = true;
		ROS_INFO("TriggerReady: Camera 0 is primed for trigger");
		return true;
	}

	bool servCam1(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
	{
		cam1_OK_ = true;
		resp.success = true;
		ROS_INFO("TriggerReady: Camera 1 is primed for trigger");
		return true;
	}

	bool cam0_OK()
	{
		return cam0_OK_;
	}

	bool cam1_OK()
	{
		return cam1_OK_;
	}

	void reset_cam()
	{
		cam0_OK_ = cam1_OK_ = false;
	}

	int enableTrigger()
	{
        //srv_.request.cycle_time = (1000.0 / framerate_hz_);
		srv_.request.trigger_enable = true;

		if (triggerClient_.call(srv_)) {
			ROS_INFO_STREAM("TriggerReady: Successfully enabled camera trigger at: " << framerate_hz_ << " Hz. NOTE: trgger rate cannot be changed from mavros now. Use QGC trigger interval instead.");

		} else {
			ROS_ERROR("TriggerReady: Failed to call trigger_control service");
			return 1;
		}

		return 0;
	}
	
	int disableTrigger()
	{
        //srv_.request.cycle_time = 0;
		srv_.request.trigger_enable = false;

		if (triggerClient_.call(srv_)) {
			ROS_INFO("TriggerReady: Successfully disabled camera trigger");

		} else {
			ROS_ERROR("TriggerReady: Failed to call trigger_control service");
			return 1;
		}

		return 0;
	}

	void advertiseService()
	{
        serverCam0_ = n_.advertiseService("/cam0/trigger_ready", &TriggerReady::servCam0, this);
        serverCam1_ = n_.advertiseService("/cam1/trigger_ready", &TriggerReady::servCam1, this);
	}


private:

	bool cam0_OK_;
	bool cam1_OK_;
	float framerate_hz_;

	ros::NodeHandle n_;

	ros::ServiceClient triggerClient_;
    slam_car::OdomTriggerControl srv_;

	ros::ServiceServer serverCam0_;
	ros::ServiceServer serverCam1_;

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "StartTrigger");
	TriggerReady tr;
	
    ros::Rate r2(5); // Hz，发送服务的request，所以速率不要太快
    ros::Rate r(100); // Hz，接受服务的request，主要用于查询，速率快点没关系

	// Send start trigger command to Pixhawk to echo the current timestamp
    // 使能飞控向相机发送触发信号，以获得时间戳
	while (tr.enableTrigger() && ros::ok()) {
		ROS_INFO("TriggerReady: Retrying reaching pixhawk");
		r2.sleep();
	}
	ROS_INFO_STREAM("Started px4 triggering");
	
	// wait for camera acknowledge
    // 等待相机准备好了触发，接着相机Node节点会睡眠一秒钟用于等待飞控停止发送触发信号
	//while (!(tr.cam0_OK() && tr.cam1_OK()) && ros::ok()) {
	while (!tr.cam0_OK() && ros::ok()) {
		ros::spinOnce();
		r.sleep();
    }
	tr.reset_cam();

	// Send stop trigger command to Pixhawk to allow measuring the offset
    // 停止飞控发送触发信号
	while (tr.disableTrigger() && ros::ok()) {
		ROS_INFO("TriggerReady: Retrying reaching pixhawk");
		r2.sleep();
	}
	ROS_INFO("TriggerReady: Stopped px4 triggering to set the offset");
	
	// wait for camera acknowledge
    //! @attention 再次等待相机已经准备好了，这时缓冲区已经清理完成
	//while (!(tr.cam0_OK() && tr.cam1_OK()) && ros::ok()) {
	while (!tr.cam0_OK() && ros::ok()) {
		ros::spinOnce();
		r.sleep();
	} 
	tr.reset_cam();
	
	// Send start trigger command to Pixhawk
	while (tr.enableTrigger() && ros::ok()) {
		ROS_INFO("TriggerReady: Retrying reaching pixhawk");
		r2.sleep();
	}
	ROS_INFO("TriggerReady: Restarted px4 triggering");
}

