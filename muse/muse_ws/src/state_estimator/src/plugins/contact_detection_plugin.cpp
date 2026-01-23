#include "state_estimator/plugin.hpp"
#include <ros/ros.h>

#include "state_estimator_msgs/ContactDetection.h"
#include <geometry_msgs/WrenchStamped.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


namespace state_estimator_plugins
{

typedef message_filters::sync_policies::ApproximateTime
<
	geometry_msgs::WrenchStamped,
	geometry_msgs::WrenchStamped,
	geometry_msgs::WrenchStamped,
	geometry_msgs::WrenchStamped
> 
ApproximateTimePolicy;

typedef message_filters::sync_policies::ExactTime
<
	geometry_msgs::WrenchStamped,
	geometry_msgs::WrenchStamped,
	geometry_msgs::WrenchStamped,
	geometry_msgs::WrenchStamped
> 
ExactTimePolicy;

#define MySyncPolicy ApproximateTimePolicy

	class ContactDetectionPlugin : public PluginBase
	{
	public:
		ContactDetectionPlugin(): 
			wrench_lf_sub_(nullptr),
			wrench_rf_sub_(nullptr),
			wrench_lh_sub_(nullptr),
			wrench_rh_sub_(nullptr),
			pub_(nullptr), 
			sync_(nullptr) 
		{ } 
	
		~ContactDetectionPlugin() 
		{
			if (wrench_lf_sub_!=nullptr) delete(wrench_lf_sub_);
			if (wrench_rf_sub_!=nullptr) delete(wrench_rf_sub_);
			if (wrench_lh_sub_!=nullptr) delete(wrench_lh_sub_);
			if (wrench_rh_sub_!=nullptr) delete(wrench_rh_sub_);
			if (pub_!=nullptr) delete(pub_);
			if (sync_!=nullptr) delete(sync_);
		}

		std::string getName() override { return std::string("ContactDetection"); }
		std::string getDescription() override { return std::string("Contact Detection Plugin"); }

		void initialize_() override {

            // Load parameters from YAML
            // LF: left front leg, RF: right front leg, LH: left hind leg, RH: right hind leg
            std::string lf_topic, rf_topic, lh_topic, rh_topic, pub_topic;
            nh_.param("contact_detection_plugin/wrench_lf_topic", lf_topic, std::string("/state_estimator/contact_force_lf_foot"));
            nh_.param("contact_detection_plugin/wrench_rf_topic", rf_topic, std::string("/state_estimator/contact_force_rf_foot"));
            nh_.param("contact_detection_plugin/wrench_lh_topic", lh_topic, std::string("/state_estimator/contact_force_lh_foot"));
            nh_.param("contact_detection_plugin/wrench_rh_topic", rh_topic, std::string("/state_estimator/contact_force_rh_foot"));
            nh_.param("contact_detection_plugin/pub_topic", pub_topic, std::string("contact_detection"));
            nh_.param("contact_detection_plugin/grf_threshold", grf_threshold_, 15.0);

            ROS_INFO_STREAM("ContactDetectionPlugin loaded with GRF threshold: " << grf_threshold_);

            // Set up subscribers
            wrench_lf_sub_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, lf_topic, 250);
            wrench_rf_sub_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, rf_topic, 250);
            wrench_lh_sub_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, lh_topic, 250);
            wrench_rh_sub_ = new message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh_, rh_topic, 250);

            // Synchronizer
            sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(250), *wrench_lf_sub_, *wrench_rf_sub_, *wrench_lh_sub_, *wrench_rh_sub_);
            sync_->registerCallback(boost::bind(&ContactDetectionPlugin::callback, this, _1, _2, _3, _4));

            // Publisher
            pub_ = new ros::Publisher(nh_.advertise<state_estimator_msgs::ContactDetection>(pub_topic, 250));

		}


		void shutdown_() override { }
		void pause_() override { }
		void resume_() override { }
		void reset_() override { }

		void callback
		(
			const geometry_msgs::WrenchStamped::ConstPtr& wrench_lf,
			const geometry_msgs::WrenchStamped::ConstPtr& wrench_rf,
			const geometry_msgs::WrenchStamped::ConstPtr& wrench_lh,
			const geometry_msgs::WrenchStamped::ConstPtr& wrench_rh
		)
		{
			// Calculate the norm of the wrenches
            wrench_lf_norm = sqrt(pow(wrench_lf->wrench.force.x,2) + pow(wrench_lf->wrench.force.y,2) + pow(wrench_lf->wrench.force.z,2));
            wrench_rf_norm = sqrt(pow(wrench_rf->wrench.force.x,2) + pow(wrench_rf->wrench.force.y,2) + pow(wrench_rf->wrench.force.z,2));
            wrench_lh_norm = sqrt(pow(wrench_lh->wrench.force.x,2) + pow(wrench_lh->wrench.force.y,2) + pow(wrench_lh->wrench.force.z,2));
            wrench_rh_norm = sqrt(pow(wrench_rh->wrench.force.x,2) + pow(wrench_rh->wrench.force.y,2) + pow(wrench_rh->wrench.force.z,2));

            if (wrench_lf_norm > grf_threshold_) stance_lf = true; else stance_lf = false;
            if (wrench_rf_norm > grf_threshold_) stance_rf = true; else stance_rf = false;
            if (wrench_lh_norm > grf_threshold_) stance_lh = true; else stance_lh = false;
            if (wrench_rh_norm > grf_threshold_) stance_rh = true; else stance_rh = false;


            // publishing
            msg_.header.stamp = ros::Time::now();

			msg_.stance_lf = stance_lf;
			msg_.stance_rf = stance_rf;
			msg_.stance_lh = stance_lh;
			msg_.stance_rh = stance_rh;

			pub_->publish(msg_);

		} // end callback


	private:
	
		message_filters::Subscriber<geometry_msgs::WrenchStamped> *wrench_lf_sub_;
		message_filters::Subscriber<geometry_msgs::WrenchStamped> *wrench_rf_sub_;
		message_filters::Subscriber<geometry_msgs::WrenchStamped> *wrench_lh_sub_;
		message_filters::Subscriber<geometry_msgs::WrenchStamped> *wrench_rh_sub_;
		message_filters::Synchronizer<MySyncPolicy> *sync_;
		ros::Publisher *pub_;

		state_estimator_msgs::ContactDetection msg_;

		bool stance_lf;
		bool stance_rf;
		bool stance_lh;
		bool stance_rh;
		double wrench_lf_norm;
		double wrench_rf_norm;
		double wrench_lh_norm;
		double wrench_rh_norm;
        double grf_threshold_;



	}; // end class ContactDetectionPlugin

} //end namespace state_estimator_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(state_estimator_plugins::ContactDetectionPlugin, state_estimator_plugins::PluginBase)
