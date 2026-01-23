#include "state_estimator/state_estimator_node.hpp"

namespace state_estimator {

void state_estimator_node::shutdown() {
	ROS_INFO("SE node: Shutting node down...");
	for (auto const&plugin:loaded_plugins) {
		ROS_INFO_STREAM("Shutting down " << plugin->getName() << "...");
		plugin->shutdown();
		ROS_INFO("Done");
	}
	ROS_INFO("State Estimator shut down.");
}

state_estimator_node::state_estimator_node(ros::NodeHandle& nh) : nh(nh), plugin_loader("state_estimator", "state_estimator_plugins::PluginBase") {
   // robot = Anymal::create();

	ros::V_string blacklist{}, whitelist{};

	getBlacklist(blacklist);
	getWhitelist(whitelist);

	//Assume all plugins are blacklisted
	if(blacklist.empty() and !whitelist.empty())
		blacklist.emplace_back("*");

	for (auto &name : plugin_loader.getDeclaredClasses()) {
		add_plugin(name, blacklist, whitelist);
	}

  services.push_back(std::make_shared<ros::ServiceServer>(nh.advertiseService("getActiveEstimators",&state_estimator_node::getActiveEstimators,this)));
  services.push_back(std::make_shared<ros::ServiceServer>(nh.advertiseService("getBlacklist",&state_estimator_node::getBlacklist,this)));
  services.push_back(std::make_shared<ros::ServiceServer>(nh.advertiseService("getWhitelist",&state_estimator_node::getWhitelist,this)));
  services.push_back(std::make_shared<ros::ServiceServer>(nh.advertiseService("listAllEstimators",&state_estimator_node::listAllEstimators,this)));
  services.push_back(std::make_shared<ros::ServiceServer>(nh.advertiseService("pauseEstimator",&state_estimator_node::pauseEstimator,this)));
  services.push_back(std::make_shared<ros::ServiceServer>(nh.advertiseService("resetEstimator",&state_estimator_node::resetEstimator,this)));
  services.push_back(std::make_shared<ros::ServiceServer>(nh.advertiseService("restartEstimator",&state_estimator_node::restartEstimator,this)));
  services.push_back(std::make_shared<ros::ServiceServer>(nh.advertiseService("startEstimator",&state_estimator_node::startEstimator,this)));
  services.push_back(std::make_shared<ros::ServiceServer>(nh.advertiseService("stopEstimator",&state_estimator_node::stopEstimator,this)));
}

bool state_estimator_node::add_plugin(std::string &name) {
    ros::V_string blacklist{}, whitelist{};
    if (!getBlacklist(blacklist)) return false;
    if (!getWhitelist(whitelist)) return false;

    //Assume all plugins are blacklisted
    if(blacklist.empty() and !whitelist.empty())
	    blacklist.emplace_back("*");
    
    return add_plugin(name,blacklist,whitelist);
}

bool state_estimator_node::add_plugin(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist) {
	if (is_blacklisted(pl_name,blacklist,whitelist)) {
		ROS_INFO_STREAM("Plugin " << pl_name << " blacklisted");
		return false;
	}

	try {
		auto plugin = plugin_loader.createInstance(pl_name);
		ROS_INFO_STREAM("Plugin " << pl_name << " loaded");
		plugin->initialize(this->nh,nullptr);
		ROS_INFO_STREAM("Plugin " << pl_name << " initialized");
		loaded_plugins.push_back(plugin);
		return true;
	} catch (pluginlib::PluginlibException &ex) {
		ROS_ERROR_STREAM("Plugin " << pl_name << " load exception: " << ex.what());
	} catch (std::exception &ex) {
		ROS_ERROR_STREAM("Plugin " << pl_name << " load exception: " << ex.what());
	} catch (...) {
		ROS_ERROR_STREAM("Plugin " << pl_name << " load exception.");
	}
	return false;
}

inline bool state_estimator_node::getBlacklist(ros::V_string &blacklist) {
    if (nh.hasParam("plugin_blacklist")) { 
	    nh.getParam("plugin_blacklist", blacklist);
    } else {
	    ROS_WARN("Blacklist parameter does not exist.");
	    return false;
    }       
    return true;  
}

inline bool state_estimator_node::getWhitelist(ros::V_string &whitelist) {
    if (nh.hasParam("plugin_whitelist")) { 
	    nh.getParam("plugin_whitelist", whitelist);
    } else {
	    ROS_WARN("Whitelist parameter does not exist.");
	    return false;
    }    
    return true;   
}

    /**
    * @brief Checks that plugin blacklisted
    *  1. if blacklist and whitelist is empty: load all
    *  2. if blacklist is empty and whitelist non empty: assume blacklist is ["*"]
    *  3. if blacklist non empty: usual blacklist behavior
    *  4. if whitelist non empty: override blacklist
    */
bool state_estimator_node::is_blacklisted(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist)
{
    for (auto &bl_pattern : blacklist) {
      if (pattern_match(bl_pattern, pl_name)) {
        for (auto &wl_pattern : whitelist) {
          if (pattern_match(wl_pattern, pl_name))
            return false;
        }
        return true;
      }
    }
    return false;
}



bool state_estimator_node::pattern_match(std::string &pattern, std::string &pl_name)
{
    int cmp = fnmatch(pattern.c_str(), pl_name.c_str(), FNM_CASEFOLD);
    if (cmp == 0)
      return true;
    else if (cmp != FNM_NOMATCH) {
      // never see that, i think that it is fatal error.
      ROS_FATAL("Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
          pattern.c_str(), pl_name.c_str(), cmp);
      ros::shutdown();
    }

    return false;
}


//**************************************************
//Services
//**************************************************

bool state_estimator_node::getActiveEstimators(state_estimator_msgs::getActiveEstimators::Request &req, state_estimator_msgs::getActiveEstimators::Response &res) {
    for (auto const&plugin:loaded_plugins) {
	res.names.push_back(plugin->getName());
    }
    return true;
}

bool state_estimator_node::getBlacklist(state_estimator_msgs::getBlacklist::Request &req, state_estimator_msgs::getBlacklist::Response &res) {
    ros::V_string blacklist{};
    if (!getBlacklist(blacklist)) return true;
    for (auto const&plugin:blacklist) {
	res.names.push_back(plugin);
    }
    return true;
}

bool state_estimator_node::getWhitelist(state_estimator_msgs::getWhitelist::Request &req, state_estimator_msgs::getWhitelist::Response &res) {
    ros::V_string whitelist{};
    if (!getWhitelist(whitelist)) return true;
    for (auto const&plugin:whitelist) {
	res.names.push_back(plugin);
    }
    return true;
}

bool state_estimator_node::listAllEstimators(state_estimator_msgs::listAllEstimators::Request &req, state_estimator_msgs::listAllEstimators::Response &res) {
    for (auto &name : plugin_loader.getDeclaredClasses()) {
	    res.names.push_back(name);
    }
    return true;
}

bool state_estimator_node::pauseEstimator(state_estimator_msgs::pauseEstimator::Request &req, state_estimator_msgs::pauseEstimator::Response &res) {
    for (auto const&plugin:loaded_plugins) {
	if(plugin->getName().compare(req.name)==0)  {
	    if (plugin->isPaused()) {
		ROS_INFO_STREAM("Plugin " << req.name << " already paused.");
		res.success=false;
		return true;	
	    }
	    ROS_INFO_STREAM("Pausing " << req.name << "...");
	    plugin->pause();
	    ROS_INFO_STREAM("Plugin " << req.name << " paused.");
	    res.success = true;
	    return true;
	}
    }
    ROS_WARN_STREAM("Plugin " << req.name << " not loaded.");
    res.success = false;
    return true;
}

bool state_estimator_node::resetEstimator(state_estimator_msgs::resetEstimator::Request &req, state_estimator_msgs::resetEstimator::Response &res) {
    for (auto const&plugin:loaded_plugins) {
	if(plugin->getName().compare(req.name)==0)  {
	    ROS_INFO_STREAM("Reseting " << req.name << "...");
	    plugin->pause();
	    ROS_INFO_STREAM("Plugin " << req.name << " reset.");
	    res.success = true;
	    return true;
	}
    }
    ROS_WARN_STREAM("Plugin " << req.name << " not loaded.");
    res.success = false;
    return true;
}

bool state_estimator_node::restartEstimator(state_estimator_msgs::restartEstimator::Request &req, state_estimator_msgs::restartEstimator::Response &res) {
    for (auto const&plugin:loaded_plugins) {
	if(plugin->getName().compare(req.name)==0)  {
	    ROS_INFO_STREAM("Shutting down " << req.name << "...");
	    plugin->shutdown();
	    ROS_INFO_STREAM("Plugin " << req.name << " shut down");
	    res.success = add_plugin(req.name);
	    return true;
	}
    }
    ROS_WARN_STREAM("Plugin " << req.name << " not loaded.");
    res.success = false;
    return true;
}

bool state_estimator_node::startEstimator(state_estimator_msgs::startEstimator::Request &req, state_estimator_msgs::startEstimator::Response &res) {
    res.success = add_plugin(req.name);
    return true;
}

bool state_estimator_node::stopEstimator(state_estimator_msgs::stopEstimator::Request &req, state_estimator_msgs::stopEstimator::Response &res) {
    for (auto const&plugin:loaded_plugins) {
        if(plugin->getName().compare(req.name)==0)  {
            if (!plugin->isRunning()) {
            ROS_INFO_STREAM("Plugin " << req.name << " is not running.");
            res.success=false;
            return true;	
            }
            ROS_INFO_STREAM("Shutting down " << req.name << "...");
            plugin->shutdown();
            ROS_INFO_STREAM("Plugin " << req.name << " shut down");
            res.success=true;
            return true;
        }
    }
    ROS_WARN_STREAM("Plugin " << req.name << " not loaded.");
    res.success=false;
    return true;
}

} //namespace state_estimator

