#include <ros/ros.h>
#include <spa_core/spa_common.h>
#include <spa_core/SpaRequestLsProbe.h>
#include <spa_core/SpaProbe.h>
#include <spa_core/SpaXteds.h>
#include <actionlib/server/simple_action_server.h>
#include <spa_core/SpaQueryAction.h>
//#include <spa_core/xteds_parser.h>
//#include <spa_core/xteds_repository.h>
#include <string>
#include <fstream>
#include <cstdlib>
#include <sys/stat.h> // to create directory

namespace spa
{

class LookupService
{
public:
	LookupService();
	/// Finding if the component's xTEDS already exists in the repository.
	bool existXteds(uint64_t xuuid);

  /// change the xTEDS repository path.
  void setXtedsRepoPath(const std::string &path)
  {
    xteds_repo_path = path;
    mkdir(xteds_repo_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

private:
    /// Callback for SM-x request to Lookup Service to perform component probe.
  bool probeCallback(spa_core::SpaRequestLsProbe::Request &req, spa_core::SpaRequestLsProbe::Response &res);

  /// Callback for component request to Lookup Service
  void queryCallback(const spa_core::SpaQueryGoalConstPtr &goal);

  // relate to ROS
	ros::NodeHandle nh;
  ros::ServiceServer probeServer;
  typedef actionlib::SimpleActionServer<spa_core::SpaQueryAction> SpaQueryServer;
  SpaQueryServer queryServer;
  ros::ServiceClient probeClient;
  ros::ServiceClient xtedsReqClient;

  // relate to xTEDS
  std::string xteds_repo_path; // = $HOME/.xteds_repo
  //XtedsRepository xtedsRepository;
  //XtedsParser xtedsParser;
};

LookupService::LookupService() :
  xteds_repo_path(std::string(std::getenv("HOME")) + "/.xteds_repo"),
  queryServer(nh, "spa_ls/spa_query", boost::bind(&LookupService::queryCallback, this, _1), false)
{
  mkdir(xteds_repo_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  ROS_INFO("set xTEDS repository path to %s", xteds_repo_path.c_str());
  probeServer = nh.advertiseService("spa_ls/request_probe", &LookupService::probeCallback, this);
  queryServer.start();
}

bool LookupService::existXteds(uint64_t xuuid)
{
  // to do
	return false;
}

bool LookupService::probeCallback(spa_core::SpaRequestLsProbe::Request &req, spa_core::SpaRequestLsProbe::Response &res)
{
	// Probe the SPA component use node name.
  spa_core::SpaProbe srv1;
  srv1.request.dialogId = 0;
  probeClient = nh.serviceClient<spa_core::SpaProbe>(req.nodeName + "/heartbeat");
  if (!probeClient.call(srv1))
  {
    ROS_ERROR("can't connect with %s", req.nodeName.c_str());
    return false;
  }

  // Find if the component's xTEDS already exists in the repository.
  if (existXteds(srv1.response.xuuid))
  {
    return true;
  }

	// Request the xTEDS of the component.
  spa_core::SpaXteds srv2;
  srv2.request.dialogId = 0;
  xtedsReqClient = nh.serviceClient<spa_core::SpaXteds>(req.nodeName + "/xteds");
  ROS_INFO("request xteds from %s", req.nodeName.c_str());
  if (!xtedsReqClient.call(srv2))
  {
    ROS_ERROR("can't get xTEDS of %s", req.nodeName.c_str());
    return false;
  }

  // parse and store
  std::string::size_type pos = req.nodeName.find_last_of('/');
  std::string uri(xteds_repo_path + req.nodeName.substr(pos) + ".xml");
  std::ofstream fout(uri.c_str());

  if (!fout.is_open())
  {
    ROS_ERROR("can't save xTEDS file of %s", req.nodeName.c_str());
    return false;
  }

  fout << srv2.response.xteds;
  fout.close();
	return true;
}

void LookupService::queryCallback(const spa_core::SpaQueryGoalConstPtr &goal)
{
  spa_core::SpaQueryFeedback feedback;
  spa_core::SpaQueryResult result;
  bool success = true;

  if (success)
  {
    result.resultMode = 0;
    queryServer.setSucceeded(result);
  }
}

} // namespace spa

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spa_ls");

  spa::LookupService spa_ls;

  ros::spin();
}
