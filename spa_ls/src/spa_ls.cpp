#include <ros/ros.h>
#include <spa_core/spa_common.h>
#include <spa_core/xteds.h>
#include <spa_core/xteds_repository.h>
#include <spa_msgs/SpaRequestLsProbe.h>
#include <spa_msgs/SpaProbe.h>
#include <spa_msgs/SpaXteds.h>
#include <actionlib/server/simple_action_server.h>
#include <spa_msgs/SpaQueryAction.h>
#include <string>
#include <fstream>
#include <memory>
#include <cstdlib>
#include <sys/stat.h> // to create directory

namespace spa
{

class LookupService
{
public:
	LookupService();
	/// Finding if the component's xTEDS already exists in the repository.
	bool existXteds(uuid_t xuuid);

  /// change the xTEDS repository path.
  void setXtedsRepoPath(const std::string &path)
  {
    xteds_repo_path = path;
    mkdir(xteds_repo_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

private:
    /// Callback for SM-x request to Lookup Service to perform component probe.
  void requestProbeCallback(const spa_msgs::SpaRequestLsProbe::ConstPtr &msg);

  /// Callback for component request to Lookup Service
  void queryCallback(const spa_msgs::SpaQueryGoalConstPtr &goal);

  // relate to ROS
	ros::NodeHandle nh;
  ros::Subscriber probeSub;
  typedef actionlib::SimpleActionServer<spa_msgs::SpaQueryAction> SpaQueryServer;
  SpaQueryServer queryServer;
  ros::ServiceClient probeClient;
  ros::ServiceClient xtedsReqClient;

  // relate to xTEDS
  std::string xteds_repo_path; // = $HOME/.xteds_repo
  spa::XtedsRepository xtedsRepository;
};

LookupService::LookupService() :
  xteds_repo_path(std::string(std::getenv("HOME")) + "/.xteds_repo"),
  queryServer(nh, "spa_ls/spa_query", boost::bind(&LookupService::queryCallback, this, _1), false)
{
  mkdir(xteds_repo_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  ROS_INFO("set xTEDS repository path to %s", xteds_repo_path.c_str());
  probeSub = nh.subscribe("spa_ls/request_ls_probe", 100, &LookupService::requestProbeCallback, this);
  queryServer.start();
}

bool LookupService::existXteds(uuid_t xuuid)
{
  // to do
	return false;
}

void LookupService::requestProbeCallback(const spa_msgs::SpaRequestLsProbe::ConstPtr &msg)
{
	// Probe the SPA component use node name.
  spa_msgs::SpaProbe srv1;
  srv1.request.dialogId = 0;
  probeClient = nh.serviceClient<spa_msgs::SpaProbe>(msg->nodeName + "/heartbeat");
  if (!probeClient.call(srv1))
  {
    ROS_ERROR("can't connect with %s", msg->nodeName.c_str());
    return;
  }

  // Find if the component's xTEDS already exists in the repository.
  uuid_t id;
  id.deserialize(srv1.response.xuuid);
  if (existXteds(id))
  {
    return;
  }

	// Request the xTEDS of the component.
  spa_msgs::SpaXteds srv2;
  srv2.request.dialogId = 0;
  xtedsReqClient = nh.serviceClient<spa_msgs::SpaXteds>(msg->nodeName + "/xteds");
  ROS_INFO("request xteds from %s", msg->nodeName.c_str());
  if (!xtedsReqClient.call(srv2))
  {
    ROS_ERROR("can't get xTEDS of %s", msg->nodeName.c_str());
    return;
  }

  // parse and store
  XtedsPtr xteds;
  std::string uri = xteds_repo_path + msg->nodeName + ".xml";
  try // Unable to parse or fail to store
  {
    xteds = std::make_shared<Xteds>(srv2.response.xteds, uri.c_str());
    ROS_INFO("saved xTEDS file to %s", uri.c_str());
  }
  catch (std::runtime_error &error)
  {
    ROS_ERROR("%s", error.what());
    return;
  }

  // index
  xtedsRepository.index(xteds);
}

void LookupService::queryCallback(const spa_msgs::SpaQueryGoalConstPtr &goal)
{
  spa_msgs::SpaQueryFeedback feedback;
  spa_msgs::SpaQueryResult result;
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
