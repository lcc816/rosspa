#include <ros/ros.h>
#include <spa_core/xteds_parser.h>
#include <spa_core/xteds_repository.h>
#include <string>

namespace spa
{

class LookupService
{
public:
	LookupService();
	/// Finding if the component's xTEDS already exists in the repository.
	bool existXteds(uint64_t xuuid);

	/// Probe the SPA component use node name.
	spa_core::SpaProbe spaProbeRequest(const std::string &nodeName);

	/// Request the xTEDS of the component.
	std_srvs::Trigger spaXtedsRequest(const std::string &nodeName);

private:
    /// Callback function for SM-x request to Lookup Service to perform component probe.
  bool probeCallback(spa_core::SpaLookupServiceProbe::Request &req, spa_core::SpaLookupServiceProbe::Response &res);

	/// Callback function for component request to Lookup Service
  bool queryCallback(spa_core::SpaQuery::Request &req, spa_core::SpaQuery::Response &res);

	///< relate to ROS
	ros::NodeHandle nh;
	ros::ServiceServer probeServer;
	ros::ServiceServer queryServer;
	ros::ServiceClient xtedsClient;

	///< relate to xTEDS
	XtedsRepository xtedsRepository;
	XtedsParser xtedsParser;
};

LookupService::LookupService()
{
	probeServer = nh.advertiseService("spa_lookup_service_probe", LookupService::probeCallback);
	queryServer = nh.advertiseService("spa_query", LookupService::queryCallback);
}

bool LookupService::existXteds(uint64_t xuuid)
{
	return false;
}

std_srvs::Trigger spaXtedsRequest(const std::string &nodeName)
{
	xtedsClient = nh.serviceClient<std_srvs::Trigger>(nodeName + "/xteds");
	std_srvs::Trigger srv;
	xtedsClient.call(srv);
	return srv;
}

bool LookupService::probeCallback(spa_core::SpaLookupServiceProbe::Request &req, spa_core::SpaLookupServiceProbe::Response &res)
{
	// Probe the SPA component use node name.
  	spa_core::SpaProbe srv1 = spaProbeRequest(req.nodeName);

	// Finding if the component's xTEDS already exists in the repository.
	if (existXteds(srv1.response.xuuid))
		return true;

	// Request the xTEDS of the component.
	std_srvs::Trigger srv2 = spaXtedsRequest(req.nodeName);
	// parse
	XtedsParser::XtedsDocPtr doc = xtedsParser.deserialize(srv2.response.message);
	// store
	char * name = xtedsParser.getXtedsName(doc);
	xtedsRepository.saveAsXteds(srv2.response.message, name);
	xtedsRepository.store(doc);

	return true;
}

bool LookupService::queryCallback(spa_core::SpaQuery::Request &req, spa_core::SpaQuery::Response &res)
{
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spa_ls");

  spa::LookupService spa_ls;

  ros::spin();
}

