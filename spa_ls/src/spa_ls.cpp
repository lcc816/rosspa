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
#include <cstdio>
#include <cstring>
#include <sys/stat.h> // to create directory

namespace spa
{

struct RegisteredComponent
{
  std::string nodeName;
  uint8_t status; // 0 represents active
};

class LookupService
{
public:
	LookupService();
	/// Finding if the component's xTEDS already exists in the repository.
	bool existXteds(const std::string &xuuid);

  /// change the xTEDS repository path.
  void setXtedsRepoPath(const std::string &path)
  {
    xteds_repo_path = path;
    mkdir(xteds_repo_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  bool matchMsgType(XtedsNode *root, XtedsNode *msgType);

  bool matchMessage(XtedsNode *root, XtedsNode *msg);

  bool matchVariable(XtedsNode *varNode, XtedsNode *var);

  bool matchAttribute(XtedsNode *attrNode, XtedsNode *var);

  bool numericCompare(double num1, double num2, char *operand);

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

  std::unordered_map<std::string, RegisteredComponent> componentTable;

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

bool LookupService::existXteds(const std::string &xuuid)
{
  for (auto &xteds : xtedsRepository.xtedsList) // Search xTEDS repository
  {
    if (xteds->xuuid() == xuuid)
      return true;
  }
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
  std::string id = srv1.response.xuuid;
  if (existXteds(id))
  {
    // Update registered component info
    RegisteredComponent cmpt;
    cmpt.nodeName = msg->nodeName;
    cmpt.status = 0;
    componentTable[id] = cmpt;
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

  feedback.dialogId = goal->dialogId;
  feedback.replyType = REGISTRATION;
  bool success = true;

  Query query(goal->query);
  XtedsNode *root = query.first_node();

  for (auto &xteds : xtedsRepository.xtedsList)
  {
    for (XtedsNode *interface = xteds->first_node("Interface"); interface; interface = interface->next_sibling())
    {
      for (XtedsNode *msgType = interface->first_node(); msgType; msgType = msgType->next_sibling())
      {
        if (!matchMsgType(root, msgType))
          continue;
        XtedsNode *msg = msgType->first_node();
        if (matchMessage(root, msg))
        {
          XtedsAttribute *msgId, *interfaceId;
          if ((msgId = msg->first_attribute("id"))&&(interfaceId = interface->first_attribute("id")))
          {
            feedback.messageId = std::atoi("msgId");
            feedback.interfaceId = std::atoi("interfaceId");
          }
          else
          {
            continue;
          }
          auto cmpt = componentTable.find(xteds->xuuid());
          if (cmpt != componentTable.end()) // 找到组件信息
          {
            if (cmpt->second.status == 0) // 且组件未失效
            {
              feedback.nodeName = cmpt->second.nodeName;
              queryServer.publishFeedback(feedback);
            }
          }
        }
      }
    }
  }

  if (success)
  {
    result.resultMode = 0;
    queryServer.setSucceeded(result);
  }
}

bool LookupService::matchMsgType(XtedsNode *root, XtedsNode *msgType)
{
  if (!(root && msgType)) // 节点为空
    return false;

  XtedsAttribute *attr;
  if (attr = root->first_attribute("msgType"))
    return false;
  
  if (std::strcmp(attr->value(), msgType->name()) != 0)
    return false;
}

bool LookupService::matchMessage(XtedsNode *root, XtedsNode *msg)
{
  if (!(root && msg))
    return false;
  
  // 逐个比较子节点
  XtedsNode *varNode = root->first_node(), *var = msg->first_node();
  for (; varNode && var; varNode = varNode->next_sibling(), var = var->next_sibling())
  {
    if (matchVariable(varNode, var))
      continue;
    else 
      return false;
  }
  return true;
}

bool LookupService::matchVariable(XtedsNode *varNode, XtedsNode *var)
{
  if (!(varNode && var))
    return false;

  XtedsNode *attrNode = var->first_node();
  // 遍历查询中的每个<Attribute>
  for (; attrNode; attrNode = attrNode->next_sibling())
  {
    if (!matchAttribute(attrNode, var))
      return false;
  }

  return true;
}

bool LookupService::matchAttribute(XtedsNode *attrNode, XtedsNode *var)
{
  if (!(attrNode && var))
    return false;
  
  XtedsAttribute *attr;
  char *name;
  char *operand;
  char *value1;
  char *value2;

  // Attribute name
  if (!(attr = attrNode->first_attribute("name")))
    return false;
  name = attr->value();

  // 搜索 var 中有无此属性
  if (!(attr = var->first_attribute(name)))
    return false;
  // 有则访问其值
  value2 = attr->value();

  // Attribute operand
  if (!(attr = attrNode->first_attribute("operand")))
    return false;
  operand = attr->value();

  // Attribute value
  if (!(attr = attrNode->first_attribute("value")))
    return false;
  value1 = attr->value();
  
  // 判断 value 是否是数值
  double num1, num2;
  if (std::sscanf(value1, "%lf", &num1)&&std::sscanf(value2, "%lf", &num2))
  {
    return numericCompare(num1, num2, operand);
  }

  // 是字符串
  if (!std::strcmp(value1, value2))
    return true;

  return false;
}

bool LookupService::numericCompare(double num1, double num2, char *operand)
{
  if (std::strcmp(operand, "eq"))
    return num1 == num2 ? true : false;
  if (std::strcmp(operand, "lt"))
    return num1 < num2 ? true : false;
  if (std::strcmp(operand, "lte"))
    return num1 <= num2 ? true : false;
  if (std::strcmp(operand, "gte"))
    return num1 >= num2 ? true : false;
  if (std::strcmp(operand, "gt"))
    return num1 > num2 ? true : false;
  return false;
}

} // namespace spa

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spa_ls");

  spa::LookupService spa_ls;

  ros::spin();
}
