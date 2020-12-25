#ifndef ACTION_SERVER_H
#define ACTION_SERVER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Empty.h>
#include <mutex>
#include <condition_variable>

namespace actionlite
{

using std_srvs::Empty;
using std_srvs::EmptyRequest;
using std_srvs::EmptyResponse;

void sigintHandler(int sig)
{
  Empty srv;
  ros::service::call("~preempt", srv);
  ros::shutdown();
}

template <class Request, class Response>
class ActionServer
{
private:
  Request req_;
  Response resp_;
  std::mutex resp_mtx_, req_mtx_;
  std::condition_variable cv_;
  ros::ServiceServer execute_srv_;
  ros::ServiceServer preempt_srv_;
  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;
  void waitForResponse(Response& resp);
  bool executeCb(Request& req, Response& resp);
  bool preemptCb(EmptyRequest&, EmptyResponse&);
protected:
  const Request& getRequest() const;
  void setResponse(const Response& resp);
  virtual bool executeSetup(const Request& req);
  virtual bool preemptSetup(Response& resp);
  virtual void cleanUp();
public:
  ActionServer();
  virtual ~ActionServer();
};

template <class Request, class Response>
ActionServer<Request, Response>::ActionServer(): spinner_(2, &callback_queue_)
{
  ros::NodeHandle nh("~");
  nh.setCallbackQueue(&callback_queue_);
  execute_srv_ = nh.advertiseService("execute", &ActionServer::executeCb, this);
  preempt_srv_ = nh.advertiseService("preempt", &ActionServer::preemptCb, this);
  spinner_.start();
}

template <class Request, class Response>
const Request& ActionServer<Request, Response>::getRequest() const
{
  return req_;
}

template <class Request, class Response>
void ActionServer<Request, Response>::setResponse(const Response& resp)
{
  std::unique_lock<std::mutex> lck(resp_mtx_);
  resp_ = resp;
  cv_.notify_all();
}

template <class Request, class Response>
void ActionServer<Request, Response>::waitForResponse(Response& resp)
{
  std::unique_lock<std::mutex> lck(resp_mtx_);
  cv_.wait(lck);
  resp = resp_;
}

template <class Request, class Response>
ActionServer<Request, Response>::~ActionServer()
{
  spinner_.stop();
  cv_.notify_all();
}

template <class Request, class Response>
bool ActionServer<Request, Response>::executeCb(Request &req, Response &resp)
{
  std::unique_lock<std::mutex> lck(req_mtx_, std::defer_lock);
  if (!lck.try_lock())
  {
    ROS_WARN("ACTION SERVER DOES NOT SUPPORT CONCURRENT CALL! ONLY ONE REQUEST HANDLED AT A TIME!");
    return false;
  }
  req_ = req;
  if (!executeSetup(req_))
  {
    ROS_WARN("EXECUTE SETUP FAIL!");
    return false;
  }
  {
    std::unique_lock<std::mutex> lck(resp_mtx_);
    cv_.wait(lck);
    resp = resp_;
  }
  cleanUp();
  return true;
}

template <class Request, class Response>
bool ActionServer<Request, Response>::preemptCb(EmptyRequest&, EmptyResponse&)
{
  std::unique_lock<std::mutex> lck(resp_mtx_);
  bool res = preemptSetup(resp_);
  cv_.notify_all();
  return res;
}

template <class Request, class Response>
bool ActionServer<Request, Response>::executeSetup(const Request& req){};

template <class Request, class Response>
bool ActionServer<Request, Response>::preemptSetup(Response& resp){};

template <class Request, class Response>
void ActionServer<Request, Response>::cleanUp(){};

}

#endif