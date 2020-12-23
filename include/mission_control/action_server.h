#ifndef ACTION_SERVER_H
#define ACTION_SERVER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <condition_variable>

template <class Request, class Response>
class ActionServer
{
private:
  std::mutex mtx_;
  std::condition_variable cv_;
  Response resp_;
  ros::ServiceServer execute_srv_;
  ros::ServiceServer preempt_srv_;
  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;
protected:
  void setResponse(Response resp);
  Response waitForResponse();
  virtual bool executeCb(Request &req, Response &resp);
  virtual bool preemptCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
public:
  ActionServer();
  ~ActionServer();
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
void ActionServer<Request, Response>::setResponse(Response resp)
{
  std::unique_lock<std::mutex> lck(mtx_);
  resp_ = resp;
  cv_.notify_all();
}

template <class Request, class Response>
Response ActionServer<Request, Response>::waitForResponse()
{
  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck);
  Response resp = resp_;
  return resp;
}

template <class Request, class Response>
ActionServer<Request, Response>::~ActionServer()
{
  cv_.notify_all();
  spinner_.stop();
};

template <class Request, class Response>
bool ActionServer<Request, Response>::executeCb(Request &req, Response &resp){};

template <class Request, class Response>
bool ActionServer<Request, Response>::preemptCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp){};

#endif