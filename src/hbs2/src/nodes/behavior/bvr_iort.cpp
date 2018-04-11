// System
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>

// Installed
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Infos.hpp>

// ROS
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "hbs2/iot.h"


#define MAX_FILE_LENGTH 20000


char *m_pBuffer = NULL;
size_t m_Size = 0;

/*  Class: Response
    desc: A class to store contents of an HTTP respone
    attrs:
        code: HTTP response code
        size: Size of the response body in bytes
        content: HTTP response body content
*/
class Response {
    public:
        long code;
        int size;
        std::string content;
};


void* Realloc(void* ptr, size_t size)
{
  if(ptr)
    return realloc(ptr, size);
  else
    return malloc(size);
};


size_t WriteMemoryCallback(char* ptr, size_t size, size_t nmemb)
{
  // Calculate the real size of the incoming buffer
  size_t realsize = size * nmemb;
  
  // (Re)Allocate memory for the buffer
  m_pBuffer = (char*) Realloc(m_pBuffer, m_Size + realsize);
  
  // Test if Buffer is initialized correctly & copy memory
  if (m_pBuffer == NULL) {
    realsize = 0;
  }
  
  memcpy(&(m_pBuffer[m_Size]), ptr, realsize);
  m_Size = realsize;
  
  // return the real size of the buffer...
  return realsize;
};

/*  Function: rest_req (REST Request)
    desc:   A funciton that performs an HTTP request with libcurl + curlpp
    inputs:
        &resp: HTTP response object
        uri: Uniform Resource Locator for the API call
    outputs:
        &resp: HTTP response object
*/
void rest_req(Response &resp, std::string uri) {
    curlpp::Cleanup cleaner;
    curlpp::Easy request;

    request.curlpp::Easy::setOpt(curlpp::options::Url(uri));
    request.curlpp::Easy::setOpt(curlpp::options::Port(5000));
    curlpp::types::WriteFunctionFunctor functor(WriteMemoryCallback);
    curlpp::options::WriteFunction *writefunc = new curlpp::options::WriteFunction(functor);

    request.setOpt(writefunc);
    request.perform();

    curlpp::infos::ResponseCode::get(request, resp.code);
    resp.size = m_Size;
    if (m_Size == 1) { resp.content = m_pBuffer[0]; }
    else { resp.content = ++m_pBuffer; m_pBuffer--; }
}

/*  Function: begin
    desc:   A function that is called from main in a loop at 2Hz rate that will perform and REST request 
            and controls the flow of the Node.
    inputs:
        &n: ROS NodeHandle for this Node
    outputs:
        &n: ROS NodeHandle for this Node
*/
void begin(ros::NodeHandle &n) {
    m_pBuffer = (char*) malloc(MAX_FILE_LENGTH * sizeof(char));
    Response resp;
    std::string get_cmd_uri = "http://54.187.205.93/api/getcommand";
    std::string get_tts_uri = "http://54.187.205.93/api/gettts";
    std::string set_resp_uri = "http://54.187.205.93/api/setdata?data=";
    m_pBuffer = NULL;
    m_Size = 0;
    resp.content = "";
    resp.size = 0;
    resp.code = 0;

    ros::ServiceClient iot_client = n.serviceClient<hbs2::iot>("iot_srv");
    hbs2::iot srv_iot;

    rest_req(resp, get_cmd_uri);

    switch (m_pBuffer[0]) {
        case 1: {
        ROS_INFO("A request to read the sonar sensor has been made.");
            /*  send request to iot_srv
                req.request.command = 1

                check for success and return data back to api client
                if res.response.success == true
                    create rest_req(resp, set_resp_uri)
            */
            srv_iot.request.command = 1;
            iot_client.call(srv_iot);
            if (srv_iot.response.success == true) {
                ROS_INFO("Sending sonar data to remote.");
                rest_req(resp, set_resp_uri + std::to_string(srv_iot.response.data));
            }
            else { ROS_ERROR("Request to iot_srv for reading sonar failed."); }
            break;
        }
        case 2: {
            ROS_INFO("A request for tts has been made.");
            rest_req(resp, get_tts_uri);
                /*  send request to iot_srv
                    req.request.command = 2
                    req.request.text = resp.content

                    check for success
                    res.resonse.success == true
                */
                srv_iot.request.command = 2;
                srv_iot.request.text = resp.content;
                iot_client.call(srv_iot);
                if (srv_iot.response.success ==  true) { ROS_INFO("Request for TTS completed."); }
                else { ROS_ERROR("Request to iot_srv for TTS has failed."); }
                break;
        }
        case 3: {
            ROS_INFO("A request to shake the robot's head has been made.");
                /*  send request to iot_srv
                    req.request.command = 3
                
                    check for success
                    if res.response.success == true
                */
                srv_iot.request.command = 3;
                iot_client.call(srv_iot);
                if (srv_iot.response.success ==  true) { ROS_INFO("Request to shake head completed."); }
                else { ROS_ERROR("Request to iot_srv to shake head has failed."); }
                break;
        }
        default:
            ROS_INFO("[ROS IOT] No action taken.");
            break;
    }
}

/*  Function: main
    desc: Entry point for the Node
    inputs:
        argc: count of command line arguments
        argv: array of command line arguments
    outputs:
        int: always 0 if exits gracefully
*/
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "iort");
    ros::NodeHandle n;
    ros::Rate loop_rate(0.5);
    
    while(ros::ok()) {
        begin(n);
        ros::spinOnce();
	    loop_rate.sleep();
    }
    return 0;
}
