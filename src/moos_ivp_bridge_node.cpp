// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "ros/package.h"
#include "geographic_msgs/GeoPoint.h"
#include "std_msgs/String.h"
#include "asv_msgs/HeadingHold.h"
#include "asv_msgs/BasicPositionStamped.h"
#include "asv_msgs/HeadingStamped.h"
#include "MOOS/libMOOS/Comms/MOOSAsyncCommClient.h"

#include "moos_ivp_bridge/gz4d_geo.h"

#include <fstream>
#include <regex>

ros::Publisher pub;
ros::Publisher asv_hh_pub;

MOOS::MOOSAsyncCommClient comms;

double lat, lon;
double last_lat_time;
double last_lon_time;
double desired_heading;

// HACK
// CCOM Pier
double LatOrigin  = 43.071959194444446;
double LongOrigin = -70.711610833333339;

gz4d::geo::LocalENU<> geoReference;

bool initializedMOOS = false;

bool OnConnect(void * param)
{
    CMOOSCommClient* c = reinterpret_cast<CMOOSCommClient*>(param);
    c->Register("DESIRED_HEADING",0.0);
    c->Register("DESIRED_THRUST",0.0);
    //c->Register("NAV_LAT",0.0);
    //c->Register("NAV_LONG",0.0);
    return true;
}

bool OnMail(void *param)
{
    CMOOSCommClient* c = reinterpret_cast<CMOOSCommClient*>(param);
    MOOSMSG_LIST ml;
    c->Fetch(ml);
    for(auto m: ml)
    {
        
        auto t = m.GetTime();
        if(m.IsName("NAV_LAT") || m.IsName("NAV_LONG"))
        {
            if(m.IsName("NAV_LAT"))
            {
                lat = m.GetDouble();
                last_lat_time = t;
            }
            if(m.IsName("NAV_LONG"))
            {
                lon = m.GetDouble();
                last_lon_time = t;
            }
            if (last_lat_time == last_lon_time)
            {
                geographic_msgs::GeoPoint gp;
                gp.altitude = 0.0;
                gp.latitude = lat;
                gp.longitude = lon;
                pub.publish(gp);
            }
        }
        else
        {
            m.Trace();
            if(m.IsName("DESIRED_HEADING"))
            {
                desired_heading = m.GetDouble();
            }
            if(m.IsName("DESIRED_THRUST"))
            {
                asv_msgs::HeadingHold asvMsg;
                asvMsg.heading.heading =  M_PI*desired_heading/180.0;
                asvMsg.thrust.type = asv_msgs::Thrust::THRUST_THROTTLE;
                asvMsg.thrust.value = m.GetDouble()/100.0;
                asvMsg.header.stamp.fromSec(t);
                asv_hh_pub.publish(asvMsg);
            }
        }
    }
    
    return true;
}


void startMOOS()
{
    initializedMOOS = true;
    
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> gr(LatOrigin,LongOrigin,0.0);
    geoReference = gz4d::geo::LocalENU<>(gr);
    
    std::string missionFileTemplate = ros::package::getPath("moos_ivp_bridge")+"/missions/ros.moos.template";
    std::string bhvFile = ros::package::getPath("moos_ivp_bridge")+"/missions/ros.bhv";

    std::ifstream infile(missionFileTemplate);
    std::stringstream incontent;
    incontent << infile.rdbuf();
    
    std::regex br("BEHAVIORS");
    
    std::string outcontent = std::regex_replace(incontent.str(),br,bhvFile);

    std::string missionFile = "ros.moos";
    
    std::ofstream outfile(missionFile);
    
    outfile << outcontent;
    
    char* argv[3];
    argv[2] = nullptr;
    argv[1] = const_cast<char *>(missionFile.c_str());
    
    std::string pa = "pAntler";
    argv[0] = const_cast<char *>(pa.c_str());
    
    int pid = fork();
    if(pid == 0)
    {
        execvp("pAntler",argv);
    }
    
}

void positionCallback(const asv_msgs::BasicPositionStamped::ConstPtr& inmsg)
{
    //std::cerr << "positionCallback: " <<  initializedMOOS << std::endl;
    if(!initializedMOOS)
        startMOOS();
    double t = inmsg->header.stamp.toSec();
    comms.Notify("NAV_LAT",inmsg->basic_position.position.latitude,t);
    comms.Notify("NAV_LONG",inmsg->basic_position.position.longitude,t);
    comms.Notify("NAV_SPEED",inmsg->basic_position.sog,t);
    
    gz4d::Point<double> position = geoReference.toLocal(gz4d::geo::Point<double,gz4d::geo::WGS84::ECEF>(gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon>(inmsg->basic_position.position.latitude,inmsg->basic_position.position.longitude,0.0)));
    
    comms.Notify("NAV_X",position[0],t);
    comms.Notify("NAV_Y",position[1],t);
}

void headingCallback(const asv_msgs::HeadingStamped::ConstPtr& inmsg)
{
    double t = inmsg->header.stamp.toSec();
    comms.Notify("NAV_HEADING",gz4d::Degrees(inmsg->heading.heading),t);
}

void waypointUpdateCallback(const std_msgs::String::ConstPtr& inmsg)
{
    comms.Notify("WPT_UPDATE",inmsg->data);
}

int main(int argc, char **argv)
{
    last_lat_time = 0.0;
    last_lon_time = 0.0;
    desired_heading = 0.0;
    
    ros::init(argc, argv, "moos_ivp_bridge_node");
    ros::NodeHandle n;
    
    pub = n.advertise<geographic_msgs::GeoPoint>("/moos/nav_point",10);
    asv_hh_pub = n.advertise<asv_msgs::HeadingHold>("/control/drive/heading_hold",1);
    
    ros::Subscriber psub = n.subscribe("/sensor/vehicle/position",10,positionCallback);
    ros::Subscriber hsub = n.subscribe("/sensor/vehicle/heading",10,headingCallback);
    ros::Subscriber wptUpdatesub = n.subscribe("/moos/wpt_updates",10,waypointUpdateCallback);

    comms.SetOnMailCallBack(OnMail,&comms);
    comms.SetOnConnectCallBack(OnConnect,&comms);
    
    comms.Run("localhost",9000,"moos_ivp_bridge");
   
    
    
    ros::spin();
    
    return 0;
    
}

