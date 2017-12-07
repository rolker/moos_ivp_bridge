// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "ros/package.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/TwistStamped.h"
#include "mission_plan/NavEulerStamped.h"
#include "MOOS/libMOOS/Comms/MOOSAsyncCommClient.h"

#include "moos_ivp_bridge/gz4d_geo.h"

#include <fstream>
#include <regex>
#include <iomanip>

ros::Publisher pub;
ros::Publisher desired_heading_pub;
ros::Publisher desired_speed_pub;
ros::Publisher appcast_pub;
ros::Publisher origin_pub;

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
    c->Register("DESIRED_SPEED",0.0);
    c->Register("APPCAST",0.0);
    comms.Notify("MOOS_MANUAL_OVERIDE","false");
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
        if(m.IsName("DESIRED_HEADING"))
        {
            mission_plan::NavEulerStamped nes;
            nes.orientation.heading = m.GetDouble();
            nes.header.stamp.fromSec(t);
            desired_heading_pub.publish(nes);
        }
        if(m.IsName("DESIRED_SPEED"))
        {
            geometry_msgs::TwistStamped ts;
            ts.twist.linear.x = m.GetDouble();
            ts.header.stamp.fromSec(t);
            desired_speed_pub.publish(ts);
        }
        if(m.IsName("APPCAST"))
        {
            std_msgs::String s;
            s.data = m.GetAsString();
            appcast_pub.publish(s);
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
    std::regex latr("LAT_ORIGIN");
    std::regex longr("LONG_ORIGIN");
    
    std::stringstream latss;
    latss << std::setprecision(15) << LatOrigin;
    std::stringstream longss;
    longss << std::setprecision(15) << LongOrigin;
    
    std::string outcontent = std::regex_replace(incontent.str(),br,bhvFile);
    outcontent = std::regex_replace(outcontent,latr,latss.str());
    outcontent = std::regex_replace(outcontent,longr,longss.str());

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

void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
{
    //std::cerr << "positionCallback: " <<  initializedMOOS << std::endl;
    if(!initializedMOOS)
    {
        LatOrigin = inmsg->position.latitude;
        LongOrigin = inmsg->position.longitude;
        startMOOS();
    }
    double t = inmsg->header.stamp.toSec();
    comms.Notify("NAV_LAT",inmsg->position.latitude,t);
    comms.Notify("NAV_LONG",inmsg->position.longitude,t);
    
    gz4d::Point<double> position = geoReference.toLocal(gz4d::geo::Point<double,gz4d::geo::WGS84::ECEF>(gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon>(inmsg->position.latitude,inmsg->position.longitude,0.0)));
    
    comms.Notify("NAV_X",position[0],t);
    comms.Notify("NAV_Y",position[1],t);
}

void headingCallback(const mission_plan::NavEulerStamped::ConstPtr& inmsg)
{
    double t = inmsg->header.stamp.toSec();
    comms.Notify("NAV_HEADING",inmsg->orientation.heading,t);
}

void sogCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
{
    double t = inmsg->header.stamp.toSec();
    comms.Notify("NAV_SPEED",inmsg->twist.linear.x,t);
}

void waypointUpdateCallback(const std_msgs::String::ConstPtr& inmsg)
{
    comms.Notify("WPT_UPDATE",inmsg->data);
}

void activeCallback(const std_msgs::Bool::ConstPtr& inmsg)
{
    if(inmsg->data)
        comms.Notify("ACTIVE","true");
    else
        comms.Notify("ACTIVE","false");
}

void appcastRequestCallback(const ros::WallTimerEvent& event)
{
    std::stringstream req;
    
    req << "node=moos_ivp_ros,app=pHelmIvP,duration=3.0,key=moos_ivp_bridge,thresh=any";
    comms.Notify("APPCAST_REQ",req.str());
}

void originCallback(const ros::WallTimerEvent& event)
{
    if(initializedMOOS)
    {
        geographic_msgs::GeoPoint gp;
        gp.latitude = LatOrigin;
        gp.longitude = LongOrigin;
        origin_pub.publish(gp);
    }
}

int main(int argc, char **argv)
{
    last_lat_time = 0.0;
    last_lon_time = 0.0;
    desired_heading = 0.0;
    
    ros::init(argc, argv, "moos_ivp_bridge_node");
    ros::NodeHandle n;
    
    pub = n.advertise<geographic_msgs::GeoPoint>("/moos/nav_point",10);
    desired_heading_pub = n.advertise<mission_plan::NavEulerStamped>("/moos/desired_heading",1);
    desired_speed_pub = n.advertise<geometry_msgs::TwistStamped>("/moos/desired_speed",1);
    appcast_pub = n.advertise<std_msgs::String>("/moos/appcast",1);
    origin_pub = n.advertise<geographic_msgs::GeoPoint>("/moos/origin",1);
    
    ros::Subscriber psub = n.subscribe("/position",10,positionCallback);
    ros::Subscriber hsub = n.subscribe("/heading",10,headingCallback);
    ros::Subscriber sogsub = n.subscribe("/sog",10,sogCallback);
    ros::Subscriber wptUpdatesub = n.subscribe("/moos/wpt_updates",10,waypointUpdateCallback);
    ros::Subscriber activesub = n.subscribe("/active",10,activeCallback);
    
    ros::WallTimer appcastRequestTimer = n.createWallTimer(ros::WallDuration(1.0),appcastRequestCallback);
    
    ros::WallTimer originTimer = n.createWallTimer(ros::WallDuration(1.0),originCallback);

    comms.SetOnMailCallBack(OnMail,&comms);
    comms.SetOnConnectCallBack(OnConnect,&comms);
    
    comms.Run("localhost",9000,"moos_ivp_bridge");
    
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
    
    return 0;
    
}

