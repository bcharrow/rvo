#define USAGE "\nUSAGE: map_server <map.yaml>\n"        \
  "  map.yaml: map description file\n"                  \
  "DEPRECATED USAGE: map_server <map> <resolution>\n"   \
  "  map: image file to load\n"                         \
  "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"

#include "player_map/rosmap.hpp"
#include "player_map/GetMap.h"

class MapServer {
public:
  /** Trivial constructor */
  MapServer(const std::string& fname, double res)
  {
    std::string mapfname = "";   
    double origin[3];
    int negate;
    double occ_th, free_th;
    std::string frame_id;
    ros::NodeHandle private_nh("~");
    private_nh.param("frame_id", frame_id, std::string("map"));
    deprecated = (res != 0);

    if (!deprecated) {
      //mapfname = fname + ".pgm";
      //std::ifstream fin((fname + ".yaml").c_str());
      std::ifstream fin(fname.c_str());
      if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s.", fname.c_str());
        exit(-1);
      }
      YAML::Parser parser(fin);   
      YAML::Node doc;
      parser.GetNextDocument(doc);
      try { 
        doc["resolution"] >> res; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["negate"] >> negate; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["occupied_thresh"] >> occ_th; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["free_thresh"] >> free_th; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["origin"][0] >> origin[0]; 
        doc["origin"][1] >> origin[1]; 
        doc["origin"][2] >> origin[2]; 
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        exit(-1);
      }
      try { 
        doc["image"] >> mapfname; 
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
          {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
          }
        if(mapfname[0] != '/')
          {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
          }
      } catch (YAML::InvalidScalar) { 
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
      }
    } else {
      private_nh.param("negate", negate, 0);
      private_nh.param("occupied_thresh", occ_th, 0.65);
      private_nh.param("free_thresh", free_th, 0.196);
      mapfname = fname;
      origin[0] = origin[1] = origin[2] = 0.0;
    }

    ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());

    nav_msgs::GetMap::Response nav_map;
      
    map_server::loadMapFromFile(&nav_map, mapfname.c_str(),res,negate,occ_th,free_th, origin);

    map_resp_.map = nav_map.map;
      
    map_resp_.map.info.map_load_time = ros::Time::now();
    map_resp_.map.header.frame_id = frame_id;
    map_resp_.map.header.stamp = ros::Time::now();
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
             map_resp_.map.info.width,
             map_resp_.map.info.height,
             map_resp_.map.info.resolution);
    meta_data_message_ = map_resp_.map.info;

    // Make player map and update distances in message
    double max_occ_dist;
    private_nh.param("max_occ_dist", max_occ_dist, 0.2);
    ROS_INFO("Building c-space w/ max_occ_dist %.3f...", max_occ_dist);
    pmap_ = map_alloc();
    rf::convertMap(map_resp_.map, pmap_);
    map_update_cspace(pmap_, max_occ_dist);
    map_resp_.max_occ_dist = max_occ_dist;
    map_resp_.occ_dist.resize(pmap_->size_y * pmap_->size_x);
    for (int j = 0; j < pmap_->size_y; j++) {
        for (int i = 0; i < pmap_->size_x; i++) {
          int ind = MAP_INDEX(pmap_, i, j);
          map_resp_.occ_dist[ind] = (pmap_->cells + ind)->occ_dist;
        }
    }
    ROS_INFO("done.");
    
    service = n.advertiseService("static_map", &MapServer::mapCallback, this);
    //pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1,

    // Latched publisher for metadata
    metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    metadata_pub.publish( meta_data_message_ );
      
    // Latched publisher for data
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_pub.publish( map_resp_.map );
  }

  ~MapServer() {
    map_free(pmap_);
  }

private:
  ros::NodeHandle n;
  ros::Publisher map_pub;
  ros::Publisher metadata_pub;
  ros::ServiceServer service;
  bool deprecated;
  map_t* pmap_;
  

  /** Callback invoked when someone requests our service */
  bool mapCallback(player_map::GetMap::Request  &req,
                   player_map::GetMap::Response &res )
  {
    // request is empty; we ignore it

    // = operator is overloaded to make deep copy (tricky!)
    res = map_resp_;
    ROS_INFO("Sending map");

    return true;
  }

  /** The map data is cached here, to be sent out to service callers
   */
  nav_msgs::MapMetaData meta_data_message_;
  player_map::GetMap::Response map_resp_;

  /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
    pub.publish( meta_data_message_ );
    }
  */

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  if(argc != 3 && argc != 2)
    {
      ROS_ERROR("%s", USAGE);
      exit(-1);
    }
  if (argc != 2) {
    ROS_WARN("Using deprecated map server interface. Please switch to new interface.");
  }
  std::string fname(argv[1]);
  double res = (argc == 2) ? 0.0 : atof(argv[2]);

  try
    {
      MapServer ms(fname, res);
      ros::spin();
    }
  catch(std::runtime_error& e)
    {
      ROS_ERROR("map_server exception: %s", e.what());
      return -1;
    }

  return 0;
}

