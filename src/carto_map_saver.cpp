
#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;

/**
 * @brief Map generation node.
 */
class MapGenerator
{

public:
MapGenerator(const std::string& mapname) : mapname_(mapname), saved_map_(false)
{
  ros::NodeHandle n;
  ROS_INFO("Waiting for the map ...");
  map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
	ROS_INFO("Received a %d X %d map @ %.3f m/pix",
				map->info.width,
				map->info.height,
				map->info.resolution);

	std::string mapdatafile = mapname_ + ".pgm";
	ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
	FILE* out = fopen(mapdatafile.c_str(), "w");
	if (!out)
	{
		ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
		return;
	}

	fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
		map->info.resolution, map->info.width, map->info.height);
	for(unsigned int y = 0; y < map->info.height; y++) 
	{
		for(unsigned int x = 0; x < map->info.width; x++) 
		{
			unsigned int i = x + (map->info.height - y - 1) * map->info.width;
			if (map->data[i] >= 0 && map->data[i] <= 100) 
			{
				unsigned int value = round((float)(100.0 - map->data[i]) * 2.55);
				if (value == 128) 
				{
					fputc(129, out);
				}
				else 
				{
					fputc(value, out);
				}
			}
			else 
			{
				fputc(128, out);
			}
		}
	}
	fclose(out);


	std::string mapmetadatafile = mapname_ + ".yaml";
	ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
	FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

	std::ostringstream oss;
	oss << "%YAML:1.0\n\n"
		<< "# Image associated to this configuration file\n"
		<< "image: " << mapdatafile.c_str() << "\n\n"
		<< "# Meters per pixel resolution\n"
		<< "resolution: " << map->info.resolution << "\n\n"
		<< "# The origin of the map [m, m, rad]\n"
		<< "origin: [" << map->info.origin.position.x << ", " << map->info.origin.position.y << ", " << map->info.origin.position.z << "]\n"
		<< "negate: 0\n"
		<< "occupied_thresh: 0.65\n"
		<< "free_thresh: 0.196\n\n"
		<< "# Pixel associated with LiDAR origin\n"
		<< "origin_px_x: \n"
		<< "origin_px_y: \n";
	fprintf(yaml, "%s\n", oss.str().c_str());
	fclose(yaml);

	ROS_INFO("Saved image and created configuration file successfully");
	saved_map_ = true;
}

std::string mapname_;
ros::Subscriber map_sub_;
bool saved_map_;

};

#define USAGE "Usage: \n" \
              "  map_saver -h\n"\
              "  map_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_saver");
	std::string mapname = "map";

	for(int i = 1; i < argc; i++)
	{
		if(!strcmp(argv[i], "-h"))
		{
			puts(USAGE);
			return 0;
		}
		else if(!strcmp(argv[i], "-f"))
		{
			if(++i < argc)
				mapname = argv[i];
			else
			{
				puts(USAGE);
				return 1;
			}
		}
		else
		{
			puts(USAGE);
			return 1;
		}
	}

	MapGenerator mg(mapname);

	while(!mg.saved_map_ && ros::ok())
		ros::spinOnce();

	return 0;
}
