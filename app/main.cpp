// main.cpp
//
// ICS 46 Spring 2015
// Project #4: Rock and Roll Stops the Traffic
//
// This is the program's main() function, which is the entry point for your
// console user interface.

#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "InputReader.hpp"
#include "RoadMap.hpp"
#include "RoadMapReader.hpp"
#include "RoadMapWriter.hpp"
#include "TripReader.hpp"



int main()
{
    // handle if Digraph function thrown an exception
    try
    {
        // construct every object as needed
        InputReader ir1(std::cin);
        InputReader ir2(std::cin);
        RoadMap rm;
        RoadMapReader rmr;
        RoadMapWriter rmw;
        TripReader tr;
        rm = rmr.readRoadMap(ir1);
        rmw.writeRoadMap(std::cout, rm);
        // store the trip infomation into a vector
        std::vector<Trip> info = tr.readTrips(ir2);
        // run every trip from the vector
        for (int i = 0; i < info.size(); i ++)
        {
            // run as distance if found TripMetric::Distance
            if (info[i].metric == TripMetric::Distance)
            {
                // create an edgeWeightFunc for calling findShortestPaths
                std::function<double(const RoadSegment &)> edgeWeightFuncD = [](const RoadSegment& edgeInfo)
                {
                    return static_cast<double>(edgeInfo.miles);
                };
                // store the total distance
				double totalDistance = 0.0;
				// store the route found from findShortestPaths
				std::vector<int> locationVector;
				// get the map from the function
                std::map<int, int> paths = rm.findShortestPaths(info[i].startVertex, edgeWeightFuncD);
                std::cout << "Shortest distance from " << rm.vertexInfo(info[i].startVertex) << " to " << rm.vertexInfo(info[i].endVertex) << std::endl;
                locationVector.push_back(info[i].endVertex);
                int check = info[i].endVertex;
                while (check != info[i].startVertex)
                {
                    locationVector.push_back(paths[check]);
                    check = paths[check];
                }
                // reverse the vector, since the route is got from the end to start
                std::reverse(locationVector.begin(), locationVector.end());
                std::cout << "Begin at " << rm.vertexInfo(info[i].startVertex) << std::endl;
                // print the infomation
                for (int i = 0; i < locationVector.size()-1; i++)
                {
                    // get the miles from edgeInfo()
					RoadSegment segment = rm.edgeInfo(locationVector[i], locationVector[i+1]);
                    std::cout << "Continue to " << rm.vertexInfo(locationVector[i+1]) << " (" << segment.miles <<" miles)"<< std::endl;
					// store the distance into total distance
					totalDistance += segment.miles;
                }
                std::cout << "Total Distance: " << totalDistance << " miles" << std::endl << std::endl;
                paths.clear();
            }
            else
            {
                // create an edgeWeightFunc for findShortestPaths getting the weight of milesPerHour
                std::function<double(const RoadSegment &)> edgeWeightFuncT = [](const RoadSegment& edgeInfo)
                {
                    return static_cast<double>(edgeInfo.milesPerHour);
                };
                // declare the time format variable as needed
				double totalTimeInSecond = 0.0;
				double currentTimeInSecond = 0.0;
				int hour = 0;
				int minute = 0;
				int tempMin = 0;
				double second = 0.0;
				double currentMiles = 0.0;
				double currentMilesPerHour = 0.0;
				// store the route as Distance
				std::vector<int> locationVector;
                std::map<int, int> paths = rm.findShortestPaths(info[i].startVertex, edgeWeightFuncT);
                std::cout << "Shortest driving time from " << rm.vertexInfo(info[i].startVertex) << " to " << rm.vertexInfo(info[i].endVertex) << std::endl;
                locationVector.push_back(info[i].endVertex);
                int check = info[i].endVertex;
                while (check != info[i].startVertex)
                {
                    locationVector.push_back(paths[check]);
                    check = paths[check];
                }
                std::reverse(locationVector.begin(), locationVector.end());
                std::cout << "Begin at " << rm.vertexInfo(info[i].startVertex) << std::endl;
                for (int i = 0; i < locationVector.size()-1; i++)
                {
					RoadSegment segment = rm.edgeInfo(locationVector[i], locationVector[i+1]);
					// translate the time in second to be formatted
					currentMiles = segment.miles;
					currentMilesPerHour = segment.milesPerHour;
					currentTimeInSecond = currentMiles / currentMilesPerHour * 60 * 60;
					totalTimeInSecond += currentTimeInSecond;
					tempMin = static_cast<int>(currentTimeInSecond) / 60;
					second = currentTimeInSecond - (tempMin*60);
					minute = (static_cast<int>(currentTimeInSecond) / 60) % 60;
					hour = (static_cast<int>(currentTimeInSecond) / 60) / 60;
                    std::cout << "Continue to " << rm.vertexInfo(locationVector[i+1])
                    	<< " (" << currentMiles << " miles @ "
						<< currentMilesPerHour << "mph = ";
					// if hour or minute is 0, don't show
					if (hour != 0)
					{
						std::cout << hour << " hrs ";
					}
					if (minute != 0)
					{
						std::cout << minute << " mins ";
					}
					std::cout << second << " secs)" << std::endl;
				}
				// get the formatted total time
				tempMin = static_cast<int>(totalTimeInSecond) / 60;
				second = totalTimeInSecond - (tempMin*60);
				minute = (static_cast<int>(totalTimeInSecond) / 60) % 60;
				hour = (static_cast<int>(totalTimeInSecond) / 60) / 60;
				std::cout << "Total time: ";
				if (hour != 0)
				{
					std::cout << hour << " hrs ";
				}
				if (minute != 0)
				{
					std::cout << minute << " mins ";
				}
				std::cout << second << " secs" << std::endl << std::endl;
            }
        }
    }
    catch (DigraphException &digraph)
    {
        // print the exception message
        std::cout << digraph.reason() << std::endl;
    }

    return 0;
}

