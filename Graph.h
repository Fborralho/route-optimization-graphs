/**
* @file Graph.h
 * @brief Defines the graph structure and algorithms for route planning
 */

#ifndef GRAPH_H
#define GRAPH_H

#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_set>
#include <limits>
#include <functional>

#define INF std::numeric_limits<int>::max()

/**
 * @struct Suggestion
 * @brief Stores alternative routes suggestions when constraints are exceeded
 */
struct Suggestion{
    std::vector<int> drivePath;
    std::vector<int> walkPath;
    int parkingNode;
    int totalTime;
    int walkingTime;
    int exceedWalkingBy;
};

struct pair_hash {
    size_t operator()(const std::pair<int, int>& p) const;
};

class Location;

/**
 * @class Road
 * @brief Represents a connection between two locations
 * @details Stores driving and walking times between locations
 */
class Road {
public:
    /**
    * @brief Constructor to initialize a Road object.
    * @param origin Pointer to the origin Location.
    * @param destination Pointer to the destination Location.
    * @param drivingTime Time required to drive.
    * @param walkingTime Time required to walk.
    */
    Road(Location* origin, Location* destination, int drivingTime, int walkingTime);

    /**
     * @brief Gets the origin Location.
     * @return Pointer to the origin Location.
    */
    Location* getOrigin() const;

    /**
     * @brief Gets the destination Location.
     * @return Pointer to the destination Location.
    */
    Location* getDestination() const;

    /**
     * @brief Gets the time to drive.
     * @return Driving time.
    */
    int getDrivingTime() const;

    /**
     * @brief Gets the time to walk.
     * @return Walking time.
    */
    int getWalkingTime() const;

    /**
     * @brief Sets driving time to a new value.
     * @param drivingTime New driving time.
    */
    void setDrivingTime(int drivingTime);

private:
    Location* origin;       ///< Pointer to the origin Location
    Location* destination;  ///< Pointer to the destination Location
    int drivingTime;        ///< Driving time cost
    int walkingTime;        ///< Walking time cost
};

/**
 * @class Location
 * @brief Represents a location in the route planning graph
 * @details Contains information about the location and its connections
 */
class Location {
public:
    Location() = default;

    Location(int id, std::string code, bool hasParking);

    /**
     * @brief Get the location ID.
     * @return ID of the location.
     */
    int getId() const;

    /**
     * @brief Get the alphanumeric location code.
     * @return Location code.
     */
    std::string getCode() const;

    /**
     * @brief Check if the location has parking.
     * @return True if it has parking, false otherwise.
     */
    bool HasParking() const;

    /**
     * @brief Get the adjacency list (roads).
     * @return Vector of Road pointers.
     */
    const std::vector<Road*>& getAdj() const;

    /**
     * @brief Get the parent road.
     * @return Pointer to the parent Road.
     */
    Road* getParent() const;

    /**
     * @brief Get the distance value.
     * @return Distance.
     */
    double getDistance() const;

    /**
     * @brief Set the parent road.
     * @param parent Pointer to the new parent Road.
     */
    void setParent(Road* parent);

    /**
     * @brief Set the distance value.
     * @param distance New distance.
     */
    void setDistance(double distance);

    /**
     * @brief Adds a road (neighbor) to this location.
     * @param road Pointer to the Road to add.
     */
    void addRoad(Road* road);
private:
    int id{};                           ///< Unique numeric ID
    std::string code;                 ///< Alphanumeric location code
    bool hasParking{};                  ///< Parking availability
    std::vector<Road*> adj;           ///< Adjacency list of roads (neighbors)
    Road* parent = nullptr;           ///< Pointer to parent road (for pathfinding)
    double distance = 0;              ///< Distance for pathfinding algorithms
};


/**
 * @class Graph
 * @brief Main graph structure containing all locations and roads
 * @details Implements route planning algorithms including Dijkstra's
 */
class Graph {
public:

    ~Graph();
    /**
     * @brief Get the list of all locations.
     * @return Vector of Location pointers.
     */
    const std::vector<Location*>& getLocations() const;

    /**
     * @brief Adds a new location to the graph.
     */
    void addLocation(const int &id, const std::string &code,const bool &hasParking);

    /**
     * @brief Finds a location by its alphanumeric code.
     */
    Location* findLocation(const std::string& code) const;

    /**
     * @brief Finds a location by its numeric ID.
     */
    Location* findLocation(const int &id) const;

    /**
     * @brief Adds a bidirectional road between two locations.
     * @param from The start of the road.
     * @param to The destination of the road.
     * @param drivingTime Time if driving
     * @param walkingTime Time if walking
     */
    void addRoad(Location* from, Location* to, int drivingTime, int walkingTime);

    /**
     * @brief Finds an alternative independent route by excluding intermediate nodes from the best route.
     * @param sourceId The starting location code.
     * @param destinationId The destination location code.
     * @param isDriving Determines if its walking or driving
     * @param blockedNodes The locations either from the BestPath or to avoid.
     * @param blockedSegments the roads to avoid
     * @return A vector of location codes representing the alternative path.
     * @complexity O((N + M) log N) where N is the number of locations and M is the number of Roads.
     */
    std::pair<std::vector<int>, int> dijkstra(
        int sourceId,
        int destinationId,
        bool isDriving,
        const std::unordered_set<int>& blockedNodes,
        std::unordered_set<std::pair<int, int>, pair_hash>& blockedSegments);

    /**
     * @brief Finds the best environmentally-friendly route combining driving and walking.
     * @param sourceId The starting location ID.
     * @param destId The destination location ID.
     * @param maxWalkingTime Maximum walking time allowed.
     * @param avoidNodes Nodes to avoid.
     * @param avoidSegments Segments to avoid.
     * @return A tuple containing the best driving route, walking route, parking node, total time, and walking time.
     * @complexity O(N (N + M) log N) where N is the number of locations and M is the number of Roads.
     */
    std::tuple<std::vector<int>, std::vector<int>, int, int, int, std::vector<Suggestion>> EnvironmentallyFriendlyRoute(
        int sourceId, int destId, int maxWalkingTime,
        const std::unordered_set<int>& avoidNodes,
        std::unordered_set<std::pair<int, int>, pair_hash>& avoidSegments);
private:
    std::vector<Location*> locations;  ///< List of all locations
};

#endif // GRAPH_H