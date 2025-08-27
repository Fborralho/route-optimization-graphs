/**
* @file FileManager.h
 * @brief Handles file input/output operations for the route planning system
 */

#ifndef FILES_MANAGER
#define FILES_MANAGER

#include "Graph.h"

/**
 * @struct Input
 * @brief Stores input parameters for route planning
 * @details Contains source, destination, constraints, and route type information
 */
struct Input {
    /**
     * @brief The ID of the source location.
     */
    int sourceId;
    /**
     * @brief The ID of the destination location.
     */
    int destId;
    /**
     * @brief Locations to avoid during route planning.
     */
    std::unordered_set<int> avoidNodes = {};
    /**
     * @brief Roads to avoid during route planning.
     */
    std::unordered_set<std::pair<int, int>, pair_hash> avoidSegments = {};
    /**
     * @brief The maximum allowed walking time for environmentally friendly routes.
     */
    int maxWalkingTime = -1;
    /**
     * @brief Location that needs to be included during route planning.
     */
    int includeNodeId = -1;
    /**
     * @brief The type of input determining the route planning mode.
     *
     * Possible values:
     * - 0: Standard driving route.
     * - 1: Restricted route (some locations may be blocked).
     * - 2: Environmentally friendly route (driving + walking).
     */
    int TypeOfInput;
};

/**
 * @struct Output
 * @brief Stores results of route planning operations
 * @details Contains best path, alternative path, and additional route information
 */
struct Output {
    /**
     * @brief The ID of the source location.
     */
    int sourceId;
    /**
     * @brief The ID of the destination location.
     */
    int destId;
    /**
     * @brief The best route found.
     */
    std::pair<std::vector<int>, int> bestPath = {};
    /**
     * @brief The alternative route found.
     */
    std::pair<std::vector<int>, int> altPath = {};
    /**
     * @brief The ID of the parking node.
     */
    int parkingNode;
    /**
     * @brief The total estimated time for the trip.
     */
    int totalTime;
    /**
     * @brief The type of input determining the route planning mode.
     *
     * Possible values:
     * - 0: Standard driving route.
     * - 1: Restricted route (some locations may be blocked).
     * - 2: Environmentally friendly route (driving + walking).
     */
    int TypeOfInput;
    /**
     * @brief Indicates if route suggestions are available.
     */
    bool hasSuggestions = false;
    /**
     * @brief A list of suggested alternative routes in case walking constraints are exceeded.
     */
    std::vector<Suggestion> suggestions;
    /**
     * @brief The maximum allowed walking time for environmentally friendly routes.
     */
    int maxWalkingTime = -1;
};

/**
 * @class FileManager
 * @brief Handles all file operations for the system
 * @details Reads input files, writes output files, and loads location data
 */
class FileManager {
public:
/**
 * @brief Reads the route planning input from a file.
 * @param filename The input file containing Mode, Source, and Destination.
 * @param typeOfInput Determines if input is normal, restricted or environmentally friendly.
 * @return A pair containing source and destination location codes.
 * @complexity O(1) since we only read a max of 10 lines.
 */
static Input readInputFile(const std::string &filename, const int& typeOfInput);

/**
 * @brief Writes the best and alternative routes to an output file.
 * @param filename The output file name.
 * @param output Contains all the possible output parameters.
 * @complexity O(N) where N is the path length.
 */
static void writeOutputFile(const std::string &filename, const Output &output);

/**
 * @brief Reads and loads location data from the Locations.csv file.
 * @param filename The name of the CSV file to read.
 * @param RoadMap Pointer to Graph where locations are being loaded.
 * @complexity O(N) where N is the number of locations in the file.
 */
static void loadLocations(const std::string &filename, Graph* RoadMap);

/**
 * @brief Reads and loads distance data from the Distances.csv.
 * @param filename The name of the CSV file to read.
 * @param RoadMap Pointer to Graph where distances are being loaded.
 * @complexity O(N) where N is the number of roads in the file.
 */
static void loadDistances(const std::string &filename, Graph* RoadMap);

};

#endif