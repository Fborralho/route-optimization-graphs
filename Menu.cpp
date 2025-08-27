/**
* @file Menu.cpp
 * @brief Implements the user interface and main program logic
 */

#include <algorithm>
#include <vector>
#include <limits>

#include "FileManager.h"
#include "Graph.h"
using namespace std;

//Global data structures
Graph* RoadMap = new Graph;  ///< Adjacency list representing the road network

/**
 * @brief Handles normal route planning.
*/
void planNormalRoute() {
    Input input = FileManager::readInputFile("input.txt", 0);
    Output output;
    output.sourceId = input.sourceId;
    output.destId = input.destId;
    output.TypeOfInput = 0;
    if (input.sourceId == -1 || input.destId == -1) {
        cerr << "Error: Invalid input file format.\n";
        return;
    }
    if (!RoadMap->findLocation(input.sourceId) || !RoadMap->findLocation(input.destId)) {
        cerr << "Error: Invalid Source or Destination ID.\n";
        return;
    }
    output.bestPath = RoadMap->dijkstra(input.sourceId, input.destId, true, input.avoidNodes,input.avoidSegments);
    if (output.bestPath.first.size() > 1) {
        output.altPath = RoadMap->dijkstra(input.sourceId, input.destId, true, input.avoidNodes, input.avoidSegments);
    }
    FileManager::writeOutputFile("output.txt", output);
    input.avoidSegments.clear();

}

/**
 * @brief Handles restricted route planning.
*/
void planRestrictedRoute() {
    Input input = FileManager::readInputFile("input.txt", 1);
    Output output;
    output.TypeOfInput = 1;
    output.sourceId = input.sourceId;
    output.destId = input.destId;
    if (input.sourceId == -1 || input.destId == -1) {
        cerr << "Error: Invalid input file format.\n";
        return;
    }
    if (!RoadMap->findLocation(input.sourceId) || !RoadMap->findLocation(input.destId)) {
        cerr << "Error: Invalid Source or Destination ID.\n";
        return;
    }

    if (input.includeNodeId != -1) {
        auto firstHalf = RoadMap->dijkstra(input.sourceId, input.includeNodeId, true, input.avoidNodes, input.avoidSegments);
        if (firstHalf.first.empty()) {
            FileManager::writeOutputFile("output.txt", output);
            return;
        }

        auto secondHalf = RoadMap->dijkstra(input.includeNodeId, input.destId, true, input.avoidNodes, input.avoidSegments);
        if (secondHalf.first.empty()) {
            FileManager::writeOutputFile("output.txt", output);
            return;
        }

        // Merge paths (remove duplicate includeNodeId)
        std::vector<int> fullPath = firstHalf.first;
        fullPath.pop_back(); // remove includeNode from the first half
        fullPath.insert(fullPath.end(), secondHalf.first.begin(), secondHalf.first.end());
        output.bestPath.first = fullPath;
        output.bestPath.second = firstHalf.second + secondHalf.second; //Sum of times
        FileManager::writeOutputFile("output.txt", output);
    }
    else {
        output.bestPath = RoadMap->dijkstra(input.sourceId, input.destId, true, input.avoidNodes, input.avoidSegments);
        FileManager::writeOutputFile("output.txt", output);
    }
}

/**
 * @brief Handles environmentally friendly route planning.
*/
void planEnvironmentallyFriendlyRoute() {
    Input input = FileManager::readInputFile("input.txt", 0);
    Output output;
    output.TypeOfInput = 2;
    output.sourceId = input.sourceId;
    output.destId = input.destId;
    output.maxWalkingTime = input.maxWalkingTime;

    if (input.sourceId == -1 || input.destId == -1 || input.maxWalkingTime == -1) {
        cerr << "Error: Invalid environmentally-friendly input file format.\n";
        return;
    }

    auto [drivePath, walkPath, parkingNode, totalTime, walkingTime, suggestions] = RoadMap->EnvironmentallyFriendlyRoute(
        input.sourceId, input.destId, input.maxWalkingTime, input.avoidNodes, input.avoidSegments);
    if (!drivePath.empty() && !walkPath.empty() && parkingNode != -1) {
        output.bestPath.first = drivePath;
        output.altPath.first = walkPath;
        output.bestPath.second = totalTime - walkingTime;
        output.altPath.second = walkingTime;
        output.parkingNode = parkingNode;
        output.totalTime = totalTime;

    } else if(!suggestions.empty()){
        output.bestPath.first = suggestions[0].drivePath;
        output.altPath.first = suggestions[0].walkPath;
        output.bestPath.second = suggestions[0].totalTime - suggestions[0].walkingTime;
        output.altPath.second = suggestions[0].walkingTime;
        output.parkingNode = suggestions[0].parkingNode;
        output.totalTime = suggestions[0].totalTime;
        output.hasSuggestions = true;
        output.suggestions = suggestions;

    }   else{
        cerr << "No feasible environmentally-friendly route found.\n";
        return;
    }

    
    FileManager::writeOutputFile("output.txt", output);
}

/**
 * @brief Loads both locations and distances from CSV files.
 * @complexity O(N + M) where N is the number of locations and M is the number of roads.
*/
void loadData() {
    FileManager::loadLocations("LocSample.txt", RoadMap);
    FileManager::loadDistances("DisSample.txt", RoadMap);
}

/**
 * @brief Displays a menu and handles user input.
*/
void showMenu() {
    cout << "\n===== Route Planning Analysis Tool =====" << endl;
    cout << "1. Plan Route" << endl;
    cout << "2. Plan Restricted Route (Avoid Nodes/Segments)" << endl;
    cout << "3. Plan Environmentally Friendly Route (driving + walking)" << endl;
    cout << "4. Exit" << endl;
    cout << "Enter your choice: ";
}

int main() {
    loadData();
    int choice;
    do {
        showMenu();
        cin >> choice;
        switch (choice) {
            case 1:
                planNormalRoute();
            break;
            case 2:
               planRestrictedRoute();
            break;
            case 3:
                planEnvironmentallyFriendlyRoute();
            break;
            case 4:
                cout << "Exiting program." << endl;
            break;
            default:
                cout << "Invalid choice. Please enter a valid option." << endl;
        }
    } while (choice != 4);
    delete RoadMap;
    return 0;
}
