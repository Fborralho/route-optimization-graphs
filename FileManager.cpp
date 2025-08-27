#include "FileManager.h"
#include "Graph.h"
#include <fstream>
#include <sstream>
using namespace std;

void FileManager::loadLocations(const string& filename, Graph* graph) {
    ifstream file(filename);
    if (!file) {
        cerr << "Error: Unable to open " << filename << endl;
        return;
    }
    string line;
    getline(file, line); // Skip header
    while (getline(file, line)) {
        stringstream ss(line);
        string name, idStr, code, parkingStr;
        getline(ss, name, ',');
        getline(ss, idStr, ',');
        getline(ss, code, ',');
        getline(ss, parkingStr, ',');
        if (idStr.empty() || code.empty() || parkingStr.empty()) continue;
        int id = stoi(idStr);
        bool hasParking = (parkingStr == "1");
        graph->addLocation(id, code, hasParking);
    }
    file.close();
}

void FileManager::loadDistances(const string& filename, Graph* graph) {
    ifstream file(filename);
    if (!file) {
        cerr << "Error: Unable to open " << filename << endl;
        return;
    }
    string line;
    getline(file, line); // Skip header
    while (getline(file, line)) {
        stringstream ss(line);
        string loc1Code, loc2Code, drivingStr, walkingStr;
        getline(ss, loc1Code, ',');
        getline(ss, loc2Code, ',');
        getline(ss, drivingStr, ',');
        getline(ss, walkingStr, ',');
        if (loc1Code.empty() || loc2Code.empty()) continue;
        auto loc1 = graph->findLocation(loc1Code);
        auto loc2 = graph->findLocation(loc2Code);
        if (!loc1 || !loc2) continue;
        int drivingTime = (drivingStr == "X") ? INF : stoi(drivingStr);
        int walkingTime = stoi(walkingStr);
        graph->addRoad(loc1, loc2, drivingTime, walkingTime);
    }
    file.close();
}

Input FileManager::readInputFile(const string& filename, const int& typeOfInput) {
    ifstream file(filename);
    if (!file) {
        cerr << "Error: Unable to open " << filename << endl;
        return {-1, -1};
    }

    Input input;
    input.TypeOfInput = typeOfInput;
    string mode, line;

    getline(file, mode);
    getline(file, line);
    int sourceId = stoi(line.substr(line.find(':') + 1));
    getline(file, line);
    int destId = stoi(line.substr(line.find(':') + 1));
    int maxWalkingTime = -1;

    if (mode == "Mode:driving-walking") {
        getline(file, line);
        maxWalkingTime = stoi(line.substr(line.find(':') + 1));
        input.TypeOfInput = 2;
    }

    input.sourceId = sourceId;
    input.destId = destId;
    input.maxWalkingTime = maxWalkingTime;

    if (typeOfInput == 1) {
        //AvoidNodes:<id>,<id>,...
        getline(file, line);
        if (line.find("AvoidNodes:") == 0 && line.length() > 11) {
            string nodes = line.substr(11);
            stringstream ss(nodes);
            string node;
            while (getline(ss, node, ',')) {
                input.avoidNodes.insert(stoi(node));
            }
        }

        //AvoidSegments:(id,id),(id,id),...
        getline(file, line);
        if (line.find("AvoidSegments:") == 0 && line.length() > 14) {
            string segments = line.substr(14); //Ignore the "AvoidSegments:" part
            size_t pos = 0;

            while ((pos = segments.find('(')) != string::npos) {
                size_t commaPos = segments.find(',', pos);
                size_t endPos = segments.find(')', commaPos);

                if (commaPos == string::npos || endPos == string::npos) break;

                int from = stoi(segments.substr(pos + 1, commaPos - pos - 1));
                int to = stoi(segments.substr(commaPos + 1, endPos - commaPos - 1));

                input.avoidSegments.insert({from, to});

                segments = segments.substr(endPos + 1); //Next segment
            }
        }
        //IncludeNode:<id>
        getline(file, line);
        if (line.find("IncludeNode:") == 0 && line.length() > 12) {
            input.includeNodeId = stoi(line.substr(12));
        }
    }
    file.close();
    return input;
}

void FileManager::writeOutputFile(const string& filename, const Output &output) {
    ofstream file(filename);
    if (!file) {
        cerr << "Error: Unable to open " << filename << endl;
        return;
    }

    file << "Source:" << output.sourceId << "\n";
    file << "Destination:" << output.destId << "\n";

    if (output.TypeOfInput == 0) {
        file << "BestDrivingRoute:";
        if (output.bestPath.first.empty()) file << "none\n";
        else {
            for (size_t i = 0; i < output.bestPath.first.size(); ++i) {
                file << output.bestPath.first[i];
                if (i < output.bestPath.first.size() - 1) file << ",";
            }
            file << " (" << output.bestPath.second << " min)\n";
        }

        file << "AlternativeRoute:";
        if (output.altPath.first.empty()) file << "none\n";
        else {
            for (size_t i = 0; i < output.altPath.first.size(); ++i) {
                file << output.altPath.first[i];
                if (i < output.altPath.first.size() - 1) file << ",";
            }
            file << " (" << output.altPath.second << " min)\n";
        }
    }

    if (output.TypeOfInput == 1) {
        file << "RestrictedDrivingRoute:";
        if (output.bestPath.first.empty()) file << "none\n";
        else {
            for (size_t i = 0; i < output.bestPath.first.size(); ++i) {
                file << output.bestPath.first[i];
                if (i < output.bestPath.first.size() - 1) file << ",";
            }
            file << " (" << output.bestPath.second << " min)\n";
        }
    }

    if (output.TypeOfInput == 2) {
        // Case 1: No route found at all
        if (output.bestPath.first.empty() && output.altPath.first.empty() && 
            output.parkingNode == -1 && !output.hasSuggestions) {
            file << "DrivingRoute:none\n";
            file << "ParkingNode:none\n";
            file << "WalkingRoute:none\n";
            file << "TotalTime:none\n";
            file << "Message:No possible route with max walking time of "
                 << output.maxWalkingTime << " minutes.\n";
        }
        // Case 2: Found exact route (within walking limit)
        else if (!output.hasSuggestions) {
            file << "DrivingRoute:";
            if (output.bestPath.first.empty()) file << "none\n";
            else {
                for (size_t i = 0; i < output.bestPath.first.size(); ++i) {
                    file << output.bestPath.first[i];
                    if (i < output.bestPath.first.size() - 1) file << ",";
                }
                file << " (" << output.bestPath.second << " min)\n";
            }

            file << "ParkingNode:" << output.parkingNode << "\n";

            file << "WalkingRoute:";
            if (output.altPath.first.empty()) file << "none\n";
            else {
                for (size_t i = 0; i < output.altPath.first.size(); ++i) {
                    file << output.altPath.first[i];
                    if (i < output.altPath.first.size() - 1) file << ",";
                }
                file << " (" << output.altPath.second << " min)\n";
            }

            file << "TotalTime:" << output.totalTime << "\n";
        }
        // Case 3: Only have suggestions (exceed walking time)
        else {
            for (auto a : output.suggestions) {
                cout << a.totalTime << endl;
            }
            for(size_t i = 0; i < 2; i++){
                const auto& suggestion = output.suggestions[i];

                file << "DrivingRoute" << i + 1 << ":";
                for (size_t j = 0; j < suggestion.drivePath.size(); j++){
                    file << suggestion.drivePath[j];
                    if (j < suggestion.drivePath.size() - 1) file << ",";
                }
                file << " (" << (suggestion.totalTime - suggestion.walkingTime) << " min)\n";

                file << "ParkingNode" << i + 1 << ":" << suggestion.parkingNode << " \n";

                file << "WalkingRoute1:";
                for (size_t j = 0; j < suggestion.walkPath.size();j++){
                    file << suggestion.walkPath[j];
                    if( j < suggestion.walkPath.size() - 1) file << ",";
                }
    
                file << " (" << suggestion.walkingTime << " min)";
                file << "(Exceeds by " << suggestion.exceedWalkingBy << " min)\n";
                file << "TotalTime" << i + 1 << ":" << suggestion.totalTime << "\n";
            }
    
        }
    file.close();
    }
}
