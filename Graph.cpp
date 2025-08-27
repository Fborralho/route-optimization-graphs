#include "Graph.h"
#include <queue>
#include <algorithm>

#define INF std::numeric_limits<int>::max()

size_t pair_hash::operator()(const std::pair<int, int>& p) const {
    return std::hash<int>()(p.first) ^ std::hash<int>()(p.second);
}

Road::Road(Location* origin, Location* destination, int drivingTime, int walkingTime)
    : origin(origin), destination(destination), drivingTime(drivingTime), walkingTime(walkingTime) {}

Location* Road::getOrigin() const { return origin; }

Location* Road::getDestination() const { return destination; }

int Road::getDrivingTime() const { return drivingTime; }

int Road::getWalkingTime() const { return walkingTime; }

void Road::setDrivingTime(int newDrivingTime) { this->drivingTime = newDrivingTime; }


Location::Location(int id, std::string code, bool hasParking)
    : id(id), code(std::move(code)), hasParking(hasParking), parent(nullptr), distance(0) {}

int Location::getId() const { return id; }

std::string Location::getCode() const { return code; }

bool Location::HasParking() const { return hasParking; }

const std::vector<Road*>& Location::getAdj() const { return adj; }

Road* Location::getParent() const { return parent; }

double Location::getDistance() const { return distance; }

void Location::setParent(Road* newParent) { this->parent = newParent; }

void Location::setDistance(double newDist) { this->distance = newDist; }

void Location::addRoad(Road* road) { adj.emplace_back(road); }


Graph::~Graph() {
    for (auto loc : locations)
        delete loc;
}

const std::vector<Location*>& Graph::getLocations() const {
    return locations;
}

void Graph::addLocation(const int &id, const std::string &code, const bool &hasParking) {
    locations.push_back(new Location(id, code, hasParking));
}

Location* Graph::findLocation(const std::string& code) const {
    for (auto* location : locations) {
        if (location->getCode() == code) return location;
    }
    return nullptr;
}

Location* Graph::findLocation(const int &id) const {
    for (auto* location : locations) {
        if (location->getId() == id) return location;
    }
    return nullptr;
}

void Graph::addRoad(Location* from, Location* to, int drivingTime, int walkingTime) {
    if (!from || !to) return;
    from->addRoad(new Road(from, to, drivingTime, walkingTime));
    to->addRoad(new Road(to, from, drivingTime, walkingTime));
}

std::pair<std::vector<int>, int> Graph::dijkstra(
    int sourceId,
    int destinationId,
    bool isDriving,
    const std::unordered_set<int>& blockedNodes,
    std::unordered_set<std::pair<int, int>, pair_hash>& blockedSegments) {

    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;

    // Initialization
    for (const auto& loc : getLocations()) {
        loc->setDistance(INF);
        loc->setParent(nullptr);
    }
    auto source = findLocation(sourceId);
    source->setDistance(0);
    pq.emplace(source->getDistance(), sourceId);

    while (!pq.empty()) {
        auto [currentDist, currentNodeId] = pq.top();
        auto currentNode = findLocation(currentNodeId);
        pq.pop();
        if (currentDist > currentNode->getDistance()) continue;

        if (blockedNodes.count(currentNodeId)) continue;

        for (const auto& road : findLocation(currentNodeId)->getAdj()) {
            int edgeWeight = isDriving ? road->getDrivingTime() : road->getWalkingTime();
            if (edgeWeight == INF) continue;
            if (blockedSegments.count({currentNodeId, road->getDestination()->getId()})) continue;
            int newDist = currentDist + edgeWeight;
            if (newDist < road->getDestination()->getDistance()) {
                road->getDestination()->setDistance(newDist);
                road->getDestination()->setParent(road);
                pq.emplace(newDist, road->getDestination()->getId());
            }
        }
    }
    auto destination = findLocation(destinationId);
    int dist = destination->getDistance();
    if (dist == INF) return {};
    // Reconstruct path
    std::vector<int> path;
    path.push_back(destination->getId());
    while (destination->getParent() != nullptr && destination->getId() != sourceId) {
        blockedSegments.insert({destination->getParent()->getOrigin()->getId(), destination->getParent()->getDestination()->getId()});
        destination = destination->getParent()->getOrigin();
        path.push_back(destination->getId());
    }
    reverse(path.begin(), path.end());
    return make_pair(path, dist);
}

std::tuple<std::vector<int>, std::vector<int>, int, int, int, std::vector<Suggestion>>
Graph::EnvironmentallyFriendlyRoute(
    const int sourceId, const int destId, const int maxWalkingTime,
    const std::unordered_set<int>& avoidNodes,
    std::unordered_set<std::pair<int, int>, pair_hash>& avoidSegments) {

    int bestTotalTime = INF, bestWalkingTime = INF;
    std::vector<int> bestDrive, bestWalk;
    int bestParking = -1;
    std::vector<Suggestion> suggestions;

    for (auto* parkingNode : locations) {
        if (!parkingNode->HasParking() || parkingNode->getId() == sourceId || parkingNode->getId() == destId) {
            continue;
        }

        auto tmpSegments = avoidSegments;
        // Driving segment
        auto driveResult = dijkstra(sourceId, parkingNode->getId(), true, avoidNodes, tmpSegments);
        if (driveResult.first.empty()) continue;

        // Walking segment
        auto walkResult = dijkstra(parkingNode->getId(), destId, false, avoidNodes, tmpSegments);
        if (walkResult.first.empty()) continue;

        int totalTime = driveResult.second + walkResult.second;
        int exceedWalk = std::max(0, walkResult.second - maxWalkingTime);

        if (exceedWalk == 0) {
            if (totalTime < bestTotalTime || (totalTime == bestTotalTime && walkResult.second < bestWalkingTime)) {
                bestTotalTime = totalTime;
                bestDrive = driveResult.first;
                bestWalk = walkResult.first;
                bestWalkingTime = walkResult.second;
                bestParking = parkingNode->getId();
            }
        } else {
            suggestions.push_back({
                driveResult.first,
                walkResult.first,
                parkingNode->getId(),
                totalTime,
                walkResult.second,
                exceedWalk
            });
        }
    }

    if (!suggestions.empty()) {
        std::sort(suggestions.begin(), suggestions.end(), [](const Suggestion& a, const Suggestion& b) {
            return a.totalTime < b.totalTime;
        });
    }

    return {bestDrive, bestWalk, bestParking, bestTotalTime, bestWalkingTime, suggestions};
}