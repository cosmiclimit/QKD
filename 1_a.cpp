/*
 * =====================================================================================
 *
 *       Filename:  CSB22055_67_T1_robotMoves_revised.cpp
 *
 *    Description:  Implementing A* Algorithm to find a path for a robot amidst
 *                  polygonal obstacles. This is a revised version focusing on
 *                  readability and minor simplifications.
 *
 *         Author:  [CSB22055 && CSB22067]
 *
 * =====================================================================================
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <iomanip>
#include <limits>

using namespace std;

// --- CONFIGURATION ---
const bool DEBUG_MODE = true; // Set to true to enable detailed step-by-step logging

// --- HELPER FOR LOGGING ---
void log_msg(const string &msg)
{
    if (DEBUG_MODE)
    {
        cout << "[LOG] " << msg << endl;
    }
}

// --- DATA STRUCTURES ---
struct Point
{
    int id;
    double x, y;
};

// Node structure for the A* priority queue
struct SearchNode
{
    int vertexId;
    double costToReach;        // g_cost: cost from start to this node
    double estimatedTotalCost; // f_cost: g_cost + heuristic
    // Overload '>' for the min-priority queue
    bool operator>(const SearchNode &other) const { return estimatedTotalCost > other.estimatedTotalCost; }
};

// A graph represented by an adjacency list
using VisibilityGraph = unordered_map<int, vector<pair<int, double>>>;

// --- PROBLEM DATA (VERTICES AND SEQUENTIAL POLYGONS) ---
const vector<Point> OBSTACLE_VERTICES = {
    {0, 188, 466}, {1, 56, 341}, {2, 78, 224}, {3, 206, 195}, {4, 276, 350}, {5, 95, 143}, {6, 95, 24}, {7, 461, 25}, {8, 459, 144}, {9, 324, 375}, {10, 276, 186}, {11, 381, 186}, {12, 477, 466}, {13, 384, 455}, {14, 384, 305}, {15, 541, 395}, {16, 473, 252}, {17, 513, 80}, {18, 590, 157}, {19, 556, 452}, {20, 556, 211}, {21, 712, 211}, {22, 712, 452}, {23, 729, 205}, {24, 643, 149}, {25, 643, 61}, {26, 713, 19}, {27, 792, 56}, {28, 792, 148}, {29, 782, 456}, {30, 727, 417}, {31, 808, 182}, {32, 829, 412}};
const vector<vector<Point>> POLYGONS = {
    {{0, 188, 466}, {1, 56, 341}, {2, 78, 224}, {3, 206, 195}, {4, 276, 350}},
    {{5, 95, 143}, {8, 459, 144}, {7, 461, 25}, {6, 95, 24}},
    {{9, 324, 375}, {11, 381, 186}, {10, 276, 186}},
    {{12, 477, 466}, {15, 541, 395}, {14, 384, 305}, {13, 384, 455}},
    {{16, 473, 252}, {18, 590, 157}, {17, 513, 80}},
    {{19, 556, 452}, {22, 712, 452}, {21, 712, 211}, {20, 556, 211}},
    {{23, 729, 205}, {24, 643, 149}, {25, 643, 61}, {26, 713, 19}, {27, 792, 56}, {28, 792, 148}},
    {{29, 782, 456}, {30, 727, 417}, {31, 808, 182}, {32, 829, 412}}};

// --- GEOMETRY & HELPER FUNCTIONS ---
double calculateEuclideanDistance(const Point &a, const Point &b) { return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)); }
double heuristic(const Point &a, const Point &b) { return calculateEuclideanDistance(a, b); }

// Determines the orientation of the ordered triplet (p, q, r)
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(const Point &p, const Point &q, const Point &r)
{
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (abs(val) < 1e-9) return 0; // Use a small epsilon for float comparison
    return (val > 0) ? 1 : 2;
}

// Checks if line segment 'p1q1' and 'p2q2' intersect.
bool lineSegmentIntersect(const Point &p1, const Point &q1, const Point &p2, const Point &q2)
{
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    // General case: if orientations are different, they must intersect
    if (o1 != o2 && o3 != o4) return true;
    return false; // Collinear cases are not treated as intersections for this problem
}

// Checks if a point is inside a polygon using the Ray Casting algorithm.
bool isInside(const Point &p, const vector<Point> &polygon)
{
    if (polygon.size() < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
    {
        if (((polygon[i].y > p.y) != (polygon[j].y > p.y)) &&
            (p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
        {
            inside = !inside;
        }
    }
    return inside;
}

// --- A* SEARCH ALGORITHM ---
vector<int> aStarSearch(const VisibilityGraph &graph, const unordered_map<int, Point> &vertexMap, int startId, int goalId)
{
    log_msg("\n--- Starting A* Search ---");
    // Priority queue to store nodes to visit, ordered by f_cost
    priority_queue<SearchNode, vector<SearchNode>, greater<SearchNode>> openSet;
    // Map to reconstruct the path
    unordered_map<int, int> pathPredecessor;
    // Map to store the cost of the cheapest path from start to a node found so far
    unordered_map<int, double> costToReach;

    for (const auto &pair : vertexMap)
    {
        costToReach[pair.first] = numeric_limits<double>::infinity();
    }
    costToReach[startId] = 0;
    openSet.push({startId, 0.0, heuristic(vertexMap.at(startId), vertexMap.at(goalId))});
    log_msg("Pushed Start Node (ID: " + to_string(startId) + ") to open set.");

    while (!openSet.empty())
    {
        SearchNode current = openSet.top();
        openSet.pop();
        log_msg("Processing node " + to_string(current.vertexId) + " with f_cost " + to_string(current.estimatedTotalCost));

        if (current.vertexId == goalId)
        {
            log_msg("Goal node " + to_string(goalId) + " reached!");
            vector<int> path;
            int at = goalId;
            while (pathPredecessor.count(at))
            {
                path.push_back(at);
                at = pathPredecessor.at(at);
            }
            path.push_back(startId);
            reverse(path.begin(), path.end());
            return path;
        }
        if (graph.count(current.vertexId) == 0)
        {
            log_msg("Node " + to_string(current.vertexId) + " has no neighbors. Skipping.");
            continue;
        }
        for (const auto &edge : graph.at(current.vertexId))
        {
            int neighborId = edge.first;
            double tentativeCostToReach = costToReach.at(current.vertexId) + edge.second;
            if (tentativeCostToReach < costToReach.at(neighborId))
            {
                pathPredecessor[neighborId] = current.vertexId;
                costToReach[neighborId] = tentativeCostToReach;
                double newEstimatedTotalCost = tentativeCostToReach + heuristic(vertexMap.at(neighborId), vertexMap.at(goalId));
                openSet.push({neighborId, tentativeCostToReach, newEstimatedTotalCost});
                log_msg("  -> Found better path to neighbor " + to_string(neighborId) + ". New f_cost: " + to_string(newEstimatedTotalCost));
            }
        }
    }
    log_msg("A* search failed. Open set is empty and goal was not reached.");
    return {}; // Return empty path if goal is not found
}

// --- MAIN EXECUTION ---
int main()
{
    // 1. GET USER INPUT
    Point startPoint, goalPoint;
    cout << "Enter Start X coordinate: ";
    cin >> startPoint.x;
    cout << "Enter Start Y coordinate: ";
    cin >> startPoint.y;
    cout << "Enter Goal X coordinate: ";
    cin >> goalPoint.x;
    cout << "Enter Goal Y coordinate: ";
    cin >> goalPoint.y;

    // 2. VALIDATE INPUT
    for (const auto &polygon : POLYGONS)
    {
        if (isInside(startPoint, polygon))
        {
            cerr << "\nError: Start point (" << startPoint.x << ", " << startPoint.y << ") is inside an obstacle." << endl;
            return 1;
        }
        if (isInside(goalPoint, polygon))
        {
            cerr << "\nError: Goal point (" << goalPoint.x << ", " << goalPoint.y << ") is inside an obstacle." << endl;
            return 1;
        }
    }

    // 3. SETUP DATA
    // Combine obstacle vertices, start, and goal points into one list for graph building
    unordered_map<int, Point> vertexMap;
    vector<Point> allVertices = OBSTACLE_VERTICES;
    startPoint.id = allVertices.size();
    allVertices.push_back(startPoint);
    goalPoint.id = allVertices.size();
    allVertices.push_back(goalPoint);

    for (const auto &p : allVertices)
    {
        vertexMap[p.id] = p;
    }

    // 4. BUILD VISIBILITY GRAPH
    VisibilityGraph visibilityGraph;

    // Lambda function to check if a direct path between two points is clear of obstacles
    auto isPathVisible = [&](const Point &p1, const Point &p2)
    {
        for (const auto &polygon : POLYGONS)
        {
            for (size_t j = 0; j < polygon.size(); ++j)
            {
                const Point &edge_p1 = polygon[j];
                const Point &edge_p2 = polygon[(j + 1) % polygon.size()];

                // Allow movement along polygon edges by ignoring intersections if the path shares a vertex with the edge
                if (p1.id == edge_p1.id || p1.id == edge_p2.id || p2.id == edge_p1.id || p2.id == edge_p2.id)
                {
                    continue;
                }

                // If the path from p1 to p2 intersects any polygon edge, it is not visible
                if (lineSegmentIntersect(p1, p2, edge_p1, edge_p2))
                {
                    log_msg("  -> Visibility FAIL: Segment intersects an edge of a polygon.");
                    return false;
                }
            }
        }
        log_msg("  -> Visibility PASS");
        return true;
    };

    log_msg("\n--- Building Full Visibility Graph ---");
    for (size_t i = 0; i < allVertices.size(); ++i)
    {
        for (size_t j = i + 1; j < allVertices.size(); ++j)
        {
            const Point &p1 = allVertices[i];
            const Point &p2 = allVertices[j];
            log_msg("Checking visibility between " + to_string(p1.id) + " and " + to_string(p2.id));
            if (isPathVisible(p1, p2))
            {
                double dist = calculateEuclideanDistance(p1, p2);
                visibilityGraph[p1.id].push_back({p2.id, dist});
                visibilityGraph[p2.id].push_back({p1.id, dist});
                log_msg("    Added edge between " + to_string(p1.id) + " and " + to_string(p2.id));
            }
        }
    }

    // 5. RUN A* SEARCH
    vector<int> pathVertexIds = aStarSearch(visibilityGraph, vertexMap, startPoint.id, goalPoint.id);

    // 6. DISPLAY RESULTS
    if (pathVertexIds.empty())
    {
        cout << "\nNo path found from Start to Goal." << endl;
    }
    else
    {
        cout << "\nOptimal path found:" << endl;
        double total_distance = 0.0;
        for (size_t i = 0; i < pathVertexIds.size(); ++i)
        {
            const auto &p = vertexMap.at(pathVertexIds[i]);
            string label;
            if (p.id == startPoint.id) label = "Start";
            else if (p.id == goalPoint.id) label = "Goal";
            else label = "Vertex " + to_string(p.id);
            
            cout << " -> " << label << " (" << p.x << ", " << p.y << ")" << endl;
            if (i > 0)
            {
                total_distance += calculateEuclideanDistance(vertexMap.at(pathVertexIds[i - 1]), p);
            }
        }
        cout << "\nTotal Path Distance: " << fixed << setprecision(2) << total_distance << endl;
    }
    return 0;
}