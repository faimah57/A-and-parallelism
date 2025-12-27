/******************************************************************************

Parallel A* maze solver with INTENTIONAL race conditions
for ARTI 503 - Assignment 2 (Laptop version)

*******************************************************************************/
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <algorithm>
#include <functional>
#include <omp.h>          // <<< OpenMP header

using namespace std;
using namespace std::chrono;

struct Node {
    int x, y;
    double f, g, h;
    Node* parent;
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

// Directions for movement (up, down, left, right, diagonals)
int dx[8] = { -1, 1, 0, 0, -1, -1, 1, 1 };
int dy[8] = { 0, 0, -1, 1, -1, 1, -1, 1 };

/*** ======== SHARED GLOBAL VARIABLES (RACE VARIABLES) ======== ***/
// These variables are shared by all threads and updated without
// any synchronization. They will produce race conditions.

// RACE VARIABLE #1: total length of all paths found
int totalPathLength = 0;

// RACE VARIABLE #2: number of mazes where a valid path was found
int successfulMazes = 0;

// RACE VARIABLE #3: number of times solveMaze() was called
int mazeAttempts = 0;
/*** ========================================================== ***/

// Heuristic: Euclidean distance
double heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

// Check if the position is valid
bool isValid(int x, int y, int rows, int cols, vector<vector<int>>& grid) {
    return (x >= 0 && x < rows && y >= 0 && y < cols && grid[x][y] == 0);
}

// Reconstruct path from goal to start
vector<pair<int, int>> reconstructPath(Node* goal) {
    vector<pair<int, int>> path;
    Node* current = goal;
    while (current != nullptr) {
        path.push_back({current->x, current->y});
        current = current->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

// A* Search function
vector<pair<int, int>> aStarSearch(vector<vector<int>>& grid,
                                   pair<int, int> start,
                                   pair<int, int> goal) {
    int rows = grid.size();
    int cols = grid[0].size();

    vector<vector<bool>> closed(rows, vector<bool>(cols, false));
    vector<vector<Node*>> allNodes(rows, vector<Node*>(cols, nullptr));

    priority_queue<Node*, vector<Node*>, function<bool(Node*, Node*)>> openList(
        [](Node* a, Node* b) { return a->f > b->f; }
    );

    Node* startNode = new Node{ start.first, start.second, 0, 0,
        heuristic(start.first, start.second, goal.first, goal.second), nullptr };
    openList.push(startNode);
    allNodes[start.first][start.second] = startNode;

    Node* goalNode = nullptr;

    // === First Nested Loop Timing: Neighbor Exploration ===
    auto start_explore = high_resolution_clock::now();

    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();

        if (current->x == goal.first && current->y == goal.second) {
            goalNode = current;
            break;
        }

        closed[current->x][current->y] = true;

        // Explore all 8 directions (neighbors)
        for (int i = 0; i < 8; i++) {
            int newX = current->x + dx[i];
            int newY = current->y + dy[i];

            if (!isValid(newX, newY, rows, cols, grid) || closed[newX][newY])
                continue;

            double gNew = current->g + heuristic(current->x, current->y, newX, newY);
            double hNew = heuristic(newX, newY, goal.first, goal.second);
            double fNew = gNew + hNew;

            if (allNodes[newX][newY] == nullptr || fNew < allNodes[newX][newY]->f) {
                Node* neighbor = new Node{ newX, newY, fNew, gNew, hNew, current };
                openList.push(neighbor);
                allNodes[newX][newY] = neighbor;
            }
        }
    }

    auto end_explore = high_resolution_clock::now();
    auto duration_explore = duration_cast<duration<double>>(end_explore - start_explore);
    cout << "Neighbor Exploration Time: " << duration_explore.count() << " s" << endl;

    // === Second Nested Loop Timing: Path Reconstruction ===
    auto start_path = high_resolution_clock::now();
    vector<pair<int, int>> path;
    if (goalNode)
        path = reconstructPath(goalNode);
    auto end_path = high_resolution_clock::now();
    auto duration_path = duration_cast<duration<double>>(end_path - start_path);
    cout << "Path Reconstruction Time: " << duration_path.count() << " s" << endl;

    // Clean up
    for (auto& row : allNodes)
        for (auto node : row)
            delete node;

    return path;
}

void solveMaze(int rows, int cols) {
    // Create a random grid
    vector<vector<int>> grid(rows, vector<int>(cols, 0));

    // Add random obstacles
    for (int i = 0; i < rows * cols / 5; i++) {
        int x = rand() % rows;
        int y = rand() % cols;
        grid[x][y] = 1;
    }

    pair<int, int> start = {0, 0};
    pair<int, int> goal = {rows - 1, cols - 1};

    auto path = aStarSearch(grid, start, goal);
    int pathLen = static_cast<int>(path.size());

    cout << "Path length: " << pathLen << " nodes\n";

    // ====== HERE ARE THE RACE CONDITIONS (writes to shared globals) ======
    // All threads call solveMaze() in parallel and execute these lines
    // at the same time without any synchronization.

    mazeAttempts++;               // RACE VARIABLE #3 write
    totalPathLength += pathLen;   // RACE VARIABLE #1 write

    if (pathLen > 0) {
        successfulMazes++;        // RACE VARIABLE #2 write
    }
    // =====================================================================
}

int main() {
    srand(time(nullptr));

    int numMazes = 3;
    int sizes[3] = {100, 500, 1000};

    int numThreads = 4; // you can change this to 1, 2, 4, 8 for TASK#4
    omp_set_num_threads(numThreads);

    // === Outer Loop Timing ===
    auto start_outer = high_resolution_clock::now();

    // Parallel loop: each thread solves one maze size.
    // The shared global variables above will be updated concurrently.
    #pragma omp parallel for
    for (int i = 0; i < numMazes; i++) {
        cout << "\nSolving Maze " << i + 1
             << " (" << sizes[i] << "x" << sizes[i] << ") ..." << endl;

        solveMaze(sizes[i], sizes[i]);
    }

    auto end_outer = high_resolution_clock::now();
    auto duration_outer = duration_cast<duration<double>>(end_outer - start_outer);
    cout << "\nOuter Loop Time (total for all mazes): "
         << duration_outer.count() << " s" << endl;

    cout << "\n=== Global Statistics (with race conditions) ===\n";
    cout << "Total maze attempts: " << mazeAttempts << endl;
    cout << "Total path length over all mazes: " << totalPathLength << endl;
    cout << "Number of successful mazes: " << successfulMazes << endl;

    return 0;
}