// COMSC-210 | lab 34 | Kent Kawashima
// IDE used: Visual Studio Code
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <climits>
using namespace std;

const int SIZE = 10; // Updated size to include airports

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair; // Creates alias 'Pair' for the pair<int,int> datatype

class Graph {
public:
    // a vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;
    unordered_map<int, string> airportNames; // Maps nodes to airport names

    // Graph Constructor
    Graph(vector<Edge> const &edges, vector<string> const &airports) {
        adjList.resize(SIZE);
        for (int i = 0; i < airports.size(); i++) {
            airportNames[i] = airports[i];
        }

        for (auto &edge : edges) {
            int src = edge.src;
            int dest = edge.dest;
            int weight = edge.weight;

            adjList[src].push_back(make_pair(dest, weight));
            adjList[dest].push_back(make_pair(src, weight));
        }
    }

    // Dijkstra's Algorithm for Shortest Path
    void dijkstra(int start) {
        vector<int> dist(SIZE, INT_MAX);
        vector<int> parent(SIZE, -1);
        priority_queue<Pair, vector<Pair>, greater<Pair>> pq;

        dist[start] = 0;
        pq.push(make_pair(0, start));

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (auto &neighbor : adjList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.push(make_pair(dist[v], v));
                }
            }
        }

        cout << "\nShortest Paths from " << airportNames[start] << ":\n";
        for (int i = 0; i < SIZE; i++) {
            if (i != start) {
                cout << airportNames[start] << " to " << airportNames[i] 
                     << " : " << dist[i] << " km" << endl;
            }
        }
    }

    // Minimum Spanning Tree (MST) using Prim's Algorithm
    void minimumSpanningTree(int start) {
        vector<int> key(SIZE, INT_MAX);
        vector<bool> inMST(SIZE, false);
        priority_queue<Pair, vector<Pair>, greater<Pair>> pq;

        key[start] = 0;
        pq.push(make_pair(0, start));

        cout << "\nMinimum Spanning Tree edges:" << endl;

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            if (inMST[u]) continue;
            inMST[u] = true;

            for (auto &neighbor : adjList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (!inMST[v] && weight < key[v]) {
                    key[v] = weight;
                    pq.push(make_pair(key[v], v));
                    cout << "Edge from " << airportNames[u] << " to " << airportNames[v]
                         << " with capacity: " << weight << " km" << endl;
                }
            }
        }
    }

    // Print the graph's adjacency list
    void printGraph() {
        cout << "Flight Network (Adjacency List Representation):" << endl;
        for (int i = 0; i < adjList.size(); i++) {
            cout << airportNames[i] << " --> ";
            for (Pair v : adjList[i])
                cout << "(" << airportNames[v.first] << ", " << v.second << " km) ";
            cout << endl;
        }
    }

    // Depth-First Search (DFS)
    void DFS(int start) {
        vector<bool> visited(SIZE, false);
        stack<int> stack;

        stack.push(start);

        cout << "\nDFS starting from " << airportNames[start] << ":\n";
        cout << "Route exploration order:\n";

        vector<string> visitedOrder;

        while (!stack.empty()) {
            int node = stack.top();
            stack.pop();

            if (!visited[node]) {
                visitedOrder.push_back(airportNames[node]);
                visited[node] = true;
            }

            for (auto &neighbor : adjList[node]) {
                int next = neighbor.first;
                if (!visited[next])
                    stack.push(next);
            }
        }

        for (size_t i = 0; i < visitedOrder.size(); i++) {
            cout << i + 1 << ". " << visitedOrder[i] << endl;
        }
    }

    // Breadth-First Search (BFS)
    void BFS(int start) {
        vector<bool> visited(SIZE, false);
        queue<int> queue;

        queue.push(start);
        visited[start] = true;

        cout << "\nBFS starting from " << airportNames[start] << ":\n";
        cout << "Closest airports visit order:\n";

        vector<string> visitedOrder;

        while (!queue.empty()) {
            int node = queue.front();
            queue.pop();

            visitedOrder.push_back(airportNames[node]);

            for (auto &neighbor : adjList[node]) {
                int next = neighbor.first;
                if (!visited[next]) {
                    queue.push(next);
                    visited[next] = true;
                }
            }
        }

        for (size_t i = 0; i < visitedOrder.size(); i++) {
            cout << i + 1 << ". " << visitedOrder[i] << endl;
        }
    }
};

// Menu-driven program
int main() {
    vector<string> airports = {
        "Los Angeles (LAX)", "New York (JFK)", "London (LHR)", "Tokyo (HND)",
        "Sydney (SYD)", "Dubai (DXB)", "Singapore (SIN)", "Paris (CDG)",
        "Frankfurt (FRA)", "Hong Kong (HKG)"
    };

    vector<Edge> edges = {
        {0, 1, 3971}, {0, 2, 8759}, {1, 3, 10853}, {2, 4, 17015},
        {3, 5, 7980}, {4, 6, 6304}, {5, 7, 5248}, {6, 8, 10528},
        {7, 9, 9452}, {8, 9, 920}
    };

    Graph graph(edges, airports);

    int choice = -1;
    while (choice != 0) {
        cout << "\nFlight Network Menu:\n";
        cout << "[1] Display flight network\n";
        cout << "[2] Perform DFS (Route Exploration)\n";
        cout << "[3] Perform BFS (Closest Airports)\n";
        cout << "[4] Calculate shortest paths\n";
        cout << "[5] Find Minimum Spanning Tree\n";
        cout << "[0] Exit\n";
        cout << "Enter your choice: ";
        cin >> choice;

        switch (choice) {
            case 1:
                graph.printGraph();
                break;
            case 2:
                graph.DFS(0); // Start DFS from Los Angeles (LAX)
                break;
            case 3:
                graph.BFS(0); // Start BFS from Los Angeles (LAX)
                break;
            case 4:
                graph.dijkstra(0); // Shortest paths from Los Angeles (LAX)
                break;
            case 5:
                graph.minimumSpanningTree(0); // Minimum Spanning Tree from LAX
                break;
            case 0:
                cout << "Exiting program.\n";
                break;
            default:
                cout << "Invalid choice, please try again.\n";
        }
    }

    return 0;
}
