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
        // resize the vector to hold SIZE elements of type vector<Edge>
        adjList.resize(SIZE);

        // Map node indices to airport names
        for (int i = 0; i < airports.size(); i++) {
            airportNames[i] = airports[i];
        }

        // add edges to the directed graph
        for (auto &edge : edges) {
            int src = edge.src;
            int dest = edge.dest;
            int weight = edge.weight;

            // insert at the end
            adjList[src].push_back(make_pair(dest, weight));

            // for an undirected graph, add an edge from dest to src also
            adjList[dest].push_back(make_pair(src, weight));
        }
    }

    // Dijkstra's Algorithm for Shortest Path
    void dijkstra(int start) {
        vector<int> dist(SIZE, INT_MAX); // Initialize distances to infinity
        vector<int> parent(SIZE, -1);    // Store parent nodes to reconstruct the path
        priority_queue<Pair, vector<Pair>, greater<Pair>> pq; // Min-heap priority queue

        dist[start] = 0;  // Distance to the start node is 0
        pq.push(make_pair(0, start)); // Push the starting node into the priority queue

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            // Process each neighbor of u
            for (auto &neighbor : adjList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                // If a shorter path is found
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;  // Store the path
                    pq.push(make_pair(dist[v], v));
                }
            }
        }

        // Print the shortest paths from the start node to all other nodes
        cout << "\nShortest Paths from " << airportNames[start] << ":\n";
        for (int i = 0; i < SIZE; i++) {
            if (i != start) {
                cout << airportNames[start] << " to " << airportNames[i] 
                     << " : " << dist[i] << " km" << endl;
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

    // Minimum Spanning Tree (MST) using Prim's Algorithm
    void minimumSpanningTree(int start) {
        vector<int> key(SIZE, INT_MAX); // Key values to pick minimum weight edge
        vector<bool> inMST(SIZE, false); // To check if a node is in MST
        priority_queue<Pair, vector<Pair>, greater<Pair>> pq; // Min-heap priority queue

        key[start] = 0; // Start from node `start`
        pq.push(make_pair(0, start));

        cout << "\nMinimum Spanning Tree edges:" << endl;

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            // Skip nodes already included in MST
            if (inMST[u]) continue;

            inMST[u] = true;

            // Explore all neighbors of u
            for (auto &neighbor : adjList[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                // If v is not in MST and weight of (u, v) is less than current key[v]
                if (!inMST[v] && weight < key[v]) {
                    key[v] = weight;
                    pq.push(make_pair(key[v], v));
                    cout << "Edge from " << airportNames[u] << " to " << airportNames[v]
                         << " with capacity: " << weight << " km" << endl;
                }
            }
        }
    }

    // Depth-First Search (DFS)
    void DFS(int start) {
        vector<bool> visited(SIZE, false);
        stack<int> stack;

        // Start DFS from the starting node
        stack.push(start);

        cout << "\nDFS starting from " << airportNames[start] << ":\n";
        cout << "Route exploration order:\n";

        vector<string> visitedOrder;

        while (!stack.empty()) {
            int node = stack.top();
            stack.pop();

            // If not visited, process the node
            if (!visited[node]) {
                visitedOrder.push_back(airportNames[node]);
                visited[node] = true;
            }

            // Add all adjacent nodes to the stack
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

        // Start BFS from the starting node
        queue.push(start);
        visited[start] = true;

        cout << "\nBFS starting from " << airportNames[start] << ":\n";
        cout << "Closest airports visit order:\n";

        vector<string> visitedOrder;

        while (!queue.empty()) {
            int node = queue.front();
            queue.pop();

            visitedOrder.push_back(airportNames[node]);

            // Add all unvisited adjacent nodes to the queue
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

int main() {
    // Airports: 0 - 9 mapped to major cities
    vector<string> airports = {
        "Los Angeles (LAX)", "New York (JFK)", "London (LHR)", "Tokyo (HND)",
        "Sydney (SYD)", "Dubai (DXB)", "Singapore (SIN)", "Paris (CDG)",
        "Frankfurt (FRA)", "Hong Kong (HKG)"
    };

    // Creates a vector of graph edges/weights
    vector<Edge> edges = {
        // (x, y, w) —> edge from x to y having weight w (distance in km)
        {0, 1, 3971}, {0, 2, 8759}, {1, 3, 10853}, {2, 4, 17015},
        {3, 5, 7980}, {4, 6, 6304}, {5, 7, 5248}, {6, 8, 10528},
        {7, 9, 9452}, {8, 9, 920}
    };

    // Creates graph
    Graph graph(edges, airports);

    // Prints adjacency list representation of the flight network
    graph.printGraph();

    // Perform DFS starting from Los Angeles
    graph.DFS(0);

    // Perform BFS starting from Los Angeles
    graph.BFS(0);

    // Calculate the shortest paths from Los Angeles (LAX) to other airports
    graph.dijkstra(0);

    // Calculate the Minimum Spanning Tree from Los Angeles (LAX)
    graph.minimumSpanningTree(0);

    return 0;
}
