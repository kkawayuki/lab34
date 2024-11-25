// COMSC-210 | lab 34 | Kent Kawashima
// IDE used: Visual Studio Code
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
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

        // Start DFS from the starting node
        stack.push(start);

        cout << "\nDFS starting from " << airportNames[start] << ":\n";
        cout << "Route exploration order from LA:\n";

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
        cout << "Closest airports relative to LA, ordered:\n";

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

    return 0;
}
