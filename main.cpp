// COMSC-210 | lab 34 | Kent Kawashima
// IDE used: Visual Studio Code
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
using namespace std;

const int SIZE = 10; // Updated size to include new nodes

struct Edge
{
    int src, dest, weight;
};

typedef pair<int, int> Pair; // Creates alias 'Pair' for the pair<int,int> datatype

class Graph
{
public:
    // a vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;

    // Graph Constructor
    Graph(vector<Edge> const &edges)
    {
        // resize the vector to hold SIZE elements of type vector<Edge>
        adjList.resize(SIZE);

        // add edges to the directed graph
        for (auto &edge : edges)
        {
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
    void printGraph()
    {
        cout << "Graph's adjacency list:" << endl;
        for (int i = 0; i < adjList.size(); i++)
        {
            cout << i << " --> ";
            for (Pair v : adjList[i])
                cout << "(" << v.first << ", " << v.second << ") ";
            cout << endl;
        }
    }

    // Depth-First Search (DFS)
    void DFS(int start)
    {
        vector<bool> visited(SIZE, false);
        stack<int> stack;

        // Start DFS from the starting node
        stack.push(start);

        cout << "DFS starting from vertex " << start << ":\n";

        while (!stack.empty())
        {
            int node = stack.top();
            stack.pop();

            // If not visited, process the node
            if (!visited[node])
            {
                cout << node << " ";
                visited[node] = true;
            }

            // Add all adjacent nodes to the stack
            for (auto &neighbor : adjList[node])
            {
                int next = neighbor.first;
                if (!visited[next])
                    stack.push(next);
            }
        }
        cout << endl;
    }

    // Breadth-First Search (BFS)
    void BFS(int start)
    {
        vector<bool> visited(SIZE, false);
        queue<int> queue;

        // Start BFS from the starting node
        queue.push(start);
        visited[start] = true;

        cout << "BFS starting from vertex " << start << ":\n";

        while (!queue.empty())
        {
            int node = queue.front();
            queue.pop();

            cout << node << " ";

            // Add all unvisited adjacent nodes to the queue
            for (auto &neighbor : adjList[node])
            {
                int next = neighbor.first;
                if (!visited[next])
                {
                    queue.push(next);
                    visited[next] = true;
                }
            }
        }
        cout << endl;
    }
};

int main()
{
    // Creates a vector of graph edges/weights
    vector<Edge> edges = {
        // (x, y, w) —> edge from x to y having weight w
        {0, 1, 12},
        {0, 2, 9},
        {1, 3, 7},
        {2, 4, 14},
        {3, 5, 11},
        {4, 6, 8},
        {5, 7, 10},
        {6, 8, 13},
        {7, 9, 15},
        {8, 9, 6}};

    // Creates graph
    Graph graph(edges);

    // Prints adjacency list representation of graph
    graph.printGraph();

    // Perform DFS starting from vertex 0
    graph.DFS(0);

    // Perform BFS starting from vertex 0
    graph.BFS(0);

    return 0;
}
