#include "Algorithms.hpp"
#include <queue>
#include <sstream>
#include <algorithm>
#include <climits>
#include <limits> 
#include <unordered_set>
#include <iostream>
#include <stack>  

using namespace std;

namespace ariel {
    
    /**
     * 
     * This function checks if all vertices in a given graph (g) are reachable from the vertex with index 0 by using BFS algorithm.
     * 
     * @param graph The graph to check.
     * @return true if all vertices are reachable; otherwise, false.
     * 
     */
    bool Algorithms::isConnected(const Graph& graph) {
        int numVertices = graph.getNumVertices();       // Get the number of vertices in g
        
        // If the graph is empty, it is connected trivially
        if (numVertices == 0){
            return true;
        }
        
        vector<bool> visited(static_cast<size_t>(numVertices), false);      // Track visited vertices
        queue<int> queue;                                                       // Queue for BFS
        
        // Start BFS from vertex 0
        visited[0] = true;
        queue.push(0);
        int count = 1;
        
        // Perform BFS
        while (!queue.empty()) {
            int currentVertex = queue.front();
            queue.pop();
            
            // Check all adjacent vertices
            for (size_t i = 0; i < numVertices; i++) {
                if (graph.getAdjacencyMatrix()[static_cast<size_t>(currentVertex)][i] != 0 && !visited[i]) {
                    visited[i] = true;
                    queue.push(static_cast<int>(i));
                    count++;
                }
            }
        }
        
        // The graph is connected if all vertices were visited
        return count == numVertices;
    }
    

    /**
     * 
     * This function finds the shortest path from a start vertex to an end vertex by using Bellman-Ford algorithm.
     * 
     * @param graph The graph to search.
     * @param start The starting vertex.
     * @param end The destination vertex.
     * @return A string representing the path or "-1" if no path exists.
     * 
     */
    string Algorithms::shortestPath(const Graph& graph, int start, int end) {
        size_t numVertices = static_cast<size_t>(graph.getNumVertices());
        vector<int> distances(numVertices, numeric_limits<int>::max());
        vector<int> predecessors(numVertices, -1);
            
        // Initialize the distance to the start vertex.
        distances[static_cast<size_t>(start)] = 0;

        // Relax edges repeatedly n-1 times according to BF algorithm
        for (size_t i = 0; i < numVertices - 1; ++i) {
            for (size_t u = 0; u < numVertices; ++u) {
                for (size_t v = 0; v < numVertices; ++v) {
                    int weight = graph.getAdjacencyMatrix()[u][v];
                    if (weight != 0 && distances[u] != numeric_limits<int>::max() &&
                        distances[u] + weight < distances[v]) {
                        distances[v] = distances[u] + weight;
                        predecessors[v] = static_cast<int>(u);
                    }
                }
            }
        }

        // Check for negative weight cycles
        for (size_t u = 0; u < numVertices; ++u) {
            for (size_t v = 0; v < numVertices; ++v) {
                int weight = graph.getAdjacencyMatrix()[u][v];
                if (weight != 0 && distances[u] != numeric_limits<int>::max() && 
                    distances[u] + weight < distances[v]) {
                    cout << "Graph contains a negative weight cycle" << endl;
                    return "Graph contains a negative weight cycle";
                }
            }
        }

        // Build the path from the end vertex to the start using predecessors
        if (distances[static_cast<size_t>(end)] == numeric_limits<int>::max()) {
            return "-1";  // No path found
        }

        vector<int> path;
        for (int at = end; at != -1; at = predecessors[static_cast<size_t>(at)]) {  // Safe cast for vector access
            path.push_back(at);
        }
        reverse(path.begin(), path.end());

        stringstream pathStream;
        for (size_t i = 0; i < path.size(); i++) {
            if (i > 0) pathStream << "->";
            pathStream << path[i];
        }

        return pathStream.str();
    }


    /**
     * 
     * This function checks if the graph contains a cycle.
     * 
     * @param graph The graph.
     * @return true if a cycle is found; otherwise, false.
     * @note see also the auxiliary functions hasCycle, dfs_cycle and printCycle in the private section below.
     * 
     */
    bool Algorithms::isContainsCycle(const Graph& g) {
        if (hasCycle(g)) {
        cout << "1" << endl;
        return true;
        } else {
            cout << "0" << endl;
            return false;
        }
    }

    

     /**
     * 
     * Determines if the graph is bipartite.
     * 
     * @param graph The graph.
     * @return A string representing the bipartite sets or "0" if not bipartite.
     * @note see also the auxiliary function bfs_bipartite in the private section below.
     * 
     */
    string Algorithms::isBipartite(const Graph& g) {
        const auto& adj = g.getAdjacencyMatrix();
        vector<int> color(static_cast<size_t>(g.getNumVertices()), -1);

        for (int v = 0; v < g.getNumVertices(); ++v) {
            if (color[static_cast<size_t>(v)] == -1) {
                if (!bfs_bipartite(g, v, color)) {
                    return "0";
                }
            }
        }

        vector<int> setA, setB;
        for (int v = 0; v < g.getNumVertices(); ++v) {
            if (color[static_cast<size_t>(v)] == 0) {
                setA.push_back(v);
            } else {
                setB.push_back(v);
            }
        }

        stringstream ss;
        ss << "The graph is bipartite: A={";
        for (size_t i = 0; i < setA.size(); ++i) {
            ss << setA[i];
            if (i < setA.size() - 1) {
                ss << ", ";
            }
        }
        ss << "}, B={";
        for (size_t i = 0; i < setB.size(); ++i) {
            ss << setB[i];
            if (i < setB.size() - 1) {
                ss << ", ";
            }
        }
        ss << "}";
        return ss.str();
    }

    
    /**
     * 
     * This function checks for negative weight cycles.
     * 
     * @param graph The graph.
     * @return true if the graph has a negative weight cycle; otherwise, false.
     * @note see also the auxiliary function detectNegativeCycle in the private section below.
     * 
     */
    bool Algorithms::negativeCycle(const Graph& g) {
        return detectNegativeCycle(g);
    }



    /*********************************************/
    ///             PRIVATE SECTION             ///
    /*********************************************/

    /**
     * 
     * This function detects if the graph contains any cycle.
     * 
     * @param graph The graph to check.
     * @return true if the graph contains a cycle; otherwise, false.
     * @note this is an auxiliary function for bool Algorithms::isContainsCycle(const Graph& g) [see above]
     * 
     **/
    bool Algorithms::hasCycle(const Graph& g) {
       size_t numVertices = static_cast<size_t>(g.getNumVertices());
        vector<bool> visited(numVertices, false);
        vector<int> parent(numVertices, -1);    // Parent vector for tracking the path of vertices

        // Check each vertex for cycles
        for (size_t v = 0; v < numVertices; ++v) {
            if (!visited[v] && dfs_cycle(g, static_cast<int>(v), visited, parent, -1)) {
                return true;
            }
        }
        return false;
    }



    /**
     * 
     * An auxiliary function for cycle detection using DFS algorithm.
     * 
     * @param graph The graph.
     * @param v The current vertex.
     * @param visited Array to keep track of visited vertices.
     * @param parent Array to store the parent of each vertex.
     * @param prev The previous vertex.
     * @return true if a cycle is found; otherwise, false.
     * @note this is an auxiliary function for the hasCycle [see above].
     * 
     **/
    bool Algorithms::dfs_cycle(const Graph& g, int v, vector<bool>& visited, vector<int>& parent, int prev) {
        visited[static_cast<size_t>(v)] = true;
        const auto& adj = g.getAdjacencyMatrix();

        // Explore each adjacent vertex
        for (size_t i = 0; i < adj[static_cast<size_t>(v)].size(); ++i) {
            if (adj[static_cast<size_t>(v)][i] != 0) {
                if (!visited[i]) {
                    parent[i] = v;
                    if (dfs_cycle(g, static_cast<int>(i), visited, parent, v)) {
                        return true;
                    }
                } else if (static_cast<int>(i) != prev) {
                    printCycle(static_cast<int>(i), v, parent);
                    return true;
                }
            }
        }

        return false;
    }



    /**
     * This function prints the cycle found in the graph (if found).
     * 
     * @param start The starting vertex of the cycle.
     * @param end The ending vertex of the cycle.
     * @param parent The array containing the parent of each vertex.
     * @note this is an auxiliary function for the dfs_cycle function [see above].
     * 
     **/
    void Algorithms::printCycle(int start, int end, const vector<int>& parent) {
        vector<int> cycle;
        int current = end;
        cycle.push_back(current);

        // Trace back the cycle path
        while (current != start) {
            current = parent[static_cast<size_t>(current)];
            cycle.push_back(current);
        }

        reverse(cycle.begin(), cycle.end());

        cout << "Cycle found: ";
        for (size_t i = 0; i < cycle.size(); ++i) {
            if (i != 0) cout << "->";
            cout << cycle[i];
        }
        cout << "->" << cycle[0] << endl;  // Print the start vertex at the end to complete the cycle
    }



    /**
     * 
     * An auxiliary function for BFS used in bipartite checking.
     * 
     * @param graph The graph.
     * @param start The starting vertex for BFS.
     * @param color Array to store the color of each vertex.
     * @return true if successful; otherwise, false.
     * @note this is an auxiliary function for string Algorithms::isBipartite(const Graph& g) [see above].
     */
    bool Algorithms::bfs_bipartite(const Graph& g, int start, vector<int>& color) {
        queue<int> q;
        q.push(start);
        color[static_cast<size_t>(start)] = 0;

        while (!q.empty()) {
            int v = q.front();
            q.pop();

            for (int w = 0; w < g.getNumVertices(); ++w) {
                if (g.getAdjacencyMatrix()[static_cast<size_t>(v)][static_cast<size_t>(w)] != 0) {
                    if (color[static_cast<size_t>(w)] == -1) {
                        color[static_cast<size_t>(w)] = 1 - color[static_cast<size_t>(v)];
                        q.push(w);
                    } else if (color[static_cast<size_t>(w)] == color[static_cast<size_t>(v)]) {
                        return false;
                    }
                }
            }
        }
        return true;
    }



     /**
     * This auxiliary function detects negative weight cycles in the graph using Bellman-Ford algorithm.
     * 
     * @param graph The graph.
     * @return true if a negative cycle is detected; otherwise, false.
     * @note this is an auxiliary function for bool Algorithms::negativeCycle(const Graph& g) [see above]
     * 
     */
    bool Algorithms::detectNegativeCycle(const Graph& g) {
        const auto& adj = g.getAdjacencyMatrix();
        size_t numVertices = static_cast<size_t>(g.getNumVertices());

        if (numVertices == 0) return false;

        vector<int> dist(numVertices, INT_MAX);

        for (size_t src = 0; src < numVertices; ++src) {
            if (dist[src] == INT_MAX) {
                dist[src] = 0;

                for (int i = 0; i < g.getNumVertices() - 1; i++) {
                    bool relaxed = false;
                    for (size_t u = 0; u < g.getNumVertices(); u++) {
                        for (size_t v = 0; v < g.getNumVertices(); v++) {
                            if (adj[u][v] != 0 && dist[u] != INT_MAX && dist[u] + adj[u][v] < dist[v]) {
                                dist[v] = dist[u] + adj[u][v];
                                relaxed = true;     
                            }
                        }
                    }
                    if (!relaxed) break;
                }

                for (size_t u = 0; u < g.getNumVertices(); u++) {
                    for (size_t v = 0; v < g.getNumVertices(); v++) {
                        if (adj[u][v] != 0 && dist[u] != INT_MAX && dist[u] + adj[u][v] < dist[v]) {
                            return true;  // Negative cycle detected
                        }
                    }
                }
            }
        }

        return false;  // No negative cycle in the graph
    }



    /*********************************************/
    ///              TO BE DELETED              ///
    /*********************************************/

    // bool Algorithms::isBipartiteUtil(int v, const vector<vector<int>>& adj, vector<int>& color, vector<bool>& visited) {
    //     std::size_t v_index = static_cast<std::size_t>(v);
    //     queue<int> q;
    //     q.push(v);
    //     color[v_index] = 0; // Start coloring with 0
    //     visited[v_index] = true;

    //     while (!q.empty()) {
    //         int u = q.front(); q.pop();
    //         std::size_t u_index = static_cast<std::size_t>(u);

    //         for (std::size_t i = 0; i < adj[u_index].size(); i++) {
    //             if (adj[u_index][i] != 0 && !visited[i]) {
    //                 color[i] = 1 - color[u_index]; // Alternate color
    //                 visited[i] = true;
    //                 q.push(static_cast<int>(i));
    //             } else if (adj[u_index][i] != 0 && color[i] == color[u_index]) {
    //                 return false;
    //             }
    //         }
    //     }
    //     return true;
    // }
}