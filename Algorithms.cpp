// ID: 200661775
// Email: origoldbsc@gmail.com

#include "Algorithms.hpp"
#include <queue>
#include <sstream>
#include <algorithm>
#include <climits>
#include <unordered_set>
#include <iostream>
#include <stack>  

using namespace std;

namespace ariel {
    
    /**
     * @brief This method checks if all vertices in a given graph are reachable from the vertex with index 0 by using BFS algorithm.
     *
     * @param graph The graph to check.
     * @return true if all vertices are reachable; otherwise, false.
     */
    bool Algorithms::isConnected(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();

        // Check if the graph is trivially connected
        if (numVertices == 1) 
        {
            return true;
        }

        vector<bool> visited(numVertices, false);   // Initialize a vector with numVertices elements and sets each of them to false
        bfs(graph, 0, visited);                     // Perform BFS starting from vertex 0

        // If any vertex was not visited, the graph is not connected
        for (size_t i = 0; i < numVertices; i++) 
        {
            if (!visited[i]) 
            {
                return false; 
            }
        }

        return true;
    }


    /**
     * This function checks if a given graph is strongly connected by applying BFS from each vertex.
     *
     * @param graph The graph to check.
     * @return true if the graph is strongly connected; otherwise, false.
     */
    bool Algorithms::isStronglyConnected(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();

        // The graph is trivially strongly connected
        if (numVertices == 1) 
        {
            return true;
        }

        // Perform BFS from each vertex
        for (size_t startVertex = 0; startVertex < numVertices; startVertex++) {
            vector<bool> visited(numVertices, false);       // Initiate a vector to keep tracking the visited vertices
            bfs(graph, startVertex, visited);

            // If any vertex was not visited, the graph is not strongly connected
            for (size_t i = 0; i < numVertices; i++) 
            {
                if (!visited[i]) 
                {
                    return false; 
                }
            }
        }

        return true;
    }


    /**
     * @brief This method finds the shortest path between two vertices in a given graph.
     *
     * @note: The method uses a strategy to choose the appropriate algorithm based on the properties of the graph (weighted/unweighted, containts negative edges or not).
     * It uses BFS for unweighted graphs (time complexity: O(|V|*|E|)), Bellman-Ford for graphs with negative weights (time complexity: O(|V|*|E|)), 
     * or Dijkstra's for non-negative weighted graphs (time complexity: O(|V|*|V|)), and thus does not waste resources.
     *
     * @param graph The graph.
     * @param start The start vertex.
     * @param end The end vertex.
     * @return A string representing the path or a message if the path doesn't exist.
     */
    string Algorithms::shortestPath(Graph& graph, size_t start, size_t end) 
    {
        size_t numVertices = graph.getNumVertices();

        // Error checking no. 1
        if (start >= numVertices || end >= numVertices) 
        {
            return "Invalid start or end vertex";
        }

        // Error checking no. 2
        if (start == end) 
        {
            return "No path exists between a vertex and itself";
        }

        // Check if the graph is unweighted
        bool isUnweighted = isGraphUnweighted(graph);

        // Choose the algorithm based on the graph type
        if (isUnweighted) 
        {
            return bfsShortestPath(graph, start, end);
        } 
        
        bool hasNegativeEdges = hasNegativeEdgesInGraph(graph);     // Check if the graph has negative edges to choose the relevant algorithm

        if (hasNegativeEdges) 
        {
            return bellmanFordShortestPath(graph, start, end);
        } 
    
        return dijkstraShortestPath(graph, start, end);        
        
        
    }

     /**
     * @brief This method checks if there is a cycle in the graph.
     *
     * This method uses DFS to find a back edge, which indicates a cycle.
     *
     * @param graph The graph to check.
     * @return A string representing the cycle if one exists; otherwise, "0".
     */
    string Algorithms::isContainsCycle(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();
        vector<bool> visited(numVertices, false);
        vector<size_t> parent(numVertices, INT_MAX);

        for (size_t vertex_v = 0; vertex_v < numVertices; vertex_v++) 
        {
            if (!visited[vertex_v]) 
            {
                string cycle = dfs_cycle(graph, vertex_v, visited, parent, vertex_v);
                if (!cycle.empty()) 
                {
                    return cycle;
                }
            }
        }

        return "0";
    }
    

    /**
     * @brief This function checks if a graph is bipartite.
     *
     * @note A graph is bipartite if it can be colored using two colors such that no two adjacent vertices with the same color.
     * This function uses DFS to color the graph.
     *
     * @param graph The graph to check.
     * @return A string indicating if the graph is bipartite; otherwise, an error message.
     */
    string Algorithms::isBipartite(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();
        vector<int> color(numVertices, -1);             // Initiate a vector with a length of the number of vertices and uncolor (-1) all of them

        for (size_t i = 0; i < numVertices; ++i) 
        {
            // If a vertex is uncolored, start a new DFS
            if (color[i] == -1) 
            {  
                if (!dfsCheck(graph, i, color, 0)) 
                {
                    return "The graph is not bipartite";
                }
            }
        }

        // Saperate the vertices based on their colors to two sets
        vector<size_t> setA;
        vector<size_t> setB;

        for (size_t i = 0; i < numVertices; i++) 
        {
            if (color[i] == 0) 
            {
                setA.push_back(i);
            }
            else
            {
                setB.push_back(i);
            }
        }
        
        // Present the results as string
        string result = "The graph is bipartite: A=" + buildSet(setA) + ", B=" + buildSet(setB);
        return result;
    }


    /**
     * @brief This method checks for a negative cycle in the graph.
     *
     * This method checks for negative cycles using a variant of the Bellman-Ford algorithm, which is capable of detecting such cycles.
     *
     * @param graph The graph to check.
     * @return A string represents a negative cycle if exists; otherwise, "No negative cycle exists".
     */
    string Algorithms::negativeCycle(Graph& graph)
    {
        size_t numVertices = graph.getNumVertices();        // A variable to store the number of vertices in t he graph   
        vector<int> distance(numVertices, 0);               // Initialize distance vector with zeros
        vector<size_t> parent(numVertices, INT_MAX);        // Initialize parent vector with INT_MAX. This vector will use us to build paths and detect cycles

        // Part 1: Check for negative self-loops
        for (size_t vertex_u = 0; vertex_u < numVertices; vertex_u++) 
        {
            if (graph.getAdjacencyMatrix()[vertex_u][vertex_u] < 0) 
            {
                return to_string(vertex_u) + "->" + to_string(vertex_u);
            }
        }

        // Part 2: Check for negative cycles between neighboring vertices with different edge weights
        for (size_t vertex_u = 0; vertex_u < numVertices; vertex_u++) 
        {
            for (size_t vertex_v = 0; vertex_v < numVertices; vertex_v++) 
            {
                // Check if there are differing weights in the edges u->v and v->u that sum to a negative value
                if (vertex_u != vertex_v && graph.getAdjacencyMatrix()[vertex_u][vertex_v] != 0 && graph.getAdjacencyMatrix()[vertex_v][vertex_u] != 0 &&
                    graph.getAdjacencyMatrix()[vertex_u][vertex_v] != graph.getAdjacencyMatrix()[vertex_v][vertex_u]) 
                    {
                    int weight = graph.getAdjacencyMatrix()[vertex_u][vertex_v] + graph.getAdjacencyMatrix()[vertex_v][vertex_u];
                    if (weight < 0) 
                    {
                        return to_string(vertex_u) + "->" + to_string(vertex_v) + "->" + to_string(vertex_u);
                    }
                }
            }
        }

        // Part 3: Check for negative cycles with with path involves more than one edge
        string result = detectNegativeCycle(graph);
        if (result != "No negative cycle exists") 
        {
            return result;
        }

        return "No negative cycle exists";
    }



    /*********************************************/
    ///             PRIVATE SECTION             ///
    /*********************************************/

    /**
     * @brief This auxiliary function preforms BFS on the graph starting from the given vertex.
     *
     * @param graph The graph to perform BFS on.
     * @param startVertex The starting vertex for BFS.
     * @param visited A vector to track visited vertices.
     */
    void Algorithms::bfs(Graph& graph, size_t startVertex, vector<bool>& visited) {
        queue<size_t> queue;
        visited[startVertex] = true;            // Mark the startVertex as visited
        queue.push(startVertex);                // Add the startVertex to the queue 


        // According to the BFS algorithm we will continue the process until there are no more vertices to explore
        while (!queue.empty()) 
        {
            size_t currentVertex = queue.front();                         // Store the front vertex and remove it from the queue
            queue.pop();

            for (size_t i = 0; i < graph.getNumVertices(); i++)           // Iterate over all vertices in the graph to find all neighbors vertices
            {
                // Check if there is an edge from currentVertex to vertex i, and it is still "true" (not visited yet)
                if (graph.getAdjacencyMatrix()[currentVertex][i] != 0 && !visited[i]) 
                {
                    visited[i] = true;
                    queue.push(i);
                }
            }
        }
    }


    /**
     * @brief This auxiliary function finds the shortest path using BFS for unweighted graphs.
     *
     * This function returns the shortest path by building the path itself.
     *
     * @param graph The graph.
     * @param start The start vertex.
     * @param end The end vertex.
     * @return A string representing the shortest path.
     */
    string Algorithms::bfsShortestPath(Graph& graph, size_t start, size_t end) 
    {
        size_t numVertices = graph.getNumVertices();
        vector<bool> visited(numVertices, false);
        vector<size_t> parent(numVertices, INT_MAX);

        queue<size_t> queue;
        visited[start] = true;
        queue.push(start);

        while (!queue.empty()) 
        {
            size_t currentVertex = queue.front();
            queue.pop();

            if (currentVertex == end) 
            {
                break;
            }

            for (size_t i = 0; i < numVertices; i++) 
            {
                if (graph.getAdjacencyMatrix()[currentVertex][i] != 0 && !visited[i]) 
                {
                    visited[i] = true;
                    queue.push(i);
                    parent[i] = currentVertex;
                }
            }
        }

        if (!visited[end]) 
        {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        // Build the shortest path
        string path = buildPath(start, end, parent);

        return path;
    }



    /**
     * @brief This auxiliary function finds the shortest path in a graph with possible negative weights using Bellman-Ford algorithm.
     *
     * This algorithm also checks for negative cycles and if detected, returns an error message.
     *
     * @param graph The graph.
     * @param start The starting vertex.
     * @param end The destination vertex.
     * @return A string representing the path or an error message if a negative cycle is detected.
     */
    string Algorithms::bellmanFordShortestPath(Graph& graph, size_t start, size_t end)
    {
        size_t numVertices = graph.getNumVertices();
        vector<int> distance(numVertices, INT_MAX);
        vector<size_t> parent(numVertices, INT_MAX);
        distance[start] = 0;

        // Relax edges V-1 times
        for (int i = 0; i < numVertices - 1; i++)
        {
            relaxEdges(graph, distance, parent);
        }

        // Check for negative cycle on the V-th iteration
        if (hasNegativeCycle(graph, distance, parent))
        {
            return "Graph contains a negative cycle";
        }

        // Check if the distance from start to end remains infinity
        if (distance[end] == INT_MAX)
        {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        // Build the shortest path
        string path = buildPath(start, end, parent);
        return path;
    }
    

    /**
     * @brief This auxiliary function relaxes the edges in the graph during the shortest path process.
     *
     * @param graph The graph on which the edges are relaxed.
     * @param distance The vector storing the distances from the source vertex to each vertex.
     * @param parent The vector storing the parent of each vertex in the shortest path tree.
     */
    void Algorithms::relaxEdges(Graph& graph, vector<int>& distance, vector<size_t>& parent)
    {
        size_t numVertices = graph.getNumVertices();
        for (size_t vertex_u = 0; vertex_u < numVertices; vertex_u++)
        {
            for (size_t vertex_v = 0; vertex_v < numVertices; vertex_v++)
            {
                int weight = graph.getAdjacencyMatrix()[vertex_u][vertex_v];
                // Check if there is an edge u->v and if the current path through u is shorter
                if (canRelax(graph, vertex_u, vertex_v, weight, distance, parent))
                {
                    distance[vertex_v] = distance[vertex_u] + weight; // Update the distance to vertex v through u
                    parent[vertex_v] = vertex_u; // Record u as the parent of v
                }
            }
        }
    }

    /**
     * @brief This auxiliary function checks if an edge can be relaxed.
     *
     * @param graph The graph containing the edge.
     * @param vertex_u The source vertex of the edge.
     * @param vertex_v The destination vertex of the edge.
     * @param weight The weight of the edge.
     * @param distance The vector storing the distances from the source vertex to each vertex.
     * @param parent The vector storing the parent of each vertex in the shortest path tree.
     * @return true if the edge can be relaxed, false otherwise.
     */
    bool Algorithms::canRelax(Graph& graph, size_t vertex_u, size_t vertex_v, int weight, vector<int>& distance, vector<size_t>& parent)
    {
        if (weight != 0 && distance[vertex_u] != INT_MAX && distance[vertex_u] + weight < distance[vertex_v])
        {
            // Check if the graph is directed or undirected when v is not the parent of u
            if (graph.isGraphDirected() || (!graph.isGraphDirected() && parent[vertex_u] != vertex_v))
            {
                return true;
            }
        }
        return false;
    }


    /**
     * @brief This auxiliary function checks if the graph contains a negative cycle.
     *
     * @param graph The graph to check for negative cycles.
     * @param distance The vector storing the distances from the source vertex to each vertex.
     * @param parent The vector storing the parent of each vertex in the shortest path tree.
     * @return true if the graph contains a negative cycle, false otherwise.
     */
    bool Algorithms::hasNegativeCycle(Graph& graph, vector<int>& distance, vector<size_t>& parent)
    {
        size_t numVertices = graph.getNumVertices();
        for (size_t vertex_u = 0; vertex_u < numVertices; vertex_u++)
        {
            for (size_t vertex_v = 0; vertex_v < numVertices; vertex_v++)
            {
                int weight = graph.getAdjacencyMatrix()[vertex_u][vertex_v];
                // If an additional relaxation is possible, a negative cycle exists
                if (canRelax(graph, vertex_u, vertex_v, weight, distance, parent))
                {
                    return true;
                }
            }
        }
        return false;
    }


    /**
     * @brief This auxiliary function finds the shortest path between two vertices using Dijkstra's algorithm.
     * 
     * Dijkstra's algorithm is used for finding the shortest path from a single source vertex
     * to a single destination vertex in a graph with non-negative edge weights.
     *
     * @param graph The graph.
     * @param start The index of the start vertex.
     * @param end The index of the end vertex.
     * @return A string representing the shortest path or a message indicating no path exists.
     */
    string Algorithms::dijkstraShortestPath(Graph& graph, size_t start, size_t end) 
    {
        size_t numVertices = graph.getNumVertices();    // A variable to store the number of vertices in the graph
        vector<int> distance(numVertices, INT_MAX);     // Initialize distance vector to infinity
        vector<size_t> parent(numVertices, INT_MAX);    // Initialize parent vector for path building
        vector<bool> visited(numVertices, false);       // Initialize visited vector to Keep track of visited nodes to avoid revisiting

        distance[start] = 0;

        // Find the unvisited vertex with the smallest distance
        for (size_t i = 0; i < numVertices; i++) 
        {
            size_t minVertex = findMinDistanceVertex(distance, visited);

            // If no vertex is found break from the loop, since the start is not connected to any vertex
            if (minVertex == INT_MAX) 
            {
                break;
            }

            visited[minVertex] = true;      // Mark the currect vertex as visited

            for (size_t vertex_v = 0; vertex_v < numVertices; vertex_v++) 
            {
                int weight = graph.getAdjacencyMatrix()[minVertex][vertex_v];                  // A variable to store the weight of the edge minVertex->v
                // Relax the edge
                if (weight != 0 && !visited[vertex_v] && distance[minVertex] != INT_MAX &&
                distance[minVertex] + weight < distance[vertex_v]) 
                {
                    distance[vertex_v] = distance[minVertex] + weight;     // Update the distance to vertex v
                    parent[vertex_v] = minVertex;                          // Set minVertex as the parent of v
                }
            }
        }

        // Check if the end point is reachable
        if (distance[end] == INT_MAX) 
        {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        // Build the shortest path
        string path = buildPath(start, end, parent);

        return path;
    }


    /**
     * @brief This auxiliary function finds the vertex with the minimum distance that has not been visited yet.
     * 
     * This is a function used by Dijkstra's algorithm to select the next vertex to visit.
     *
     * @param distance A vector of distances from the start vertex.
     * @param visited A vector indicating whether each vertex has been visited.
     * @return The index of the vertex with the smallest distance that has not been visited.
     */
    size_t Algorithms::findMinDistanceVertex(vector<int>& distance, vector<bool>& visited) {
        size_t minVertex = INT_MAX;
        int minDistance = INT_MAX;          // We will start with the largest possible distance

        // Iterate through all vertices
        for (size_t vertex_v = 0; vertex_v < distance.size(); vertex_v++) 
        {
            // Update the vertex with the minimum distance that has not been visited
            if (!visited[vertex_v] && distance[vertex_v] < minDistance) 
            {
                minVertex = vertex_v;
                minDistance = distance[vertex_v];
            }
        }

        return minVertex;
    }


    /**
     * @brief This auxiliary function builds the path from the start vertex to the end vertex.
     *
     * @param start The start vertex of the path.
     * @param end The end vertex of the path.
     * @param parent The vector containing each vertex's parent in the path.
     * @return A string representing the path or an empty string if no path exists.
     */
    string Algorithms::buildPath(size_t start, size_t end, vector<size_t>& parent) {
        string path;
        size_t current = end;

        // If no path exists from start to end, return an empty string
        if (parent[current] == INT_MAX) 
        {
            return "";
        }

        // Traverse backwards from end to start using the parent vector
        while (current != start)
        {
            path.insert(0, "->" + to_string(current));
            current = parent[current];
        }

        // Add the start vertex to the path
        path = to_string(start) + path;

        return path;
    }

    /**
     * @brief This auxiliary function uses DFS to detect a cycle in the graph starting from a given vertex.
     *
     * @param graph The graph.
     * @param vertex The starting vertex for DFS.
     * @param visited A vector to keep track of visited vertices.
     * @param parent A vector to keep track of the parents of each vertex.
     * @param start The original starting vertex for cycle detection.
     * @return A string representing the cycle or an empty string if no cycle is found.
     */
    string Algorithms::dfs_cycle(Graph& graph, size_t vertex, vector<bool>& visited, vector<size_t>& parent, size_t start) 
    {
        visited[vertex] = true;

        for (size_t i = 0; i < graph.getNumVertices(); i++) 
        {
            if (graph.getAdjacencyMatrix()[vertex][i] != 0)      // Check if there is an edge v->i
            {
                if (!visited[i]) 
                {
                    parent[i] = vertex;                                                  // Set the parent of vertex i to v
                    string cycle = dfs_cycle(graph, i, visited, parent, start);     // Recurse into vertex i
                    
                    // Check if a cycle is detected in recursion, and if so - return it
                    if (!cycle.empty()) 
                    {
                        return cycle;
                    }
                } 
                else if (i != parent[vertex] && i == start) 
                {
                    // Found a cycle returning to the start vertex
                    string cycle = to_string(i);
                    size_t current = vertex;
                    while (current != i)
                    {
                        cycle.insert(0, to_string(current) + "->");
                        current = parent[current];
                    }
                    cycle.insert(0, to_string(i) + "->");
                    return cycle;
                }
            }
        }

        // Return empty if no cycle is found
        return "";
    }


    /**
     * @brief This auxiliary function checks if a graph is bipartite using DFS by trying to color it with 2-colors.
     * 
     * A graph is bipartite if it can be colored using two colors such that no two adjacent vertices
     * share the same color.
     *
     * @param graph The graph to check.
     * @param currectVertex The vertex to start coloring.
     * @param colorVec A vector storing the color of each vertex, initialized to -1 (uncolored).
     * @param color The current color to use (0 or 1).
     * @return true if the graph can be colored bipartitely; otherwise, false.
     */
    bool Algorithms::dfsCheck(Graph& graph, size_t currectVertex, vector<int>& colorVec, int color) 
    {
        if (colorVec[currectVertex] != -1) 
        {
            return colorVec[currectVertex] == color;       // Check if the currect vertex is colored correctly
        }

        // Color the current vertex
        colorVec[currectVertex] = color;

        // Check all adjacent vertices for a valid coloring
        for (size_t vertex_v = 0; vertex_v < graph.getNumVertices(); vertex_v++) 
        {
            // There's an edge from u to v
            if (graph.getAdjacencyMatrix()[currectVertex][vertex_v] != 0)      
            {  
                // Color the adjacent vertex with the opposite color, and return false is contradiction discovered
                if (!dfsCheck(graph, vertex_v, colorVec, 1 - color)) 
                {
                    return false;
                }
            }

            // Check reverse direction for undirected graphs
            if (graph.getAdjacencyMatrix()[vertex_v][currectVertex] != 0) 
            {  
                // Color the adjacent vertex with the opposite color, and return false is contradiction discovered
                if (!dfsCheck(graph, vertex_v, colorVec, 1 - color)) 
                {
                    return false;
                }
            }
        }

        return true;
    }



    /**
     * @brief This auxiliary function builds a string representation of a set of vertices.
     *
     * @param set A vector containing the indices of vertices in the set.
     * @return A string representation of the set.
     */
    string Algorithms::buildSet(vector<size_t>& set) 
    {
        string result = "{";
        for (size_t i = 0; i < set.size(); i++) 
        {
            result += to_string(set[i]);
            if (i < set.size() - 1)             // We use this condition to determine whether to add "," to the string
            {
                result += ",";
            }
        }
        result += "}";
        return result;
    }



    /**
     * @brief This auxiliary function checks if a graph is unweighted.
     * 
     * This function checks if all the edges in the graph have weights of either 0 or 1,
     *
     * @param graph The graph to be checked.
     * @return true if the graph is unweighted, false otherwise.
     */
    bool Algorithms::isGraphUnweighted(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();

        // Iterate over all pairs of vertices
        for (size_t i = 0; i < numVertices; i++) 
        {
            for (size_t j = 0; j < numVertices; j++) 
            {
                // Check if the weight is neither 0 nor 1
                if (graph.getAdjacencyMatrix()[i][j] != 0 && graph.getAdjacencyMatrix()[i][j] != 1) 
                {
                    return false;
                }
            }
        }
        return true;
    }


    /**
     * @brief This auxiliary function checks if a graph contains any negative edges.
     *
     *
     * @param graph The graph to be checked.
     * @return true if there is at least one negative edge, false otherwise.
     */
    bool Algorithms::hasNegativeEdgesInGraph(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();

        // Iterate over all pairs of vertices
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) 
            {
                // Check if the weight of the edge is negative
                if (graph.getAdjacencyMatrix()[i][j] < 0) 
                {
                    return true;
                }
            }
        }
        return false;
    }
    

    /**
     * @brief This auxiliary function finds a negative cycle (if exists) in a graph using a modified Bellman-Ford algorithm.
     * 
     * This function attempts to detect a negative cycle by relaxing edges repeatedly and checking
     * for changes in the shortest path estimate that indicate a cycle.
     *
     * @param graph The graph in which to detect negative cycles.
     * @return A string describing the cycle if found, or a message indicating no cycle exists.
     */

    string Algorithms::detectNegativeCycle(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();
        vector<int> distance(numVertices, 0);
        vector<size_t> parent(numVertices, INT_MAX);

        // Check each vertex as a potential start point
        for (size_t start = 0; start < numVertices; start++) 
        {
            distance.assign(numVertices, INT_MAX);     // Reset distances to infinity
            parent.assign(numVertices, INT_MAX);       // Reset parent pointers
            distance[start] = 0;                       // Distance to self is zero

            // Relax edges for all vertices
            for (int i = 0; i < numVertices - 1; i++) 
            {
                relaxEdges(graph, distance, parent);
            }

            // Check for cycle on the last iteration
            if (hasNegativeCycle(graph, distance, parent)) 
            {
                return buildNegativeCycle(graph, distance, parent);
            }
        }

        return "No negative cycle exists";
    }

    /**
     * @brief This auxiliary function builds a negative cycle found in the graph.
     *
     * @param graph The graph containing the negative cycle.
     * @param distance The vector storing the distances from the source vertex to each vertex.
     * @param parent The vector storing the parent of each vertex in the shortest path tree.
     * @return A string representing the negative cycle.
     */
    string Algorithms::buildNegativeCycle(Graph& graph, vector<int>& distance, vector<size_t>& parent)
    {
        size_t numVertices = graph.getNumVertices();
        for (size_t vertex_u = 0; vertex_u < numVertices; vertex_u++)
        {
            for (size_t vertex_v = 0; vertex_v < numVertices; vertex_v++)
            {
                int weight = graph.getAdjacencyMatrix()[vertex_u][vertex_v];
                // A cycle is found if further relaxation is possible
                if (canRelax(graph, vertex_u, vertex_v, weight, distance, parent))
                {
                    size_t current = vertex_v;
                    vector<size_t> cycle;

                    // Trace back the cycle
                    while (current != INT_MAX)
                    {
                        cycle.push_back(current);
                        current = parent[current];
                        if (find(cycle.begin(), cycle.end(), current) != cycle.end())
                        {
                            cycle.push_back(current);
                            break;
                        }
                    }

                    string cycleString;
                    cycleString += to_string(cycle.back());
                    for (size_t i = cycle.size() - 2; i != static_cast<size_t>(-1); i--)
                    {
                        cycleString += "->" + to_string(cycle[i]);
                    }
                    return cycleString;
                }
            }
        }
        return "";
    }

}