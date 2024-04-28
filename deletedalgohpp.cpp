#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <string>
#include "Graph.hpp"

using namespace std;

namespace ariel {

    class Algorithms {
        
        public:
            /**
             * This function checks if all vertices in a graph are connected.
             * 
             * @param g The graph to check.
             * @return true if the graph is connected, false otherwise.
             */
            static bool isConnected(Graph& g);

            /**
             * This function finds the shortest path between two vertices using Bellman-Ford algorithm.
             * 
             * @param g The graph.
             * @param start The starting vertex.
             * @param end The ending vertex.
             * @return A string representation of the shortest path or "-1" if no path exists.
             */

            static string shortestPath(Graph& graph, size_t start, size_t end);

            /**
             * This function finds if the graph contains any cycle.
             * 
             * @param g The graph.
             * @return true if the graph contains at least one cycle, false otherwise.
             */
            static bool isContainsCycle(Graph& g);

            /**
             * This function finds if the graph is bipartite.
             * 
             * @param g The graph.
             * @return A string representing the two sets of vertices if the graph is bipartite, "0" otherwise.
             */
            static string isBipartite(Graph& g);

            /**
             * This function finds a presence of any negative weight cycles in the graph.
             * 
             * @param g The graph.
             * @return true if there is at least one negative weight cycle, false otherwise.
             */
            static bool negativeCycle(Graph& g);

        private:


            static string bfsShortestPath(Graph& graph, size_t start, size_t end);
            static string bellmanFordShortestPath(Graph& graph, size_t start, size_t end);
            static string dijkstraShortestPath(Graph& graph, size_t start, size_t end);

            /**
             * Auxiliary function to detect cycles in the graph.
             * 
             * @param g The graph.
             * @return true if a cycle is found, false otherwise.
             */
            static bool hasCycle(Graph& g);

            /**
             * Auxiliary function using DFS algorithm to detect cycles in the graph.
             * 
             * @param g The graph.
             * @param v The current vertex.
             * @param visited Vector to track visited vertices.
             * @param parent Vector to store parent vertices to trace back the cycle path.
             * @param prev The previous vertex in the DFS traversal.
             * @return true if a cycle is found, false otherwise.
             */
            static bool dfs_cycle(Graph& g, size_t v, vector<bool>& visited, vector<int>& parent, size_t prev);

            /**
             * Auxiliary function to print the cycle found in the graph.
             * 
             * @param start The start vertex of the cycle.
             * @param end The end vertex of the cycle.
             * @param parent Vector containing parent relations to reconstruct the cycle path.
             */
            static void printCycle(size_t start, size_t end, vector<int>& parent);

            /**
             * Auxiliary function using BFS algorithm to check if the graph is bipartite.
             * 
             * @param g The graph.
             * @param start The starting vertex for BFS.
             * @param color Vector to store colors of vertices.
             * @return true if the graph can be colored using two colors without any adjacent vertices having the same color, false otherwise.
             */
            static bool bfs_bipartite(Graph& g, size_t start, vector<int>& color);

            /**
             * Auxiliary function using Bellman-Ford algorithm to detect negative weight cycles in the graph.
             * 
             * @param g The graph.
             * @return true if a negative weight cycle is detected, false otherwise.
             */
            static bool detectNegativeCycle(Graph& g);


            static bool relaxEdges(vector<vector<int>>& adj, vector<int>& dist, std::size_t numVertices);

            static bool checkForNegativeCycle(vector<vector<int>>& adj, vector<int>& dist, std::size_t numVertices);


    };
}

#endif