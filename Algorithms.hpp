#ifndef ALGORITHMS_H
#define ALGORITHMS_H

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
            static bool isConnected(const Graph& g);

            /**
             * This function finds the shortest path between two vertices using Bellman-Ford algorithm.
             * 
             * @param g The graph.
             * @param start The starting vertex.
             * @param end The ending vertex.
             * @return A string representation of the shortest path or "-1" if no path exists.
             */
            static string shortestPath(const Graph& g, int start, int end);

            /**
             * This function finds if the graph contains any cycle.
             * 
             * @param g The graph.
             * @return true if the graph contains at least one cycle, false otherwise.
             */
            static bool isContainsCycle(const Graph& g);

            /**
             * This function finds if the graph is bipartite.
             * 
             * @param g The graph.
             * @return A string representing the two sets of vertices if the graph is bipartite, "0" otherwise.
             */
            static string isBipartite(const Graph& g);

            /**
             * This function finds a presence of any negative weight cycles in the graph.
             * 
             * @param g The graph.
             * @return true if there is at least one negative weight cycle, false otherwise.
             */
            static bool negativeCycle(const Graph& g);

        private:
            /**
             * Auxiliary function to detect cycles in the graph.
             * 
             * @param g The graph.
             * @return true if a cycle is found, false otherwise.
             */
            static bool hasCycle(const Graph& g);

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
            static bool dfs_cycle(const Graph& g, int v, vector<bool>& visited, vector<int>& parent, int prev);

            /**
             * Auxiliary function to print the cycle found in the graph.
             * 
             * @param start The start vertex of the cycle.
             * @param end The end vertex of the cycle.
             * @param parent Vector containing parent relations to reconstruct the cycle path.
             */
            static void printCycle(int start, int end, const vector<int>& parent);

            /**
             * Auxiliary function using BFS algorithm to check if the graph is bipartite.
             * 
             * @param g The graph.
             * @param start The starting vertex for BFS.
             * @param color Vector to store colors of vertices.
             * @return true if the graph can be colored using two colors without any adjacent vertices having the same color, false otherwise.
             */
            static bool bfs_bipartite(const Graph& g, int start, vector<int>& color);

            /**
             * Auxiliary function using Bellman-Ford algorithm to detect negative weight cycles in the graph.
             * 
             * @param g The graph.
             * @return true if a negative weight cycle is detected, false otherwise.
             */
            static bool detectNegativeCycle(const Graph& g);
    };
}

#endif
