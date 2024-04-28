#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <string>
#include "Graph.hpp"

using namespace std;

namespace ariel {

    class Algorithms {
        
        public:
            static bool isConnected(Graph& g);
            static bool isStronglyConnected(Graph& graph);
            static string shortestPath(Graph& graph, size_t start, size_t end);
            static string isContainsCycle(Graph& graph);
            static string isBipartite(Graph& graph);
            static string negativeCycle(Graph& graph);

        private:
            static size_t bfs(Graph& graph, size_t startVertex, vector<bool>& visited);
            static string bfsShortestPath(Graph& graph, size_t start, size_t end);
            static string bellmanFordShortestPath(Graph& graph, size_t start, size_t end);
            static string dijkstraShortestPath(Graph& graph, size_t start, size_t end);
            static string dfs_cycle(Graph& graph, size_t v, vector<bool>& visited, vector<size_t>& parent, size_t start);
            // static bool bfs_bipartite(Graph& graph, size_t start, vector<int>& color);
            // static bool bfsBipartite(Graph& graph, size_t start, vector<int>& color);
            static bool dfsCheck(Graph& graph, size_t u, std::vector<int>& color, int col) ;
            




            

    };
}

#endif
