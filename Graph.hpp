#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>

using namespace std;

namespace ariel {
    
    /**
     * 
     * This class represents a graph with vertices and edges.
     * The edges can be directed or undirected, and weighted or unweighted.
     * 
     */
    class Graph {

        private:
            /**
             * This data structure stores the graph as an adjacency matrix, where the value at adjacencyMatrix[i][j] 
             * represents the weight of the edge from vertex i to vertex j. 
             */
            vector<vector<int>> adjacencyMatrix;

            /**
             * This variable stores the number of vertices in the graph.
             */
            int numVertices;

            /**
             * This variable stores the number of edges in the graph.
             */
            int numEdges;

            /**
             * This boolean variable indicates whether the graph is directed (true) or undirected (false).
             */
            bool isDirected;
            
        public:
            /**
             * Default constructor that initializes an empty graph.
             */

            Graph();


            /**
             * This function loads a graph from a given adjacency matrix, where each entry in the matrix
             * specifies the edge weight between vertices.
             * 
             * @param matrix The adjacency matrix representing the graph.
             */
            void loadGraph(const vector<vector<int>>& matrix);


            /**
             * This function prints the graph's adjacency matrix.
             */
            void printGraph() const;


            /**
             * This function returns the number of vertices in the graph.
             * 
             * @return The number of vertices.
             */
            int getNumVertices() const;


            /**
             * This function returns the number of edges in the graph.
             * 
             * @return The number of edges.
             */
            int getNumEdges() const;


            /**
             * This function provides a "read-only" access to the graph's adjacency matrix.
             * 
             * @return A constant reference to the graph's adjacency matrix.
             */
            const vector<vector<int>>& getAdjacencyMatrix() const;
    };
}

#endif
