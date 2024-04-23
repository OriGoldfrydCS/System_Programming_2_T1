#include "Graph.hpp"
#include <iostream>
#include <stdexcept>

using namespace std;

namespace ariel {

    /**
     * 
     * Constructor for the Graph class.
     * Initializes a new graph with 0 vertices and edges, and is not directed.
     * 
     */
    Graph::Graph() {
            this->numVertices = 0;
            this->numEdges = 0;
            this->isDirected = false;
    }
    
    /**
     * 
     * This function loads a graph from a given adjacency matrix.
     * @param matrix A square matrix representing the adjacency of vertices, where matrix[i][j] represents
     *               the weight of the edge from vertex i to vertex j.
     * @throws invalid_argument if the matrix is not square.
     * 
     */
    void Graph::loadGraph(const vector<vector<int>>& matrix) {
        // Check if the matrix is square
        int rows = matrix.size();
        for (const auto& row : matrix) {
            if (row.size() != rows) {
                throw invalid_argument("Invalid graph: The graph is not a square matrix.");
            }
        }
        
        adjacencyMatrix = matrix;
        numVertices = rows;
        
        // Count the number of edges
        numEdges = 0;
        for (vector<vector<int>>::size_type i = 0; i < numVertices; i++) {
            for (vector<vector<int>>::size_type j = i + 1; j < numVertices; j++) {
                if (adjacencyMatrix[i][j] != 0) {
                    numEdges++;
                    if (!isDirected) {
                        numEdges++;
                    }
                }
            }
        }
    }


    /**
     * 
     * This function prints the graph information to the standard output.
     * 
     */
    void Graph::printGraph() const {
        cout << "Graph with " << numVertices << " vertices and " << numEdges << " edges." << endl;
    }


    /**
     * 
     * This function returns the number of vertices in the graph.
     * @return int The number of vertices.
     * 
     */
    int Graph::getNumVertices() const {
        return numVertices;
    }


    /**
     * This function returns the number of edges in the graph.
     * @return int The number of edges.
     * 
     */
    int Graph::getNumEdges() const {
        return numEdges;
    }


    /**
     * 
     * This function provides a "read-only" access to the graph's adjacency matrix.
     * @return const vector<vector<int>>& The adjacency matrix of the graph.
     * 
     */
    const vector<vector<int>>& Graph::getAdjacencyMatrix() const {
        return adjacencyMatrix;
    }

}