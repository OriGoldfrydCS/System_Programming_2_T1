#include "Graph.hpp"
#include <iostream>
#include <stdexcept>

using namespace std;

namespace ariel {

    Graph::Graph() : numVertices(0), numEdges(0), isDirected(false) {}

    void Graph::loadGraph(vector<vector<int>>& matrix) {
        

        // if (matrix.empty()) {
        //     throw invalid_argument("Invalid graph: The graph matrix is empty.");
        // }

        numVertices = matrix.size();
        numEdges = 0;

        // Check if the matrix is square
        for (const auto& row : matrix) {
            if (row.size() != numVertices) {
                throw invalid_argument("Invalid graph: The graph is not a square matrix.");
            }
        }

    adjacencyMatrix = matrix;
        
        adjacencyMatrix = matrix;

        // Check if the graph is directed
        isDirected = false;
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) {
                if (adjacencyMatrix[i][j] != adjacencyMatrix[j][i]) {
                    isDirected = true;
                    break;
                }
            }
            if (isDirected) {
                break;
            }
        }

        // Count the number of edges
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = i + 1; j < numVertices; j++) {
                if (adjacencyMatrix[i][j] != 0) {
                    numEdges++;
                    if (isDirected) {
                        numEdges++; // Only increment again if the graph is directed
                    }
                }
            }
        }
    }

    void Graph::printGraph() {
        cout << "Graph with " << numVertices << " vertices and " << numEdges << " edges." << endl;
    }

    size_t Graph::getNumVertices() {
        return numVertices;
    }

    size_t Graph::getNumEdges() {
        return numEdges;
    }

    vector<vector<int>>& Graph::getAdjacencyMatrix() {
        return adjacencyMatrix;
    }

    bool Graph::isGraphDirected() {
        return isDirected;
    }

}