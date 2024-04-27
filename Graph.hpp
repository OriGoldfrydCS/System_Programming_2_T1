#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>

using namespace std; 

namespace ariel {

class Graph {
private:
    vector<vector<int>> adjacencyMatrix;
    size_t numVertices;
    size_t numEdges;
    bool isDirected;

public:
    Graph();
    void loadGraph(vector<vector<int>>& matrix);
    void printGraph();
    size_t getNumVertices();
    size_t getNumEdges();
    bool isGraphDirected();
    vector<vector<int>>& getAdjacencyMatrix();
};

}

#endif