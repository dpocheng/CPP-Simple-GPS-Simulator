// Digraph_MyGtestTests.cpp
//
// ICS46 Spring 2015
// Project #4: Rock and Roll Stops the Traffic

#include <gtest/gtest.h>
#include "Digraph.hpp"


TEST(Digraph_MyGtest, canConstructWithNoArguments)
{
    Digraph<int, double> d1;
    Digraph<unsigned int, double> d2;
}


TEST(Digraph_MyGtest, canCopyConstructToCompatibleType)
{
    Digraph<std::string, std::string> d1;
    Digraph<std::string, std::string> d2{d1};
}


TEST(Digraph_MyGtest, canAssignToCompatibleType)
{
    Digraph<unsigned int, double> d1;
    Digraph<unsigned int, double> d2;
    d1 = d2;
}


TEST(Digraph_MyGtest, canGetAllVertexNumbers)
{
    Digraph<double, unsigned long> d1;
    const Digraph<double, unsigned long>& d = d1;
    std::vector<int> v = d.vertices();
    v.clear();
}


TEST(Digraph_MyGtest, canGetAllEdges)
{
    Digraph<long, std::string> d1;
    const Digraph<long, std::string>& d = d1;
    std::vector<std::pair<int, int>> e = d.edges();
    e.clear();
}


TEST(Digraph_MyGtest, canGetAllEdgesForExistingVertex)
{
    Digraph<float, float> d1;
    d1.addVertex(1, 9.4);
    d1.addVertex(2, 7.9);
    d1.addEdge(1, 2, -1.5);
    const Digraph<float, float>& d = d1;
    std::vector<std::pair<int, int>> e = d.edges(1);
    e.clear();
}


TEST(Digraph_MyGtest, canGetVertexInfoForExistingVertex)
{
    Digraph<std::string, std::string> d1;
    d1.addVertex(1, "Boo is great");
    const Digraph<std::string, std::string>& d = d1;
    std::string vinfo = d.vertexInfo(1);
    vinfo += "Boo is great";
}


TEST(Digraph_MyGtest, canGetEdgeInfoForExistingEdge)
{
    Digraph<std::string, std::string> d1;
    d1.addVertex(1, "Boo");
    d1.addVertex(2, "Bear");
    d1.addEdge(1, 2, "go");
    const Digraph<std::string, std::string>& d = d1;
    std::string einfo = d.edgeInfo(1, 2);
    einfo += "Hello";
}


TEST(Digraph_MyGtest, canAddVertex)
{
    Digraph<int, int> d1;
    d1.addVertex(1, 10);
}


TEST(Digraph_MyGtest, canAddEdge)
{
    Digraph<int, int> d1;
    d1.addVertex(1, 10);
    d1.addVertex(2, 9);
    d1.addEdge(1, 2, -1);
}


TEST(Digraph_MyGtest, canRemoveExistingVertex)
{
    Digraph<std::string, std::string> d1;
    d1.addVertex(1, "Boo");
    d1.removeVertex(1);
}


TEST(Digraph_MyGtest, canRemoveExistingEdge)
{
    Digraph<float, float> d1;
    d1.addVertex(1, 3.5);
    d1.addVertex(2, 4.75);
    d1.addEdge(1, 2, -1.5);
    d1.removeEdge(1, 2);
}


TEST(Digraph_MyGtest, canGetVertexCount)
{
    const Digraph<int, int> d1;
    int vcount = d1.vertexCount();
    ++vcount;
}


TEST(Digraph_MyGtest, canGetTotalEdgeCount)
{
    const Digraph<int, int> d1;
    int ecount = d1.edgeCount();
    ++ecount;
}


TEST(Digraph_MyGtest, canGetEdgeCountForExistingVertex)
{
    Digraph<int, int> d1;
    d1.addVertex(1, 10);
    const Digraph<int, int>& d = d1;
    int ecount = d.edgeCount(1);
    ++ecount;
}


TEST(Digraph_MyGtest, canFindShortestPaths)
{
    Digraph<std::string, std::string> d1;
    d1.addVertex(1, "Boo");
    d1.addVertex(2, "Bear");
    d1.addEdge(1, 2, "hello");

    const Digraph<std::string, std::string>& d = d1;

    std::function<double(const std::string&)> edgeWeightFunc =
        [](const std::string& edgeInfo)
        {
            return static_cast<double>(edgeInfo.length());
        };

    std::map<int, int> paths = d.findShortestPaths(1, edgeWeightFunc);
    paths.clear();
}


TEST(Digraph_MyGtest, canDetermineStrongConnectedness)
{
    const Digraph<int, int> d1;
    bool connected = d1.isStronglyConnected();
    connected = !connected;
}
