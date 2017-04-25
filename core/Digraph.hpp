// Digraph.hpp
//
// ICS 46 Spring 2015
// Project #4: Rock and Roll Stops the Traffic
//
// This header file declares a class template called Digraph, which is
// intended to implement a generic directed graph.  The implementation
// uses the adjacency lists technique, so each vertex stores a linked
// list of its outgoing edges.
//
// Along with the Digraph class template is a class DigraphException
// and a couple of utility structs that aren't generally useful outside
// of this header file.
//
// In general, directed graphs are all the same, except in the sense
// that they store different kinds of information about each vertex and
// about each edge; these two types are the type parameters to the
// Digraph class template.

#ifndef DIGRAPH_HPP
#define DIGRAPH_HPP

#include <functional>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>



// DigraphExceptions are thrown from some of the member functions in the
// Digraph class template, so that exception is declared here, so it
// will be available to any code that includes this header file.

class DigraphException
{
public:
    DigraphException(const std::string& reason): reason_{reason} { }

    std::string reason() const { return reason_; }

private:
    std::string reason_;
};



// A DigraphEdge lists a "from vertex" (the number of the vertex from which
// the edge points), a "to vertex" (the number of the vertex to which the
// edge points), and an EdgeInfo object.  Because different kinds of Digraphs
// store different kinds of edge information, DigraphEdge is a template
// struct.

template <typename EdgeInfo>
struct DigraphEdge
{
    int fromVertex;
    int toVertex;
    EdgeInfo einfo;
};



// A DigraphVertex includes two things: a VertexInfo object and a list of
// its outgoing edges.  Because different kinds of Digraphs store different
// kinds of vertex and edge information, DigraphVertex is a template struct.

template <typename VertexInfo, typename EdgeInfo>
struct DigraphVertex
{
    VertexInfo vinfo;
    std::list<DigraphEdge<EdgeInfo>> edges;
};



// Digraph is a class template that represents a directed graph implemented
// using adjacency lists.  It takes two type parameters:
//
// * VertexInfo, which specifies the kind of object stored for each vertex
// * EdgeInfo, which specifies the kind of object stored for each edge
//
// You'll need to implement the member functions declared here; each has a
// comment detailing how it is intended to work.
//
// Each vertex in a Digraph is identified uniquely by a "vertex number".
// Vertex numbers are not necessarily sequential and they are not necessarily
// zero- or one-based.

template <typename VertexInfo, typename EdgeInfo>
class Digraph
{
public:
    // The default constructor initializes a new, empty Digraph so that
    // contains no vertices and no edges.
    Digraph();

    // The copy constructor initializes a new Digraph to be a deep copy
    // of another one (i.e., any change to the copy will not affect the
    // original).
    Digraph(const Digraph& d);

    // The destructor deallocates any memory associated with the Digraph.
    ~Digraph();

    // The assignment operator assigns the contents of the given Digraph
    // into "this" Digraph, with "this" Digraph becoming a separate, deep
    // copy of the contents of the given one (i.e., any change made to
    // "this" Digraph afterward will not affect the other).
    Digraph& operator=(const Digraph& d);

    // vertices() returns a std::vector containing the vertex numbers of
    // every vertex in this Digraph.
    std::vector<int> vertices() const;

    // edges() returns a std::vector of std::pairs, in which each pair
    // contains the "from" and "to" vertex numbers of an edge in this
    // Digraph.  All edges are included in the std::vector.
    std::vector<std::pair<int, int>> edges() const;

    // This overload of edges() returns a std::vector of std::pairs, in
    // which each pair contains the "from" and "to" vertex numbers of an
    // edge in this Digraph.  Only edges outgoing from the given vertex
    // number are included in the std::vector.  If the given vertex does
    // not exist, a DigraphException is thrown instead.
    std::vector<std::pair<int, int>> edges(int vertex) const;

    // vertexInfo() returns the VertexInfo object belonging to the vertex
    // with the given vertex number.  If that vertex does not exist, a
    // DigraphException is thrown instead.
    VertexInfo vertexInfo(int vertex) const;

    // edgeInfo() returns the EdgeInfo object belonging to the edge
    // with the given "from" and "to" vertex numbers.  If either of those
    // vertices does not exist *or* if the edge does not exist, a
    // DigraphException is thrown instead.
    EdgeInfo edgeInfo(int fromVertex, int toVertex) const;

    // addVertex() adds a vertex to the Digraph with the given vertex
    // number and VertexInfo object.  If there is already a vertex in
    // the graph with the given vertex number, a DigraphException is
    // thrown instead.
    void addVertex(int vertex, const VertexInfo& vinfo);

    // addEdge() adds an edge to the Digraph pointing from the given
    // "from" vertex number to the given "to" vertex number, and
    // associates with the given EdgeInfo object with it.  If one
    // of the vertices does not exist *or* if the same edge is already
    // present in the graph, a DigraphException is thrown instead.
    void addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo);

    // removeVertex() removes the vertex (and all of its incoming
    // and outgoing edges) with the given vertex number from the
    // Digraph.  If the vertex does not exist already, a DigraphException
    // is thrown instead.
    void removeVertex(int vertex);

    // removeEdge() removes the edge pointing from the given "from"
    // vertex number to the given "to" vertex number from the Digraph.
    // If either of these vertices does not exist *or* if the edge
    // is not already present in the graph, a DigraphException is
    // thrown instead.
    void removeEdge(int fromVertex, int toVertex);

    // vertexCount() returns the number of vertices in the graph.
    int vertexCount() const;

    // edgeCount() returns the total number of edges in the graph,
    // counting edges outgoing from all vertices.
    int edgeCount() const;

    // This overload of edgeCount() returns the number of edges in
    // the graph that are outgoing from the given vertex number.
    // If the given vertex does not exist, a DigraphException is
    // thrown instead.
    int edgeCount(int vertex) const;

    // isStronglyConnected() returns true if the Digraph is strongly
    // connected (i.e., every vertex is reachable from every other),
    // false otherwise.
    bool isStronglyConnected() const;

    // findShortestPaths() takes a start vertex number and a function
    // that takes an EdgeInfo object and determines an edge weight.
    // It uses Dijkstra's Shortest Path Algorithm to determine the
    // shortest paths from the start vertex to every other vertex
    // in the graph.  The result is returned as a std::map<int, int>
    // where the keys are vertex numbers and the value associated
    // with each key k is the precedessor of that vertex chosen by
    // the algorithm.  For any vertex without a predecessor (e.g.,
    // a vertex that was never reached, or the start vertex itself),
    // the value is simply a copy of the key.
    std::map<int, int> findShortestPaths(
        int startVertex,
        std::function<double(const EdgeInfo&)> edgeWeightFunc) const;


private:
    // Add whatever member variables you think you need here.  One
    // possibility is a std::map where the keys are vertex numbers
    // and the values are DigraphVertex<VertexInfo, EdgeInfo> objects.

    // create a map for Digraph class
    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> digraphMap;

    // create a class for priority queue, the priority quees will use it for getting the priority for each vertice
    class Compare
    {
    public:
        bool operator()(std::pair<int, int> vertice1, std::pair<int, int> vertice2)
        {
            if(vertice1.first > vertice2.first)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    };

    // create typedef const iterator with list and map
    // so that the code won't look so messy
    typedef typename std::list<DigraphEdge<EdgeInfo>>::const_iterator listIterator;
    typedef typename std::map<int, DigraphVertex<VertexInfo, EdgeInfo>>::const_iterator mapIterator;

    // You can also feel free to add any additional member functions
    // you'd like (public or private), so long as you don't remove or
    // change the signatures of the ones that already exist.

    // helper functions for checking the digraph is strongly connected
    const bool isAllConnected(int location) const;
    void checkEachLocation(int index, std::vector<std::pair<int, int>> localVec, std::vector<bool>& visited) const;
};


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph()
{
    // to make sure the map is empty when the map object is decleared
    digraphMap.clear();
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(const Digraph& d)
{
    // copy the map object from Digraph d
    this->digraphMap = d.digraphMap;
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::~Digraph()
{
    // clear the memory buffer from the map object
    digraphMap.clear();
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(const Digraph& d)
{
    // copy and return it to new class object (assign)
    this->digraphMap = d.digraphMap;
    return *this;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<int> Digraph<VertexInfo, EdgeInfo>::vertices() const
{
    // create a vector for return
    std::vector<int> verticesVector;
    // create a map const iterator
    mapIterator verticesMap;
    for (verticesMap = digraphMap.begin(); verticesMap != digraphMap.end(); verticesMap++)
    {
        // push the vertice into the vector for return
        verticesVector.push_back(verticesMap->first);
    }
    return verticesVector;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges() const
{
    // create a vector for return
    std::vector<std::pair<int, int>> edgesVector;
    // create a map const iterator
    mapIterator edgesMap;
    for (edgesMap = digraphMap.begin(); edgesMap != digraphMap.end(); edgesMap++)
    {
        // create a list const iterator
        listIterator edgesList;
        for (edgesList = edgesMap->second.edges.begin(); edgesList != edgesMap->second.edges.end(); ++edgesList)
        {
            // find the edge and push into the vector for return
            edgesVector.push_back(std::make_pair(edgesList->fromVertex, edgesList->toVertex));
        }
    }
    return edgesVector;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges(int vertex) const
{
    // create a vector for return
    std::vector<std::pair<int, int>> edgesVector;
    // create the iterator with find()
    mapIterator edgesMap = digraphMap.find(vertex);
    // create a list const iterator
    listIterator edgesList;
    // the program will execute the command within the if loop if the map iterator is not equal to the end of the class map
    if (edgesMap != digraphMap.end())
    {
        for (edgesList = edgesMap->second.edges.begin(); edgesList != edgesMap->second.edges.end(); ++edgesList)
        {
            // will find and push the edge into the vector
            edgesVector.push_back(std::make_pair(edgesList->fromVertex, edgesList->toVertex));
        }
    }
    else
    {
        // if the map iterator is equal to the end of the map, throw an exception
        throw DigraphException("edges() cannot find the specific vertex");
    }
    return edgesVector;
}


template <typename VertexInfo, typename EdgeInfo>
VertexInfo Digraph<VertexInfo, EdgeInfo>::vertexInfo(int vertex) const
{
    // create a map iterator
    mapIterator vertexInfoMap = digraphMap.find(vertex);
    if (vertexInfoMap != digraphMap.end())
    {
        // return the vertex information if the map iterator is not equal to the end of the class map
        return vertexInfoMap->second.vinfo;
    }
    else
    {
        // throw an excpetion if the map iterator is equal to the end of class map
        throw DigraphException("vertexInfo() cannot find the specific vertex");
    }
}


template <typename VertexInfo, typename EdgeInfo>
EdgeInfo Digraph<VertexInfo, EdgeInfo>::edgeInfo(int fromVertex, int toVertex) const
{
    // create a map iterator with find()
    mapIterator edgeInfoMap = digraphMap.find(fromVertex);
    // create a list iterator
    listIterator edgeInfoList;
    if (edgeInfoMap != digraphMap.end())
    {
        // iterate each key in the map to find the correspoing toVertex
        for (edgeInfoList = edgeInfoMap->second.edges.begin(); edgeInfoList != edgeInfoMap->second.edges.end(); ++edgeInfoList)
        {
            // if matching, return the information
            if(edgeInfoList->toVertex == toVertex)
            {
                return edgeInfoList->einfo;
            }
        }
    }
    else
    {
        // throw an exception if the map iterator is equal to the end of digraphMap
        throw DigraphException("edgeInfo() cannot find the specific fromVertice");
    }
    // throw and exception if nothing is found
    throw DigraphException("edgeInfo() cannot find the specific edge");
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addVertex(int vertex, const VertexInfo& vinfo)
{
    // create a map iterator
    mapIterator addVertexMap = digraphMap.find(vertex);
    // execute the command if matching the end of digraphMap
    if(addVertexMap == digraphMap.end())
    {
        // assign the vertex information to the class map
        digraphMap[vertex].vinfo = vinfo;
    }
    else
    {
        // throw an exception if the vertex is already existed in the class map
        throw DigraphException("addVertex() found the vertex is existed");
    }
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo)
{
    // create a fromVertex map iterator with find()
    mapIterator fromVertexMap = digraphMap.find(fromVertex);
    // create a toVertex map iterator with find()
    mapIterator toVertexMap = digraphMap.find(toVertex);
    // create a list iterator
    listIterator addEdgeList;
    if (fromVertexMap == digraphMap.end())
    {
        // throw an exception if fromVertex map iterator is equal to the end of digraphMap
        throw DigraphException("addEdge() cannot find the specific fromVertex");
    }
    else if (toVertexMap == digraphMap.end())
    {
        // throw an exception if toVertex map iterator is equal to the end of digraphMap
        throw DigraphException("addEdge() cannot find the specific toVertex");
    }
    else
    {
        for (addEdgeList = fromVertexMap->second.edges.begin(); addEdgeList != fromVertexMap->second.edges.end(); ++addEdgeList)
        {
            if (addEdgeList->toVertex == toVertex)
            {
                // throw an exception if the edge is existed
                throw DigraphException("addEdge() cannot add the specific edge");
            }
        }
        // create a new DigraphEdge struct
        DigraphEdge<EdgeInfo> copyDigraphEdge;
        copyDigraphEdge.fromVertex = fromVertex;
        copyDigraphEdge.toVertex = toVertex;
        copyDigraphEdge.einfo = einfo;
        // push the new struct into the class map digraphMap
        digraphMap[fromVertex].edges.push_back(copyDigraphEdge);
    }
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeVertex(int vertex)
{
    // create map iterator for remove search
    mapIterator removeVertexMap = digraphMap.find(vertex);
    if(removeVertexMap != digraphMap.end())
    {
        // erase the specific vertex if the vertex is existed in the map
        digraphMap.erase(vertex);
    }
    else
    {
        // throw an exception if the specific is not in the map
        throw DigraphException("removeVertex() cannot find the specific vertex");
    }
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeEdge(int fromVertex, int toVertex)
{
    // create a fromVertex map iterator
    mapIterator fromVertexMap = digraphMap.find(fromVertex);
    // create a toVertex map iterator
    mapIterator toVertexMap = digraphMap.find(toVertex);
    // create a list iterator
    listIterator removeEdgeList;
    if (fromVertexMap == digraphMap.end())
    {
        // throw an exception if fromVertexMap is at the end of the map
        throw DigraphException("removeEdge() cannot find the specific fromVertex");
    }
    else if (toVertexMap == digraphMap.end())
    {
        // throw an exception if toVertexMap is equal to the end of the map
        throw DigraphException("removeEdge() cannot find the specific toVertex");
    }
    else
    {
        for (removeEdgeList = fromVertexMap->second.edges.begin(); removeEdgeList != fromVertexMap->second.edges.end(); ++removeEdgeList)
        {
            if(removeEdgeList->toVertex == toVertex)
            {
                // erase the corresponding edges if found
                digraphMap[fromVertex].edges.erase(removeEdgeList);
                return;
            }
        }
        // throw an exception if the specific edges is not found
        throw DigraphException("removeEdge() cannot find the specific edge");
    }
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::vertexCount() const
{
    // return the size of the map
    return digraphMap.size();
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount() const
{
    int count = 0;
    // create a map iterator for counting the size of all edges
    mapIterator edgeCountMap;
    for (edgeCountMap = digraphMap.begin(); edgeCountMap != digraphMap.end(); edgeCountMap++)
    {
        // count every edges's size
        count = count + edgeCountMap->second.edges.size();
    }
    // and return
    return count;
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount(int vertex) const
{
    // create the map iterator with find
    mapIterator edgeCountMap = digraphMap.find(vertex);
    if (edgeCountMap != digraphMap.end())
    {
        int count = 0;
        // count the specific vertex's size
        count = count + edgeCountMap->second.edges.size();
        return count;
    }
    else
    {
        // throw an exception if the vertex does not exist
        throw DigraphException("edgeCount() cannot find the specific vertex");
    }
}


template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::isStronglyConnected() const
{
    // create a local bool variable for return
    bool isConnected;
    // create a map iterator for search the map
    mapIterator strongMap;
    for (strongMap = digraphMap.begin(); strongMap != digraphMap.end(); strongMap++)
    {

        // store the bool value to isConnected from the helper function
        isConnected = isAllConnected(strongMap->first);
        if (!isConnected)
        {
            // if false, return false
            return isConnected;
        }
    }
    // if check all and did not return false, return true
    return isConnected;
}


template <typename VertexInfo, typename EdgeInfo>
std::map<int, int> Digraph<VertexInfo, EdgeInfo>::findShortestPaths(int startVertex, std::function<double(const EdgeInfo&)> edgeWeightFunc) const
{
    // create a map iterator start from the startVertex
    mapIterator findMap = digraphMap.find(startVertex);
    if (findMap != digraphMap.end())
    {
        // create a priority queue with custom compare constructor (class)
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, Compare> priority;
        // shortestBooleanMap (kv) is a boolean flag that indicates
        // whether the shortest path to vertex v is known. Initially
        // kv is false for all vertices
        std::map<int, bool> shortestBooleanMap;
        // shortestLengthMap (dv) is the length of the shortest path
        // found thusfar from the start vertex to v. When the algorithm
        // begins, no paths have conssidered, so dv is initially set to
        // +INF for all vertices, except the start vertex, for which
        // dv = 0
        std::map<int, int> shortestLengthMap;
        // shortestPathMap (pv) is the predecessor of the vertex v on
        // the shortest path found thusfar from the start vertex to v.
        // Initially, pv is unknown for all vertices, except for the
        // start vertex, for which pv is none;
        std::map<int, int> shortestPathMap;
        mapIterator shortMap;
        for (shortMap = digraphMap.begin(); shortMap != digraphMap.end(); shortMap++)
        {
            // set kv to false
            shortestBooleanMap[shortMap->first] = false;
            // set dv to +INF
            shortestLengthMap[shortMap->first] = std::numeric_limits<int>::max();
        }
        // set the first element in dv to be 0 since v is the start vertex
        shortestLengthMap[findMap->first] = 0;
        // set the first element in pv to be +INF to assume none
        shortestPathMap[findMap->first] = std::numeric_limits<int>::max();
        // push the start vertex into the priority queue with priority 0
        priority.push(std::make_pair(0, findMap->first));
        // it will keep searching if the queue is not empty
        while (!priority.empty())
        {
            // vertice location is the vertex in queue with the smallest priority
            int location = priority.top().second;
            priority.pop();
            mapIterator queueMap = digraphMap.find(location);
            // check if the key has been checked before or not
            if (shortestBooleanMap[queueMap->first] == false)
            {
                // set the value to be true in order not to check this key again
                shortestBooleanMap[queueMap->first] = true;
                listIterator queueList;
                // check every path from startVertex to pick up the smallest path
                for (queueList = queueMap->second.edges.begin(); queueList != queueMap->second.edges.end(); queueList++)
                {
                    // the corresponding miles or milesPerHour by using edgeWeightFunc which is pass by parameter
                    if (shortestLengthMap[queueList->toVertex] > (shortestLengthMap[queueMap->first] + edgeWeightFunc(edgeInfo(queueMap->first, queueList->toVertex))))
                    {
                        // set the edge weight and push into the priority queue
                        // so that the loop will check next round to see which
                        // one is the smallest
                        shortestLengthMap[queueList->toVertex] = shortestLengthMap[queueMap->first] + edgeWeightFunc(edgeInfo(queueMap->first, queueList->toVertex));
                        shortestPathMap[queueList->toVertex] = queueMap->first;
                        priority.push(std::make_pair(shortestLengthMap[queueList->toVertex], queueList->toVertex));
                    }
                }
            }
        }
        // return pv after choosing the smallest path
        return shortestPathMap;
    }
    else
    {
        // throw an exception if the startVertex can't find any toVertex
        throw DigraphException("findShortestPaths cannot find the start vertex");
    }

}


template <typename VertexInfo, typename EdgeInfo>
const bool Digraph<VertexInfo, EdgeInfo>::isAllConnected(int location) const
{
    // create a vector that initialize every element is false
    std::vector<bool> visited;
    for (int i = 0; i < digraphMap.size(); i++)
    {
        visited.push_back(false);
    }
    // get the edges infomation from edges()
    std::vector<std::pair<int, int>> edgesVector = edges();
    // call the helper function to check each vertice recursively
    checkEachLocation(location, edgesVector, visited);
    // check each element in the vector if found false, return the program that
    // the graph is not strongly connected
    // In the other hand, true
    for (int j = 0; j < visited.size(); j++)
    {
        if (visited[j] == false)
        {
            return false;
        }
    }
    return true;
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::checkEachLocation(int index, std::vector<std::pair<int, int>> localVec, std::vector<bool>& visited) const
{
    // set the current index to be true
    visited[index] = true;
    for (int i = 0; i < localVec.size(); i ++)
    {
        // check if the next element is false, then call itself to check the next vertice
        if (index == localVec[i].first && visited[localVec[i].second] == false)
        {
            checkEachLocation(localVec[i].second, localVec, visited);
        }
    }
}



#endif // DIGRAPH_HPP

