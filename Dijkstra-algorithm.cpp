#include <stdio.h>
#include <stdlib.h>
#define INFINITY 65535

// Structure to represent a node in the adjacency list
struct AdjacencyListNode
{
    int destinationVertex;
    int edgeWeight;
    struct AdjacencyListNode* next;
};

// Structure to represent an adjacency list
struct AdjacencyList
{
    struct AdjacencyListNode *head;
};

// Structure to represent a graph
struct Graph
{
    int noOfVertices;
    struct AdjacencyList* array;
};

// Structure to represent a min heap node
struct MinHeapNode
{
    int vertex;
    int distance;
};

// Structure to represent a min heap
struct MinHeap
{
    int size;
    int capacity;
    int *position;
    struct MinHeapNode **array;
};

// Function to creates a graph of vertices specified by the parameter 'noOfVertices'
struct Graph* CreateGraph(int noOfVertices)
{
    struct Graph* graph = (struct Graph*) malloc(sizeof(struct Graph));
    graph->noOfVertices = noOfVertices;
    graph->array = (struct AdjacencyList*) malloc(noOfVertices * sizeof(struct AdjacencyList));

    for (int i=0; i<noOfVertices; ++i)
        graph->array[i].head = NULL;

    return graph;
}

// Function to create a new adjacency list node
struct AdjacencyListNode* CreateNewAdjacencyListNode(int destinationVertex, int edgeWeight)
{
    struct AdjacencyListNode* newNode = (struct AdjacencyListNode*) malloc(sizeof(struct AdjacencyListNode));
    newNode->destinationVertex = destinationVertex;
    newNode->edgeWeight = edgeWeight;
    newNode->next = NULL;
    return newNode;
}

// Function to add an edge in the graph
void AddAnEdge(struct Graph* graph, int sourceVertex, int destinationVertex, int edgeWeight)
{
    struct AdjacencyListNode* newNode = CreateNewAdjacencyListNode(destinationVertex, edgeWeight);
    newNode->next = graph->array[sourceVertex].head;
    graph->array[sourceVertex].head = newNode;
}

// Function to print the adjacency list representation of graph and the corresponding edge weights for each edge of each adjacency list
void PrintGraph(struct Graph* graph)
{
    if(graph != NULL && graph->noOfVertices != 0)
    {
        for (int i=0; i<graph->noOfVertices; ++i)
        {
            //print the adjacency list
            struct AdjacencyListNode* node = graph->array[i].head;
            printf("\n Adjacency List of vertex %d \n %d", i, i);
            while (node)
            {
                printf(" -> %d", node->destinationVertex);
                node = node->next;
            }

            printf(": ");

            //print the edge weights
            struct AdjacencyListNode* nodeEdgeWeight = graph->array[i].head;
            while (nodeEdgeWeight)
            {
                printf(" %d", nodeEdgeWeight->edgeWeight);
                printf(" ");
                nodeEdgeWeight = nodeEdgeWeight->next;
            }
            printf("\n");
        }
    }
}

// Function to create a new Min Heap Node
struct MinHeapNode* CreateNewMinHeapNode(int vertex, int distance)
{
    struct MinHeapNode* minHeapNode = (struct MinHeapNode*) malloc(sizeof(struct MinHeapNode));
    minHeapNode->vertex = vertex;
    minHeapNode->distance = distance;
    return minHeapNode;
}

// Function to create a Min Heap
struct MinHeap* CreateMinHeap(int capacity)
{
    struct MinHeap* minHeap = (struct MinHeap*) malloc(sizeof(struct MinHeap));
    minHeap->position = (int *)malloc(capacity * sizeof(int));
    minHeap->size = 0;
    minHeap->capacity = capacity;
    minHeap->array = (struct MinHeapNode**) malloc(capacity * sizeof(struct MinHeapNode*));
    return minHeap;
}

// Function to swap two nodes of Min heap
void SwapMinHeapNode(struct MinHeapNode** firstNode, struct MinHeapNode** secondNode)
{
    struct MinHeapNode* tempNode = *firstNode;
    *firstNode = *secondNode;
    *secondNode = tempNode;
}

// Function to decrease distance value of a given vertex
void DecreaseDistance(struct MinHeap* minHeap, int vertex, int dist)
{
    int posiion = minHeap->position[vertex];
    minHeap->array[posiion]->distance = dist;

    while (posiion && minHeap->array[posiion]->distance < minHeap->array[(posiion-1)/2]->distance)
    {
        minHeap->position[minHeap->array[posiion]->vertex] = (posiion-1)/2;
        minHeap->position[minHeap->array[(posiion-1)/2]->vertex] = posiion;
        SwapMinHeapNode(&minHeap->array[posiion],  &minHeap->array[(posiion-1)/2]);
        posiion = (posiion-1)/2;
    }
}

// Function to do Min heapify at the given index
void DoMinHeapify(struct MinHeap* minHeap, int index)
{
    int smallest, left, right;
    smallest = index;
    left = 2*index + 1;
    right = 2*index + 2;

    if (left < minHeap->size && (minHeap->array[left]->distance < minHeap->array[smallest]->distance))
    {
          smallest = left;
    }

    if (right < minHeap->size && (minHeap->array[right]->distance < minHeap->array[smallest]->distance))
    {
          smallest = right;
    }

    if (smallest != index)
    {
        MinHeapNode *smallestNode = minHeap->array[smallest];
        MinHeapNode *indexNode = minHeap->array[index];

        // Swap positions
        minHeap->position[smallestNode->vertex] = index;
        minHeap->position[indexNode->vertex] = smallest;

        // Swap nodes
        SwapMinHeapNode(&minHeap->array[smallest], &minHeap->array[index]);

        DoMinHeapify(minHeap, smallest);
    }
}

// Function to extract minimum node from the heap
struct MinHeapNode* ExtractMinimumNode(struct MinHeap* minHeap)
{
    if (minHeap->size == 0)
        return NULL;

    struct MinHeapNode* root = minHeap->array[0];
    struct MinHeapNode* lastNode = minHeap->array[minHeap->size-1];
    minHeap->array[0] = lastNode;
    minHeap->position[root->vertex] = minHeap->size-1;
    minHeap->position[lastNode->vertex] = 0;
    --minHeap->size;
    DoMinHeapify(minHeap, 0);

    return root;
}

// Function to print the calculated shortest distances
void PrintShortestDistance(int distance[], int noOfVertices)
{
    for(int i=0; i < noOfVertices; ++i)
    {
        printf("Distance of %d from source 0 is: %d \n", i, distance[i]);
    }
}

// Function to compute shortest path using Djikstra's algorithm
void ComputeShortestPathUsingDijkstraAlgo(struct Graph* graph, int source)
{
    int noOfVertices = graph->noOfVertices;
    int distance[noOfVertices];

    struct MinHeap* minHeap = CreateMinHeap(noOfVertices);

    for (int v=0; v<noOfVertices; ++v)
    {
        distance[v] = INFINITY;
        minHeap->array[v] = CreateNewMinHeapNode(v, distance[v]);
        minHeap->position[v] = v;
    }

    minHeap->array[source] = CreateNewMinHeapNode(source, distance[source]);
    minHeap->position[source] = source;
    distance[source] = 0;
    DecreaseDistance(minHeap, source, distance[source]);

    minHeap->size = noOfVertices;

    while (minHeap->size != 0)
    {
        struct MinHeapNode* minHeapNode = ExtractMinimumNode(minHeap);
        int u = minHeapNode->vertex;

        struct AdjacencyListNode* node = graph->array[u].head;
        while (node != NULL)
        {
            int v = node->destinationVertex;
            if ((minHeap->position[v] < minHeap->size) && (distance[u] != INFINITY && node->edgeWeight + distance[u] < distance[v]))
            {
                distance[v] = distance[u] + node->edgeWeight;
                DecreaseDistance(minHeap, v, distance[v]);
            }
            node = node->next;
        }
    }

    PrintShortestDistance(distance, noOfVertices);
}

// Main Function
int main()
{
    int noOfVertices = 8;

    //A digraph is created here
    struct Graph* graph = CreateGraph(noOfVertices);

    //Add edges for vertex 0
    AddAnEdge(graph, 0, 1, 9);
    AddAnEdge(graph, 0, 5, 14);
    AddAnEdge(graph, 0, 6, 15);

    //Add edges for vertex 1
    AddAnEdge(graph, 1, 2, 24);

    //Add edges for vertex 2
    AddAnEdge(graph, 2, 7, 19);
    AddAnEdge(graph, 2, 4, 2);

    //Add edges for vertex 3
    AddAnEdge(graph, 3, 2, 6);
    AddAnEdge(graph, 3, 7, 6);

    //Add edges for vertex 4
    AddAnEdge(graph, 4, 3, 11);
    AddAnEdge(graph, 4, 7, 16);

    //Add edges for vertex 5
    AddAnEdge(graph, 5, 2, 18);
    AddAnEdge(graph, 5, 4, 30);
    AddAnEdge(graph, 5, 6, 5);

    //Add edges for vertex 6
    AddAnEdge(graph, 6, 4, 20);
    AddAnEdge(graph, 6, 7, 44);

    // print the adjacency list representation of the above graph
    PrintGraph(graph);

    ComputeShortestPathUsingDijkstraAlgo(graph, 0);

    return 0;
}
