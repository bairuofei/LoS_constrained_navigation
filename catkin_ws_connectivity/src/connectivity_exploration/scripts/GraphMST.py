# Python program for Kruskal's algorithm to find minimum Spanning Tree of a given connected, undirected and weighted graph
# It is used in discreted graph connectivity maintainance work, to find the desired target configuration of robots.
import time
import networkx as nx
from typing import List


class GraphMST:
    def __init__(self, num_vertices: int):
        self.num_vertices = num_vertices
        self.edges: List[list] = []

    # Function to add an edge to graph
    def addEdge(self, u: int, v: int, w: float):
        self.edges.append([u, v, w])

    # A utility function to find set of an element i (truly uses path compression technique)
    def find(self, parent, i):
        if parent[i] != i:
            # Reassignment of node's parent to root node as path compression requires
            parent[i] = self.find(parent, parent[i])
        return parent[i]

    # A function that does union of two sets of x and y (uses union by rank)
    def union(self, parent, rank, x, y):
        # Attach smaller rank tree under root of high rank tree (Union by Rank)
        if rank[x] < rank[y]:
            parent[x] = y
        elif rank[x] > rank[y]:
            parent[y] = x
        # If ranks are same, then make one as root and increment its rank by one
        else:
            parent[y] = x
            rank[x] += 1
        return

    def is_graph_connected(self) -> bool:
        G = nx.Graph()
        edges = []
        for edge in self.edges:
            edges.append((edge[0], edge[1]))
        G.add_edges_from(edges)  # 连通图
        if G.number_of_nodes() < self.num_vertices:
            return False
        if G.number_of_nodes() == 0:
            print("\033[91m Communication graph is empty! \033[0m")
            return False
        return nx.is_connected(G)

    # The main function to construct MST using Kruskal's algorithm
    def KruskalMST(self) -> List[tuple]:
        selected_edge = []
        # Check connectivity of the graph first
        if not self.is_graph_connected():
            print(
                f"\u001b[91m Communcation graph is not connected. Failed to find minimum spainning tree! \u001b[0m"
            )
            for edge in self.edges:
                selected_edge.append((edge[0], edge[1]))
            return selected_edge

        # This will store the resultant MST
        result = []
        # Sort all the edges in non-decreasing order of their weight
        self.edges = sorted(self.edges, key=lambda item: item[2])

        parent = []
        rank = []
        # Create V subsets with single elements
        parent = [i for i in range(self.num_vertices)]
        rank = [0 for i in range(self.num_vertices)]

        # An index variable, used for sorted edges
        idx_edge = 0
        # An index variable, used for result[]
        num_edge = 0
        # Number of edges to be taken is less than to V-1
        while num_edge < self.num_vertices - 1:
            # Pick the smallest edge and increment the index for next iteration
            u, v, w = self.edges[idx_edge]
            idx_edge += 1
            x = self.find(parent, u)
            y = self.find(parent, v)

            # If including this edge doesn't cause cycle, then include it in result and increment the index of result
            # for next edge
            if x != y:  # 说明不会形成环
                num_edge += 1
                result.append([u, v, w])
                self.union(parent, rank, x, y)
            # Else discard the edge

        minimumCost = 0
        for one_edge in result:
            minimumCost += one_edge[2]
            selected_edge.append((one_edge[0], one_edge[1]))
        # print("Minimum Spanning Tree", minimumCost)
        return selected_edge


# Driver code
if __name__ == "__main__":

    time_start = time.time()
    g = GraphMST(4)
    g.addEdge(0, 1, 10)
    g.addEdge(0, 2, 6)
    g.addEdge(0, 3, 5)
    g.addEdge(1, 2, 15)
    g.addEdge(1, 3, 15)
    g.addEdge(2, 3, 4)

    # Function call
    selected_edges = g.KruskalMST()
    print(f"time spend: {time.time() - time_start} seconds")
