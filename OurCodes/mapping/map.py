import networkx as nx
import matplotlib.pyplot as plt
import math
import csv

nodes = {}
edges = []

# Import data
with open('node_map.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        edges += [(row['Source'], row['Dest'],
                   {'Dist': float(row['Dist'])})]

with open('BuildingPosition.csv', newline='') as csvfile:
    reader = csvfile.readlines()
    for rows in reader:
        row = rows.split(',')
        nodes[row[0]] = (float(row[1]), float(row[2]), float(row[3]))
    

def create_graph(nodes: dict, edges) -> nx.Graph:
    "Converts the given nodes and edges into a NetworkX Graph."
    graph = nx.Graph()
    node_keys = list(nodes.keys())
    for key in node_keys:
        graph.add_node(key, pos=(nodes[key][0], nodes[key][1]))
    graph.add_edges_from(edges)
    return graph


def draw_graph(graph: nx.Graph):
    "Draws the given graph using pyplot."
    pos = nx.get_node_attributes(graph, 'pos')
    labels = nx.get_edge_attributes(graph, 'distance')
    nx.draw(graph, pos, with_labels=True)
    #nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels)
    plt.show()

g = create_graph(nodes,edges)
draw_graph(g)