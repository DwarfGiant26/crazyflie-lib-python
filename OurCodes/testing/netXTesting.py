import networkx as nx
import matplotlib.pyplot as plt

G=nx.Graph()
G.add_node(1,pos=(0,0))
G.add_node(2,pos=(2,2))
G.add_edge(1,2)
G.add_node(3,pos=(3,2))
G.add_edge(2,3)
pos=nx.get_node_attributes(G,'pos')
nx.draw(G,pos)
plt.show()
