import networkx as nx
import matplotlib.pyplot as plt
import math

#parse the csv file with format -> BuildingCode, x,y,z 
#Note that x,y,and z here is refering to the x,y,and z of the top of the building
import csv

file = open('BuildingPosition.csv')
csvreader = csv.reader(file)
rows = []
for row in csvreader:
        rows.append(row)
file.close()

#add nodes
G=nx.Graph()
for row in rows:
    if row[1] != "none":
        G.add_node(row[0],pos=(float(row[1]),float(row[2])))

#add all edges
for i in range(len(list(G))-1):
    for j in range(i+1,len(list(G))):
        G.add_edge(list(G)[i],list(G)[j])

print(G)
#draw
pos=nx.get_node_attributes(G,'pos')
nx.draw(G,pos)
plt.show()  

#create function to calculate euclidean distance from pointA to pointB. 
#Both pointA and pointB has form of (x,y,z) tuple
def euclidean_dist(a,b):
    total = 0
    for i in range(len(a)):
        total += (a[i] - b[i])**2
    return math.sqrt(total)

#create function to calculate travel distance
