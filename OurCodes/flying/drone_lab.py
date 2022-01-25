import networkx as nx
import matplotlib.pyplot as plt
import csv
import random
import math
from enum import Enum

# Constants
RAND_SEED = 1

# Drone Parameters
CRUISE_SPEED = 30                   # cm/s
TAKEOFF_LANDING_TIME = 5            # s
MAX_VOLT = 4.23                     # V
MIN_VOLT = 3.0                      # V
CHARGING_TIME = 600                 # s
BASE_BAT_DRAIN = 0.00145            # V/s
PAYLOAD_WEIGHT = 0                  # g
PAYLOAD_BAT_DRAIN = 0.00004927      # V/g

# Environment Parameters
WIND_SPEED = 1                      # km/h
WIND_ANGLE = -90                    # Degrees
WIND_BAT_DRAIN_BASE = 0.00001143    # V/(km/h)
WIND_BAT_DRAIN_DI_HEAD = 0.0038473  # V/(km/h)
WIND_BAT_DRAIN_DI_TAIL = -0.001525  # V/(km/h)
WIND_BAT_DRAIN_CROSS = 0.000986     # V/(km/h)

# Globals
START = 'A1'
END = 'N1'

nodes = {}
edges = []

# Import data
with open('drone_lab_nodes.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        nodes[row['src']] = (int(row['x']), int(row['y']), int(row['z']))
        edges += [(row['src'], row['destination'],
                   {'distance': float(row['distance'])})]


class WindDirection(Enum):
    NONE = 0
    HEAD = 1
    TAIL = 2
    CROSS = 3
    DIAGONAL_HEAD = 4
    DIAGONAL_TAIL = 5

# Miscellaneous helpers


def get_battery_percentage(cur_volts: float) -> int:
    "Converts the current battery volts to a percentage."
    return (cur_volts - MIN_VOLT) / (MAX_VOLT - MIN_VOLT)


def normalise_vector(x: float, y: float) -> tuple[float, float]:
    "Converts the vector to length one"
    if x == 0 and y == 0:
        return 0, 0
    length = math.sqrt(x*x + y*y)
    return x/length, y/length


def kmh_to_cms(kmh: float) -> float:
    "Converts km/h to cm/s"
    return kmh * 27.7778


def cms_to_kmh(cms: float) -> float:
    "Converts cm/s to km/h"
    return cms*0.36


def find_angle(x1, y1, x2, y2) -> float:
    "Finds the angle between two points."
    myradians = math.atan2(y2-y1, x2-x1)
    mydegrees = math.degrees(myradians)
    return mydegrees


def determine_direction(graph: nx.Graph,
                        a: str, b: str) -> WindDirection:
    "Determines if the wind is a head wind, tail wind, side wind, diagonal against wind, diagonal with wind, or no wind."

    if WIND_SPEED == 0:
        return WindDirection.NONE

    # Calculate drone angle
    start = graph.nodes[a]['pos']
    end = graph.nodes[b]['pos']
    drone_angle = find_angle(start[0], start[1], end[0], end[1])

    # Calculate wind angle relative to drone angle
    rel_wind_angle = WIND_ANGLE - drone_angle
    if rel_wind_angle > 180:
        rel_wind_angle -= 360
    elif rel_wind_angle < -180:
        rel_wind_angle += 360

    # Round to nearest category
    if (rel_wind_angle >= -180 and rel_wind_angle < -150) or \
       (rel_wind_angle <= 180 and rel_wind_angle > 150):
        return WindDirection.HEAD

    if (rel_wind_angle >= -150 and rel_wind_angle < -120) or \
       (rel_wind_angle <= 150 and rel_wind_angle > 120):
        return WindDirection.DIAGONAL_HEAD

    if (rel_wind_angle >= -120 and rel_wind_angle < -60) or \
       (rel_wind_angle <= 120 and rel_wind_angle > 60):
        return WindDirection.CROSS

    if (rel_wind_angle >= -60 and rel_wind_angle < -30) or \
       (rel_wind_angle <= 60 and rel_wind_angle > 30):
        return WindDirection.DIAGONAL_TAIL

    if (rel_wind_angle >= -30 and rel_wind_angle <= 0) or \
       (rel_wind_angle <= 30 and rel_wind_angle >= 0):
        return WindDirection.TAIL


def polar_to_cartesian(radius: float, angle: float) -> tuple[float, float]:
    "Converts polar coordinate radius and angle to Cartesian X and Y"
    x = radius * math.cos(angle * (math.pi / 180))
    y = radius * math.sin(angle * (math.pi / 180))
    return x, y

# Graph methods


def create_graph(nodes: dict, edges: list[tuple]) -> nx.Graph:
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
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels)
    plt.show()


def get_shortest_path(graph: nx.Graph, start: str, end: str) -> list[str]:
    "Returns the shortest path between the given start and end node."
    return nx.shortest_path(graph, source=start, target=end, weight='distance')


def get_k_shortest_paths(graph: nx.Graph, start: str, end: str, k: int) -> list[list[str]]:
    "Returns the k shortest paths between the given start and end node."
    generator = nx.shortest_simple_paths(graph, start, end, 'distance')
    paths = []
    for i, path in enumerate(generator):
        paths.append(path)
        if i >= k - 1:
            break
    return paths


def get_path_distance(graph: nx.Graph, path: list[str]) -> float:
    "Gets the total distance of the given path."
    return nx.path_weight(graph, path, 'distance')


def get_leg_length(graph: nx.Graph, a: str, b: str) -> float:
    "Gets the distance of the edge between the given nodes."
    try:
        return graph.edges[a, b]['distance']
    except:
        return None


def estimate_leg_time(graph: nx.graph, a: str, b: str) -> float:
    length = get_leg_length(graph, a, b)
    if not length:
        return -1
    return length / CRUISE_SPEED


def estimate_wind_bat_naive(graph: nx.Graph, a: str, b: str, time: float) -> float:
    "Estimates the naive battery drain due to the wind."
    start = graph.nodes[a]['pos']
    end = graph.nodes[b]['pos']
    dx = end[0] - start[0]
    dy = end[1] - start[1]

    x_vel = dx / time
    y_vel = dy / time

    # Relative wind speed to the drone
    wind_x, wind_y = polar_to_cartesian(WIND_SPEED, WIND_ANGLE)
    rel_x_vel = wind_x - cms_to_kmh(x_vel)
    rel_y_vel = wind_y - cms_to_kmh(y_vel)

    bat = 0
    bat += time * (WIND_BAT_DRAIN_BASE * rel_x_vel)
    bat += time * (WIND_BAT_DRAIN_BASE * rel_y_vel)

    return bat


def estimate_wind_bat_paths(graph: nx.Graph, a: str, b: str, time: float) -> float:
    "Estimates the battery drain due to the wind using path data."

    wind_effect = determine_direction(graph, a, b)
    drain_rate = 0
    if wind_effect == WindDirection.HEAD:
        drain_rate = WIND_BAT_DRAIN_BASE
    elif wind_effect == WindDirection.TAIL:
        drain_rate = -WIND_BAT_DRAIN_BASE
    elif wind_effect == WindDirection.CROSS:
        drain_rate = WIND_BAT_DRAIN_CROSS
    elif wind_effect == WindDirection.DIAGONAL_HEAD:
        drain_rate = WIND_BAT_DRAIN_DI_HEAD
    elif wind_effect == WindDirection.DIAGONAL_TAIL:
        drain_rate = WIND_BAT_DRAIN_DI_TAIL

    return time * (drain_rate * WIND_SPEED)


def estimate_leg_bat(graph: nx.Graph, a: str, b: str, time: float = None) -> float:
    "Estimates the voltage drop of the drone for the given flight leg."
    leg_time = time
    if not leg_time:
        leg_time = estimate_leg_time(graph, a, b)

    # Calculate base battery drain
    leg_bat = leg_time * BASE_BAT_DRAIN

    # Consider payload
    leg_bat += leg_time * (PAYLOAD_BAT_DRAIN * PAYLOAD_WEIGHT)

    # Consider wind
    # leg_bat += estimate_wind_bat_naive(graph, a, b, leg_time)
    leg_bat += estimate_wind_bat_paths(graph, a, b, leg_time)

    return leg_bat


def estimate_total_time_bat(graph: nx.Graph, path: list[str]) -> float:
    "Estimates the total flight time and battery usage for the given path."
    if len(path) < 2:
        return -1
    time = 0.0
    battery = 0.0
    for i in range(1, len(path)):
        # Calculate flying time
        leg_time = estimate_leg_time(graph, path[i - 1], path[i])
        if leg_time < 0:
            continue

        # Consider take of and landing time
        time += leg_time + TAKEOFF_LANDING_TIME

        # Calculate battery drain
        battery += estimate_leg_bat(graph, path[i - 1], path[i], leg_time)
    return (time, battery)


random.seed(RAND_SEED)

G = create_graph(nodes, edges)
paths = get_k_shortest_paths(G, START, END, 10)

estimates = [estimate_total_time_bat(G, path) for path in paths]

for i in range(len(estimates)):
    print("Path {}:\n  Time (s): {}\n  Battery (V): {}".format(
        paths[i], estimates[i][0], estimates[i][1]
    ))
