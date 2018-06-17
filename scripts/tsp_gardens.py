#!/usr/bin/python3
# coding: utf-8

'''
Find a route of a large garden
usage:
python3 tsp_gardens.py [--input input-png-filename] [--map input-map-filename]
                       [--output output-png-basename] [--solver 0 or 1]
+ input-png-filename : PNG file describing a geographical map
+ input-map-filename : Plane text file describing vertices and edges of the map
+ output-png-basename : Basename of output PNG files
+ solver : Using OR-Tools (1) or default (0)
'''

import math
import re
from collections import defaultdict
from collections import namedtuple
from optparse import OptionParser
import cv2
from dijkstar import Graph, find_path
import networkx as nx
import numpy as np
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

## Elements of routes
## Crossing
Vertex = namedtuple('Vertex', ('x', 'y', 'index'))
## Two-way road without crossings except its ends
Edge = namedtuple('Edge', ('start', 'end', 'cost', 'index'))
## Chain of roads : 'via' contains vertices and includes each end
Path = namedtuple('Path', ('via'))

NOTICES = [[30, -235, 0.8, 2, 'This image is a derived work and modified from a map of which'],
           [30, -205, 0.8, 2, 'Geospatial Information Authority of Japan owes copyright under the license shown below'],
           [30, -175, 0.8, 2, 'http://www.gsi.go.jp/kikakuchousei/kikakuchousei40182.html'],
           [30, -145, 0.8, 2, 'https://maps.gsi.go.jp/development/ichiran.html'],
           [30, -115, 0.8, 2, 'The original map is available at'],
           [30, -95, 0.5, 2, 'https://maps.gsi.go.jp/#18/35.051342/135.759140/&base=std&ls=std&disp=1&vs=c1j0h0k0l0u0t0z0r0s0f1&reliefdata=0G000000']]

SOLVER_DEFAULT = 0
SOLVER_OR_TOOLS_TSP = 1

# Distance callback
class CreateDistanceCallback(object):
    def __init__(self, matrix):
        self.matrix = matrix

    def __call__(self, from_node, to_node):
        return self.matrix[from_node][to_node]

class RoadMap(object):
    '''Road map which consists of vertices and edges'''

    def __init__(self, options):
        self.options = options
        self.solver = options.solver
        self.vertices, edges = self.parse(options.input_map_filename)
        self.edges, self.edge_costs, self.vertex_to_edge = self.calculate_costs(self.vertices, edges)
        if self.solver == SOLVER_OR_TOOLS_TSP:
            self.graph, self.alt_vertices, self.edge_distances = self.make_graph(self.edges, self.edge_costs)

    def solve(self):
        '''Solve to find a route to visit all edges'''
        if self.solver == SOLVER_OR_TOOLS_TSP:
            self.paths, self.cost_matrix = self.find_shortest_paths_in_graph(self.graph)
            self.route = self.find_route_in_graph(self.alt_vertices, self.edge_distances,
                                                  self.paths, self.cost_matrix)
        else:
            self.paths, self.cost_matrix = self.find_shortest_paths(self.vertices, self.edges, self.vertex_to_edge)
            route = self.find_route(self.vertices[0].index, self.edges,
                                    self.vertex_to_edge, self.paths, self.cost_matrix)
            self.route = self.fill_route(self.vertices[0].index, self.paths, route)
        self.print_route(self.route, self.edge_costs)

    def draw_map(self, in_image_filename, out_image_filename):
        '''Superimposes vertex numbers'''
        self._draw_map(in_image_filename, out_image_filename, self.vertices)

    def draw_route(self, in_image_filename, out_image_filename):
        '''Superimposes the route'''
        self._draw_route(in_image_filename, out_image_filename, self.vertices, self.route)

    def parse(self, filename):
        '''Parses a garden map'''
        vertices = []
        edges = []
        parser = self.parse_nothing
        target = []

        # Discard BOM
        with open(filename, 'r', encoding='utf_8_sig') as infile:
            line = infile.readline()
            while line:
                # Detecting #s and a keyword
                match_obj = re.match(r'^\s*#+\s*(\S+)', line)
                if match_obj is not None:
                    # Input vertices or edges the following lines
                    keyword = match_obj[1].lower()
                    if keyword in ['vertex', 'vertexes', 'vertices']:
                        parser = self.parse_vertex
                        target = vertices
                    elif keyword in ['edge', 'edges']:
                        parser = self.parse_edge
                        target = edges
                else:
                    parser(line, target)
                line = infile.readline()

        return vertices, edges

    def parse_nothing(self, line, _):
        '''Parses nothing'''
        pass

    def parse_vertex(self, line, vertices):
        '''Parses a vertex in a 2D position'''
        words = list(filter(None, re.split('\s+', line)))
        if len(words) >= 2:
            nums = list(map(float, words[0:2]))
            index = len(vertices)
            vertices.append(Vertex(nums[0], nums[1], index))

    def parse_edge(self, line, edges):
        '''Parses an edge with optional cost'''
        words = list(filter(None, re.split('\s+', line)))
        if len(words) >= 2:
            start_vertex = int(words[0])
            end_vertex = int(words[1])
            cost = None if len(words) <= 2 else float(words[2])
            index = len(edges)
            edges.append(Edge(start_vertex, end_vertex, cost, index))

    def calculate_costs(self, vertices, edges):
        '''Calculate implicit costs of edges in the Euclidean distances'''
        new_edges = []
        edge_costs = {}
        vertex_to_edge = defaultdict(list)

        for index, edge in enumerate(edges):
            cost = edge.cost
            if edge.cost is None:
                vertex_start = vertices[edge.start]
                vertex_end = vertices[edge.end]
                x_diff = (vertex_start.x - vertex_end.x)
                y_diff = (vertex_start.y - vertex_end.y)
                cost = math.sqrt(x_diff * x_diff + y_diff * y_diff)

            # Notice that tuples are immutable
            new_edge = Edge(start=edge.start, end=edge.end, cost=cost, index=index)
            new_edges.append(new_edge)
            edge_costs[(edge.start, edge.end)] = cost
            edge_costs[(edge.end, edge.start)] = cost
            vertex_to_edge[edge.start].append(index)
            vertex_to_edge[edge.end].append(index)

        return new_edges, edge_costs, vertex_to_edge

    def make_graph(self, edges, edge_costs):
        '''Make a graph object'''
        graph = nx.Graph()
        for edge in edges:
            vertex_u, vertex_v = (edge.start, edge.end) if edge.start < edge.end else (edge.end, edge.start)
            graph.add_edge(vertex_u, vertex_v, weight=edge.cost)

        # Graph which has edges as vertices of 'graph' and vertices as edges of 'graph'
        alt_graph = nx.line_graph(graph)

        # Original vertex pair indexes to original edges
        alt_vertix_index_to_original_edges = {}
        # Original edges (pairs index) to original vertex pair indexes
        original_edge_to_alt_vertex_index = {}
        for index, original_edge in enumerate(alt_graph.nodes()):
            # The vertex is an original edge and pair of original vertex
            vertex_u, vertex_v = original_edge
            edge = (vertex_u, vertex_v) if vertex_u < vertex_v else (vertex_v, vertex_u)
            alt_vertix_index_to_original_edges[index] = edge
            original_edge_to_alt_vertex_index[(vertex_u, vertex_v)] = index
            original_edge_to_alt_vertex_index[(vertex_v, vertex_u)] = index

        alt_edges = {}
        max_weight = 0
        for index, alt_edge in enumerate(alt_graph.edges()):
            edge0_index = original_edge_to_alt_vertex_index[alt_edge[0]]
            edge1_index = original_edge_to_alt_vertex_index[alt_edge[1]]
            weight = (edge_costs[alt_edge[0]] + edge_costs[alt_edge[1]]) / 2.0
            alt_edges[(edge0_index, edge1_index)] = weight
            alt_edges[(edge1_index, edge0_index)] = weight
            max_weight = max(weight, max_weight)

        size = len(alt_graph.nodes())
        penalty = size * max_weight * 3
        edge_distances = []
        for from_index in range(0, size):
            row = []
            for to_index in range(0, size):
                edge0 = (from_index, to_index)
                edge1 = (to_index, from_index)
                if from_index == to_index:
                    weight = 0.0
                elif edge0 in alt_edges:
                    weight = alt_edges[edge0]
                elif edge1 in alt_edges:
                    weight = alt_edges[edge1]
                else:
                    weight = penalty
                row.append(weight)
            edge_distances.append(row)

        return graph, alt_vertix_index_to_original_edges, CreateDistanceCallback(edge_distances)

    def find_shortest_paths(self, vertices, edges, vertex_to_edge):
        '''Search paths for each vertices and its rests'''
        size = len(vertices)

        # Make an Path[size][size]
        paths = []
        for _ in range(0, size):
            paths.append([])

        # Non-directed graph
        graph = Graph()
        for edge in edges:
            graph.add_edge(edge.start, edge.end, {'cost': edge.cost})
            graph.add_edge(edge.end, edge.start, {'cost': edge.cost})

        # Measuring costs in the Dijkstra algorithm
        # We can halve counts of this loop for non-directed graphs
        cost_matrix = np.zeros((size, size), dtype=float)
        cost_func = lambda _1, _2, edge, _3: edge['cost']
        for from_index in range(0, size):
            for to_index in range(0, size):
                result = find_path(graph, from_index, to_index, cost_func=cost_func)
                nodes = result.nodes
                # No detour
                if from_index in vertex_to_edge:
                    if to_index in vertex_to_edge[from_index]:
                        nodes = [from_index, to_index]
                if to_index in vertex_to_edge:
                    if from_index in vertex_to_edge[to_index]:
                        nodes = [from_index, to_index]
                paths[from_index].append(Path(nodes))
                cost_matrix[from_index, to_index] = result.total_cost

        return paths, cost_matrix

    def find_shortest_paths_in_graph(self, graph):
        '''Search paths for each vertices and its rests'''
        paths = dict(nx.all_pairs_dijkstra_path(graph), weight='weight')
        cost_matrix = dict(nx.all_pairs_dijkstra_path_length(graph), weight='weight')
        return paths, cost_matrix

    def find_route(self, initial_vertex, edges, edge_map, paths, cost_matrix):
        '''Find a route to visit each edge once or more'''
        from_index = initial_vertex
        route = [from_index]
        unvisited_edges = set([edge.index for edge in edges])

        while unvisited_edges:
            next_index = None
            candidate_edges = set(edge_map[from_index]) & unvisited_edges
            if candidate_edges:
                # Can visit an unvisited edge directly
                next_index = self.visit_adjacent_edge(from_index, edges, candidate_edges, route, unvisited_edges)
            if next_index is None:
                # Cannot visit an unvisited edge directly
                next_index = self.visit_far_edge(from_index, edges, paths, cost_matrix, route, unvisited_edges)
            from_index = next_index

        route.append(initial_vertex)
        return route

    def fill_route(self, initial_vertex, paths, route):
        '''Fill paths for non-directly connected vertices'''
        new_route = []
        pre_vertex = None
        for vertex in route:
            if pre_vertex is not None:
                path = paths[pre_vertex][vertex].via
                new_route.extend(path[:-1])
            pre_vertex = vertex

        if new_route[-1] != initial_vertex:
            new_route.append(initial_vertex)
        return new_route

    def visit_adjacent_edge(self, from_index, edges, candidate_edges, route, unvisited_edges):
        '''Visits an unvisited edge directly'''
        next_index = None
        start_edge_index = None
        end_edge_index = None
        start_edge = None
        end_edge = None

        candidates = list(sorted(candidate_edges, key=lambda edge: edges[edge].cost))
        for edge_index in candidates:
            edge = edges[edge_index]
            if edge.start == from_index:
                start_edge_index = edge_index
                start_edge = edge
                break

        for edge_index in candidates:
            edge = edges[edge_index]
            if edge.end == from_index:
                end_edge_index = edge_index
                end_edge = edge
                break

        if start_edge is not None:
            if end_edge is None or start_edge.cost < end_edge.cost:
                unvisited_edges.remove(start_edge_index)
                next_index = start_edge.end

        if next_index is None and end_edge is not None:
            unvisited_edges.remove(end_edge_index)
            next_index = end_edge.start

        if next_index is not None:
            route.append(next_index)
        return next_index

    def visit_far_edge(self, from_index, edges, paths, cost_matrix, route, unvisited_edges):
        '''Visits an far unvisited edge'''
        candidates = list(unvisited_edges)
        edge_start_index = sorted(candidates, key=lambda edge: cost_matrix[from_index, edges[edge].start])[0]
        edge_end_index = sorted(candidates, key=lambda edge: cost_matrix[from_index, edges[edge].end])[0]
        edge_start = edges[edge_start_index]
        edge_end = edges[edge_end_index]
        cost_start = cost_matrix[from_index, edge_start.start]
        cost_end = cost_matrix[from_index, edge_end.end]

        if cost_start < cost_end:
            route.extend(paths[from_index][edge_start.start].via[1:])
            unvisited_edges.remove(edge_start_index)
            next_index = edge_start.end
        else:
            route.extend(paths[from_index][edge_end.end].via[1:])
            unvisited_edges.remove(edge_end_index)
            next_index = edge_end.start

        if next_index is not None:
            route.append(next_index)
        return next_index

    def find_route_in_graph(self, alt_vertices, edge_distances, paths, cost_matrix):
        '''
        Find a route to visit each edge once or more.
        modified from https://developers.google.com/optimization/routing/tsp
        '''
        routing = pywrapcp.RoutingModel(len(alt_vertices), 1, 0)
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        routing.SetArcCostEvaluatorOfAllVehicles(edge_distances)
        assignment = routing.SolveWithParameters(search_parameters)

        index = routing.Start(min(list(alt_vertices.keys())))
        route = []
        is_last = False

        while True:
            if is_last:
                vertex_u, vertex_v = (route[-1], route[0])
            else:
                vertex_u, vertex_v = alt_vertices[routing.IndexToNode(index)]

            if route:
                sub_route = []
                if route[-1] == vertex_u:
                    route.append(vertex_v)
                elif route[-1] == vertex_v:
                    route.append(vertex_u)
                else:
                    vertex_last = route[-1]
                    cost_u = cost_matrix[vertex_last][vertex_u]
                    cost_v = cost_matrix[vertex_last][vertex_v]
                    if cost_u < cost_v:
                        route.extend(paths[vertex_last][vertex_u][1:])
                        if route[-1] != vertex_v:
                            route.append(vertex_v)
                    else:
                        route.extend(paths[vertex_last][vertex_v][1:])
                        if route[-1] != vertex_u:
                            route.append(vertex_u)
            else:
                route = [vertex_u, vertex_v]
            if is_last:
                break
            else:
                index = assignment.Value(routing.NextVar(index))
                is_last = True if routing.IsEnd(index) else False
        return route

    def print_route(self, route, edge_costs):
        '''Print a route and its cost'''
        print(self.route)

        cost = 0.0
        pre_vertex = route[0]
        for vertex in route[1:]:
            cost += edge_costs[(pre_vertex, vertex)]
            pre_vertex = vertex
        print(cost)

    def _draw_map(self, in_image_filename, out_image_filename, vertices):
        '''Superimposes vertex numbers'''
        # Count when each vertex is visited on the route
        img = cv2.imread(in_image_filename).copy()

        for index, vertex in enumerate(vertices):
            px = int(vertex.x)
            py = int(vertex.y)
            tx = '{}'.format(index)
            cv2.putText(img, tx, (px, py), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (64, 64, 64), thickness=2)

        self._draw_notices(img)
        img = self.add_alpha(img)
        cv2.imwrite(out_image_filename, img)

    def _draw_route(self, in_image_filename, out_image_filename, vertices, route):
        '''Superimposes the route'''
        # Count when each vertex is visited on the route
        vertex_visited = defaultdict(list)
        for route_index, vertex_index in enumerate(route):
            vertex_visited[vertex_index].append(route_index)

        img = cv2.imread(in_image_filename).copy()

        def scale(x, y):
            return int(x), int(y)

        prev_px = None
        prev_py = None
        ofs = 0
        for route_index, vertex_index in enumerate(route):
            vertex = vertices[vertex_index]
            count = max([0, vertex_visited[vertex_index].index(route_index)])
            new_ofs = count * 4

            hue = int(route_index * 254 / len(route))
            hsv = np.uint8([[[hue, 255, 160]]])
            color = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0,0]
            color = (int(color[0]), int(color[1]), int(color[2]))
            px, py = scale(vertex.x, vertex.y)
            if prev_px is not None:
                cv2.arrowedLine(img=img, pt1=(prev_px + ofs, prev_py + ofs),
                                pt2=(px + new_ofs, py + new_ofs), color=color,
                                thickness=1, tipLength=0.1)
            prev_px = px
            prev_py = py
            ofs = new_ofs

        for vertex in vertices:
            px, py = scale(vertex.x, vertex.y)
            tx = ','.join(map(str, vertex_visited[vertex.index]))
            cv2.putText(img, tx, (px, py), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), thickness=1)

        self._draw_notices(img)
        img = self.add_alpha(img)
        cv2.imwrite(out_image_filename, img)

    def _draw_notices(self, img):
        '''Superimposes notices'''
        for notice in NOTICES:
            x, y, fontsize, thickness, line = notice
            cv2.putText(img, line, (x, img.shape[1] + y), cv2.FONT_HERSHEY_SIMPLEX,
                        fontsize, (32, 32, 32), thickness=thickness)

    def add_alpha(self, img):
        '''Add transparent pixels to prevent from being converted to JPEGs'''
        # https://stackoverflow.com/questions/32290096/python-opencv-add-alpha-channel-to-rgb-image
        b_channel, g_channel, r_channel = cv2.split(img)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255
        alpha_channel[0,0] = 0
        img_BGRA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
        return img_BGRA

def main():
    parser = OptionParser()
    parser.add_option('-i', '--input', dest='input_image_filename', default='data/in.png',
                      help='PNG file describing a geographical map')
    parser.add_option('-m', '--map', dest='input_map_filename', default='data/tsp_gardens.txt',
                      help='Plane text file describing vertices and edges of the map')
    parser.add_option('-o', '--output', dest='output_image_basename', default='out',
                      help='Basename of output PNG files')
    parser.add_option('-s', '--solver', type='int', dest='solver', default=0,
                      help='Select solver (0 or 1)')
    (options, args) = parser.parse_args()

    road_map = RoadMap(options)
    road_map.solve()
    road_map.draw_map(options.input_image_filename, options.output_image_basename + '_map.png')
    road_map.draw_route(options.input_image_filename, options.output_image_basename + '_route.png')

if __name__ == '__main__':
    main()
