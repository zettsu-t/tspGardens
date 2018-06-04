#!/usr/bin/python3
# coding: utf-8

'''
Find a route of a large garden
usage:
python3 tsp_gardens.py [--input input-png-filename] [--map input-map-filename] [--output output-png-basename]
+ input-png-filename : PNG file describing a geographical map
+ input-map-filename : Plane text file describing vertices and edges of the map
+ output-png-basename : Basename of output PNG files
'''

import math
import re
from collections import defaultdict
from collections import namedtuple
from optparse import OptionParser
import cv2
from dijkstar import Graph, find_path
import numpy as np

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

class RoadMap(object):
    '''Road map which consists of vertices and edges'''

    def __init__(self, options):
        self.options = options
        self.vertices, self.edges = self.parse(options.input_map_filename)
        self.edges, self.vertex_to_edge = self.calculate_costs(self.vertices, self.edges)

    def solve(self):
        '''Solve to find a route to visit all edges'''
        self.paths, self.cost_matrix = self.find_shortest_paths(self.vertices, self.edges, self.vertex_to_edge)
        self.route = self.find_route(self.vertices[0].index, self.edges,
                                     self.vertex_to_edge, self.paths, self.cost_matrix)
        self.route = self.fill_route(self.vertices[0].index, self.paths, self.route)
        print(self.route)

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
            vertex_to_edge[edge.start].append(index)
            vertex_to_edge[edge.end].append(index)

        return new_edges, vertex_to_edge

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
            new_ofs = count * 8

            span = 32
            phase = (route_index // span) % 2
            weight = (route_index % span) / span
            color_value = weight if (phase == 0) else (1.0 - weight)
            color_value = 64 + int(color_value * 128)
            color = (255, color_value, color_value)
            px, py = scale(vertex.x, vertex.y)
            if prev_px is not None:
                cv2.arrowedLine(img=img, pt1=(prev_px + ofs, prev_py + ofs),
                                pt2=(px + new_ofs, py + new_ofs), color=color,
                                thickness=2, tipLength=0.2)
            prev_px = px
            prev_py = py
            ofs = new_ofs

        for vertex in vertices:
            px, py = scale(vertex.x, vertex.y)
            tx = ','.join(map(str, vertex_visited[vertex.index]))
            cv2.putText(img, tx, (px, py), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), thickness=1)

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
    (options, args) = parser.parse_args()

    road_map = RoadMap(options)
    road_map.solve()
    road_map.draw_map(options.input_image_filename, options.output_image_basename + '_map.png')
    road_map.draw_route(options.input_image_filename, options.output_image_basename + '_route.png')

if __name__ == '__main__':
    main()
