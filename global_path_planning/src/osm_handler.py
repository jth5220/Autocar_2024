#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import osmium as osm

class OSMHandler(osm.SimpleHandler):
    def __init__(self):
        osm.SimpleHandler.__init__(self)
        self.way_nodes = {} # 주행유도선 노드 저장 딕셔너리
        self.ways = {} # 주행유도선 way 저장 딕셔너리

        self.mission_nodes = {} # 미션구역 노드 저장 딕셔너리 / node_id:(x,y)
        self.mission_areas = {} # 미션구역 area 저장 딕셔너리 (area: 폐곡선 way) / way_id:[node1, node2, ...]
        self.mission_types = {} # 미션구역 타입 저장 딕셔너리 / way_id:mission_type

        self.codes = {}

        self.index_node = None
        self.i = 1
        self.indicies_node = {}

        self.index_way = 1

    def node(self, n):
        code, x, y = int(n.tags.get('code', '-1')), float(n.tags['x']), float(n.tags['y'])
        self.index_node = code * 10000 + self.i
        self.i += 1
        self.indicies_node[n.id] = self.index_node

        # 주행유도선일 때
        if code == 0 or code == 1:
            self.way_nodes[self.index_node] = (x, y)

        # 미션 구역일 때
        elif code == 2:
            self.mission_nodes[self.index_node] = (x, y)
            
        # 신호등 구역일 때
        
        # code 저장
        self.codes[self.index_node] = code
        return

    def way(self, w):
        # print("Way ID: ", w.id)
        # print("Start Node ID: ", w.nodes[0].ref)
        # print("End Node ID: ", w.nodes[-1].ref)

        # 하나의 way에서 모든 node가 같은 code를 가진다고 가정
        # => 첫 번째 node를 way의 code로 판단
        w_code = int(w.tags.get('code', '-1'))

        # 주행유도선일 때
        if w_code == 0 or w_code == 1:
            self.ways[self.index_way] = [self.indicies_node[node.ref] for node in w.nodes]
        
        # 미션구역일 때
        elif w_code == 2:
            self.mission_areas[self.index_way] = [self.indicies_node[node.ref] for node in w.nodes]
            self.mission_types[self.index_way] = w.tags.get('mission', 'None')

        self.index_way += 1
        return

    def reset_index(self):
        self.i = 1
        self.indicies_node = {}

    def import_file(self, file):
        self.apply_file(file)
        self.reset_index()