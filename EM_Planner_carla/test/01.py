# -*- coding: utf-8 -*-
# @Time    : 2023/5/23 21:11
# @Author  : Star
# @File    : 01.py
# @Software: PyCharm
class Human:
    def __init__(self, name, height):
        self.name = name
        self.height = height
        print(self.name)
        print(self.height)

    def show_all(self, name, height):
        pass


# Human("star", 180)

import matplotlib.pyplot as plt
import networkx as nx

G = nx.DiGraph()
G.add_node('A')
G.add_node('B')
G.add_edge('A', 'B')
print(G.nodes())
print(G.edges())

nx.draw(G, with_labels=True)
plt.show()
