# -*- coding: utf-8 -*-
"""
Created on Mon Jul  5 21:33:35 2021

@author: dhruv
"""

# Nearest Neighbor
import math
# THIS is our current position. Could be any coordinate.
# PLEASE assume all coordinates are positive.
our_position = [0, 0]
neighbors = [
[1, 1],
[2, 1],
[2, 2],
[2, 3],
]
def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
def euler(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
# This dict will use neighbors coordinate as key,
# and use `[<manhattan distance>, <euler distance>]` as value in dict.
# Since we may use this dict for other purposes, we have use `dict` to
# find the distance relations with original neighbors.
distances = {}
for nei in neighbors:
    dist_man = manhattan(our_position, nei)
    dist_eu = euler(our_position, nei)
    distances[tuple(nei)] = [dist_man, dist_eu]
# Output sorted result.
result = list(distances.values())
result.sort()
print("This is my result:")
print(result)