### Rutgers Algorithmic Robotics and Control Lab
### Author: Jingjin Yu Email: jingjinyu@gmail.com
### Coded and tested using Python 3.8.3
import helper;
import math;
import problemgen;
import networkx as nx;

def _process_cycle(plan, cycles, cycle_index, start_cell, cycle_tree, jump_map):
    cycle = helper.reorder_cycle(cycles[cycle_index], start_cell)
    for i in range(len(cycle)):
        item = cycle[i]
        plan.append(item)
        
        # Check wether the item is a place to jump to another cycle 
        if item in jump_map:
            next_jump = jump_map[item]
            for j in range(len(next_jump)):
                [next_cycle, next_cell, _] = next_jump[j]
                _process_cycle(plan, cycles, next_cycle, next_cell, cycle_tree, jump_map)

    plan.append(start_cell)

'''
Compute LTR solution
pltr: the LTR problem instance, as a list representing a permutation
n: the side length of the n x n lattice
''' 
def compute_ltr_solution(pltr, n):
    # Collect all cycles 
    cycles = helper.get_cycles(pltr)

    # Compute # of pick-n-swaps
    pns = helper.compute_pns(cycles)

    # Compute distances between cycles by processing each cycle 
    # For each pair of consecutive vertices, we compute its distance 
    # to all other vertices of all other cycles. This take O(n^3)
    # but can be made more efficient 
    cycle_dist_map = dict()
    for ci in range(len(cycles)):
        # Work with the first cycle, pair by pair 
        c1 = cycles[ci]
        darr = [n*2 for i in range(len(cycles))]
        darr2 = [n*2 for i in range(len(cycles))]
        for si in range(len(c1)):
            v1 = c1[si]
            v2 = c1[(si+1)%len(c1)]
            d12 = helper.compute_dist_2d(n, v1, v2)

            for cj in range(len(cycles)):
                if ci == cj : continue
                c2 = cycles[cj]
                for sj in range(len(c2)):
                    d1 = helper.compute_dist_2d(n, v1, c2[sj])
                    d2 = helper.compute_dist_2d(n, v2, c2[sj])
                    d = d1 + d2 - d12
                    #print(d, v1, v2, c2[sj])
                    darr2[cj] = darr2[cj] if darr[cj] <= d else [v1, v2, c2[sj]]
                    darr[cj] = darr[cj] if darr[cj] <= d else d
    
        # Now we have darr containing shorest distance from ci to 
        # all other cycles, add to map
        cycle_dist_map[ci] = dict()
        for cj in range(len(cycles)):
            if ci == cj : continue
            cycle_dist_map[ci][cj]= [darr[cj] + 0.0001, darr2[cj]]

    # Compute distance from end-effector rest position to all cycles
    cycle_dist_map[-1] = dict()
    for i in range(len(cycles)):
        d = n*n
        start_cell = -1
        for v in cycles[i]:
            dv = helper.compute_dist_2d(n, 0, v)
            start_cell = start_cell if d <= dv else v
            d = d if d <= dv else dv
        cycle_dist_map[-1][i] = [d*2 + 0.0001, start_cell]

    # Construct a directed graph for directed MST computation 
    g = nx.DiGraph()    
    for v1 in cycle_dist_map.keys():
        v2s = cycle_dist_map[v1]
        for v2 in v2s.keys():
            v1v2d = v2s[v2][0]
            g.add_edge(v1, v2, weight=v1v2d)
    branching = nx.minimum_spanning_arborescence(g)

    # Construct the plan, starting from -1 (end-effector rest)
    jump_map = dict()
    mst_tree = dict()
    mst_tree [-1] = []
    for i in range(len(cycles)):
        mst_tree[i] = []

    for (v1, v2) in branching.edges():
        mst_tree[v1].append(v2)
        # Populate jump_map
        if v1 != -1:
            jump = cycle_dist_map[v1][v2][1]
            if jump[0] not in jump_map:
                jump_map[jump[0]] = []
            jump_map[jump[0]].append([v2, jump[2], jump[1]])

    plan = [-1]
    start_cycle = mst_tree[-1][0]
    start_cell = cycle_dist_map[-1][start_cycle][1]
    _process_cycle(plan, cycles, start_cycle, start_cell, mst_tree, jump_map)

    cyclecopy = plan.copy()
    cyclecopy[0] = 0
    mst_distance = helper.compute_cycle_dist_2d_basic(cyclecopy, n)

    # Compute solution distance 
    (cycle_sweep_plan, _, cycle_sweep_distance) = helper.compute_cycle_sweep_solution_distance_tr(pltr, cycles, n)

    return (pns, plan, mst_distance, cycle_sweep_plan, cycle_sweep_distance )


plor = problemgen.get_lor_instance(16)
print(f"LTR instance (4 x 4): {plor}")
(pns, plan, mst_distance, cycle_sweep_plan, cycle_sweep_distance) = compute_ltr_solution(plor, 4)

print (f"\n# of pick-n-swaps: {pns}, mst distance: {mst_distance}, cycle sweep distance: {cycle_sweep_distance}\n")
print (f"MST cycle merge rearrangement plan:\n {plan} \n")
print (f"Cycle sweep rearrangement plan:\n {cycle_sweep_plan} \n")

