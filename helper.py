### Rutgers Algorithmic Robotics and Control Lab
### Author: Jingjin Yu Email: jingjinyu@gmail.com
### Coded and tested using Python 3.8.3
import math;
import numpy as np;
from scipy.optimize import linear_sum_assignment;
import networkx as nx;


def get_cycles(pi):
    # Collect all cycles 
    n = len(pi)

    cycles = []                 # List of cycles
    processedItemSet = set()    # Processed items 

    # Add all items into a list for processing 
    queue = []
    for i in range(0, n):
        queue.append(i)

    # Process the queue and get cycles 
    while (len(queue) > 0):
        # Retrive first item 
        index = queue.pop(0)
        item = pi[index]

        # Check whether item has already been processed 
        if item in processedItemSet:
            continue

        # Add item as processed
        processedItemSet.add(item)
       
        # Create a cycle with the first item 
        cycle = [item]

        # Follow the cycle 
        while index != item:
            # Get the next item in the cycle, add as processed
            item = pi[item]
            processedItemSet.add(item)

            # Add to cycle
            cycle.append(item)

        # We have the cycle, add to proper data structure
        # if it is longer than 1 
        if(len(cycle)>1):
            cycles.append(cycle)

    return cycles

def compute_pns(cycles):
    pns = 0
    for c in cycles:
        pns = pns + len(c) + 1
    return pns

def get_cycle_range(cycle, n):
    min = n 
    max = -1
    for item in cycle:
        min = min if min < item else item
        max = max if max > item else item
    return (min, max)

def get_cycle_group_range(cycles, n):
    min = n 
    max = -1
    for cycle in cycles:
        for item in cycle:
            min = min if min < item else item
            max = max if max > item else item
    return (min, max)

def compute_range(cycles, n):
    ranges = []
    for c in range(0, len(cycles)):
        cycle = cycles[c]
        min = n
        max = -1
        for i in range(0, len(cycle)):
            if cycle[i] < min: 
                min = cycle[i]
            if cycle[i] > max:
                max = cycle[i]
        ranges.append([min, max])

    return ranges

def compute_cycle_dist(cycle):
    dist = 0
    for i in range(len(cycle)-1):
        dist = dist + abs(cycle[i+1] - cycle[i])
    dist = dist + abs(cycle[0] - cycle[len(cycle) - 1])
    return dist

def compute_dist_2d(n, a, b):
    ax = a // n
    ay = a % n
    bx = b // n
    by = b % n
    return math.sqrt((bx - ax)*(bx - ax) + (by - ay)*(by - ay))

def compute_cycle_dist_2d(cycle, n, iis):
    dist = 0
    for i in range(len(cycle)-1):
        dist = dist + compute_dist_2d(n, iis[cycle[i+1]], iis[cycle[i]])
    dist = dist + compute_dist_2d(n, iis[cycle[0]], iis[cycle[len(cycle) - 1]])
    return dist

def compute_cycle_dist_2d_basic(cycle, n):
    dist = 0
    for i in range(len(cycle)-1):
        dist = dist + compute_dist_2d(n, cycle[i+1], cycle[i])
    dist = dist + compute_dist_2d(n, cycle[0], cycle[len(cycle) - 1])
    return dist

def get_closest_cycle(n, v, cycles):
    d0 = n*n
    c0 = -1
    v0 = -1
    for c in range(len(cycles)):
        for v in cycles[c]:
            dv = compute_dist_2d(n, v, 0)
            if dv < d0: 
                d0 = dv
                c0 = cycles[c]
                v0 = v
    return (d0, c0, v0)

def compute_cycle_sweep_solution_distance_tr(ins, cycles, n):
    dist = 0
    distcs = 0

    # Compute a inverse array that gives locations of items
    iis = [0 for i in range(len(ins))]
    for i in range(len(ins)):
        iis[ins[i]] = i

    # Compute distances for all cycles
    for c in range(len(cycles)):
        dist = dist + compute_cycle_dist_2d(cycles[c], n, iis)

    # Compute sweeping distance & plan, first locate cycle closest to 0
    cyclescopy = cycles.copy()
    v0 = 0
    plan = [-1]
    while len(cyclescopy) > 0: 
        (d0, c0, v0) = get_closest_cycle(n, v0, cyclescopy)
        distcs = distcs + d0
        cyclescopy.remove(c0)
        new_cycle = reorder_cycle(c0, v0)
        plan.extend(new_cycle)
        plan.append(new_cycle[0])
    distcs = distcs + compute_dist_2d(n, v0, 0)

    return (plan, dist, dist + distcs)

def get_cycles_por(pi, n, k):
    # Collect all cycles 
    cycles = []                 # List of cycles
    processedIndexSet = set()    # Processed items 

    # Add all items into a list for processing 
    queue = []
    for i in range(0, n*k):
        queue.append(i)

    # Process the queue and get cycles 
    while (len(queue) > 0):
        # Retrive first index and the item in there
        index = queue.pop(0)
        item = pi[index]

        # Check whether item has already been processed or does not need 
        # processing
        if index in processedIndexSet:
            continue

        # Do we need to do anything? 
        if item == index//n:
            processedIndexSet.add(index)
            continue

        # Create a cycle with the first item 
        cycle = [index]

        # Follow the cycle 
        firstIndex = index
        while True:
            # Find the next index for swapping 
            if item > index // n:       
                for ni in range(n*item, n*(item+1)):
                    # If processes, go to next
                    if ni in processedIndexSet:
                        continue
                    
                    # Process the new index 
                    index = ni
                    item = pi[index]

                    # Do we need to do anything? 
                    if item == index//n:
                        processedIndexSet.add(index)
                        continue
                        
                    break
            else:
                for ni in range(n*(item+1)-1, n*item-1, -1):
                    # If processes, go to next
                    if ni in processedIndexSet:
                        continue
                    
                    # Process the new index 
                    index = ni
                    item = pi[index]

                    # Do we need to do anything? 
                    if item == index//n:
                        processedIndexSet.add(index)
                        continue
                
                    break

            # If the same as the first index, we complete a cycle 
            if index == firstIndex:
                break

            # We must reach here
            cycle.append(index)
            processedIndexSet.add(index)
            
        processedIndexSet.add(firstIndex)

        # Add cycle
        cycles.append(cycle)

    return cycles

def group_cycles(cycles, ranges):
    # Group cycles based on range. 
    groupRanges = []
    cycleGroups = []
    rp = -1
    for r in range(0, len(ranges)):
        if r <= rp:
            continue
        rp = r

        min = ranges[r][0]
        max = ranges[r][1]
        cycle = cycles[r]
        cycleGroup = [cycle]
        while (r != len(ranges) - 1) and (ranges[r+1][0] < max):
            r += 1
            rp = r
            max = ranges[r][1] if ranges[r][1] > max else max
            cycle = cycles[r]
            cycleGroup.append(cycle)

        cycleGroups.append(cycleGroup)
        groupRanges.append([min, max])

    return(cycleGroups, groupRanges)

def compute_cycle_sweep_solution_distance(cycles, ranges):
    dist = 0
    maxmin = 0

    # Compute distances for all cycles
    for c in range(len(cycles)):
        dist = dist + compute_cycle_dist(cycles[c])
        maxmin = maxmin if maxmin > ranges[c][0] else ranges[c][0]

    # Add overhead
    dist = dist + maxmin*2
    
    return dist

def compute_solution_distance(cgs, grs):
    dist = 0

    # Compute distances for all cycles
    for g in range(len(cgs)):
        cg = cgs[g]
        for c in range(len(cg)):
            dist = dist + compute_cycle_dist(cg[c])

    # Add up the distance between the ranges
    for g in range(len(grs)):
        gr = grs[g]
        if g == 0:
            dist = dist + gr[0]*2
        else:
            #print(gr[0] - grs[g-1][1])
            dist = dist + (gr[0] - grs[g-1][1])*2
    
    return dist

def get_cycles_ptr_block(pi, n, sqrtn, greedy):
    # Collect all cycles 
    cycles = []                 # List of cycles

    # For each type, collect sources and targets
    sources = []
    tomove = []
    for t in range(n):
        sources.append([])
        tomove.append([])

    tyss = [i for i in range(n*n)]
    tyts = [i for i in range(n*n)]
    tiMap = dict()

    for idx in range(n*n):
        sx = idx // n
        sy = idx % n 
        stx = sx//sqrtn
        sty = sy//sqrtn
        st = sqrtn*stx + sty
        tyss[idx] = st

        tidx = pi[idx]
        tx = tidx // n
        ty = tidx % n 
        ttx = tx//sqrtn
        tty = ty//sqrtn
        tt = sqrtn*ttx + tty
        tyts[idx] = tt

        if st != tt:
            sources[tt].append(idx)
            tomove[st].append(idx)

            if not st in tiMap.keys():
                tiMap[st] = []
            tiMap[st].append(idx)

    # Compute pairwise distances for each type and do matching
    # which forms the initial cycles 
    fp = [i for i in range(n*n)]
    bp = [i for i in range(n*n)]

    for t in range(n):
        ni = len(tomove[t])

        # Compute distance 2D array
        arr = np.zeros((ni, ni))
        idArr = np.zeros((ni, ni))
        for si in range(ni):
            for ti in range(ni):
                sidx = sources[t][si]
                tidx = tomove[t][ti]
                arr[si][ti] = compute_dist_2d(n, sidx, tidx)

        # Compute matching
        row_ind, col_ind = linear_sum_assignment(arr)

        # Assign 
        # Assign 
        if(greedy == False):
            for mi in range(ni):
                fp[sources[t][row_ind[mi]]] = tomove[t][col_ind[mi]]
                bp[tomove[t][col_ind[mi]]] = sources[t][row_ind[mi]]
        else:
            for mi in range(ni):
                fp[sources[t][mi]] = tomove[t][mi]
                bp[tomove[t][mi]] = sources[t][mi]

    #print(fp)
    #print(bp)

    # Follow the pointers to retrieve cycles 
    processedIndexSet = set()    # Processed items 

    # Add all items into a list for processing 
    queue = [i for i in range(n*n)]

    # Process the queue and get cycles 
    while (len(queue) > 0):
        # Retrive first index and the item in there
        idx = queue.pop(0)
        sty = tyss[idx]
        tty = tyts[idx]

        # Check whether item has already been processed or does not need 
        # processing
        if idx in processedIndexSet:
            continue

        # Do we need to do anything? 
        if sty == tty:
            processedIndexSet.add(idx)
            continue

        # Create a cycle with the first item 
        cycle = [idx]

        # Follow the cycle 
        while True:
            idx = fp[idx]
            cycle.append(idx)
            processedIndexSet.add(idx)
            if fp[idx] == cycle[0]:
                break

        cycles.append(cycle)
        
    #print(cycles)

    return (tiMap, tyss, tyts, fp, bp, cycles)

def get_cycles_ptr_column(pi, n, greedy):
    # Collect all cycles 
    cycles = []                 # List of cycles

    # For each type, collect sources and targets
    sources = []
    tomove = []
    for t in range(n):
        sources.append([])
        tomove.append([])

    tyss = [i for i in range(n*n)]
    tyts = [i for i in range(n*n)]
    tiMap = dict()

    for idx in range(n*n):
        tyss[idx] = st = idx // n

        tidx = pi[idx]
        tyts[idx] = tt = tidx //n

        if st != tt:
            sources[tt].append(idx)
            tomove[st].append(idx)

            if not st in tiMap.keys():
                tiMap[st] = []
            tiMap[st].append(idx)

    # Compute pairwise distances for each type and do matching
    # which forms the initial cycles 
    fp = [i for i in range(n*n)]
    bp = [i for i in range(n*n)]

    for t in range(n):
        ni = len(tomove[t])

        # Compute distance 2D array
        arr = np.zeros((ni, ni))
        idArr = np.zeros((ni, ni))
        for si in range(ni):
            for ti in range(ni):
                sidx = sources[t][si]
                tidx = tomove[t][ti]
                arr[si][ti] = compute_dist_2d(n, sidx, tidx)

        # Compute matching
        row_ind, col_ind = linear_sum_assignment(arr)

        # Assign 
        if(greedy == False):
            for mi in range(ni):
                fp[sources[t][row_ind[mi]]] = tomove[t][col_ind[mi]]
                bp[tomove[t][col_ind[mi]]] = sources[t][row_ind[mi]]
        else:
            for mi in range(ni):
                fp[sources[t][mi]] = tomove[t][mi]
                bp[tomove[t][mi]] = sources[t][mi]

    #print(fp)
    #print(bp)

    # Follow the pointers to retrieve cycles 
    processedIndexSet = set()    # Processed items 

    # Add all items into a list for processing 
    queue = [i for i in range(n*n)]

    # Process the queue and get cycles 
    while (len(queue) > 0):
        # Retrive first index and the item in there
        idx = queue.pop(0)
        sty = tyss[idx]
        tty = tyts[idx]

        # Check whether item has already been processed or does not need 
        # processing
        if idx in processedIndexSet:
            continue

        # Do we need to do anything? 
        if sty == tty:
            processedIndexSet.add(idx)
            continue

        # Create a cycle with the first item 
        cycle = [idx]

        # Follow the cycle 
        while True:
            idx = fp[idx]
            cycle.append(idx)
            processedIndexSet.add(idx)
            if fp[idx] == cycle[0]:
                break

        cycles.append(cycle)
        
    #print(cycles)

    return (tiMap, tyss, tyts, fp, bp, cycles)

def reorder_cycle(cycle, start_element):
    split_point = 0
    for i in range(len(cycle)):
        if cycle[i] == start_element:
            split_point = i
    #print(split_point)

    new_cycle = cycle[split_point:]
    new_cycle.extend(cycle[0:split_point])
    return new_cycle

#print(reorder_cycle([1,2,3,4,5,6,7], 2))


def _process_cycle(plan, cycles, cycle_index, start_cell, cycle_tree, jump_map):
    cycle = reorder_cycle(cycles[cycle_index], start_cell)
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

def retrieve_mst_solution(cycles, n):
    # Compute # of pick-n-swaps
    pns = compute_pns(cycles)

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
            d12 = compute_dist_2d(n, v1, v2)

            for cj in range(len(cycles)):
                if ci == cj : continue
                c2 = cycles[cj]
                for sj in range(len(c2)):
                    d1 = compute_dist_2d(n, v1, c2[sj])
                    d2 = compute_dist_2d(n, v2, c2[sj])
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
            dv = compute_dist_2d(n, 0, v)
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
    mst_distance = compute_cycle_dist_2d_basic(cyclecopy, n)

    return (pns, plan, mst_distance)

def extract_basic_plan(tplan):
    plan = [-1]
    for [a, b, c] in tplan:
        plan.append(a)
    return plan