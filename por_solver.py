### Rutgers Algorithmic Robotics and Control Lab
### Author: Jingjin Yu Email: jingjinyu@gmail.com
### Coded and tested using Python 3.8.3
import helper;
import math;
import copy;
import problemgen;

def _get_pointers(ppor, n, k, cycles):
    # Compute back pointers
    bp = [0 for i in range(n*k)]
    fp = [0 for i in range(n*k)]
    for c in cycles:
        for i in range(0, len(c)):
            if i == 0:
                bp[c[0]] = c[len(c)-1]
                fp[c[len(c)-1]] = c[0]
            else:
                bp[c[i]] = c[i-1]
                fp[c[i-1]] = c[i]
    return (fp, bp)

def _merge_cycles(cycles, c, cc, idx, cidx, fp,bp, map):
    # Update cycles, insert c into cc
    for i in range(len(cc)):
        if cc[i] == cidx:
            for j in range(len(c)):
                cc.insert(i+j, idx)
                idx = fp[idx]
            break

    # Update bp and fp
    fpidx = fp[idx]
    fpcidx = fp[cidx]
    bpidx = bp[idx]
    bpcidx = bp[cidx]

    fp[bpcidx] = idx
    bp[idx] = bpcidx

    fp[bpidx] = cidx
    bp[cidx] = bpidx

    # Update map 
    for i in range(len(c)):
        map[c[i]] = cc
    
    # Remove cycle 
    cycles.remove(c)

def _por_solver(ppor, n, k):
    ''' Collect all cycles  '''
    cycles = helper.get_cycles_por(ppor, n, k)
    cycle_copy = copy.deepcopy(cycles)

    ''' Merge cycles (not the most efficient implementation) '''
    # Compute back pointers for a given index 
    (fp, bp) = _get_pointers(ppor, n, k, cycles)

    # Add to map for look up
    memberCycleMap = dict()     # Cycles indexed by the members 
    for c in range(0, len(cycles)):
        cycle = cycles[c]
        for i in range(0, len(cycle)):
            memberCycleMap[cycle[i]] = cycle

    # print(cycles)

    # Merge cycles
    for t in range(0, k):
        currentCycle = []
        currentIndex = -1
        for idx in range(n*t, n*(t+1)):
            # If the item does not need rearrangement, skip
            if ppor[idx] == idx//n:
                continue

            # First item to process for the type? 
            if currentIndex == -1:
                currentIndex = idx
                currentCycle = memberCycleMap[currentIndex]
                continue
            # Later item to process for the type?
            else:
                c = memberCycleMap[idx]
                if c[0] != currentCycle[0]: 
                    # Merge two cycles?
                    if (idx - bp[idx])*(currentIndex - bp[currentIndex]) > 0:
                        _merge_cycles(cycles, c, currentCycle, idx, currentIndex, fp, bp, memberCycleMap)
                        currentIndex = idx
                    else:
                        currentCycle = c
                        currentIndex = idx

    ''' Merge cycles, MST '''
    # Create a map for holding distances 
    cdMap = dict()

    # Compute distances between cycles by processing each type 
    for t in range(0, k):
        currentCycle = []
        currentIndex = -1
        for idx in range(n*t, n*(t+1)):
            # If the item does not need rearrangement, skip
            if ppor[idx] == idx//n:
                continue

            # First item to process for the type? 
            if currentIndex == -1:
                currentIndex = idx
                currentCycle = memberCycleMap[currentIndex]
                continue
            # Later item to process for the type?
            else:
                c = memberCycleMap[idx]
                if c[0] != currentCycle[0]: 
                    # Need to compute distance?
                    if (idx - bp[idx])*(currentIndex - bp[currentIndex]) < 0:
                        dist = idx - currentIndex
                        # Add a new list if cycle pairs with same distance 
                        # is not arelady in cdMap
                        if not dist in cdMap.keys():
                            cdMap[dist] = []

                        # Add the curent pair
                        cdMap[dist].append([currentIndex, idx])
                        break

    # Create cycle groups where each cycle form a cycle group
    cgMap = dict()
    for c in cycles:
        cgMap[c[0]] = c[0]

    # Merge 
    for i in sorted (cdMap.keys()) :  
        for pair in cdMap[i]:
            c0 = memberCycleMap[pair[0]]
            c1 = memberCycleMap[pair[1]]
            if cgMap[c0[0]] != cgMap[c1[0]]:
                _merge_cycles(cycles, c1, c0, pair[1], pair[0], fp, bp, memberCycleMap)
                cgMap.pop(c1[0])

    ''' Group cycles ''' 
    # Compute ranges for cycles 
    ranges = helper.compute_range(cycles, n)

    # Group cycles based on range. 
    (cycleGroups, groupRanges) = helper.group_cycles(cycles, ranges)
    
    # Compute solution distance 
    dist = helper.compute_solution_distance(cycleGroups, groupRanges)
    #distcs = helper.compute_cycle_sweep_solution_distance(cycles, ranges)
    
    return (dist, helper.compute_pns(cycles), cycleGroups, groupRanges, cycle_copy)


def _por_solver_greedy(ppor, n, k):
    ''' Collect all cycles  '''
    cycles = helper.get_cycles_por(ppor, n, k)

    # Compute ranges for cycles 
    ranges = helper.compute_range(cycles, n)
    distcs = helper.compute_cycle_sweep_solution_distance(cycles, ranges)
    
    return (distcs, helper.compute_pns(cycles), cycles)

 
def _process_cycle(plan, plor,  p, c, cg, cgs):
    # Get next cycle and next cycle group if any 
    has_more_cycles = True if len(cg) > 0 else False
    nc = cg[0] if has_more_cycles else []
    (ncmin, ncmax) = helper.get_cycle_range(nc, len(plor)) if has_more_cycles else (0, 0)
    has_more_cg = True if len(cgs) > 0 else False
    ncg = cgs.pop if has_more_cg else []

    # Get cycle range 
    (min, max) = helper.get_cycle_range(c, len(plor))
    (cgmin, cgmax) = helper.get_cycle_group_range(cg, len(plor))

    # Build cycle index 
    cMap = dict()
    for idx in range(len(c)):
        if idx == 0:
            cMap[c[len(c) - 1]] = c[0]
        else:
            cMap[c[idx - 1]] = c[idx]

    i = min
    plan.append([i, p, plor[i]])
    g = plor[i] # plor is the item 
    ni = cMap[i]
    while not ni == i:
        while has_more_cycles and g > ncmin:
            cg.pop(0)
            _process_cycle(plan, plor, g, nc, cg, cgs)
            has_more_cycles = True if len(cg) > 0 else False
            nc = cg[0] if has_more_cycles else []
            (ncmin, ncmax) = helper.get_cycle_range(nc, len(plor)) if has_more_cycles else (0, 0)
    
        if g == cgmax and has_more_cg:
            _process_cycle_group(g, ncg, plor, cgs)
        
        plan.append([ni, g, plor[ni]])
        g = plor[ni]
        ni = cMap[ni]
    plan.append([i, g, p])


def _process_cycle_group(plan, plor, p, cg, cgs):
    c = cg.pop(0)
    _process_cycle(plan, plor, p, c, cg, cgs)


def _plor_from_ppor(ppor, n, k):
    itemStore = dict()
    for t in range(0, k):
        itemStore[t] = []
        for i in range(0, n):
            itemStore[t].append(t*n + i)

    plor = []
    for i in range(0, len(ppor)):
        t = ppor[i]
        plor.append(itemStore[t].pop())

    return plor

'''
Compute POR solution
ppor: a POR instance in a list form
n: number of items per type
k: number of types (labels)
'''
def compute_por_solution(ppor, n, k):

    # Compute solution 
    (mst_distance, mst_pns, cycle_groups, group_ranges, mst_cycles) = _por_solver(ppor, n, k) 
    (cycle_sweep_distance, cycle_sweep_pns, cycle_sweep_cycles) = _por_solver_greedy(ppor, n, k)

    # Retrieve plan, MST optimal strategy 
    plor = _plor_from_ppor(ppor, n, k)
    mst_plan = []
    _process_cycle_group(mst_plan, plor, -1, cycle_groups.pop(0), cycle_groups)

    # Cycle sweep plan
    cycle_sweep_plan = []
    last = 0
    for c in cycle_sweep_cycles:
        (min, max) = helper.get_cycle_range(c, len(plor))
        # Build cycle index 

        cMap = dict()
        for idx in range(len(c)):
            if idx == 0:
                cMap[c[len(c) - 1]] = c[0]
            else:
                cMap[c[idx - 1]] = c[idx]


        i = min
        cycle_sweep_plan.append([i, -1, plor[i]])
        g = plor[i]
        ni = cMap[i]
        while not ni == i:
            cycle_sweep_plan.append([ni, g, plor[ni]])
            g = plor[ni]
            ni = cMap[ni]

        last = g
        cycle_sweep_plan.append([i, g, -1])

    #if not g == 0:
    #    cycle_sweep_plan.append([0, -1, -1])


    return (mst_pns, helper.extract_basic_plan(mst_plan), mst_distance, cycle_sweep_pns, helper.extract_basic_plan(cycle_sweep_plan), cycle_sweep_distance)


ppor = problemgen.get_por_instance(4,4)
print(f"POR instance (4 types): {ppor}")
mst_pns, mst_plan, mst_distance, cycle_sweep_pns, cycle_sweep_plan, cycle_sweep_distance = compute_por_solution(ppor, 4, 4)

print (f"\n# of pick-n-swaps, opt:greedy: {mst_pns}:{cycle_sweep_pns}, optimal distance: {mst_distance}, cycle sweep distance: {cycle_sweep_distance}\n")
print (f"Optimal rearrangement plan:\n {mst_plan} \n")
print (f"Cycle sweep rearrangement plan:\n {cycle_sweep_plan} \n")


