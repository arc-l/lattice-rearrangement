### Rutgers Algorithmic Robotics and Control Lab
### Author: Jingjin Yu Email: jingjinyu@gmail.com
### Coded and tested using Python 3.8.3
import problemgen;
import helper;

def _merge_cycles(cycles, c, cc, idx, cidx, fp, bp, map):
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
 
    #print("FP", fp)
    #print("BP", bp)

    # Update map 
    for i in range(len(c)):
        map[c[i]] = cc
    
    # Remove cycle 
    cycles.remove(c)

def ptr_solver_greedy(pptr, n, sqrtn, block):
    ''' Collect all cycles  '''
    (tiMap, tyss, tyts, fp, bp, cycles) = helper.get_cycles_ptr_block(pptr, n, sqrtn, True) \
        if block == True else helper.get_cycles_ptr_column(pptr, n, True)

    (plan, dist, distgreedy) = helper.compute_cycle_sweep_solution_distance_tr(pptr, cycles, n)
    return (distgreedy, helper.compute_pns(cycles), plan)

def ptr_solver(pptr, n, sqrtn, block):
    ''' Collect all cycles  '''
    (tiMap, tyss, tyts, fp, bp, cycles) = helper.get_cycles_ptr_block(pptr, n, sqrtn, False) \
        if block == True else helper.get_cycles_ptr_column(pptr, n, False)

    # Add to map for look up
    idxCycleMap = dict()     # Cycles indexed by the members 

    for c in range(0, len(cycles)):
        cycle = cycles[c]
        for i in range(0, len(cycle)):
            idxCycleMap[cycle[i]] = cycle

    ''' Merge cycles, MST '''
    # Create a map for holding distances 
    dcMap = dict()
    cdMap = dict()
    dist = 0

    # Compute distances between cycles by processing each type 
    for t in range(0, n):
        if not t in tiMap.keys():
            continue

        # Retrieve the list of indices that needs processing 
        ids = tiMap[t]

        if len(ids) <= 1:
            continue

        for i in range(len(ids)):
            idi = ids[i]
            ci = idxCycleMap[idi]
            for j in range(len(ids)):
                idj = ids[j]
                if idj <= idi:
                    continue 

                cj = idxCycleMap[idj]
                if(ci[0] == cj[0]):
                    continue
                    
                dist = helper.compute_dist_2d(n, idi, idj)
                    
                if not idi in cdMap:
                    cdMap[idi]  = dict()
                    
                cdmap = cdMap[idi]

                if not idj in cdmap:
                    cdmap[idj] = dist
                else:
                    cdmap[idj] = dist if dist < cdmap[idj] else cdmap[idj]

    for c1 in cdMap.keys():
        cdmap = cdMap[c1]
        for c2 in cdmap.keys():
            dist = cdmap[c2]
            if not dist in dcMap.keys():
                dcMap[dist] = []
            dcMap[dist].append([c1, c2])

    # Merge 
    cm = 0
    cmdist = 0
    for dist in sorted (dcMap.keys()) :  
        for pair in dcMap[dist]:
            c0 = idxCycleMap[pair[0]]
            c1 = idxCycleMap[pair[1]]
            if c0[0] != c1[0]:
                cm = cm + 1
                cmdist = cmdist + dist
                #print(cycles)
                _merge_cycles(cycles, c1, c0, pair[1], pair[0], fp, bp, idxCycleMap)

    (pns, plan, mst_distance) = helper.retrieve_mst_solution(cycles, n)

    return (mst_distance, pns, plan)


(pltr, pptr) = problemgen.get_ptr_instance(4)
print(f"PTR instance (4 x 4): {pptr}")

# We use the labeled instance as the input 
(mst_distance, opt_pns, mst_plan) = ptr_solver(pltr, 4, 2, True)
(greedy_distance, greedy_pns, greedy_plan) = ptr_solver_greedy(pltr, 4, 2, True)

print (f"\n# of pick-n-swaps, opt:greedy: {opt_pns}:{greedy_pns}, optimal distance: {mst_distance}, cycle sweep distance: {greedy_distance}\n")
print (f"MST cycle merge  rearrangement plan:\n {mst_plan} \n")
print (f"Cycle sweep rearrangement plan:\n {greedy_plan} \n")


