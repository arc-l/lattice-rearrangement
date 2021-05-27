### Rutgers Algorithmic Robotics and Control Lab
### Author: Jingjin Yu Email: jingjinyu@gmail.com
### Coded and tested using Python 3.8.3
import problemgen;
import helper;
import math;

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

    i = min
    plan.append([i, p, plor[i]])
    g = plor[i]
    while not g == i:
        while has_more_cycles and g > ncmin:
            cg.pop(0)
            _process_cycle(plan, plor, g, nc, cg, cgs)
            has_more_cycles = True if len(cg) > 0 else False
            nc = cg[0] if has_more_cycles else []
            (ncmin, ncmax) = helper.get_cycle_range(nc, len(plor)) if has_more_cycles else (0, 0)
    
        if g == cgmax and has_more_cg:
            _process_cycle_group(g, ncg, plor, cgs)
        
        plan.append([g, g, plor[g]])
        g = plor[g]
    plan.append([i, i, p])


def _process_cycle_group(plan, plor, p, cg, cgs):
    c = cg.pop(0)
    _process_cycle(plan, plor, p, c, cg, cgs)


def _compute_cycle_sweep_plan(cycles, plor):
    cs_plan = []
    last = 0
    for c in cycles:
        (min, max) = helper.get_cycle_range(c, len(plor))
        i = min
        cs_plan.append([i, -1, plor[i]])
        g = plor[i]
        while not g == i:
            cs_plan.append([g, g, plor[g]])
            g = plor[g]
        last = g
        cs_plan.append([i, i, -1])
    #if not g == 0:
    #    cs_plan.append([0, -1, -1])
    return cs_plan

'''
Compute LOR solution
plor: the LOR problem instance, a list representing a permutation
'''
def compute_lor_solution(plor):

    # n is the number of items, equaling the length of the list
    n = len(plor)

    # Collect all cycles 
    cycles = helper.get_cycles(plor)

    # Compute ranges for cycles 
    ranges = helper.compute_range(cycles, n)

    # Group cycles based on range, yielding cycle groups and ranges of these groups
    (cycle_groups, group_ranges) = helper.group_cycles(cycles, ranges)

    # Compute cycle sweep solution distance and optimal solution distance 
    pick_n_swaps = helper.compute_pns(cycles)
    cycle_sweep_distance = helper.compute_cycle_sweep_solution_distance(cycles, ranges)
    optimal_distance = helper.compute_solution_distance(cycle_groups, group_ranges)

    # Obtain cycle sweep plan
    cycle_sweep_plan = helper.extract_basic_plan(_compute_cycle_sweep_plan(cycles, plor))

    # Obtain optimal plan
    optmal_plan = []
    _process_cycle_group(optmal_plan, plor, -1, cycle_groups.pop(0), cycle_groups)

    return (pick_n_swaps, helper.extract_basic_plan(optmal_plan), optimal_distance, cycle_sweep_plan, cycle_sweep_distance)


plor = problemgen.get_lor_instance(16)
print(f"LOR instance: {plor}")
(pns, opt_plan, opt_dist, cs_plan, cs_dist) = compute_lor_solution(plor)

print (f"\n# of pick-n-swaps: {pns}, optimal distance: {opt_dist}, cycle sweep distance: {cs_dist}\n")
print (f"Optimal rearrangement plan:\n {opt_plan} \n")
print (f"Cycle sweep rearrangement plan:\n {cs_plan} \n")



