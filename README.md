# Rearrangement on Lattices with Pick-n-Swaps: Optimality Structures and Efficient Algorithms

## Jingjin Yu ([arxiv preprint](https://arxiv.org/abs/2105.05366), R:SS 2021)

This is the code that accompanies the above paper, where algorithms are developed for optimally solving rearrangement tasks where items are stored in lattices. In the most basic setups, LOR (**L**abeled **O**ne-dimensional **R**earrangement), the items are stored in a line at integer locations, out of order, and must be sorted, for example

![alt text](https://github.com/rutgers-arc-lab/lattice-rearrangement/blob/main/media/lor.png?raw=true)


## Dependencies

The code base is written in Python and tested under v3.8.3. It has dependencies on numpy, scipy, and networkx. These should be installed with pip

`> pip install numpy scipy networkx`

## Usage

The four main solvers are named *_solvers.py, where * corresponds to LOR, POR, LTR, PTR. Each file has a default demo that works with a randomly generated problem instance. For example, running 

`> python lor_solver.py`

will produce something like 


```
LOR instance: [2, 7, 3, 0, 14, 5, 1, 11, 13, 6, 9, 4, 12, 8, 10, 15]

# of pick-n-swaps: 16, optimal distance: 56, cycle sweep distance: 72

Optimal rearrangement plan:
 [-1, 0, 1, 7, 8, 13, 8, 11, 4, 14, 10, 9, 6, 1, 2, 3, 0]

Cycle sweep rearrangement plan:
 [-1, 0, 2, 3, 0, 1, 7, 11, 4, 14, 10, 9, 6, 1, 8, 13, 8]
 ```

which contains the randomly generated instance, the optimal number of pick-n-swaps, the optimal end-effector travel distance, and the distance incurred by a cycle-sweeping algorithm. The plans from both algorithms are also given. To interpret the plan, "-1" indicates the end-effector at rest. The other numbers are cell labels. So, for the plan  [-1, 0, 1, 7, 8, 13, 8, 11, 4, 14, 10, 9, 6, 1, 2, 3, 0], the end-effector first travels to cell 0 (and picks up an item there), then cell 1, 7, 8, and so on. It is clear whether the operation at a given cell location is a pick, drop, or pick-n-swap. The same intepretation applies to other solvers. For the 2D solvers, the cells should be interpreted based on column-major ordering (see the paper). 



