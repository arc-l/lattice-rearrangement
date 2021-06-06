# Rearrangement on Lattices with Pick-n-Swaps: Optimality Structures and Efficient Algorithms

## Jingjin Yu ([arxiv preprint](https://arxiv.org/abs/2105.05366), R:SS 2021)

This is the Python codebase that accompanies the above paper, where algorithms are developed for optimally solving rearrangement tasks where items are stored in lattices. In the most basic setup, **LOR** (**L**abeled **O**ne-dimensional **R**earrangement), the items are stored in a line at integer locations, out of order, and must be sorted, for example:

![lor](https://user-images.githubusercontent.com/23622170/120925830-6ec2d600-c6a8-11eb-922a-9e461c318255.png)


Here, it is assumed that the robot end-effector initially rests at the leftmost end. It can pick up objects, move around, and make object swaps. The goal is to minimize the time it takes to complete the rearrangement task, assuming that each pick-n-swap operation takes a fixed amount of time and the robot end-effector travel time is proportional to end-effector travel distance (as measured by some distance metric). We assume that the pick-n-swap operation is more costly, yielding a sequential optimization problem. A solution, when executed, looks like the following: 

https://user-images.githubusercontent.com/23622170/120925809-518e0780-c6a8-11eb-9931-5f97f4c119ca.mp4


Including LOR, this codebase contains greedy and optimized implementations of algorithms described in the paper for solving four problems. For the one-dimensional setting, partially-labeled setting is also supported; we call this the **POR** (**P**artially-labeled **O**ne-dimensional **R**earrangement) problem. In a partially-labeled problem, items of the same type/color are interchangeable. An example POR instance is as follows: 

![por](https://user-images.githubusercontent.com/23622170/120927745-f2cc8c00-c6af-11eb-8a20-a74d3e217e80.png)

An animated solution for POR looks like:

https://user-images.githubusercontent.com/23622170/120927857-55be2300-c6b0-11eb-99ad-65a574035edf.mp4

The rest two of the four problems are two-dimensional versions of LOR and POR, whare are named as **LTR** and **PTR**. Animated solutions, which also demonstrate the problem setup, are given below. 

https://user-images.githubusercontent.com/23622170/120928336-4213bc00-c6b2-11eb-8f30-cc0ef23b4981.mp4



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



