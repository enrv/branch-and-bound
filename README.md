# The Branch & Bound Algorithm

## Authors

2023 Integer Programming Class of Prof. Street at PUC-Rio:

[@AngeloMilfont](https://github.com/AngeloMilfont)

[@enrv](https://github.com/enrv)

[@JoaoAmorim2604](https://github.com/JoaoAmorim2604)

Thanks also [@lucasximenes](https://github.com/lucasximenes) for insights.

## Idea

Branch-and-bound is an algorithmic paradigm for solving integer and mixed-integer linear programming (MILP) problems. Its strategy involves instancing a relaxation of the original optimization problem by converting integer and binary variables into real ones, which solution gives us a lower (in minimization problems) or an upper (in maximization problems) bound for the original problem.

Thus, we begin to tighten the restrictions of the relaxed problem, generating a list of candidate solutions through a rooted tree. This tree consists of the fully relaxed problem as the main root successively branched into two nodes defined by the most fractional integer heuristic - i.e. the value closest to the .5 between the ceiling and the floor.

In order to find the optimal solution without inneficiently searching for all nodes - which would require an exponential computational time in all cases - we call upon a strategy to reduce our search space for solutions through pruning the tree according to the following criteria:

- by optimality: when we find an integer solution that has an objective value lower than some integer solution previously found, this solution is suboptimal though valid
- by limit: when we find a non-integer solution that has an objective value lower than some integer solution previously found, there's no need to further branch it as no branch can lead to a better solution than its generating node: any integer solution that may be found by diving the branches will be suboptimal
- by infeasibility: when we find an infeasible node, all of its branches will also be infeasible nodes

Employing this ideia we are able to more efficiently find an optimal solution to MILP problems. This is crucial in operations research because those problems are NP-hard, meaning they may take a long - perhaps unreasonable - computational time to solve for larger inputs often encountered in real-world applications.

## Implementation

We implemented the algorithm in [The Julia Programming Language](https://julialang.org), using the [JuMP.jl](https://github.com/jump-dev/JuMP.jl) optimization package. [Gurobi Optimizer](https://www.gurobi.com) may be easily replaced by any other solver that supports linear programming.

`branch_and_bound.jl`: the functions and structs implemented

`branch_and_bound_test.jl`: the instances of different problems solved by branch and bound

`branch_and_bound_test_comparison.jl`: the instances of different problems solved by Gurobi