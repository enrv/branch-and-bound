include("branch_and_bound.jl")

# Wolsey example
m = Model(Gurobi.Optimizer)
set_silent(m)
@variable(m, x[1:2], Int)
@constraint(m, x .>= 0)
@constraint(m, 7*x[1] - 2*x[2] <= 14)
@constraint(m, x[2] <= 3)
@constraint(m, 2*x[1] - 2*x[2] <= 3)
@objective(m, Max, 4*x[1] - x[2])

prob, outmsg = solveBranchAndBound(m)

foreach(println, outmsg)
println(prob)

# Production problem
m = Model(Gurobi.Optimizer)
set_silent(m)
@variable(m, x[1:2], Int)
@constraint(m, x .>= 0)
@constraint(m, 2 * x[1] + x[2] <= 4)
@constraint(m, x[1] + 2 * x[2] <= 4)
@objective(m, Max, 4 * x[1] + 3 * x[2])

prob, outmsg = solveBranchAndBound(m)

foreach(println, outmsg)
println(prob)

# Knapsack problem
m = Model(Gurobi.Optimizer)
set_silent(m)
@variable(m, x[1:3], Bin)
@constraint(m, x .>= 0)
@constraint(m, 6*x[1] + 5*x[2] + 5*x[3] <= 10)
@objective(m, Max, 6*x[1] + 4*x[2] + 3*x[3])

prob, outmsg = solveBranchAndBound(m)

foreach(println, outmsg)
println(prob)

# Knapsack problem - first variation
m = Model(Gurobi.Optimizer)
set_silent(m)
@variable(m, x[1:4], Bin)
@constraint(m, x .>= 0)
@constraint(m, 0.8*x[1] + 0.4*x[2] + 0.9*x[3] + 0.4*x[4] <= 3)
@objective(m, Max, 0.6*x[1] + 0.4*x[2] + 0.3*x[3] + 0.9*x[4])

prob, outmsg = solveBranchAndBound(m)

foreach(println, outmsg)
println(prob)

# Knapsack problem - second variation
m = Model(Gurobi.Optimizer)
set_silent(m)
@variable(m, x[1:8], Bin)
@constraint(m, x .>= 0)
@constraint(m, 0.6*x[1] + 0.5*x[2] + 0.4*x[3] + 0.3*x[4] + 0.2*x[5] + 0.1*x[6] + 0.9*x[7] + 0.8*x[8] <= 3)
@objective(m, Max, 0.2*x[1] + 0.8*x[2] + 0.4*x[3] + 0.3*x[4] + 0.5*x[5] + 0.6*x[6] + 0.7*x[7] + 0.9*x[8])

prob, outmsg = solveBranchAndBound(m)

foreach(println, outmsg)
println(prob)

# TSP problem
m = Model(Gurobi.Optimizer)
set_silent(m)
using TSPLIB
tsp_tokens = [:burma14, :ulysses16, :gr17, :gr21]
instance = readTSPLIB(:ulysses16)
n = instance.dimension
@variable(m, x[1:n^2], Bin)
@variable(m, u[1:n], Int)
@constraint(m, x .>= 0)
@constraint(m, u .>= 0)
@constraint(m, [i in 1:n], sum(x[(i-1)*n + j] for j in 1:n if j != i) == 1)
@constraint(m, [j in 1:n], sum(x[(i-1)*n + j] for i in 1:n if i != j) == 1)
@constraint(m, [i in 2:n, j in 2:n], u[i] - u[j] + 1 <= n*(1 - x[(i-1)*n + j]))
@constraint(m, u[1] == 1)

@objective(m, Max, -sum(instance.weights[i,j]*x[(i-1)*n + j] for i in 1:n, j in 1:n if i != j))

prob, outmsg = solveBranchAndBound(m, 1000000)

foreach(println, outmsg)
println(prob)