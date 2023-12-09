using JuMP, Gurobi

mutable struct Node
    model::Model
    left::Union{Node, Nothing}
    right::Union{Node, Nothing}
end

mutable struct Problem
    UB::Float64
    root::Node
    z_BEST::Float64
    x_BEST::Vector{Float64}
end

function searchForMostFractional(x)
    _, i = findmin(abs.(x - floor.(x) .- 0.5))
    return i
end

function createSons!(node::Node)
    model_left, ref_left = copy_model(node.model)
    model_right, ref_right = copy_model(node.model)
    set_optimizer(model_left, Gurobi.Optimizer)
    set_optimizer(model_right, Gurobi.Optimizer)

    x_val = value.(node.model[:x])
    x_left = ref_left[x]
    x_right = ref_right[x]

    i = searchForMostFractional(x_val)

    @constraint(model_left, x_left[i] <= floor(value.(x_val)[i]))
    @constraint(model_right, x_right[i] >= ceil(value.(x_val)[i]))

    node.left = Node(model_left, nothing, nothing)
    node.right = Node(model_right, nothing, nothing)
end

function problemSelectionCriteria(problem1::Model, problem2::Model)
    if objective_value(problem1) > objective_value(problem2)
        return :left
    else
        return :right
    end
end

function isXInteger(x)
    δ = 10e-4
    for i in 1:length(x)
        if abs(x[i] - round(x[i])) > δ
            return false
        end
    end
    return true
end

# Problem global variables

ϵ = 0.01
M = 10e9
MAX_ITER = 100
k = 1

# Initialization

m = Model(Gurobi.Optimizer)
set_silent(m)
@variable(m, x[1:2], Int)
@constraint(m, 2 * x[1] + x[2] <= 4)
@constraint(m, x[1] + 2 * x[2] <= 4)
@objective(m, Max, 4 * x[1] + 3 * x[2])

# Solve relaxed problem

undo = relax_integrality(m)
optimize!(m)

# Save solution

root = Node(m, nothing, nothing)
problem = Problem(objective_value(m), root, -Inf, [NaN])

# Loop

next_problem = problem.root

while (problem.UB - problem.z_BEST) > ϵ && k < MAX_ITER
    # Create sons
    createSons!(next_problem)

    # Solve sons
    optimize!(next_problem.left.model)
    optimize!(next_problem.right.model)

    # Select son
    which_one = problemSelectionCriteria(next_problem.left.model, next_problem.right.model)
    if which_one == :left
        next_problem = next_problem.left
    else
        next_problem = next_problem.right
    end

    # Update problem information
    problem.UB = objective_value(next_problem.model)

    # Criteria if is integer
    if isXInteger(value.(next_problem.model[:x]))
        problem.z_BEST = objective_value(next_problem.model)
        problem.x_BEST = value.(next_problem.model[:x])
    end

    # Check if unbounded
    if objective_value(next_problem.model) > M
        println("Ilimitado")
        break
    end

    # Check if infeasible
    if objective_value(next_problem.model) < -M
        println("Inviável")
        break
    end

    k += 1
end

println(problem.z_BEST)
println(problem.x_BEST)