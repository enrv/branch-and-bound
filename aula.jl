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

# Initialization

m = Model(Gurobi.Optimizer)
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


# while (problem.UB - problem.z_BEST) > ϵ


# Create sons

createSons!(problem.root)

# Solve sons

optimize!(problem.root.left.model)
optimize!(problem.root.right.model)

value.(problem.root.left.model[:x])
objective_value(problem.root.left.model)

value.(problem.root.right.model[:x])
objective_value(problem.root.right.model)

which_one = problemSelectionCriteria(problem.root.left.model, problem.root.right.model)

if which_one == :left
    next_problem = problem.root.left
    other_problem = problem.root.right
else
    next_problem = problem.root.right
    other_problem = problem.root.left
end

problem.UB = objective_value(next_problem.model)

createSons!(next_problem)

optimize!(next_problem.left.model)
optimize!(next_problem.right.model)

value.(next_problem.left.model[:x])
objective_value(next_problem.left.model)

value.(next_problem.right.model[:x])
objective_value(next_problem.right.model)



createSons!(other_problem)

optimize!(other_problem.left.model)
optimize!(other_problem.right.model)

value.(other_problem.left.model[:x])
objective_value(other_problem.left.model)

value.(other_problem.right.model[:x])
objective_value(other_problem.right.model)

# Check
"""
if isa(value.(x), Integer)
    println("Ótimo")
    println(x)
elseif objective_value(m) > 10e9
    println("Ilimitado")
elseif objective_value(m) < -10e9
    println("Inviável")
end
"""