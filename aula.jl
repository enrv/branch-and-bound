using JuMP, Gurobi

mutable struct Node
    model::Model
    left::Union{Node, Nothing}
    right::Union{Node, Nothing}
end

mutable struct Problem
    UB::Float64
    LB::Float64
    root::Node
    z_BEST::Float64
    x_BEST::Vector{Float64}
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
problem = Problem(Inf, -Inf, root, -Inf, [NaN])

# Create sons

function createSons!(node::Node)
    model_left, ref_left = copy_model(node.model)
    model_right, ref_right = copy_model(node.model)
    set_optimizer(model_left, Gurobi.Optimizer)
    set_optimizer(model_right, Gurobi.Optimizer)

    x_val = value.(node.model[:x])
    x_left = ref_left[x]
    x_right = ref_right[x]

    i = 1 # findMostFractional

    @constraint(model_left, x_left[i] <= floor(value.(x_val)[i]))
    @constraint(model_right, x_right[i] >= ceil(value.(x_val)[i]))

    node.left = Node(model_left, nothing, nothing)
    node.right = Node(model_right, nothing, nothing)
end

createSons!(problem.root)

# Solve sons

optimize!(problem.root.left.model)
optimize!(problem.root.right.model)

value.(problem.root.left.model[:x])
objective_value(problem.root.left.model)

value.(problem.root.right.model[:x])
objective_value(problem.root.right.model)

createSons!(problem.root.left)
createSons!(problem.root.right)

# Deep

optimize!(problem.root.left.left.model)
optimize!(problem.root.left.right.model)

optimize!(problem.root.right.left.model)
optimize!(problem.root.right.right.model)

value.(problem.root.left.left.model[:x])
objective_value(problem.root.left.left.model)

value.(problem.root.left.right.model[:x])
objective_value(problem.root.left.right.model)

value.(problem.root.right.left.model[:x])
objective_value(problem.root.right.left.model)

value.(problem.root.right.right.model[:x])
objective_value(problem.root.right.right.model)

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