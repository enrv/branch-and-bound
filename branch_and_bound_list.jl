using JuMP, Gurobi

mutable struct Node
    model::Model
    left::Union{Node, Nothing}
    right::Union{Node, Nothing}
end

mutable struct Problem
    root::Node
    z_UB::Float64
    z_BEST::Float64
    x_BEST::Vector{Float64}
    status::Symbol
end

function searchForMostFractional(x)
    _, i = findmin(abs.(x - floor.(x) .- 0.5))
    return i
end

function createSons!(node::Node, optimizer)
    model_left, ref_left = copy_model(node.model)
    model_right, ref_right = copy_model(node.model)
    set_optimizer(model_left, optimizer)
    set_optimizer(model_right, optimizer)

    x_val = value.(node.model[:x])
    x_left = ref_left[x]
    x_right = ref_right[x]

    i = searchForMostFractional(x_val)

    @constraint(model_left, x_left[i] <= floor(value.(x_val)[i]))
    @constraint(model_right, x_right[i] >= ceil(value.(x_val)[i]))

    node.left = Node(model_left, nothing, nothing)
    node.right = Node(model_right, nothing, nothing)
end

function problemSelectionCriteria(problems::Vector{Node})
    _, i = findmax([objective_value(p.model) for p in problems])
    return i
end

function isXInteger(x, δ=10e-4)
    return all(abs.(x - round.(x)) .< δ)
end

function solveBranchAndBound(m::Model, optimizer, ϵ=0.01, MAX_ITER=100, M=10e9)
    # Solve relaxed problem
    _ = relax_integrality(m)
    optimize!(m)

    # Save solution
    root = Node(m, nothing, nothing)
    problem = Problem(root, objective_value(m), -Inf, [NaN], :unsolved)

    # Loop
    k = 1
    L = [problem.root]
    next_problem = 1
    while (problem.z_UB - problem.z_BEST) > ϵ && k < MAX_ITER
        # Optimal solution found if x is all integer
        if isXInteger(value.(L[next_problem].model[:x]))
            problem.z_BEST = objective_value(L[next_problem].model)
            problem.x_BEST = value.(L[next_problem].model[:x])
            problem.status = :optimal
        end

        # Check if unbounded
        if objective_value(L[next_problem].model) > M
            problem.status = :unbounded
            break
        end

        # Check if infeasible
        if objective_value(L[next_problem].model) < -M
            problem.status = :infeasible
            break
        end

        # Create sons
        createSons!(L[next_problem], optimizer)

        # Solve sons
        optimize!(L[next_problem].left.model)
        optimize!(L[next_problem].right.model)

        # Add sons to list and remove current
        push!(L, L[next_problem].left)
        push!(L, L[next_problem].right)
        deleteat!(L, next_problem)

        # Update upper bound
        problem.z_UB = objective_value(L[next_problem].model)

        # Select next problem
        next_problem = problemSelectionCriteria(L)

        k += 1
    end

    return problem
end

# Original integer programming model
m = Model(Gurobi.Optimizer)
set_silent(m)
@variable(m, x[1:2], Int)
@constraint(m, 2 * x[1] + x[2] <= 4)
@constraint(m, x[1] + 2 * x[2] <= 4)
@objective(m, Max, 4 * x[1] + 3 * x[2])

# Apply branch and bound
problem = solveBranchAndBound(m, Gurobi.Optimizer)
println(problem.status)
println(problem.z_BEST)
println(problem.x_BEST)