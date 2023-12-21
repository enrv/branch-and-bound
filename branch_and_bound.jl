using JuMP, Gurobi

mutable struct Problem
    zBEST::Float64
    xBEST::Array{Float64}
    zUP::Float64
    status::Symbol
end

mutable struct S
    xUB::Array{Float64}
    xLB::Array{Float64}
    xRL::Array{Float64}
    zRL::Float64
    ifrac::Int
    xfrac::Float64
end

function solveBranchAndBound(m::Model, MAX_ITER=100, optimizer=Gurobi.Optimizer, M=10e9)
    output_msg = []
    push!(output_msg, "========== List created with 0 unsolved problem(s) ==========")

    # First relaxed problem
    _ = relax_integrality(m)
    optimize!(m)

    # 3 questions
    if termination_status(m) == MOI.OPTIMAL
        if isXInteger(objective_value(m))
            return Problem(objective_value(m), value.(x), objective_value(m), :optimal)
        end
    elseif termination_status(m) == MOI.INFEASIBLE
        push!(output_msg, "Initial problem is infeasible.")
        return Problem(-Inf, [NaN], -Inf, :infeasible)
    elseif termination_status(m) == MOI.UNBOUNDED
        push!(output_msg, "Initial problem is unbounded.")
        return Problem(-Inf, [NaN], -Inf, :unbounded)
    end

    first = S(M*ones(length(x)), -M*ones(length(x)), value.(x), objective_value(m), 0, NaN)

    prob = Problem(-Inf, [NaN], first.zRL, :unsolved)

    L = []
    push!(L, first)
    push!(output_msg, "Initial Problem pushed into List.")
    next = pop!(L)
    push!(output_msg, "One problem withdrawn from list.")
    push!(output_msg, "List contains $(length(L)) problem(s).")  

    searchForMostFractional!(next)
    l, r = createSons(next)

    push!(L, l)
    push!(L, r)

    k = 1
    while length(L) > 0 && k < MAX_ITER
        push!(output_msg, "List contains $(length(L)) problem(s).")
        push!(output_msg, "========== Iteration $k ==========")

        next = pop!(L)
        push!(output_msg, "One problem withdrawn from list.")
        push!(output_msg, "List contains $(length(L)) problem(s).")
        result = solveProblem!(next, m, optimizer)
        if result == :possible
            if isXInteger(next.xRL)
                # Candidate solution
                if next.zRL > prob.zBEST
                    prob.zBEST = next.zRL
                    prob.xBEST = next.xRL
                    msg = "Iteration $k -> Possible and Integer result -> Z_best updated."
                    push!(output_msg, msg)
                else
                    msg = "Iteration $k -> Possible and Integer result -> Pruned by optimality."
                    push!(output_msg, msg)
                end
            else
                if next.zRL > prob.zBEST
                    msg = "Iteration $k -> Possible and non-Integer result -> create sons."
                    push!(output_msg, msg)
                    searchForMostFractional!(next)
                    l, r = createSons(next)
                    push!(L, l)
                    push!(L, r)
                else                
                    msg = "Iteration $k -> Pruned by limit."
                    push!(output_msg, msg)
                end
            end
        else
            msg = "Iteration $k -> Infeasible."
            push!(output_msg, msg)
        end
        k += 1
    end

    push!(output_msg, "List contains $(length(L)) problem(s).")        
    push!(output_msg, "========== Final Result ==========")

    if prob.zBEST > -Inf
        prob.status = :optimal
    else
        prob.status = :infeasible
    end

    return prob, output_msg
end

function searchForMostFractional!(problem::S)
    x = problem.xRL
    _, i = findmin(abs.(x - floor.(x) .- 0.5))

    problem.ifrac = i
    problem.xfrac = x[i]
end

function createSons(father::S)
    xUB = copy(father.xUB)
    xUB[father.ifrac] = floor(father.xfrac)

    xLB = copy(father.xLB)
    xLB[father.ifrac] = ceil(father.xfrac)

    l = S(father.xUB, xLB, [], Inf, 0, NaN)
    r = S(xUB, father.xLB, [], Inf, 0, NaN)
    return l, r
end

function solveProblem!(problem::S, m::Model, optimizer)
    model, ref = copy_model(m)
    set_optimizer(model, optimizer)

    x_ref = ref[x]

    for i in 1:length(x)
        @constraint(model, x_ref[i] <= problem.xUB[i])
        @constraint(model, x_ref[i] >= problem.xLB[i])
    end

    optimize!(model)

    if termination_status(model) == MOI.OPTIMAL
        problem.xRL = value.(x_ref)
        problem.zRL = objective_value(model)
        return :possible
    else
        return :infeasible
    end
end

function isXInteger(x, δ=10e-4)
    return all(abs.(x - round.(x)) .< δ)
end