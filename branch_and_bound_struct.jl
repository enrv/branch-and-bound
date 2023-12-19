using JuMP, Gurobi

mutable struct Problem
    zBEST::Float64
    xBEST::Vector{Float64}
    zUP::Float64
    status::Symbol
end

mutable struct S
    xUB::Vector{Float64}
    xLB::Vector{Float64}
    xRL::Vector{Float64}
    zRL::Float64
    ifrac::Int
    xfrac::Float64
end

function solveBranchAndBound(m::Model, optimizer=Gurobi.Optimizer, ϵ=0.01, MAX_ITER=100, M=10e9)
    # First relaxed problem
    _ = relax_integrality(m)
    optimize!(m)

    # 3 questions
    if termination_status(m) == MOI.OPTIMAL
        if isXInteger(objective_value(m))
            return Problem(objective_value(m), value.(x), objective_value(m), :optimal)
        end
    elseif termination_status(m) == MOI.INFEASIBLE
        return Problem(-Inf, [NaN], -Inf, :infeasible)
    elseif termination_status(m) == MOI.UNBOUNDED
        return Problem(-Inf, [NaN], -Inf, :unbounded)
    end

    first = S(M*ones(length(x)), -M*ones(length(x)), value.(x), objective_value(m), 0, NaN)

    prob = Problem(-Inf, [NaN], first.zRL, :unsolved)

    L = []
    push!(L, first)
    next = pop!(L)

    searchForMostFractional!(next)
    l, r = createSons(next)

    push!(L, l)
    push!(L, r)

    # Iteration
    k = 1
    while length(L) > 0 && k < MAX_ITER # && (prob.zUP - prob.zBEST) > ϵ
        next = pop!(L)
        result = solveProblem!(next, m, optimizer)
        if result == :possible
            if isXInteger(next.xRL)
                # Candidate solution
                if next.zRL > prob.zBEST
                    prob.zBEST = next.zRL
                    prob.xBEST = next.xRL
                end
            else
                if next.zRL > zbest
                    searchForMostFractional!(next)
                    l, r = createSons(next)
                    push!(L, l)
                    push!(L, r)
                # else Prune by limit
                end
            end
            prob.zUP = next.zRL
        # else Infeasible
        end
        k += 1
    end

    if prob.zBEST > -Inf
        prob.status = :optimal
    else
        prob.status = :infeasible
    end

    return prob
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

# Testing
m = Model(Gurobi.Optimizer)
set_silent(m)
@variable(m, x[1:2], Int)
@constraint(m, x .>= 0)
@constraint(m, 7*x[1] - 2*x[2] <= 14)
@constraint(m, x[2] <= 3)
@constraint(m, 2*x[1] - 2*x[2] <= 3)
@objective(m, Max, 4*x[1] - x[2])

solveBranchAndBound(m)
