using JuMP, Gurobi

include("Tree.jl")
include("Utils.jl")

function main()

    m = Model(Gurobi.Optimizer)

    @variable(m, x[1:2], Int)
    @constraint(m, 2 * x[1] + x[2] <= 1)
    @constraint(m, x[1] + 2 * x[2] <= 1)
    @objective(m, Min, 4 * x[1] + 3 * x[2])

    tree = createTree(m)
    solve!(tree)

    return tree.opt.val, value.(tree.opt.model[:x])
end