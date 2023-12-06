##using JuMP, Gurobi
using JuMP, HiGHS

include("Tree.jl")
include("Utils.jl")

function main()

    #m = Model(Gurobi.Optimizer)
    m = Model(HiGHS.Optimizer)
    
    #@variable(m, x[1:2], Int)
    @variable(m, x[1:2] >=0)
    
    @constraint(m, 2 * x[1] + x[2] <= 4)
    @constraint(m, x[1] + 2 * x[2] <= 4)
    @objective(m, Min, 4 * x[1] + 3 * x[2])

    


    tree = createTree(m)

    println(tree)
    
    
    solve!(tree)

    tree


    return tree.opt.val, value.(tree.opt.model[:x])
end