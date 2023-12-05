mutable struct BB
    UB::Float64
    LB::Float64
    root::BBNode
    opt::Union{BBNode, Nothing}
end

mutable struct BBNode
    parent::Union{BBNode, Nothing}
    sons::Vector{Union{BBNode, Nothing}}
    model::Model
    val::Float64
    status::Symbols # :
end

function createTree(model::Model)
    root = BBNode(nothing, [nothing, nothing], model, 0.0, :open)
    tree = BB(Inf, -Inf, root, nothing)
    root.tree = tree
    return tree
end

function solve!(tree::BB)
    candidates = [tree.root]
    while !isempty(candidates)
        node = pop!(candidates)
        solveNode!(node, tree)
        if node.status == :infeasible
            continue
        elseif node.status == :optimal
            return
        elseif node.status == :closed
            generateSons!(node)
            push!(candidates, node.sons[1])
            push!(candidates, node.sons[2])
        end
    end
    return
end

function solveNode!(parent::BBNode, tree::BB)
    optimize!(parent.model)
    if termination_status(parent.model) == MOI.OPTIMAL
        parent.val = objective_value(parent.model)
        
        if checkIntegerSolution(value.(parent.model[:x]))
            if tree.UB > parent.val
                tree.UB = parent.val
            elseif tree.LB < parent.val
                tree.LB = parent.val
            end
        end

        parent.status = :closed
        
        if tree.UB == tree.LB
            tree.opt = parent
            parent.status = :optimal
        end
    else
        parent.val = Inf
        parent.status = :infeasible
    end
end

function generateSons!(parent::BBNode)
    x = findMostFractional(value.(parent.model[:x]))
    for i in 1:2
        model = copy(parent.model)
        if i == 1
            @constraint(model, x >= ceil(x))
        else
            @constraint(model, x <= floor(x))
        end
        son = BBNode(parent, [nothing, nothing], model, 0.0, :open)
        parents.sons[i] = son
    end
end
