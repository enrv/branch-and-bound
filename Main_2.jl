#
#
#

##using JuMP, Gurobi
using JuMP, HiGHS

A = [2 1; 1 2]
b = [4;4]
c = [4; 3]

m = Model(HiGHS.Optimizer)
    
#@variable(m, x[1:2] >=0)
@variable(m, x[1:2], Int)


@constraint(m, A * x <= b)
@objective(m, Min, c'*x)

optimize!(m)


#Passo 0 
#Input epsilon, qtde iterações Maxiter, e critério de seleção do subproblema.




#Passo 1
#Crie S

mutable struct BB
    UB::Float64
    LB::Float64
    root::BBNode
    opt::Union{BBNode, Nothing}
end




#S:	
#xMax 	= [ ∞, ∞, ..., ∞ ]' , xMin = [ 0, 0, ..., 0 ]'
#i(frac) = posição do x fracionado de maior "incerteza" (~0.5)
#x(frac) = valor do x fracionado 
#z(RL) = valor da função objetivo do problema linear relaxado (RL = relaxação linear)

#L 	recebe S
#z(up) 	recebe S.z(RL)
#z(best) recebe -∞
#x(best) recebe NaN

#Passo 2
#While (z(up) - z(best) > epsilon) & (K <= Maxiter)
#	2.1. Seleciona um problema S na lista sobre um critério (por exemplo z(up) ou último inserido)
#	2.2. Quebre o problema em dois usando o S.i(frac)
#	2.3. Faça duas cópias de S(i).
#	2.4. Ajuste os limites de xMax ou xMin de acordo com a estratégia, de um x posicionado.
#	2.5. Atualize as duas cópias com os novos valores de z(frac), i(frac), x(frac), com a relaxação linear de cada problema
#	2.6. verifique as podas para cada um e insira na lista dos não podados.


