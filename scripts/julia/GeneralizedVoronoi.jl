using JuMP
import JuMP: optimize!, objective_value
import ECOS
using LinearAlgebra: Symmetric, eigen

struct AgentModel
    model::Model
    x::Array{VariableRef,1}
    x_curr::Array{Float64,1}
    x_goal::Array{Float64,1}
end

function quad_over_lin(m, x, y)
    n = size(x, 1)
    t = @variable(m, [1:n])

    for i = 1:n
        @constraint(m, [y[i] + t[i]; y[i] - t[i]; 2*x[i]] ∈ SecondOrderCone())
    end

    return t
end

"Generate ellipsoidal constraint"
function add_ellipsoid(agent::AgentModel, μ, Σ::Symmetric)
    n = size(μ, 1)

    factor = eigen(Σ)
    D = factor.values
    U = factor.vectors

    λ = @variable(agent.model)
    @constraint(agent.model, λ ≥ 0)

    μ_new = Σ \ μ
    t = quad_over_lin(agent.model, U'*(agent.x + λ.*μ_new), 1 ./ D .+ λ)

    @constraint(agent.model, sum(agent.x_curr.^2) - 2*agent.x_curr'*agent.x
                + sum(t) ≤ λ.*(μ_new'*(D.*μ_new) - 1))
end

"Generate a model with the current starting position and goal."
function generate_model(x_curr, x_goal; optimizer=ECOS.Optimizer, feastol=1e-5)
    n = size(x_goal, 1)
    model = Model(with_optimizer(optimizer, verbose=false, feastol=feastol))

    @variable(model, x[1:n])

    obj_t = quad_over_lin(model, x - x_goal, ones(n))

    @objective(model, Min, sum(obj_t))

    return AgentModel(model, x, x_curr, x_goal)
end

optimize!(a::AgentModel) = optimize!(a.model)
objective_value(a::AgentModel) = objective_value(a.model)
