using LinearAlgebra: I
using BenchmarkTools
import Random

include("GeneralizedVoronoi.jl")

n_dims = 3
Random.seed!(1234)

# Obvious test
function model1()
    test_1 = generate_model(ones(n_dims), zeros(n_dims))
    optimize!(test_1)
    @assert isapprox(objective_value(test_1), 0.0, atol=1e-5)
end

# Slightly less obvious test
function model2()
    test_2 = generate_model(ones(n_dims), zeros(n_dims))
    add_ellipsoid(test_2, 4*ones(n_dims), Symmetric(Matrix(I, n_dims, n_dims)))
    optimize!(test_2)
    @assert isapprox(objective_value(test_2), 0.0, atol=1e-5)
end

# Generating a bunch of random ones
function model3()
    test_3 = generate_model(ones(n_dims), zeros(n_dims))

    for i=1:100
        add_ellipsoid(test_3, 10*randn(n_dims), Symmetric(Matrix(I, n_dims, n_dims)))
    end
    optimize!(test_3)
end

@benchmark model1()
@benchmark model2()
@benchmark model3()

function test_4()
    test_model = generate_model(ones(n_dims), zeros(n_dims))
    n_ellipse = 100

    for i=1:n_ellipse
        add_ellipsoid(test_model, 10*randn(n_dims), Symmetric(Matrix(I, n_dims, n_dims)))
    end

    return test_model
end

t4 = test_4()
@time optimize!(t4)
