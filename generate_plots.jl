using PyPlot
using Statistics
import LinearAlgebra: I
import Random

include("GeneralizedVoronoi.jl")

rc("text", usetex=true)

n_dims = 3
Random.seed!(1234)

# Generate plots of timings
function generate_ellipse(n_ellipse)
    test_3 = generate_model(10*ones(n_dims), zeros(n_dims))

    for i=1:n_ellipse
        add_ellipsoid(test_3, 100*randn(n_dims), Symmetric(Matrix(I, n_dims, n_dims)))
    end
    optimize!(test_3)
end

const n_ellipse_list = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]
const n_timings = 100
all_times = zeros(length(n_ellipse_list), n_timings)

# Precompile
generate_ellipse(1)

for (i, n) ∈ enumerate(n_ellipse_list)
    for j ∈ 1:n_timings
        all_times[i, j] = @elapsed generate_ellipse(n)
    end
end

bottom_5_percent = zeros(length(n_ellipse_list))
top_5_percent = zeros(length(n_ellipse_list))

for i ∈ 1:length(n_ellipse_list)
    (bottom_5_percent[i], top_5_percent[i]) = quantile(all_times[i,:], [.05, .95])
end

figure(figsize=(4, 3))
title("Time vs. number of obstacles")
loglog(n_ellipse_list, bottom_5_percent, "-.", linewidth=.75, label="5\\%")
loglog(n_ellipse_list, mean(all_times, dims=2), linewidth=.75, label="Mean")
loglog(n_ellipse_list, top_5_percent, "-.", linewidth=.75, label="95\\%")
ylabel("Time (s)")
xlabel("Number of obstacles")
legend()
savefig("plots/timings.pdf", bbox_inches="tight")
close()
