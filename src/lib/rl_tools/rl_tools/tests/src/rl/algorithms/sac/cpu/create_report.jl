using Pkg
Pkg.add("JSON")
Pkg.add("Plots")

using JSON
using Plots
using Statistics

test_result_path = ENV["RL_TOOLS_TEST_RESULT_PATH"]
current_test_result_path = joinpath(test_result_path, "rl/algorithms/sac/full_training_blas/")
results = JSON.parsefile(joinpath(current_test_result_path, "results.json"))

evaluation_returns = Float64.(hcat(results["evaluation_returns"]...))
println(evaluation_returns)
plot(mean(evaluation_returns, dims=2)[:], ribbon=std(evaluation_returns, dims=2)[:])
savefig(joinpath(current_test_result_path, "evaluation_returns.png"))

