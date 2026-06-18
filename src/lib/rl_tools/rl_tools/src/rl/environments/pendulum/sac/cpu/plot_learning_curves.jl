using Plots
using HDF5
using Statistics

file = "rl_environments_pendulum_sac_learning_curves.h5"

f = h5open(file, "r")


plt = plot(legend=false)
episode_returns = []
for (run_i, run) in enumerate(f)
    plot!(run["episode_returns"][:], label="run $run_i", linecolor = :grey, linewidth=2)
    push!(episode_returns, run["episode_returns"][:])
end
episode_returns_stacked = stack(episode_returns, dims=2)
mean_returns = mean(episode_returns_stacked, dims=2)
plot(plt, mean_returns, label="mean", linecolor=:black, linewidth=10)
title!(plt, file)
xlabel!(plt, "Evaluation #")
ylabel!(plt, "Return")
display(plt)

close(f)