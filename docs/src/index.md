# GraphsOfConvexSets

GraphsOfConvexSets is a [MathOptInterface](https://github.com/jump-dev/MathOptInterface.jl/) and [Graphs.jl](https://juliagraphs.org/Graphs.jl/stable/) extension for optimization over graphs of convex sets.

## What is a Graph of Convex Sets?
Graphs of Convex Sets (GCS) is a framework [Marcucci2025](@cite) for modelling optimization problems mixing graph problems and convex optimization: vertices of a graph correspond to convex programs, and those are linked through edges that may put additional convex constraints on the variables of its extremities programs.
All standard graph problems (Shortest Path Problem, Travelling Salesman Problem, Minimum Spanning Tree Problem, etc.) can be generalized to GCS.