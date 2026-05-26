# Quickstart

The following code shows how to solve a Shortest Path Problem on a GCS.

```jldoctest quickstart
julia> import GraphsOfConvexSets as GCS

julia> import Graphs

julia> import JuMP

julia> import MathOptInterface as MOI

julia> import Pajarito, HiGHS, Hypatia
```

This problem corresponds to a graph with five vertices, on which we want the shortest path between the vertices 1 and 2.

## Create the GCS

First, we create a GCS structure

```jldoctest quickstart
julia> g = GCS.GraphModel(Graphs.SimpleDiGraph(),
    optimizer_with_attributes(
        Pajarito.Optimizer,
        "oa_solver" => optimizer_with_attributes(
            HiGHS.Optimizer,
            MOI.Silent() => false,
            "mip_feasibility_tolerance" => 1e-8,
            "mip_rel_gap" => 1e-6,
        ),
        "conic_solver" =>
            optimizer_with_attributes(Hypatia.Optimizer, MOI.Silent() => false),
    )
)
GraphModel
{0, 0} directed simple Int64 graph
A JuMP Model
├ solver: unknown
├ objective_sense: FEASIBILITY_SENSE
├ num_variables: 0
├ num_constraints: 0
└ Names registered in the model: none
```

!!! note
    Solving a problem on a GCS requires a Mixed Integer solver
    If all the constraints are linear, you may choose a MILP solver.
    Yet, if your GCS has feasible sets described by conic (non-linear) constraints, you must choose a MISOCP.
    See [Supported solvers](https://jump.dev/JuMP.jl/stable/installation/#Supported-solvers) for a list of available solvers.

Next, we specify the number of vertices and the edges of the graph.
```jldoctest quickstart
julia> Graphs.add_vertices!(g, 5);

julia> edges = [(1, 3), (1, 4), (3, 4), (3, 5), (4, 5), (4, 2), (5, 2)];

julia> for (u,v) in edges
    Graphs.add_edge!(g, u, v)
end;
```

### Vertices
On each vertex, we define the associated conic program.
This is done in two steps

 1. Create the variables of the conic program
```jldoctest quickstart
julia> x = Vector{Vector{VariableRef}}(undef, 0);

julia> for v in Graphs.vertices(g)
    push!(x, @variable(GCS.Vertex(g, v), [1:2]))
end;
```

 2. Define the constraints
```jldoctest quickstart
julia> # Constraint on the vertex 1

julia> # The convex set is the ellipse of equation (x - 1)^2 + y^2 / 4 <= 1.

julia> @constraint(GCS.Vertex(g, 1), [1; Diagonal([1, 1/2]) * (x[1][:] - [1, 0])] in SecondOrderCone());

julia> ... # Define constraints on all vertices
```

### Edges
Similarly, we first proceed by adding the variables, then the constraints, and finally an objective.

!!! warning
    As of today, the problem must be given in conic form.
    Hence, if an edge has an objective ${\lVert x_1 - x_0 \rVert}^2$, introduce an auxiliary variable $t$, add the constraint ${\lVert x_1 - x_0 \rVert}^2 \leq t$, and put $t$ in the objective.

```jldoctest quickstart
julia> # Example on the edge 1 -> 3

julia> cost_1_3 = @variable(GCS.Edge(g,1,3));

julia> # t >= ||x_3 - x_1||^2

julia> @constraint(GCS.Edge(g, 1, 3), [edge_cost_1_3; x[:][3] - x[:][1]] in SecondOrderCone());

julia> JuMP.set_objective_function(GCS.Edge(g,1,3), cost_1_3);
```

## Solve the Shortest Path Problem
```julia
julia> GCS.shortest_path(g,1,2); # Compute the shortest path between vertices 1 and 2
```

There are two information two extract:

 1. Which vertices are explored
```julia
julia> sol = MOI.get(g.model, GCS.SubGraph());
```

 2. The chosen point in the explored vertices
```julia
julia> value_x = value.(x);
```

!!! info
    `value_x` also contains the values of the variables of the non-chosen vertices.
    On the non-chosen vertices, the corresponding point may not be feasible.