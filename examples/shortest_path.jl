# Inspired from https://github.com/TobiaMarcucci/gcspy/blob/main/examples/shortest_path/shortest_path.py

using JuMP
import Pajarito, HiGHS, Hypatia
using LinearAlgebra
import GraphsOfConvexSets as GCS
import Graphs

g = GCS.GraphModel(Graphs.SimpleDiGraph(),
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
println(g)

# Construct the graph
Graphs.add_vertices!(g, 5)

# Create programs on vertices
x = Vector{Vector{VariableRef}}(undef, 0)
for v in Graphs.vertices(g) push!(x, @variable(GCS.Vertex(g, v), [1:2])) end

####################################################

# Centers
C = [
     1    0
    10    0
     4    2
     5.5 -2
     7    2
]

# source vertex
D = Diagonal([1, 1/2]) # scaling matrix

@constraint(GCS.Vertex(g, 1), [1; D * (x[1][:] - C[1, :])] in SecondOrderCone())

# target vertex
D = Diagonal([1/2, 1]) # scaling matrix
@constraint(GCS.Vertex(g, 2), [1; D * (x[2][:] - C[2, :])] in SecondOrderCone())
@constraint(GCS.Vertex(g, 2), x[2][1] <= C[2, 1]) # cut right half of the set

# vertex 1
@constraint(GCS.Vertex(g, 3), [1; x[3][:] - C[3, :]] in MOI.NormInfinityCone(3))

# vertex 2
@constraint(GCS.Vertex(g, 4), [1.2; x[4][:] - C[4, :]] in MOI.NormOneCone(3))
@constraint(GCS.Vertex(g, 4), [1; x[4][:] - C[4, :]] in SecondOrderCone())

# vertex 3
@constraint(GCS.Vertex(g, 5), [1; x[5][:] - C[5, :]] in SecondOrderCone())

edges = [(1, 3), (1, 4), (3, 4), (3, 5), (4, 5), (4, 2), (5, 2)]
for (u,v) in edges Graphs.add_edge!(g, u, v) end

cost = Vector{VariableRef}(undef, 0)
for (src, dst) in edges
    edge = GCS.Edge(g, src, dst)
    # Cost of the edge
    edge_cost = @variable(edge)
    push!(cost, edge_cost)
    JuMP.set_objective_function(edge, edge_cost) # TODO : find better
    @constraint(edge, [edge_cost; x[:][dst] - x[:][src]] in SecondOrderCone())

    # Constraint on the edge
    @constraint(edge, x[src][2] <= x[dst][2])
end

GCS.shortest_path(g, 1, 2)

sol = MOI.get(g.model, GCS.SubGraph())
xv = value.(x)


############################################
## Visualize the solution
############################################

using Plots
p = plot()

rgb(r,g,b) = RGB(r/255, g/255, b/255)

# Draw the five convex vertices
# S_1
c1 = rgb(56, 130, 221)
plot!(p,
    Shape(1 .+ cos.(range(0, 2π, 300)),
          2 .* sin.(range(0, 2π, 300))),
    aspect_ratio = :equal,
    label = "",
    linecolor = c1,
    fillcolor = c1,
    fillalpha = 0.3
)

# S_2
c2 = rgb(220, 90, 56)
plot!(p,
    Shape([10 .+ 2 .* cos.(range(π/2, 3π/2, 300)); 10; 10],
          [sin.(range(π/2, 3π/2, 300)); -1; -1]),
    aspect_ratio = :equal,
    label = "",
    linecolor = c2,
    fillcolor = c2,
    fillalpha = 0.3
)

# S_3
c3 = rgb(80, 170, 120)
plot!(p,
    Shape([3, 5, 5, 3, 3],
          [1, 1, 3, 3, 1]),
    aspect_ratio = :equal,
    label = "",
    linecolor = c3,
    fillcolor = c3,
    fillalpha = 0.3
)

# S_4
c4 = rgb(150, 90, 200)
plot!(p,
    Shape([4.3, 5.5, 6.7, 5.5, 4.3],
          [-2, -3.2, -2, -0.8, -2]),
    aspect_ratio = :equal,
    label = "",
    linecolor = c4,
    fillcolor = c4,
    fillalpha = 0.3
)
plot!(p, 
    Shape(5.5 .+ cos.(range(0, 2π, 300)),
          -2  .+ sin.(range(0, 2π, 300))),
    aspect_ratio = :equal,
    label = "",
    linecolor = c4,
    fillcolor = c4,
    fillalpha = 0.3
)

# S_5
c5 = rgb(230, 160, 30)
plot!(p, 
    Shape(7 .+ cos.(range(0, 2π, 300)),
          2 .+ sin.(range(0, 2π, 300))),
    aspect_ratio = :equal,
    label = "",
    linecolor = c5,
    fillcolor = c5,
    fillalpha = 0.3
)

# Plot the shortest path

for v in filter(x -> Graphs.degree(sol,x) > 0, Graphs.vertices(sol))
    scatter!(p, [getindex(xv[v], 1)], [getindex(xv[v], 2)], label = "", markersize=5, markercolor = :red)
end

for edge in edges
    if Graphs.has_edge(sol, edge)
        v = [edge...]
        plot!(p, getindex.(xv[v], 1), getindex.(xv[v], 2), arrow = true, label = "", color = :grey, linewidth=2)
    end
end

display(p);