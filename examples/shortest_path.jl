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

# Construct the graph
Graphs.add_vertices!(g, 5)

# Create programs on vertices
x = Vector{Vector{VariableRef}}(undef, 0)
for v in Graphs.vertices(g) push!(x, @variable(GCS.Vertex(g, v), [1:2])) end

####################################################
# Test with variables on multiple vertices at once
vs = GCS.Vertices(g, Graphs.vertices(g));
@variable(vs, y[i = 1:5])
@constraint(vs, test, y[1] + y[3] .== 1)
println(MOI.get.(g.model, GCS.ConstraintVertexOrEdge(), test))

f(y) = [1, y]

# Problem: JuMP want to parse the vector [1; y[1]] to broadcast it on all elements of vs,
# but vs has 5 elements while [1; y[1]] has 6.
@constraint(vs, in.(f.(y[1]), Ref(SecondOrderCone())))

h(y) = JuMP.build_constraint(error, [1, y], SecondOrderCone())
JuMP.add_constraint.(vs, h.(y[1]))

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

using GraphPlot
gplot(sol, nodelabel = 1:5, layout = shell_layout)

xv = value.(x)

using Plots
p = plot()

for edge in edges
    if Graphs.has_edge(sol, edge)
        v = [edge...]
        plot!(p, getindex.(xv[v], 1), getindex.(xv[v], 2), arrow = true, label = "", color = :grey, linewidth=2)
    end
end

scatter!(p, [C[:, 1]], [C[:, 2]], label = "", markerstrokewidth = 0, markersize = 3)

for v in filter(x -> Graphs.degree(sol,x) > 0, Graphs.vertices(sol))
    plot!(p, [C[v, 1], getindex(xv[v], 1)], [C[v, 2], getindex(xv[v], 2)], label = "", linestyle = :dash, color = :lightgrey)
    scatter!(p, [getindex(xv[v], 1)], [getindex(xv[v], 2)], label = "$v", markerstrokewidth=0)
end

p
