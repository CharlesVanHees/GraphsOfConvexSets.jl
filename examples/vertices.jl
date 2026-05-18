"""
Example of use of the GraphsOfConvexSets.jl package,
in which we add the same variables and constraints on several vertices at once.
"""

using JuMP
import GraphsOfConvexSets as GCS
import Graphs

# Create the GCS
g = GCS.GraphModel(Graphs.SimpleDiGraph())
Graphs.add_vertices!(g,5) # Add vertices

# We create an auxiliary model, which contains the same JuMP model as g,
# but specifying the vertices on which it applies.
# All the variables and constraints defined on this model will be
# duplicated on all the vertices of the model.
vs = GCS.Vertices(g, Graphs.vertices(g)); # This model concerns all the vertices of the graph
@variable(vs, 0 ≤ x[i = 1:2] ≤ 2) # Define a vertex of 2 variables on each vertex of vs
# Remark: x is a Vector with 2 Vectors of 5 VariableRef
x = collect.(eachrow(stack(x)))


# Now, we will add the same constraints on several vertices
v_2_3_4 = GCS.Vertices(g, 2:4); # Constraints will go on vertices 2, 3 and 4
c1(x) = JuMP.build_constraint(error, [1; x], SecondOrderCone())
JuMP.add_constraint.(v_2_3_4, c1.(x[2:4]))

v_1_5 = GCS.Vertices(g, [1,5]); # Those constraints will be on vertices 1 and 5
c2(x) = JuMP.build_constraint(error, 1 - x[1] + 3x[2], MOI.EqualTo(0))
JuMP.add_constraint.(v_1_5, c2.(x[[1,5]]))