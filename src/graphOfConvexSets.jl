struct GraphModel{T, G <: Graphs.AbstractGraph{T}, M <: JuMP.AbstractModel} <: Graphs.AbstractGraph{T}
    graph::G
    model::M

    function GraphModel(
        g::G,
        @nospecialize(optimizer_factory = nothing);
        kwargs...
    ) where {T, G <: Graphs.AbstractGraph{T}}
        new{T, G, JuMP.Model}(g, JuMP.Model(optimizer_factory, kwargs...))
    end
end

Graphs.nv(g::GraphModel) = Graphs.nv(g.graph)
Graphs.vertices(g::GraphModel) = Graphs.vertices(g.graph)
Graphs.has_vertex(g::GraphModel, v::Integer) = Graphs.has_vertex(g.graph, v)
Graphs.add_vertex!(g::GraphModel) = Graphs.add_vertex!(g.graph)
Graphs.rem_vertex!(g::GraphModel, v::Integer) = Graphs.rem_vertex!(g.graph, v)
# TODO : Remove variables, constraints and cost associated to v when removing v

Graphs.ne(g::GraphModel) = Graphs.ne(g.graph)
Graphs.edgetype(g::GraphModel) = Graphs.edgetype(g.graph)
Graphs.edges(g::GraphModel) = Graphs.edges(g.graph)
Graphs.has_edge(g::GraphModel, s::I, d::I) where {I <: Integer} = Graphs.has_edge(g.graph, s, d)
Graphs.add_edge!(g::GraphModel, s::I, d::I) where {I <: Integer} = Graphs.add_edge!(g.graph, s, d)
Graphs.rem_edge!(g::GraphModel, s::I, d::I) where {I <: Integer} = Graphs.rem_edge!(g.graph, s, d)
# TODO : Remove variables, constraints and cost associated to (s,d) when removing (s,d)

Graphs.is_directed(g::GraphModel) = Graphs.is_directed(g.graph)
Graphs.inneighbors(g::GraphModel, v::I) where {I <: Integer} = Graphs.inneighbors(g.graph, v)
Graphs.outneighbors(g::GraphModel, v::I) where {I <: Integer} = Graphs.outneighbors(g.graph, v)

struct Vertex{M <: JuMP.AbstractModel, T<:Integer} <: JuMP.AbstractModel
    model::M
    vertex::T

    function Vertex(g::GraphModel{T,G,M}, v::T) where {M <: JuMP.AbstractModel, T<:Integer, G<:Graphs.AbstractGraph{T}}
        Graphs.has_vertex(g,v) || throw(BoundsError("The graph g does not have a vertex $(v). You can add it through the add_vertex! method."))
        return new{M,T}(g.model, v)
    end
end
Base.broadcastable(v::Vertex) = Ref(v)

function JuMP.add_variable(v::Vertex, var...)
    var_ref = JuMP.add_variable(v.model, var...)
    MOI.set(v.model, VariableVertexOrEdge(), var_ref, v.vertex)
    return var_ref
end

function JuMP.add_constraint(v::Vertex, con, var...)
    _check(JuMP.backend(v.model), con, JuMP.moi_function(con), v.vertex)
    con_ref = JuMP.add_constraint(v.model, con, var...)
    MOI.set(v.model, ConstraintVertexOrEdge(), con_ref, v.vertex)
    return con_ref
end

function JuMP.set_objective_function(v::Vertex, func)
    _check(JuMP.backend(v.model), func, JuMP.moi_function(func), v.vertex)
    MOI.set(v.model, VertexOrEdgeObjective(v.vertex), JuMP.moi_function(func))
end

struct Edge{M <: JuMP.AbstractModel, T<:Integer} <: JuMP.AbstractModel
    model::M
    edge::Tuple{T, T}

    function Edge(g::GraphModel{T,G,M}, s::T, d::T) where {M <: JuMP.AbstractModel, T <: Integer, G<: Graphs.AbstractGraph{T}}
        Graphs.has_edge(g,s,d) || throw(BoundsError("The graph g does not have an edge from $(s) to $(d). You can add it through the add_edge! method."))
        return new{M,T}(g.model, (s,d))
    end
end
Base.broadcastable(e::Edge) = Ref(e)

function JuMP.add_variable(e::Edge, var...)
    var_ref = JuMP.add_variable(e.model, var...)
    MOI.set(e.model, VariableVertexOrEdge(), var_ref, e.edge)
    return var_ref
end

function JuMP.add_constraint(e::Edge, con, var...)
    _check(JuMP.backend(e.model), con, JuMP.moi_function(con), e.edge)
    con_ref = JuMP.add_constraint(e.model, con, var...)
    MOI.set(e.model, ConstraintVertexOrEdge(), con_ref, e.edge)
    return con_ref
end

function JuMP.set_objective_function(e::Edge, func)
    _check(JuMP.backend(e.model), func, JuMP.moi_function(func), e.edge)
    MOI.set(e.model, VertexOrEdgeObjective(e.edge), JuMP.moi_function(func))
end

# TODO : extract model associated to a given vertex/edge (see what has been done for the serializer)

function shortest_path(g::GraphModel{T,G,M}, s::T, t::T) where {M,T,G}
    Graphs.has_vertex(g, s) || Graphs.has_vertex(g, t) || throw(ArgumentError("The graph g does not have either vertex $(s), either vertex $(t)."))

    JuMP.set_objective_sense(g.model, MOI.MIN_SENSE)
    MOI.set(g.model, Problem(), ShortestPathProblem(1,2))
    JuMP.optimize!(g.model)
end
