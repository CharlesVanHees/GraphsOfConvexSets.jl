using Test

using JuMP
using GraphsOfConvexSets

@testset "Dummy" begin
    @test GCSOptimizer(MOI.Utilities.Model{Float64}) isa MOI.ModelLike
end
