using Documenter
using DocumenterInterLinks
using GraphsOfConvexSets

# links = InterLinks("MathOptInterface" => "https://jump.dev/MathOptInterface.jl/stable/")

makedocs(
    sitename = "GraphsOfConvexSets.jl",
    format = Documenter.HTML(
        assets = ["assets/favicon.ico"],
        prettyurls = Base.get(ENV, "CI", nothing) == "true",
        mathengine = Documenter.KaTeX(),
    ),
    modules = [GraphsOfConvexSets],
    repo = "https://github.com/CharlesVanHees/GraphsOfConvexSets.jl/blob/{commit}{path}#{line}",
    # checkdocs = :none,
    # clean = true,
    pages = [
        "Home" => "index.md",
        "Quickstart" => "quickstart.md",
        "Bibliography" => "references.bib"
    ],
    # plugins = [links],
)

deploydocs(
    repo = "github.com/CharlesVanHees/GraphsOfConvexSets.jl.git",
    target = "build",
    devbranch = "main",
#     devurl = "dev",
    push_preview = true,
)