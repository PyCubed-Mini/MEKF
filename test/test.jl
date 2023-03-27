using GNCTestSever
using PyCall

@testset "Estimation" begin
    py"""
    import sys
    sys.path.insert(0, "..")
    """
    mekf = pyimport(".src")
    MEKF = mekf.MEKF
    function control_law(state, params, t)
        return [0.0, 0.0, 0.0]
    end
end