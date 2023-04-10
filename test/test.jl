using PyCall
using Test
using Plots
using SatellitePlayground
SP = SatellitePlayground
using SatelliteDynamics
using LinearAlgebra

@testset "Estimation" begin
    py"""
    import sys
    sys.path.insert(0, "..")
    """
    mekf = pyimport("src")
    MEKF = mekf.MEKF()

    δt = 0.1

    q_err_hist = []

    function measure(state, params, t)
        ω = state.angular_velocity
        nr_mag = params.b
        nr_sun = SatelliteDynamics.sun_position(t)

        ᵇQⁿ = SP.quaternionToMatrix(state.attitude)'

        br_sun = ᵇQⁿ * normalize(nr_sun - state.position)
        br_mag = ᵇQⁿ * normalize(nr_mag)
        MEKF.step(ω, nr_mag, nr_sun, br_mag, br_sun)
        q_err = state.attitude - MEKF.attitude
        push!(q_err_hist, q_err) 
    end

    function control_law(measure, t)
        return zero(SP.Control)
    end

    SP.simulate(control_law, max_iterations=10000, measure=measure, dt=δt)
    q_err_hist = q_err_hist[1:100:end]
    q_err_hist = reduce(hcat, q_err_hist)
    q_err_hist = q_err_hist'
    println(q_err_hist)
    display(
        plot(
            q_err_hist,
            title="Quaternion Error",
            xlabel="Simulation Steps",
            ylabel="Error",
            label=["x" "y" "z" "w"],
            legend=:bottomright
        )
    )
end