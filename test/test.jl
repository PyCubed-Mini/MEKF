using PyCall
using Test
using Plots
using SatellitePlayground
SP = SatellitePlayground
using SatelliteDynamics
using LinearAlgebra

function randomMatrix(covariance)
    ϕ = √covariance * randn(3)
    return exp(hat(ϕ))
end

""" hat(v)

    Converts a 3-element vector into a cross-product matrix.

    Arguments:
     - v:  3-element vector                   |  [3,] 

    Returns:
     - M:  A [3 × 3] skew-symmetric matrix    |  [3, 3]
"""
function hat(v)
    M = [0.0 -v[3] v[2]
        v[3] 0.0 -v[1]
        -v[2] v[1] 0.0]

    return M
end

function qErr(q₁, q₂)
    return norm((L(q₁)'*q₂)[2:4])
end

""" L(q) 

      Converts a scalar-first unit quaternion into the left-side matrix for 
    quaternion multiplication, as described in "Planning with Attiude" (Jackson)

    Arguments:
     - q:  A scalar-first unit quaternion                         |  [4,]

    Returns: 
     - M:  Left-side matrix representing the given quaternion     |  [4, 4]
"""
function L(q)
    qₛ, qᵥ = q[1], q[2:4]

    M = [qₛ -qᵥ'
        qᵥ qₛ*I(3)+hat(qᵥ)]

    return M
end

function eulerError(e1, e2)
    return acos(dot(e1, e2) / (norm(e1) * norm(e2)))
end

@testset "Estimation" begin
    py"""
    import sys
    sys.path.insert(0, "..")
    """
    mekf = pyimport("src")
    MEKF = mekf.MEKF()

    δt = 0.1

    hist = []

    x_osc_0 = [400e3 + SatelliteDynamics.R_EARTH, 0.0, deg2rad(50), deg2rad(-1.0), 0.0, 0.0] # a, e, i, Ω, ω, M
    q0 = normalize([ 0.030, 0.502, 0.476, 0.780])
    ω0 = 0.1 * [0.3, 0.1, -0.2]
    ω0 = ω0 / norm(ω0) * deg2rad(50.0)

    x0 = SP.state_from_osc(x_osc_0, q0, ω0)

    function measure(state, params, t)
        ω = state.angular_velocity
        nr_mag = params.b
        nr_sun = SatelliteDynamics.sun_position(t)

        ᵇQⁿ = SP.quaternionToMatrix(state.attitude)'

        br_sun = randomMatrix(0.001) * ᵇQⁿ * normalize(nr_sun - state.position)
        br_mag = randomMatrix(0.001) * ᵇQⁿ * normalize(nr_mag)
        nr_mag = normalize(nr_mag)
        nr_sun = normalize(nr_sun)
        MEKF.update(ω, nr_mag, nr_sun, br_mag, br_sun, δt)
        q_err = qErr(state.attitude, MEKF.attitude)
        bias = norm(MEKF.gyro_bias)
        push!(hist, [
            q_err; bias
        ]) 
    end

    function control_law(measure, t)
        return zero(SP.Control)
    end

    SP.simulate(control_law, max_iterations=10000, measure=measure, dt=δt, initial_condition=x0)
    hist = hist[1:10:end]
    hist = reduce(hcat, hist)
    hist = hist'
    # println(hist)
    display(
        plot(
            hist,
            title="Quaternion Error",
            xlabel="Simulation Steps",
            ylabel="Error",
            label=["quaternion_error" "gyro_bias"],
            legend=:topright
        )
    )
end