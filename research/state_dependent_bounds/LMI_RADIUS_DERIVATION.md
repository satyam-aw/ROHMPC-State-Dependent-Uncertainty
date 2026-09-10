# Recovering the original observer certificate as dynamic radius propagation

## Scope and result

This derivation uses the saved constant X, P_delta=X^{-1}, L, lambda_epsilon,
W_bar and H1_bar from the reproduced offline design. No new SDP is solved.
The original process and noise bounds are the supplied UQ bounds; local
acceleration bounds are prescribed synthetic subsets, not reidentified data.
The original submodules and baseline MAT file are not modified.

The old prototype used separate metric-norm maxima and gave a larger bound
than the original SDP. This second version retains the original LMI quadratic
structure and recovers its epsilon in the global case.

## 1. The inequalities actually implemented

Write n=nx, lambda=lambda_epsilon, P=P_delta, X=P^{-1}, A_o=A-LC, and

    S(b) = [0_(n x n), b; b', 0].

compute_check_epsilon_RPI_LMIs.m builds three families:

    S(E*w) <= W,                                           (1)
    S(-L*F*eta) <= H,                                      (2)
    W + H + diag_blocks(X*A_o' + A_o*X + lambda*X,
                        -lambda*epsilon^2) <= 0.           (3)

Here W=W_bar and H=H1_bar are symmetric auxiliary matrices. These are matrix
inequalities in the positive-semidefinite order, not entrywise inequalities.
This derivation is for constant X: the baseline dotX term is zero.

Adding (1)-(3) eliminates W and H. For b=E*w-L*F*eta:

    [X*A_o' + A_o*X + lambda*X, b;
      b',                         -lambda*epsilon^2] <= 0. (4)

Apply the quadratic form with vector [P*e;1], where e=x-xhat:

    e'*(A_o'*P + P*A_o)*e + 2*e'*P*b
          <= -lambda*(e'*P*e) + lambda*epsilon^2.           (5)

For a nonlinear f0, f0(x,u)-f0(xhat,u) equals a segment-averaged Jacobian
acting on e. Consequently the Jacobian inequalities must hold along the
whole segment joining xhat and x, not merely at one nominal state. If that
coverage holds, the left side of (5) is V_dot for V=e'*P*e.

## 2. Dynamic bound that exactly recovers the baseline

Use R=r^2 as a scalar squared-radius variable (not the Cholesky factor used
in the first prototype):

    R_dot = -lambda*R + q0,   q0=lambda*epsilon^2,
    R(0) >= V(0).

Then V(t)<=R(t), under the stated derivative inequality. Explicitly,

    R(t)=epsilon^2 + (R(0)-epsilon^2)*exp(-lambda*t).

If R(0)=epsilon^2, r(t)=epsilon for all t. Hence this recovers the original
invariant bound without the triangle-inequality conservatism of prototype 1.
A zero initial radius is only valid if initial estimation error is zero.
Propagating R avoids division by r at r=0.

## 3. Reducing the forcing for a local disturbance set

Partition the SAVED process envelope as

    W = [Q,h;h',c].

The numerical baseline has Q positive definite. By the Schur complement,

    S(E*w) <= W  iff  (E*w-h)'*Q^{-1}*(E*w-h) <= c.

Keep Q and h unchanged, and scale only acceleration-channel residuals by
alpha in [beta,1], beta=0.25. Define

    g(alpha) = max over original disturbance vertices w of
               (E*D(alpha)*w-h)'*Q^{-1}*(E*D(alpha)*w-h).

D(alpha) is diagonal: alpha in channels 7:9, one in all other channels.
The maximum over vertices covers the entire box since the quadratic is
convex in w. Each vertex expression is convex in alpha, so their maximum
is convex. The sets are nested, hence g(beta)<=g(1). Convexity gives

    g(alpha) <= g(1) - (1-alpha)/(1-beta)*(g(1)-g(beta)).

Let Delta=0.99*(g(1)-g(beta)); the 1% retained saving is a numerical cushion,
not a substitute for validating numerical feasibility. Given c>=g(1), set

    c_local(alpha) = c - Delta*(1-alpha)/(1-beta),
    W_local(alpha) = [Q,h;h',c_local(alpha)].

Then W_local covers all local disturbance realizations. Define

    q(alpha) = q0 - Delta*(1-alpha)/(1-beta).

The reduction in W's bottom-right entry cancels exactly with the reduction
in q, so the dynamics LMI (3) is unchanged when using W_local and -q(alpha).
The noise envelope H stays fixed. This is a local modification of the
auxiliary process envelope, not just sampling smaller disturbances while
retaining one global W. No new decision variables or metric are required.

At alpha=1, W_local=W and q=q0. At lower alpha, forcing can decrease while
retaining the original quadratic error argument. For this example q remains
positive throughout [beta,1]. A different model must recheck that property.

## 4. Unknown true velocity and coupled tracking tube

Let T=chol(P), so T'*T=P, and S_v select velocity states. Define

    c_v=||S_v/T||_2, gamma=||T*L*C/T||_2,
    b_noise=max_eta ||T*L*F*eta||_2.

With ||x-xhat||_P<=sqrt(R), ||xhat-z||_P<=s:

    v_upper=min(vmax, ||z_velocity|| + c_v*(sqrt(R)+s)),
    alpha_upper=beta+(1-beta)*(v_upper/vmax)^2,
    R_dot=-lambda*R+q(alpha_upper),
    s_dot=-rho_c*s+gamma*sqrt(R)+b_noise.

The cap vmax=sqrt(12) assumes the true state remains in the original
componentwise velocity domain [-2,2]^3. The nominal, observer and true states
and relevant feedback inputs must also remain in the domain supporting the
LMIs. These equations alone DO NOT prove domain preservation.

Because q is nondecreasing with alpha and the off-diagonal radius dependence
is nonnegative, a coupled comparison argument applies with valid initial
bounds. The local solution can be compared with the global solution using
the SAME metric, gains, lambda, rho_c, noise and initial radii.

## 5. Numerical results (2026-09-10)

Run run_lmi_radius_recovery.m with Windows MATLAB and the WSL files.
It saves each run to a new timestamped results/lmi_* folder.

    lambda                  0.497029 (rounded)
    original epsilon        0.07442638619
    q0                      0.002753187883
    recovered sqrt(q0/lambda) 0.07442638619
    g(1)                    0.00148773 (rounded)
    g(beta)                 0.00104178 (rounded)
    retained reduction Delta 0.000441487 (rounded)

Under a prescribed nominal-speed schedule 0 -> 2.5 -> 0 m/s over 32 seconds,
starting at R(0)=epsilon^2, s(0)=0:

    final local r           0.06945456129
    global r                0.07442638619
    final radius reduction  6.68019 percent
    final local s           0.367532
    final global s          0.392384

This schedule is only a tube-ODE illustration, NOT a simulated dynamically
feasible quadrotor trajectory. The percentage is a final-time comparison,
not an average closed-loop improvement or a universal performance claim.

## 6. What was checked and what is not proved

- All original process/noise envelope vertices and 1024 original dynamics
  points checked with the saved solver tolerance 1e-6.
- Maximum original matrix residuals are approximately +5.6e-8, NOT strictly
  negative. This is numerical feasibility within tolerance, not exact or
  interval-certified feasibility. The Schur-envelope residual is similar.
- All 4096 disturbance vertices checked at 31 scale values; the convex-chord
  derivation above, conditional on the original exact inequalities, covers
  intermediate scale values analytically.
- Global scalar ODE checked against its analytical solution.
- 1968 admissible random nonlinear error configurations checked directly;
  the maximum tested squared-error derivative excess was -0.00229595.
- Coupled local radius trajectories stayed below their baseline-equivalent
  global comparisons in the illustrative schedule.

The underlying Jacobian coverage between operating points remains unproved.
The earlier expanded-grid audit does not replace a rigorous enclosure.
Finite tests cannot prove nonlinear closed-loop safety or recursive
feasibility. Terminal-set compatibility and model/data validation remain
separate tasks. Numerical residuals also require rigorous treatment before
claiming an exact certificate.