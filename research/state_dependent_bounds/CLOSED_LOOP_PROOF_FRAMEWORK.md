# Closed-loop properties of a candidate state-dependent output-feedback tube MPC

Status: analytical design note, 18 September 2026. This is a conditional theorem
for a NEW MPC formulation. It is not a proof that the current Falcon implementation
satisfies all assumptions. No new MPC implementation, simulation, or SDP solve is
performed here. In particular the original z(0)=xhat reset is replaced with a tube
consistency constraint. The full planning/tracking ROHMPC hierarchy is not covered.

## 1. Scope, notation and uncertainty assumptions

Consider a fixed regulation target (x_ref,u_ref) satisfying f0(x_ref,u_ref)=0.
The bias-corrected nonlinear nominal model and plant/observer are

    x_dot = f0(x,u) + d(x,u,w),       y = C*x + F*eta,
    xhat_dot = f0(xhat,u) + L*(y-C*xhat),
    z_dot = f0(z,v),                 u = v + K*(xhat-z).

K,L and P=P_delta>0 are fixed. Disturbances are measurable and belong to explicitly
specified sets W(x,u), H(x,u) at every time. State dependence is assumed known; it
is not inferred from containment in the baseline UQ box. No dependence on the
Falcon model or speed specifically is needed in the theorem.

Let e=x-xhat, delta=xhat-z, V_e=e'*P*e, r=sqrt(R), and t_delta=||delta||_P.
Use a finite horizon H>0 and a fixed sample period h with 0<h<=H. Constraints and
the target are time invariant. Online obstacles or changing references need their
own consistency assumptions and are outside this first theorem.

## 2. The certificate assumptions (must be verified, not merely sampled)

A1. Solutions exist uniquely and remain well-defined for the controls and bounded
uncertainties under consideration. Nominal, observer, and error dynamics have
appropriate regularity. Conditions below hold on a neighborhood of the certified
state/input domain, including all segments used in incremental arguments.

A2. For every nominal (z,v), radii (R,s), and states with

    ||x-xhat||_P^2 <= R,    ||xhat-z||_P <= s,

and all allowed disturbance/noise realizations, the following hold:

    V_e_dot <= -lambda*V_e + q(z,v,R,s),
    D+ t_delta <= -rho*t_delta + gamma*sqrt(V_e) + b_eta.

lambda,rho>0 and gamma,b_eta>=0 are constants. q is a uniformly valid local
forcing allowance over the WHOLE uncertainty tube, not just at z or xhat.
These conditions are the certificate input to the MPC theorem. They include the
relevant state/input dependence of uncertainty and feedback inputs. Upper Dini
derivatives handle the tracking norm at zero.

A3. There are constants 0<q_min<=q(z,v,R,s)<=q_max on the admissible augmented
domain. Choose

    0 < R_min <= q_min/lambda,
    R_max = q_max/lambda,
    s_max = (gamma*sqrt(R_max)+b_eta)/rho.

Restrict R to [R_min,R_max], s to [0,s_max]. The positive lower bound avoids the
square-root regularity issue at zero in the augmented ODE. It is a design choice:
an exactly known initial state may still be assigned the small positive R_min.
If q_min=0, the theorem needs a separate well-posedness/comparison treatment;
this note does not silently assume local Lipschitz continuity of sqrt(R) at zero.
Assume q is locally Lipschitz on the positive-R augmented domain. The radius ODE is

    R_dot = -lambda*R + q(z,v,R,s),
    s_dot = -rho*s + gamma*sqrt(R) + b_eta.                  (A)

The rectangular radius set B=[R_min,R_max] x [0,s_max] is invariant: R_dot>=0 on
R_min, R_dot<=0 on R_max, s_dot>=0 on 0, and s_dot<=0 on s_max.

A4. At t=0 an available number Rbar_0 in [R_min,R_max] satisfies
||x(0)-xhat(0)||_P^2<=Rbar_0. This is a prior bound, not access to the unknown x.
The observer is not reset during an interval. A measurement reset requires a
separately certified jump update for Rbar.

## 3. Tightened constraints and domain preservation

For a general mixed constraint g_j(x,u)<=0, define the robust tightening by the
supremum over the tube:

    sup g_j(z+delta+e, v+K*delta) <= 0,
    where ||e||_P<=sqrt(R), ||delta||_P<=s.                  (B)

For affine constraints a_j'*x+b_j'*u<=c_j, (B) has the explicit support form

    a_j'*z+b_j'*v
    + ||(a_j'+b_j'*K)/T||_2*s
    + ||a_j'/T||_2*sqrt(R) <= c_j,                          (C)

where T'*T=P. This separates the effects correctly: input uncertainty is due to
delta, while e affects the true state. The existing baseline c_s,c_o can be used
when they upper-bound the corresponding support coefficients; they are not
implicitly exact.

Also require the complete true-state tube, observer-state tube, and feedback-input
sets to lie in the domain supporting A1-A3. Thus speed caps and other domain bounds
are enforced by the construction rather than assumed solely because a nominal
state satisfies a box. For a convex common domain the required state/input
segments then remain inside. These constraints must hold for all prediction times,
not only at collocation nodes; discretized implementations need an intersample
argument.

## 4. A conservative terminal construction that preserves local benefits

Assume a nonempty nominal terminal set Z_f and a locally Lipschitz nominal terminal
controller kappa_f(z) exist with the following properties:

T1. z_dot=f0(z,kappa_f(z)) leaves Z_f positively invariant.
T2. Every z in Z_f with v=kappa_f(z) satisfies (B) and the certificate-domain
constraints at the worst-case radii R_max,s_max.
T3. There is V_f>=0 and a nominal stage cost ell(z,v)>=0, positive definite about
the reference on the relevant domain, such that

    grad V_f(z)*f0(z,kappa_f(z)) <= -ell(z,kappa_f(z)).       (D)

Then the augmented terminal set

    Omega_f = Z_f x [R_min,R_max] x [0,s_max]

is invariant under (A), z_dot=f0(z,kappa_f(z)). T1 keeps z in Z_f; A3 keeps radii
in B; T2 guarantees tightened constraints. Terminal invariance suffices for the
feasibility argument. T3 is used for the value-decrease/performance argument.

This is deliberately conservative at the terminal end. It retains local q and
smaller radii elsewhere along the horizon, and avoids claiming that the original
sum-of-radii terminal condition automatically remains invariant.

CRITICAL PRACTICAL LIMITATION: the reproduced Falcon global tightening had empty
input intervals. For those unchanged numerical ingredients T2 may be impossible,
so we have not demonstrated a nonempty Z_f. A theorem requiring T2 cannot cure that.
A redesign, different assumptions/domain, or a rigorously verified local terminal
construction may be necessary. Empirical calibration alone is not a proof of T2.

## 5. The precise MPC problem and online radius bookkeeping

At sample k the controller has xhat_k and the certified prior Rbar_k. Optimize
z(.),v(.),R(.),s(.) over [0,H] with nominal/radius ODEs and

    R(0) = Rbar_k,
    ||xhat_k-z(0)||_P <= s(0),
    (R(tau),s(tau)) in B,
    tightened and domain constraints for all tau in [0,H],
    (z(H),R(H),s(H)) in Omega_f.

The nominal initial state z(0) is a decision variable; it is NOT required to equal
xhat_k. The objective for the present proof is

    J = integral_0^H ell(z(tau),v(tau)) d tau + V_f(z(H)).    (E)

There are no radius penalties in this first theorem. Adding them requires their
contribution to the terminal decrease inequality; they must not be ignored.

Apply u(t)=v_k*(tau)+K*(xhat(t)-z_k*(tau)) for tau in [0,h]. At the next sample set

    Rbar_(k+1) = R_k*(h).

This number is computable and remains a valid estimation bound by A2 and (A).
It avoids needing the unknown true error. A tighter independently certified bound
could be used, but requires an additional comparison argument if equality is kept
in the initialization rule. That enhancement is not assumed here.

## 6. Lemma: containment during an applied interval

Initially V_e<=R and t_delta<=s. At the boundary V_e=R, A2 gives

    d/dt(V_e-R) <= 0.

At t_delta=s and V_e<=R, A2 gives

    D+(t_delta-s) <= gamma*(sqrt(V_e)-sqrt(R)) <= 0.

Under A1 and the stated regularity, the moving tube is forward invariant (the
standard tangent/comparison argument). Non-strict boundary inequalities rely on
well-posedness; bare numerical tests are insufficient. Hence V_e<=R and
||delta||_P<=s throughout the applied interval. This is a coupled tube lemma and
does not require resetting s to zero at each MPC update.

By (B), actual (x,u) satisfy their constraints. The domain constraints ensure the
certificate hypotheses hold over the tube; a standard first-exit continuation
argument closes the domain/containment induction rather than assuming safety as
a conclusion's premise.

## 7. Theorem: recursive feasibility and robust constraint satisfaction

Assume A1-A4 and T1-T2, and an initially feasible MPC problem. At every sample a
feasible solution is used and its first h seconds are executed as described. Then
the MPC problem remains feasible at every subsequent sample, and true state/input
constraints hold for all times, for every uncertainty realization in the assumed
sets. This is conditional on the continuous-time enforcement and regularity above.

Proof. At the next sample construct a candidate on [0,H]:

(a) For tau in [0,H-h], shift the previous solution: z_c(tau)=z_k*(tau+h), and
likewise v_c,R_c,s_c. Time invariance makes the ODEs and constraints unchanged.
Because Rbar_(k+1)=R_k*(h), the initial R equality holds. The containment lemma gives
||xhat_(k+1)-z_k*(h)||_P<=s_k*(h), so initial tube consistency also holds.

(b) Over the remaining h seconds, start from the previous terminal augmented
state and apply kappa_f with the radius ODE. Terminal invariance keeps this tail
in Omega_f and satisfies all tightened and domain constraints.

Thus the shifted-and-extended candidate is feasible. Repeating gives recursive
feasibility. The containment lemma plus robust tightening gives true constraint
satisfaction on every applied interval. No disturbance realization has to be
predicted; only membership in the prescribed sets is used. QED.

This proof depends materially on the free nominal initial center. It is NOT a
proof for the original ROHMPC reset z(0)=xhat_k, s(0)=0. That formulation needs a
separate recentering/shrinkage argument; smaller q by itself does not supply one.

## 8. Terminal decrease and the limits of the stability claim

If T3 also holds and the MPC is solved to an attained global optimum, the candidate
cost satisfies

    J_candidate,k+1 <= J_k* - integral_0^h ell(z_k*,v_k*) d tau.

This follows by cancelling the common shifted horizon portion and integrating
(D) on the terminal tail. Since the next optimum is no larger than the feasible
candidate, the same inequality holds with J_(k+1)* on the left. Consequently the
sum of applied nominal stage-cost integrals is bounded by J_0*.

For a suboptimal solver, feasibility still follows if a feasible candidate is
retained. The value inequality requires accepting a plan whose cost is no greater
than that candidate; arbitrary feasible suboptimal solutions need not satisfy it.

This alone is NOT a full asymptotic stability proof for true x. Nominal trajectory
recentering can create jumps between predicted nominal trajectories; additional
regularity/detectability arguments are needed to convert summable costs to the
appropriate convergence statement. Persistent disturbances also generally permit
only practical, rather than exact, state convergence. At every applied instant,

    ||x-x_ref||_P <= ||z-x_ref||_P + sqrt(R)+s.

If nominal tracking converges under additional proven conditions, this yields a
practical bound by sqrt(R_max)+s_max. We do not claim that convergence here.

## 9. How the present local-envelope model fits

Our formula q=q0-Delta*(1-a_upper)/(1-beta), with a_upper in [beta,1], has
q_max=q0=lambda*epsilon^2 and q_min=q0-Delta>0 in the recorded experiment.
Thus R_max=epsilon^2, and the terminal worst-case radius matches the original
uncalibrated one. q depends on ||z_velocity||+c_v*(sqrt(R)+s) and is locally
Lipschitz for R>=R_min>0; the norm and min are Lipschitz even at their kinks.
The cap must be justified by the domain constraints in section 3. The local
Schur-envelope proof supplies only part of A2; continuous-domain Jacobian coverage
and numerical residual treatment remain required.

The prior global-versus-local radius ordering result is useful but is NOT the
recursive-feasibility theorem. Here feasibility is obtained by candidate
construction, exact continuation of the augmented dynamics, and terminal
invariance. Monotonicity of q is not needed to shift the identical solution; it is
useful when comparing different radius trajectories or tightening initial bounds.

## 10. Next verification tasks

1. Establish a continuous-domain enclosure for the observer and tracking Jacobians
   and account rigorously for numerical matrix residuals (A2).
2. Decide whether to use this free-center initialization or prove recentering for
   the original reset formulation before modifying the MPC implementation.
3. Find a NONEMPTY terminal construction compatible with actual state/input
   limits; the current Falcon global margins do not supply it (T1-T2).
4. Verify a terminal cost inequality and appropriate assumptions for the desired
   stability/performance statement (T3 and section 8).
5. Only then implement and simulate the complete MPC, retaining the previous
   feasible candidate as a fallback for numerical solver failures.

No assumption above has been discharged merely because the sampled Falcon LMIs
passed. The theorem identifies the conditions needed for a generalized robust MPC
result and which pieces of the current derivation can be reused.