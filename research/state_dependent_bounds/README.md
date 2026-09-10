# Start here: baseline-equivalent LMI derivation

The newer experiment is `run_lmi_radius_recovery.m`. Read
[LMI_RADIUS_DERIVATION.md](LMI_RADIUS_DERIVATION.md) for the equation-by-equation
derivation and validation limits. It recovers the original uncalibrated
SDP epsilon exactly in the global formula and locally reduces the saved
process envelope. The older norm-based prototype is retained below for comparison.
# Dynamic estimation and tracking bounds: first research prototype

This experiment reuses the saved ROHMPC constant metric, feedback gain and
observer gain. It does not redesign the SDP, change upstream submodules,
claim full-domain certification, or simulate a closed-loop quadrotor/MPC.

## Model assumptions

Use the original bias-corrected nominal dynamics f0, constant C,F,L, and
prescribed zero-centered residual disturbance boxes. Scale only columns of
E that inject exclusively into velocity derivatives (channels 7:9 in this
baseline). All other disturbance channels and measurement-noise bounds stay
unchanged. The acceleration scale is

    a(v) = beta + (1-beta)*||v||^2/vmax^2, beta = 0.25.

The baseline componentwise velocity box is [-2,2]^3, hence vmax=sqrt(12).
This describes synthetic bounded uncertainty; the smaller sets have not
been identified or validated by the original UQ data. Constant bias is not
scaled. Each scaled set is contained in the original centered box.

## Derivation

Let e=x-xhat, delta=xhat-z, P=P_delta, and R=chol(P), so R'*R=P.
The control is u=v_nom+K*delta; the observer is
xhat_dot=f0(xhat,u)+L*(y-C*xhat). Then

    e_dot = f0(x,u)-f0(xhat,u)-LC*e + E*w - LF*eta,
    delta_dot = f0(xhat,u)-f0(z,v_nom)+LC*e+LF*eta.

For a constant P, sufficient contraction conditions are

    sym(R*(A-LC)/R) <= -rho_o*I,
    sym(R*(A+B*K)/R) <= -rho_c*I.

Here sym(M)=(M+M')/2. Equivalently, these are the usual Lyapunov matrix
inequalities with +2*rho*P. They must hold along all relevant state/input
segments, not merely at their endpoints. The rectangular design domain is
convex; the actual, estimated and nominal states and relevant inputs must
remain in that domain. The prototype audits these conditions numerically
but has NOT proved them on the continuous domain.

Define constants from box-vertex maximizations:

    b_acc = max ||R*E_acc*w_acc||,
    b_rest = max ||R*E_rest*w_rest||,
    b_noise = max ||R*L*F*eta||,
    gamma = ||R*L*C/R||_2,
    c_v = ||S_v/R||_2.

The vertex maxima are exact for each supplied finite box and fixed matrix;
summing separate maxima uses the triangle inequality and is conservative.
It is NOT the same construction as the original auxiliary SDP matrix W_bar.

If ||e||_P <= r and ||delta||_P <= s, the unknown true speed obeys

    ||v_true|| <= ||v_nominal_state|| + c_v*(r+s).

The additional bound vmax is valid only while the actual state remains in
the stated design domain. Set

    v_upper = min(vmax, ||v_nominal_state|| + c_v*(r+s)),
    a_upper = beta + (1-beta)*(v_upper/vmax)^2.

The proposed comparison dynamics are

    r_dot = -rho_o*r + b_rest + b_acc*a_upper + b_noise,
    s_dot = -rho_c*s + gamma*r + b_noise.

This is a dynamic bound, not an instantaneous assignment r=epsilon(v).
For online observer-only propagation, use ||vhat||+c_v*r; the nominal
prediction version above also includes s because the nominal trajectory is
separated from the observer estimate. Appropriate initial bounds are required.
The right-hand sides have nondecreasing off-diagonal dependence for r,s>=0,
so the coupled comparison construction applies under the stated derivative
bounds. Noise remains constant in this first experiment.

The shared worst-case growth scalar is replaced by a local forcing bound
that retains speed dependence. The constant metric itself is retained.

## Implementation and audits

- run_dynamic_bound_demo.m loads the previous saved baseline without overwriting it.
- Checks the original 1024 operating points, 3456 angle-midpoint/vertex points,
  and 2000 reproducibly sampled interior points with nonzero velocities.
- Chooses rates at 95% of the smallest audited rates (controller rate also
  capped by the baseline rho_c). This margin is NOT an interpolation proof.
- check_error_dynamics.m checks the actual nonlinear error derivatives on
  admissible state/estimate/nominal triples with disturbance/noise vertices.
- Checks the true-speed tube envelope and the global-envelope equilibrium.
- Propagates r,s through a prescribed low/high/low nominal-speed schedule.
  This schedule is only a comparison-ODE illustration; no dynamically
  feasible plant trajectory or MPC feasibility claim is made.
- Saves MAT, CSV, log, and PNG under a new timestamped results directory.

## Run in Windows MATLAB using the WSL filesystem

Add your installed Windows CasADi MATLAB folder to the MATLAB path, then:

    addpath('//wsl.localhost/Ubuntu/home/satyam/ROHMPC-State-Dependent-Uncertainty/research/state_dependent_bounds');
    result = run_dynamic_bound_demo();

The current baseline input is work/offline-sanity-20260910/offline_design.mat.
No FORCESPRO, agiclean, new MOSEK solve, or new UQ computation is required.
The model directory must stay on a case-sensitive filesystem.

## Interpretation and remaining work

Compare the dynamic radius against the global-envelope radius obtained from
THE SAME comparison inequality. Also report the original SDP epsilon, but
do not conflate the two bound constructions or claim that the new bound is
tighter than the original unless the numerical values support that claim.

To call this a continuous-domain certificate, replace the finite-point audit
with a justified domain enclosure (e.g. suitable polytopic/interval bounds)
and verify the contraction inequalities on that enclosure. A state-dependent
metric is not required. Incorporating these tubes into MPC additionally needs
initialization, domain preservation, terminal conditions, and recursive
feasibility analysis. Existing terminal ingredients cannot simply be assumed
valid with the new radii. An external simulator test remains to be done.