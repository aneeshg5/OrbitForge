# OrbitForge — Mathematical Reference

All equations used in the C++ engine, with derivations. Equation numbers are cited in source comments.

---

## §1 Equations of Motion

### §1.1 Two-Body Gravity

$$\mathbf{a}_\text{grav} = -\frac{\mu}{|\mathbf{r}|^3} \mathbf{r}$$

- $\mu = 3.986004418 \times 10^{14}$ m³/s²
- $\mathbf{r}$ in ECI frame (meters)

### §1.2 J2 Oblateness

$$\text{factor} = \frac{3}{2} \frac{J_2 \mu R_E^2}{|\mathbf{r}|^5}$$

$$a_{J_2,x} = \text{factor} \cdot x \left[5\left(\frac{z}{|\mathbf{r}|}\right)^2 - 1\right]$$
$$a_{J_2,y} = \text{factor} \cdot y \left[5\left(\frac{z}{|\mathbf{r}|}\right)^2 - 1\right]$$
$$a_{J_2,z} = \text{factor} \cdot z \left[5\left(\frac{z}{|\mathbf{r}|}\right)^2 - 3\right]$$

- $J_2 = 1.08262668 \times 10^{-3}$, $R_E = 6.3781 \times 10^6$ m

### §1.3 Atmospheric Drag

$$\mathbf{a}_\text{drag} = -\frac{1}{2} \rho(h) C_D \frac{A}{m} |\mathbf{v}_\text{rel}|^2 \hat{\mathbf{v}}_\text{rel}$$

$$\mathbf{v}_\text{rel} = \mathbf{v} - \boldsymbol{\omega}_E \times \mathbf{r}$$

7-band exponential density model (bands defined in `perturbations.cpp`).

### §1.4 Solar Radiation Pressure

$$\mathbf{a}_\text{srp} = -P_{sr} C_R \frac{A}{m} \hat{\mathbf{r}}_\odot$$

- $P_{sr} = 4.56 \times 10^{-6}$ N/m², $C_R = 1.3$

Solar direction uses simplified analytical ephemeris (Vallado §5.1):
$$\lambda = 280.460° + 36000.771° T + (1.915° \sin M + 0.020° \sin 2M)$$

---

## §2 Integrators

### §2.1 RK4 (Fixed Step)

$$k_1 = f(t, \mathbf{x})$$
$$k_2 = f\!\left(t + \tfrac{h}{2},\, \mathbf{x} + \tfrac{h}{2} k_1\right)$$
$$k_3 = f\!\left(t + \tfrac{h}{2},\, \mathbf{x} + \tfrac{h}{2} k_2\right)$$
$$k_4 = f(t + h,\, \mathbf{x} + h\, k_3)$$
$$\mathbf{x}(t+h) = \mathbf{x}(t) + \tfrac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$

### §2.2 RK45 Dormand-Prince (Adaptive Step)

Embedded 4th/5th order pair. 6 function evaluations per step (7 stages, FSAL not used).

**Butcher tableau** (Hairer & Wanner "Solving ODEs I", Table 5.2):

| c     | a                                                                      |
|-------|------------------------------------------------------------------------|
| 0     |                                                                        |
| 1/5   | 1/5                                                                    |
| 3/10  | 3/40          9/40                                                     |
| 4/5   | 44/45         −56/15         32/9                                      |
| 8/9   | 19372/6561    −25360/2187    64448/6561    −212/729                    |
| 1     | 9017/3168     −355/33        46732/5247    49/176     −5103/18656      |
| 1     | 35/384        0              500/1113      125/192    −2187/6784       11/84   |

**5th-order solution:**
$$\mathbf{x}^{(5)} = \mathbf{x} + h\!\left(\tfrac{35}{384}k_1 + \tfrac{500}{1113}k_3 + \tfrac{125}{192}k_4 - \tfrac{2187}{6784}k_5 + \tfrac{11}{84}k_6\right)$$

**Error estimate** $\mathbf{e} = \mathbf{x}^{(5)} - \mathbf{x}^{(4)}$, coefficients $e_i = b_i^{(5)} - b_i^{(4)}$:

$$e_1 = \frac{71}{57600},\quad e_3 = -\frac{71}{16695},\quad e_4 = \frac{71}{1920},\quad e_5 = -\frac{17253}{339200},\quad e_6 = \frac{22}{525},\quad e_7 = -\frac{1}{40}$$

**Step acceptance** (mixed absolute/relative tolerance):
$$\|\mathbf{e}\|_\infty < \tau, \quad \tau = \text{atol} + \text{rtol}\cdot\|\mathbf{x}\|_\infty$$

**Step size control** (PI-free, safety factor 0.9):
$$h_\text{new} = h \cdot \min\!\left(5,\, \max\!\left(0.1,\, 0.9 \cdot \left(\frac{\tau}{\|\mathbf{e}\|_\infty}\right)^{1/5}\right)\right)$$

Default tolerances: $\text{atol} = 10^{-6}$ m, $\text{rtol} = 10^{-9}$.

---

## §3 Filter Jacobians

### §3.1 Two-Body Gravity Jacobian (KF / EKF)

Continuous-time system matrix:

$$F = \begin{bmatrix} 0_3 & I_3 \\ \partial\mathbf{a}/\partial\mathbf{r} & 0_3 \end{bmatrix}$$

Two-body gravity Jacobian (exact):

$$\frac{\partial \mathbf{a}_\text{grav}}{\partial \mathbf{r}} = -\frac{\mu}{|\mathbf{r}|^3}\left(I_3 - 3\hat{\mathbf{r}}\hat{\mathbf{r}}^\top\right)$$

**KF discrete transition** (first-order, causes visible linearisation drift):
$$\Phi \approx I + F \cdot \Delta t$$

**EKF discrete transition** (state propagated nonlinearly via RK4; $\Phi$ used only for $P$ update — see §3.2).

### §3.2 J2 Oblateness Jacobian (EKF)

Define $C = \tfrac{3}{2} J_2 \mu R_E^2$, $\text{factor} = C/|\mathbf{r}|^5$, $A = 5z^2/|\mathbf{r}|^2$.

Starting from $a_{J_2,x} = \text{factor} \cdot x(A-1)$ (and cyclic for $y$; $a_{J_2,z} = \text{factor}\cdot z(A-3)$), apply the product rule with $\partial\text{factor}/\partial x_j = -5\,\text{factor}\,x_j/|\mathbf{r}|^2$ and $\partial A/\partial x_j = (10z/|\mathbf{r}|^2)(\delta_{jz} - z\,x_j/|\mathbf{r}|^2)$:

**Diagonal entries:**

$$\frac{\partial a_{J_2,x}}{\partial x} = \text{factor}\!\left[\frac{5(x^2+z^2)}{|\mathbf{r}|^2} - 1 - \frac{35\,x^2 z^2}{|\mathbf{r}|^4}\right]$$

$$\frac{\partial a_{J_2,y}}{\partial y} = \text{factor}\!\left[\frac{5(y^2+z^2)}{|\mathbf{r}|^2} - 1 - \frac{35\,y^2 z^2}{|\mathbf{r}|^4}\right]$$

$$\frac{\partial a_{J_2,z}}{\partial z} = \text{factor}\!\left[-3 + \frac{30\,z^2}{|\mathbf{r}|^2} - \frac{35\,z^4}{|\mathbf{r}|^4}\right]$$

**Off-diagonal entries** (Jacobian is symmetric — conservative force):

$$\frac{\partial a_{J_2,x}}{\partial y} = \frac{\partial a_{J_2,y}}{\partial x} = \text{factor}\cdot\frac{5xy}{|\mathbf{r}|^2}\!\left(1 - \frac{7z^2}{|\mathbf{r}|^2}\right)$$

$$\frac{\partial a_{J_2,x}}{\partial z} = \frac{\partial a_{J_2,z}}{\partial x} = \text{factor}\cdot\frac{5xz}{|\mathbf{r}|^2}\!\left(3 - \frac{7z^2}{|\mathbf{r}|^2}\right)$$

$$\frac{\partial a_{J_2,y}}{\partial z} = \frac{\partial a_{J_2,z}}{\partial y} = \text{factor}\cdot\frac{5yz}{|\mathbf{r}|^2}\!\left(3 - \frac{7z^2}{|\mathbf{r}|^2}\right)$$

Note: $\text{trace}(\partial\mathbf{a}_{J_2}/\partial\mathbf{r}) = 0$ (same as gravity — Liouville's theorem holds).

---

## §4 UKF — Square-Root Form (SR-UKF)

### §4.1 Parameters

$$\alpha = 10^{-3},\quad \kappa = 0,\quad \beta = 2,\quad n = 6$$

$$\lambda = \alpha^2(n+\kappa) - n \approx -5.999994 \qquad (n+\lambda = \alpha^2 n \approx 6\times10^{-6})$$

$$\gamma = \sqrt{n+\lambda} \approx 2.449\times10^{-3}$$

Weights:
$$W_0^{(m)} = \frac{\lambda}{n+\lambda},\quad W_0^{(c)} = W_0^{(m)} + (1-\alpha^2+\beta)$$
$$W_i^{(m)} = W_i^{(c)} = \frac{1}{2(n+\lambda)}\quad i=1,\ldots,2n$$

With $\alpha=10^{-3}$: $W_0^{(m)}\approx -999999$, $W_i\approx 83333$.  The large negative $W_0^{(c)}$ requires a Cholesky **downdate** (not update) for the $i=0$ sigma-point term.

### §4.2 Sigma Point Generation

$$\chi_0 = \hat{\mathbf{x}},\quad \chi_i = \hat{\mathbf{x}} + \gamma S_{:,i},\quad \chi_{i+n} = \hat{\mathbf{x}} - \gamma S_{:,i},\quad i=1,\ldots,n$$

where $S$ is the lower-triangular Cholesky factor: $P = SS^\top$.

**Symmetry identity** (verified by test): $\sum_{i=0}^{2n} W_i^{(m)} \chi_i = \hat{\mathbf{x}}$ exactly.

### §4.3 Predict Step (SR Form)

1. Generate $\{\chi_i\}$ from $\hat{\mathbf{x}}, S$ as in §4.2.
2. Propagate via RK4: $\chi_i^* = f(\chi_i, \Delta t)$.
3. Predicted mean: $\hat{\mathbf{x}}^- = \sum_i W_i^{(m)} \chi_i^*$.
4. Form matrix $A^\top \in \mathbb{R}^{(2n+n)\times n}$:
   $$A^\top = \begin{bmatrix} \sqrt{W_1^{(c)}}(\chi_1^*-\hat{\mathbf{x}}^-)^\top \\ \vdots \\ \sqrt{W_{2n}^{(c)}}(\chi_{2n}^*-\hat{\mathbf{x}}^-)^\top \\ S_Q \end{bmatrix}$$
   where $S_Q = \operatorname{chol}(Q)$.  Thin QR of $A^\top$: $A^\top = \hat{Q}R$, giving $S^- = R^\top$ (lower-triangular Cholesky of $A A^\top$).
5. Since $W_0^{(c)}<0$: downdate $S^-$ with $\sqrt{|W_0^{(c)}|}(\chi_0^*-\hat{\mathbf{x}}^-)$ via `chol_rank1_update(..., -1)`.

### §4.4 Rank-1 Cholesky Update/Downdate (Givens Rotation)

Given lower-triangular $L$ and vector $\mathbf{v}$, compute $L'$ such that $L'L'^\top = LL^\top + \sigma\mathbf{v}\mathbf{v}^\top$ ($\sigma=\pm1$):

```
for k = 0..n-1:
    r = sqrt(L(k,k)^2 + sigma * v(k)^2)   // requires r^2 > 0 for downdate
    c = L(k,k)/r;   s = v(k)/r             // Givens cosine, sine
    L(k,k) = r
    for i = k+1..n-1:
        L(i,k) <-- c*L(i,k) + sigma*s*v(i)   // using old L(i,k), v(i)
        v(i)   <-- c*v(i)   - s*L(i,k)        // using old L(i,k), v(i)
```

Returns false (downdate failed) if $r^2 \le 0$, i.e., the matrix would become non-positive-definite.

### §4.5 Update Step (SR Form)

For the linear GPS measurement $H = [I_3\;0_3]$, sigma-point sums reduce to standard Kalman equations:

$$P_{xy} = PH^\top,\quad S_\text{innov} = HPH^\top + R,\quad K = PH^\top S_\text{innov}^{-1}$$

SR covariance update via 3 rank-1 Cholesky downdates:

$$P_\text{new} = P - K S_\text{innov} K^\top = SS^\top - UU^\top,\quad U = K\,\operatorname{chol}(S_\text{innov})$$

Apply `chol_rank1_update(S, U[:,j], -1)` for $j=0,1,2$.

---

## §5 Sensor Models

### §5.1 GPS (Position-Only)

$$\mathbf{z}_\text{gps} = R_\text{ECEF/ECI}\,\mathbf{r}_\text{ECI} + \boldsymbol{\eta},\quad \boldsymbol{\eta}\sim\mathcal{N}(\mathbf{0},\,\sigma_\text{gps}^2 I_3)$$

GAST (Greenwich Apparent Sidereal Time):
$$\theta_\text{GAST} = 280.46061837° + 360.98564736629°\,(T_\text{JD} - 2451545.0)$$

$$R_\text{ECEF/ECI} = R_z(\theta_\text{GAST}) = \begin{bmatrix}\cos\theta & \sin\theta & 0 \\ -\sin\theta & \cos\theta & 0 \\ 0 & 0 & 1\end{bmatrix}$$

Default $\sigma_\text{gps} = 10$ m; configurable 1–100 m.

### §5.2 IMU Accelerometer

$$\mathbf{z}_\text{imu}(t) = \mathbf{a}_\text{true}(t) + \mathbf{b}(t) + \boldsymbol{\eta}_\text{acc},\quad \boldsymbol{\eta}_\text{acc}\sim\mathcal{N}(\mathbf{0},\sigma_\text{acc}^2 I_3)$$

Bias random walk with spectral density $\sigma_b$:
$$\mathbf{b}(t+\Delta t) = \mathbf{b}(t) + \boldsymbol{\eta}_b,\quad \boldsymbol{\eta}_b\sim\mathcal{N}(\mathbf{0},\sigma_b^2\,\Delta t\, I_3)$$

Defaults: $\sigma_\text{acc} = 0.05$ m/s², $\sigma_b = 0.001$ m/s²/√s.

### §5.3 Magnetometer (IGRF-13 Dipole Approximation)

Uses Gauss coefficients $\{g_{10}, g_{11}, h_{11}, g_{20}\}$ (IGRF-13, 2020, Alken et al. 2021):

| Coefficient | Value [nT] |
|-------------|-----------|
| $g_{10}$ | −29 404.5 |
| $g_{11}$ | −1 450.7 |
| $h_{11}$ | +4 652.9 |
| $g_{20}$ | −2 499.7 |

**Field in geocentric spherical coords** ($a = R_E/r$):

$$B_r = 2a^3\!\left(g_{10}\cos\theta + D_1\sin\theta\right) + 3a^4 g_{20}\tfrac{3\cos^2\theta-1}{2}$$

$$B_\theta = a^3\!\left(g_{10}\sin\theta - D_1\cos\theta\right) + 3a^4 g_{20}\sin\theta\cos\theta$$

$$B_\lambda = a^3(g_{11}\sin\lambda - h_{11}\cos\lambda)$$

where $D_1 = g_{11}\cos\lambda + h_{11}\sin\lambda$, $\theta$ = geocentric colatitude, $\lambda$ = east longitude.

**Cartesian conversion** (ECEF):
$$\mathbf{B}_\text{ECEF} = B_r\hat{\mathbf{r}} + B_\theta\hat{\boldsymbol{\theta}} + B_\lambda\hat{\boldsymbol{\lambda}}$$

$$\hat{\mathbf{r}} = (\sin\theta\cos\lambda,\;\sin\theta\sin\lambda,\;\cos\theta)^T$$
$$\hat{\boldsymbol{\theta}} = (\cos\theta\cos\lambda,\;\cos\theta\sin\lambda,\;-\sin\theta)^T\quad\text{[toward south]}$$
$$\hat{\boldsymbol{\lambda}} = (-\sin\lambda,\;\cos\lambda,\;0)^T\quad\text{[east]}$$

ECEF → ECI via $R_\text{ECI/ECEF} = R_z(-\theta_\text{GAST})$.  Default $\sigma_\text{mag} = 100$ nT.

## §6 Filter Consistency — NEES Monte Carlo (CLAUDE.md §14)

For a consistent filter, the estimation error $\mathbf{e}_i = \mathbf{x}_{\text{true},i} - \hat{\mathbf{x}}_i$ satisfies $\mathbf{e}_i \sim \mathcal{N}(\mathbf{0}, P_i)$, so the Normalized Estimation Error Squared

$$\text{NEES}_i = \mathbf{e}_i^T P_i^{-1} \mathbf{e}_i \sim \chi^2(n), \quad n = 6$$

Averaged over $N$ independent runs, $N \cdot \overline{\text{NEES}} \sim \chi^2(nN)$. The 95% bounds on $\overline{\text{NEES}}$ are $\chi^2(nN, 0.025)/N$ and $\chi^2(nN, 0.975)/N$, approximated via Wilson–Hilferty:

$$\chi^2(\nu, p) \approx \nu\left(1 - \frac{2}{9\nu} + z_p\sqrt{\frac{2}{9\nu}}\right)^3$$

For $N=100$, $n=6$ ($\nu = 600$): bounds $\approx [5.35,\, 6.69]$.

**Process noise must be injected into the true trajectory to match filter $Q$.** If the true trajectory is purely deterministic (e.g. two-body RK4 with no random forcing) but the filter's $Q > 0$, the filter is provably underconfident: $P$ converges to a value set by $Q$ while the actual error converges toward zero (deterministic dynamics + the same integrator the filter assumes), so $\text{NEES} \to 0 \ll 6$. Conversely $Q=0$ makes $P \to 0$ as GPS updates accumulate while the actual error remains bounded by GPS noise statistics, so $\text{NEES} \to \infty$. The test (`test_filter_consistency.cpp`) resolves this by drawing $\mathbf{w} \sim \mathcal{N}(0, Q)$ each step and adding it directly to the true trajectory — making the stochastic model the filter assumes exactly the one realized, which is the textbook construction for an NEES Monte Carlo test on an otherwise-deterministic system.

Test parameters: $N=100$ runs, 500 steps, $dt=10$ s, ISS circular orbit (two-body only), $\sigma_\text{gps}=10$ m, $\sigma_{Q,\text{pos}}=1$ m/step, $\sigma_{Q,\text{vel}}=0.01$ m/s/step, $P_0 = \text{diag}(100^2,100^2,100^2,1,1,1)$. Initial filter state error drawn from $P_0$. Pass criterion: $\geq 90\%$ of the 500 per-step averaged NEES values fall within $[5.35, 6.69]$.

---

## §7 Phase 5 — 6DOF Attitude Estimation

KF is unaffected by this section (CLAUDE.md §6.1) — stays the 6-state $[\mathbf{r},\mathbf{v}]$ filter from §3 forever. Everything below applies only to EKF/UKF's new 12-state multiplicative-EKF (MEKF) attitude block, stacked with the *unchanged* §3.1 orbital block.

### §7.1 Attitude Convention and Quaternion Kinematics

$q$ is the unit quaternion rotating BODY-frame vectors into ECI: for body components $\mathbf{v}_b$, $\mathbf{v}_i = q\,\mathbf{v}_b\,q^{-1}$ (Hamilton product). Composition: $R(q_1 q_2) = R(q_1)R(q_2)$ (same order as the quaternion product); the ECI→body DCM is $A(q) = R(q)^\top$, which therefore composes in **reversed** order, $A(q_1 q_2) = A(q_2)A(q_1)$ — this is what fixes the MEKF reset-step composition order in §7.3.

Quaternion kinematics:
$$\dot{q} = \tfrac{1}{2}\, q \otimes [0,\boldsymbol{\omega}]$$
where $\boldsymbol{\omega}$ is body-frame angular velocity and $[0,\boldsymbol{\omega}]$ is the pure-vector quaternion. Implemented in `math/quaternion.cpp::quat_kinematics_coeffs_dot` operating on `Eigen::Quaterniond::coeffs()` (note: **coeffs() order is $(x,y,z,w)$, constructor order is $(w,x,y,z)$** — an easy-to-invert Eigen footgun, flagged explicitly in the header).

### §7.2 Euler's Equation (Torque-Free Rigid Body) and Its Jacobian

Diagonal principal-axis inertia tensor $I = \text{diag}(I_x,I_y,I_z)$, no external torque (§18 — no actuators/gravity-gradient):
$$\dot{\boldsymbol{\omega}} = I^{-1}\left(-\boldsymbol{\omega}\times(I\boldsymbol{\omega})\right)$$

**Verification against the textbook component form** (sanity check before trusting the vector form): expanding $-\boldsymbol{\omega}\times(I\boldsymbol{\omega})$ component-wise reproduces the classical Euler's equations exactly, e.g. $I_y\dot\omega_y = (I_z-I_x)\omega_z\omega_x$ — confirms no sign error in the cross-product form used in code (`rigid_body.cpp` uses `omega.cross(i_omega)` directly rather than expanding components by hand, precisely to avoid this class of sign slip).

**Jacobian** $\partial\dot{\boldsymbol{\omega}}/\partial\boldsymbol{\omega}$, needed for the EKF $F$ matrix (§7.3). Let $g(\boldsymbol{\omega}) = \boldsymbol{\omega}\times(I\boldsymbol{\omega})$. Differentiating with $I$ constant:
$$dg = d\boldsymbol{\omega}\times(I\boldsymbol{\omega}) + \boldsymbol{\omega}\times(I\,d\boldsymbol{\omega}) = \big(-[(I\boldsymbol{\omega})\times] + [\boldsymbol{\omega}\times]I\big)\,d\boldsymbol{\omega}$$
using $\mathbf{a}\times\mathbf{b} = -[\mathbf{b}\times]\mathbf{a}$ for the first term. So:
$$\frac{\partial g}{\partial\boldsymbol{\omega}} = [\boldsymbol{\omega}\times]I - [(I\boldsymbol{\omega})\times], \qquad \frac{\partial\dot{\boldsymbol{\omega}}}{\partial\boldsymbol{\omega}} = -I^{-1}\left([\boldsymbol{\omega}\times]I - [(I\boldsymbol{\omega})\times]\right)$$
**Validated in code, not just on paper**: `test_rigid_body.cpp`'s `EulerJacobianMatchesFiniteDifference` cross-checks this analytical form against central finite differences (passes to $10^{-6}$) — the same discipline as Phase 1's J2 Jacobian, satisfying CLAUDE.md §22 rule 6 ("don't guess at Jacobian terms").

Note $\dot{\boldsymbol{\omega}}$ has **no dependence on attitude** — torque-free rotation is attitude-independent, so the Jacobian's $\partial\dot{\boldsymbol{\omega}}/\partial(\delta\theta)$ block is exactly zero (§7.3's $F$ matrix).

### §7.3 MEKF Error Kinematics, $F$ Matrix, and Reset Step

**Error definition** (body-frame multiplicative error, Lefferts–Markley–Shuster convention): let $A_\text{true}, A_\text{ref}$ be the true/reference ECI→body DCMs. Define $\delta A = A_\text{true}A_\text{ref}^\top$ (small misalignment, ref-body→true-body). Equivalently, in quaternion form (using §7.1's reversed composition rule):
$$q_\text{true} = q_\text{ref}\otimes\delta q$$

**Linearized error kinematics** — derived here directly from the unambiguous DCM kinematic law $\dot{A} = -[\boldsymbol{\omega}\times]A$ (rather than recalled from memory, since this filter's setup — $\boldsymbol{\omega}$ as a filter state via Euler's equation, not a directly-measured/bias-only quantity — is non-standard relative to most textbook MEKF derivations and deserves a from-scratch check):

$$\dot{\delta A} = \dot{A}_\text{true}A_\text{ref}^\top + A_\text{true}\dot{A}_\text{ref}^\top = -[\boldsymbol{\omega}_\text{true}\times]\,\delta A + \delta A\,[\boldsymbol{\omega}_\text{ref}\times]$$

Substituting the small-angle approximation $\delta A \approx I_3 - [\delta\boldsymbol{\theta}\times]$ and keeping first-order terms only (using the skew-commutator identity $[\mathbf{a}\times][\mathbf{b}\times]-[\mathbf{b}\times][\mathbf{a}\times] = [(\mathbf{a}\times\mathbf{b})\times]$, evaluated at $\boldsymbol{\omega}_\text{true}\approx\boldsymbol{\omega}_\text{ref}\approx\hat{\boldsymbol{\omega}}$ since the difference is already first-order):

$$\boxed{\dot{\delta\boldsymbol{\theta}} = -\hat{\boldsymbol{\omega}}\times\delta\boldsymbol{\theta} + \delta\boldsymbol{\omega}}, \qquad \delta\boldsymbol{\omega} := \boldsymbol{\omega}_\text{true}-\hat{\boldsymbol{\omega}}\ \text{(the }\boldsymbol{\omega}\text{-state's own estimation error)}$$

This is structurally identical to the orbital block's $\dot{\mathbf{e}_r} = \mathbf{e}_v$ (position-error rate equals velocity error) — attitude-error rate depends on both the current attitude error *and* the angular-velocity estimation error, not on $\delta\boldsymbol{\theta}$ alone.

**Full 12×12 continuous Jacobian** (block-diagonal between attitude and orbital blocks — no coupling terms, §18):
$$F = \begin{bmatrix} -[\hat{\boldsymbol{\omega}}\times] & I_3 & 0_3 & 0_3 \\ 0_3 & -I^{-1}([\hat{\boldsymbol{\omega}}\times]I-[(I\hat{\boldsymbol{\omega}})\times]) & 0_3 & 0_3 \\ 0_3 & 0_3 & 0_3 & I_3 \\ 0_3 & 0_3 & \partial\mathbf{a}/\partial\mathbf{r} & 0_3 \end{bmatrix}$$
(state order $[\delta\boldsymbol{\theta},\boldsymbol{\omega},\mathbf{r},\mathbf{v}]$; the bottom-right 6×6 is exactly §3.1's unchanged orbital block.)

**Reset step**, applied after every measurement update:
$$q_\text{ref} \leftarrow \left(q_\text{ref}\otimes\delta q(\hat{\delta\boldsymbol{\theta}})\right)\!/\!\left|q_\text{ref}\otimes\delta q(\hat{\delta\boldsymbol{\theta}})\right|, \qquad \hat{\delta\boldsymbol{\theta}}\leftarrow \mathbf{0}$$
**Right**-multiplication — directly forced by $q_\text{true}=q_\text{ref}\otimes\delta q$ above (substituting the posterior $\hat{\delta\boldsymbol{\theta}}$ for the true-but-unknown $\delta\boldsymbol{\theta}$ folds the estimated correction into $q_\text{ref}$ exactly). An earlier draft of this section had this backwards (left-multiplication) before this derivation was carried out carefully — corrected in CLAUDE.md to match.

$\delta q(\delta\boldsymbol{\theta})$ is the quaternion exponential map (`math/quaternion.cpp::quat_exp`): exact axis-angle form $[\cos(|\delta\boldsymbol{\theta}|/2),\ \sin(|\delta\boldsymbol{\theta}|/2)\,\hat{\delta\boldsymbol{\theta}}]$, with a first-order Taylor fallback near $\delta\boldsymbol{\theta}=\mathbf{0}$ to avoid a 0/0 in the axis normalization.

**On verifying this against the literature — a real convention trap, documented honestly.** The standard references for this exact algorithm — Lefferts, Markley & Shuster (1982), *"Kalman Filtering for Spacecraft Attitude Estimation,"* JGCD 5(5); and Markley (2003), *"Attitude Error Representations for Kalman Filtering,"* JGCD 26(2) — were located and read (NASA NTRS) specifically to cross-check this section line-by-line. That check surfaced something worth recording rather than glossing over: **the spacecraft-attitude literature's quaternion product convention (often called the Shuster/JPL convention) negates the cross-product term relative to the Hamilton convention this codebase's quaternion library (Eigen) uses** — Markley's Eq. (6) is $p\otimes q = [p_4\mathbf{q}+q_4\mathbf{p}-\mathbf{p}\times\mathbf{q};\ p_4q_4-\mathbf{p}\cdot\mathbf{q}]$, with a **minus** sign on the cross term where Hamilton's product has a plus. Hand-translating his equations into Eigen's convention to check them term-by-term turned out to be genuinely error-prone — two independent translation attempts during this work produced *different* answers for the reset composition direction, which is exactly the failure mode this cross-check was trying to catch.

Rather than trust further hand algebra, this was resolved with two convention-independent checks that don't require translating between notations at all:
1. **The kinematics formula** ($\dot q=\tfrac12 q\otimes[0,\boldsymbol\omega]$) is checked against an unambiguous physical fact in `test_quaternion.cpp` (`KinematicsIntegrationProducesExpectedRotation`): integrating it for constant $\boldsymbol\omega=(0,0,\omega_z)$ must reproduce a $z$-axis rotation of angle $\omega_z T$ — true regardless of which paper's convention anyone uses.
2. **The reset step's composition direction** is checked geometrically in `test_ekf.cpp`/`test_ukf.cpp` (`ResetComposesCorrectionInBodyFrameRightMultiply`): $\delta\boldsymbol\theta$ is, by construction, a correction expressed in the *current body frame* (it's exactly what `H_gyro`/`H_mag` map state perturbations to). Composing "apply the existing attitude, then a further small rotation expressed in the frame that attitude produces" is the standard intrinsic-rotation rule $R_\text{total}=R_\text{existing}\cdot R_\text{correction}$ — independent of any paper's notation, relying only on Eigen's own documented property that its rotation matrices compose in the same order as its quaternion product. Both tests pass, with the expected value in test 2 built independently via `Eigen::AngleAxisd` directly, not via this codebase's own `quat_exp`/composition code.

This is solid evidence the implementation is internally and geometrically correct. It is *not* the same as a verified line-by-line match against Lefferts-Markley-Shuster — that comparison is what triggered this whole investigation, and was abandoned as unreliable once it started producing self-contradictory results, rather than forced to a possibly-wrong conclusion.

### §7.4 Gyro and Magnetometer Measurement Jacobians

**Gyro** — direct measurement of the $\boldsymbol{\omega}$ state (sensor-side bias only, no filter bias state — CLAUDE.md §6.3, mirrors the existing IMU pattern):
$$H_\text{gyro} = \begin{bmatrix}0_3 & I_3 & 0_3 & 0_3\end{bmatrix} \quad (3\times12)$$

**Magnetometer** — predicted measurement $\hat{\mathbf{z}}_\text{mag} = A(q_\text{ref})\,\mathbf{B}_\text{ECI}(\hat{\mathbf{r}})$ (existing IGRF dipole model, §5.3, now finally consumed). Linearizing the rotation of a fixed vector by a perturbed quaternion ($A(q_\text{ref}\otimes\delta q)\mathbf{B} \approx (I_3-[\delta\boldsymbol{\theta}\times])A(q_\text{ref})\mathbf{B} = \hat{\mathbf{z}}_\text{mag} - \delta\boldsymbol{\theta}\times\hat{\mathbf{z}}_\text{mag} = \hat{\mathbf{z}}_\text{mag} + [\hat{\mathbf{z}}_\text{mag}\times]\delta\boldsymbol{\theta}$):
$$H_\text{mag} = \begin{bmatrix}[\hat{\mathbf{z}}_\text{mag}\times] & 0_3 & 0_3 & 0_3\end{bmatrix} \quad (3\times12)$$
$\partial\mathbf{B}_\text{ECI}/\partial\mathbf{r}$ is deliberately **not** included (treated as zero) — GPS already observes $\mathbf{r}$ directly and the true coupling is second-order; this is what keeps $F$ exactly block-diagonal between the attitude and orbital blocks (§18 simplification).
