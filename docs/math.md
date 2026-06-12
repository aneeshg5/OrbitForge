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

---

## §3 Filter Jacobians

*(To be filled in during EKF implementation — Phase 1 Step 6)*

---

## §4 UKF Sigma Points

*(To be filled in during UKF implementation — Phase 1 Step 7)*
