# Formula reference

## Estimated Augmented States

1. Payload Trim
    $$
      \hat{\mathbf{f}}_d = -\left(m_pg\mathbf{1}_3 + \hat{\mathbf{d}}_\perp\right)
    $$

2. Total Trim
    $$
      \hat{\mathbf{f}}_t = -m_bg\mathbf{1}_3 + \hat{\mathbf{f}}_d - \hat{\mathbf{d}}_T
    $$

3. Equilibrium Swing
    $$
      \hat{\mathbf{r}}_d = l\frac{{\hat{\mathbf{f}}_d}_{1:2}}{\left\lVert\hat{\mathbf{f}}_d\right\rVert}
    $$

## Error Quantities

1. Translational Error
    $$
    \mathbf{s}_p = k_p\mathbf{e}_p + \mathbf{e}_v
    $$

2. Swing Error
    $$
    \hat{\boldsymbol{\mu}} = k_L\left(\mathbf{r} - \hat{\mathbf{r}}_d\right)
    $$

3. Cross Feeding
    $$
    \hat{\mathtt{R}} = \mathbf{B}\left(\mathbf{v} + \hat{\boldsymbol{\mu}}\right)
    $$

4. Filtered Cross Feeding
    $$
    \hat{\mathtt{F}} = \frac{k_r}{s + \lambda}\hat{\mathtt{R}}(s) \; \mid \; \dot{\hat{\mathtt{F}}} = -\lambda\hat{\mathtt{F}} + k_r\hat{\mathtt{R}}
    $$

5. Generalized Cross Feeding
    $$
    \hat{\boldsymbol{\zeta}} = k_p\mathbf{e}_p + \hat{\mathtt{F}} - \mathbf{v}_d \;\mid\;\dot{\hat{\boldsymbol{\zeta}}} = k_p\dot{\mathbf{e}}_p - \lambda\hat{\mathtt{F}} + k_r\hat{\mathtt{R}}
    $$

## Disturbance Estimation Law

1. Disturbance on UAV
    $$
        \hat{\mathbf{d}}_b = \kappa\int_0^t\boldsymbol{\mathfrak{B}}\left(m_b\dot{\mathbf{v}}_b - \mathbf{f}_L - m_bg\mathbf{1}_3 - \hat{\mathbf{d}}_b\right)d\tau
    $$

2. Disturbance projected to cable perpendicular
    $$
        \hat{\mathbf{d}}_\perp = \boldsymbol{\ell} - \frac{\hat{\mathbf{d}}_b^\top\boldsymbol{\ell}}{l^2}\boldsymbol{\ell}
    $$

3. Total disturbance
    $$
        \hat{\mathbf{d}}_T = \lambda_T\left(\underbrace{(m_p+m_b)}_{m_{sys}}\mathbf{v}_p + m_b\mathbf{B}\mathbf{v} - \int_0^t\left(\mathbf{f}_L + \hat{\mathbf{d}}_\perp + \hat{\mathbf{d}}_T + \left(m_p+m_b\right)g\mathbf{1}_3\right)d\tau\right)
    $$


## Control Law

1. Sync Force
    $$
        \hat{\mathbf{f}}_0 = -m_b\left(\hat{\boldsymbol{\zeta}} + k_L\mathbf{B}\mathbf{v} + \dot{\mathbf{B}}\hat{\boldsymbol{\mu}}\right)
    $$

2. Swing Compensator
    $$
        \hat{\mathbf{f}}_a = K_0\left(\mathbf{v}_p + \hat{\boldsymbol{\zeta}} + \underbrace{\mathbf{B}\left(\mathbf{v} + \hat{\boldsymbol{\mu}}\right)}_{\mathtt{R}}\right)
    $$

3. Translational Compensator
    $$
        \hat{\mathbf{f}}_b = m_p\left(\hat{\boldsymbol{\zeta}} + k_p\mathbf{s}_p\right)
    $$
