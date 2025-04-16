# Solar Radiation Pressure
---

## Nomenclature
---

* $\hat{r}$: source-to-satellite normalized vector in ECI frame,
* $\hat{n}_j$: normal vector of satellite surface $j$ (of the macomodel) in ECI frame,
* $\gamma _s$: specular coefficient,
* $\gamma _d$: diffuse coefficient,
* $\gamma _a$: absorbed coefficient


## CNES Model
---

Reference: Cerri L., Couhert A., Ferrage P., DORIS satellites models implemented
in POE processing, Ed. 1, Rev. 19, https://ids-doris.org/documents/BC/satellites/DORISSatelliteModels.pdf

$$
\vec{a} = \sum_{j=1}^{m} A_j \cdot \langle \hat{r}, \hat{n}_j \rangle 
\cdot \left[ 
  2 \gamma_s \langle \hat{r}, \hat{n}_j \rangle \hat{n}_j + 
  \gamma_d \left( \hat{r} - \frac{2}{3} \hat{n}_j \right) + 
  \gamma_a \hat{r} 
\right]
$$

## Woske et al
---

Reference: Wöske F., Kato T., Rievers B., List M., GRACE accelerometer calibration by high precision non-gravitational force modeling,
Advances in Space Research, Volume 63, Issue 3, 2019, Pages 1318-1335, ISSN 0273-1177, https://doi.org/10.1016/j.asr.2018.10.025.

$$
\vec{F} = \sum_{j=1}^{m} A_j \cdot \langle \hat{r}, \hat{n}_j \rangle 
\cdot \left[
  (\gamma_a + \gamma_d) \hat{r} + 2 \left[ \frac{\gamma_d}{3} + \gamma_s \langle \hat{r}, \hat{n}_j \rangle \right] \hat{n}_j
\right]
$$
