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
\vec{a} = \sum_{j=1}^{m} A_j \cdot <\hat{r}, \hat{n}_j> \left\{ 2 \gamma _s <\hat{r}, \hat{n}_j> \hat{n}_j + \gamma _d \left\( \hat{r} - \frac{2}{3} \hat{n}_j \right) + \gamma _a \hat{r} \right\} 
$$
