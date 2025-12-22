# TODO:




___
Mathematical/phisical descriptios

1) Polar to cartesian transformation
...


2) Speckle noise
   
2.1) Signal Representation (Coherent Component)

Acoustic echoes in sonar are fundamentally waves. 

While the input image $I_{in}(x,y)$ represents intensity (energy), the physical interference phenomena responsible for speckle occur in the amplitude domain (pressure).

We model the input image as the coherent component (the determined structure of the seabed/objects):
$$A_{coh}(x,y) = \sqrt{I_{in}(x,y)}$$

This represents the magnitude of the coherent phasor, assuming a zero initial phase for simplicity.

2.2)  Statistical Modeling of Scatterers (Incoherent Component)

Speckle arises from the interference of sub-resolution scatterers (sand, roughness) within a resolution cell. 

According to the Burckhardt model, this is a "random walk" in the complex plane.

Instead of simulating $M$ individual scatterers iteratively, we apply the Central Limit Theorem. 

The sum of $M$ random independent phasors converges to a complex Gaussian distribution.

For a pixel with $M$ scatterers (where $M \sim U(M_{min}, M_{max})$), the noise components $u$ (real) and $v$ (imaginary) are generated as:$$u_{raw}, v_{raw} \sim \mathcal{N}(0, \sigma_{noise}^2 \cdot M)$$Here, $\sigma_{noise}^2 \cdot M$ represents the aggregate variance of the scatterers within that cell.

2.3) Beam Geometry Simulation (Spatial Correlation) 

Real sonar systems do not measure single points; they measure the convolution of the scene with the system's Point Spread Function (PSF).

Range axis ($r$): The pulse is very short, resulting in high resolution (no blurring).

Azimuth axis ($\theta$): The beam width is significant, causing returns to "smear" across the arc.

We simulate this by convolving the raw white noise with an anisotropic Gaussian kernel (the approximated PSF):$$u_{corr} = u_{raw} * h(x,y), \quad v_{corr} = v_{raw} * h(x,y)$$Where the kernel $h(x,y)$ is defined by the beam width $\sigma_{\theta}$:$$h(x,y) = \delta(y) \cdot \frac{1}{\sqrt{2\pi}\sigma_{\theta}} e^{-\frac{x^2}{2\sigma_{\theta}^2}}$$Note: In the code, a normalization factor is applied after filtration to compensate for the peak amplitude loss inherent in discrete convolution

2.4) Image Formation (Interference and Detection) 

The total signal received by the transducer is the vector sum of the coherent structure (signal) and the spatially correlated incoherent scattering (noise):

$$\mathbf{A}_{total} = (A_{coh} + u_{corr}) + i(v_{corr})$$

Finally, the sonar system performs envelope detection to display the image. 
Mathematically, this corresponds to calculating the squared modulus of the complex phasor:

$$I_{out}(x,y) = |\mathbf{A}_{total}|^2 = (A_{coh} + u_{corr})^2 + v_{corr}^2$$



3) Energy loss
