# Estimating Centripetal Acceleration
An Extended Kalman Filter to estimate centripetal acceleration from GNSS and IMU readings

**Michael Nix**, Allegory Technology, Toronto, Canada, 2021

---

The first real telematics solutions for use in auto insurance came as devices that were physically attached to cars, using their own onboard sensors, or plugging into the car to get its data--primarily speed with some measure of turn angle.  This allowed for some very basic metrics that could be used to categorize driver behaviour in order to give drivers discounts on their insurance premiums.  The metrics calculated were not very good, cars with GPS didn't fare much better, and these first attempts at behaviour based insurance largely failed.

When smartphones then brought more powerful sensors and computing power into everyone's pocket (fast CPUs, GPS, and 6- or 9-axis inertial measurement units), it was thought that, finally, behaviour based insurance was possible... as long as the driver had their phone on them while driving.  The industry then began adapting state estimation techniques developed for planes, missiles, and then drones for use in analyzing driver behaviour.  

The problem with planes is that they fly far above the ground.  Cars drive along the ground, and the ground is where all the electromagnetic interference is.  That is: magnetometers won't work if there are power lines nearby (especially if you're inside a metal box); GPS won't work if there's a building in the way or you're underground; estimation techniques fail catastrophically if you hit a pot hole or you drop your phone... So the focus shifted to using raw accelerometer data if you could guarantee that the phone was physically attached to the car in a known good orientation.  With that, you could just throw AI at raw accelerometer data, and fuse it with GPS data to... identify intersections where people stop too fast.

Other than a few press releases, I haven't seen much happen with that technology.

And yes, there are successful auto insurance telematics companies, and they work with their customers to hopefully improve the profitability of their portfolios (where regulation permits it).  But there's still a lot of mystery in what's truly being measured, and then how it's used to evaluate driver behaviour; particularly when classifying drivers into risk deciles.

The real trouble with telematics for auto insurance is that GPS can really get you most of the way there.  *If* you have good signal throughout your trip, you can get the speed, heading, latitude, and longitude data; match it to a half decent map; and, you're off to the races.  This will get you some decent metrics like:

 - General trip metadata (duration, distance, basic stats),
 - Distance traveled in known high risk areas,
 - Road segments spent speeding,
 - Average acceleration in one second intervals,
 - Periods of high linear acceleration or deceleration,
 - Periods of high lateral acceleration.

And a lot of this data can be directly correlated with accident risk.  People that decelerate, on average, significantly faster than everyone else are much more prone to accidents, for instance.  Add in some basic time-series analysis to raw accelerometer or gyroscope data--or with built-in phone features--and you can even estimate when drivers are distracted by looking at their phone.  The problem with this GPS-type approach is that there is no real failsafe in times where GPS is unreliable (and accident risk is high; read: parking garages), nor is there a way to correct for its shortcomings such as long sampling period (i.e. 1 Hz).  

A long sampling period generally leads to false positives in lateral acceleration events when turning corners--missing aggressive lane changes entirely--or false negatives in linear acceleration events when not coming to a complete stop, or when accelerating from zero kilometers per hour.  Since these play a big part in measuring the risk of an individual driver's behaviour, improving the resolution of their measurements can make telematics for auto insurance truly useful.  If we can make it work on a smartphone in an app that's easy to use, then it will actually get used by drivers.  This is the holy grail of auto insurance telematics.

From here on out we will use the terms centripetal acceleration, radial acceleration, and lateral acceleration to all mean the same thing.  It is close enough.

When reading through the literature, there has been considerable effort in mapping the pose of the car of interest to a known good coordinate system such as the North-East-Down (NED) reference frame.  When using a phone to estimate the pose of the car, there have been many different ways of trying to determine the orientation of the phone relative to the car, as well as the position of the phone within the car.  Usually this means taking the phone's GPS speed and heading (relative to magnetic north pole), and somehow mapping it to its accelerometer and gyroscope readings.  Because of the use of the NED reference frame, the problem tends to be simplified by estimating 2D acceleration, speed, and a heading angle; assuming a good vector for down can be estimated and removed from the sensor readings.  These various approaches don't quite get us what we need; however, we still need to translate these estimates to estimates of centripetal acceleration as that's what all the studies on dangerous driving behaviour use in order to classify dangerous turns or lane changes.  So what we really need is a way to separate lateral and linear acceleration from smartphone sensors, while correcting for the fact that the smartphone might not be secure within the car.  To do this, we can use the following assumptions:

1. Cars and phones are different things, and behave in different ways,
2. Phones can be contained completely within a car,
3. A phone's sensor's measurements can be fused to estimate the state of the car,
4. All measured rotations are extrinsic, and short enough that they're circular,
5. Movement of the phone within the car can be treated as another defect or source of noise.

Now, because of how tires work, when a car is turning (but not slipping, sliding, or drifting), it is always traveling tangent to some curve, which we assume to be a circle.  However, the total acceleration vector does not have to be tangential to a curve, as it may contain linear acceleration (e.g. when starting from a stop; getting up to speed), as well as centripetal acceleration (turning a corner; changing a lane); the common scenario for this being coming to a stop before turning, e.g. at a stop sign or red light. By taking advantage of the physical characteristics of the car (i.e. acknowledging its tires) as well as the distinct characteristics of a mobile phone’s sensors (GPS is fundamentally different from an accelerometer), it should be possible to use basic kinematics and notions of three-dimensional angular velocity to estimate both linear and centripetal acceleration.

A simple map of this could look something like:

<p align="center"><img src="./figures/car motion.png" width="50%"></p>

Where we want to estimate the car's 3D velocity vector, $\mathbf{v}$, it's lateral acceleration, $\mathbf{a}_{\mathrm{lat}}$.  To do that, we'll also need to estimate the phone's raw acceleration (assuming gravity is removed), and its raw angular velocity (assuming bias is removed).  The measurements we'll have access to will be the same, but also include the speed reading from the GPS itself.  It's possible to gain some additional insight if also using the heading reading from the GPS, but that adds too much complexity for too little value if all we want to do is use centripetal acceleration estimates to do an offline classification of driver behaviour.

<p align="center"><img src="./figures/Circular_motion_vectors.svg" alt="By Jmarini - Own work, CC BY 3.0, https://commons.wikimedia.org/w/index.php?curid=5827902" width="50%"></p>

...

```math
\mathbf{v}_{\mathrm{car}} = \mathbf{v}'_{\mathrm{car}} + \Delta t \, \mathbf{a}'_{\mathrm{phone}}
```

```math
\mathbf{\omega}_{\mathrm{car}} = \mathbf{\omega}'_{\mathrm{car}}
```

```math
\mathbf{a}_{\mathrm{lat}} = \mathbf{\omega}' \times \mathbf{v}_{\mathrm{car}}
```

```math
\mathbf{a}_{\mathrm{phone}} = \mathbf{a}'_{\mathrm{phone}}
```
Where a prime indicates an estimate from a previous time step, and all other quantities are for this current time step.  We can abstract these relationships into a linear operator by concatenating all of these equations, and then finding the Jacobian such that:

```math
\begin{bmatrix} \mathbf{v}_{\mathrm{car}} \\
\mathbf{\omega}_{\mathrm{car}} \\
\mathbf{a}_{\mathrm{lat}} \\
\mathbf{a}_{\mathrm{phone}} 
\end{bmatrix} = 
\begin{bmatrix} \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 \\
\mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\
\mathbf{W} & \mathbf{V} & \mathbf{0}_3 & \mathbf{W} \Delta \, t \\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 
\end{bmatrix} 
\begin{bmatrix} \mathbf{v}'_{\mathrm{car}} \\
\mathbf{\omega}'_{\mathrm{car}} \\
\mathbf{a}'_{\mathrm{lat}} \\
\mathbf{a}'_{\mathrm{phone}} 
\end{bmatrix} 
```

Where $\mathbf{I}$ is the identity matrix, $\mathbf{0}$ is a matrix of zeros, $\mathbf{W}$ is a skew-symmetric matrix representing the rate of change of the lateral acceleration with respect to vector velocity: 

$$ 
\mathbf{W} = \frac{d}{d\mathbf{v}}(\mathbf{\omega} \times \mathbf{v}) = 
\begin{bmatrix} 0 & -\omega_z & \omega_y \\
\omega_z & 0 & -\omega_x \\
-\omega_y & \omega_x & 0 
\end{bmatrix} 
 $$

And $\mathbf{V}$ is a skew-symmetric matrix representing the rate of change of lateral acceleration with respect to angular velocity:

$$ 
\mathbf{V} = \frac{d}{d\mathbf{\omega}}(\mathbf{\omega} \times \mathbf{v}) = 
\begin{bmatrix} 0 & v_z & -v_y \\
-v_z & 0 & v_x \\
v_y & -v_x & 0 
\end{bmatrix} 
$$

In the parlance of Kalman filters, we can abstract this such that our state transition matrix, **F**, is simply: 

$$ 
\mathbf{F} = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 \\
\mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\
\mathbf{W} & \mathbf{V} & \mathbf{0}_3 & \mathbf{W} \Delta \, t \\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 \end{bmatrix} 
$$

We also need a way to map our predictions to measurements by using them to predict what the measurements might be. Since we have a scalar speed from our GPS, vector angular velocity from our gyroscope, and vector acceleration from accelerometer. This means that our measurement predictions can be:

```math
v_{\mathrm{GPS}} = (\mathbf{v}^\mathrm{T}_{\mathrm{car}}\mathbf{v}_{\mathrm{car}})^{\frac{1}{2}} \\

\mathbf{\omega}_{\mathrm{gyro}} = \mathbf{\omega}_{\mathrm{car}} \\

\mathbf{a}_{\mathrm{accel}} = \mathbf{a}_{\mathrm{phone}}
```

Where again, we combine these relationships into a vector, then find its Jacobian to linearize them so that measurement predictions can be related to estimate predictions by:

```math
\begin{bmatrix}
v_\mathrm{GPS} \\
\mathbf{\omega}_\mathrm{gyro} \\
\mathbf{a}_\mathrm{accel}
\end{bmatrix} = 
\begin{bmatrix}
|\partial\mathbf{v}| & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} \\
\mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3
\end{bmatrix}
\begin{bmatrix}
\mathbf{v}_\mathrm{car} \\
\mathbf{\omega}_\mathrm{car} \\
\mathbf{a}_\mathrm{lat} \\
\mathbf{a}_\mathrm{phone}
\end{bmatrix}
```

Where the Jacobian of a speed scalar with respect to its underlying velocity vector is:

```math
|\partial\mathbf{v}| = \frac{d}{d\mathbf{v}}(\mathbf{v}^\mathrm{T}\mathbf{v})^\frac{1}{2}
= (\mathbf{v}^\mathrm{T}\mathbf{v})^{-\frac{1}{2}}
\begin{bmatrix}
v_x & v_y & v_z
\end{bmatrix}
```

We can then abstract this back to get our observation matrix, H:

```math
\mathbf{H} = 
\begin{bmatrix}
|\partial\mathbf{v}| & \mathbf{0}_{1\times 3} & \mathbf{0}_{1\times 3} & \mathbf{0}_{1\times 3} \\
\mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3
\end{bmatrix}
```

Since we’re using a constant acceleration and constant angular velocity model, that makes our model uncertainty matrix:

```math
\mathbf{Q}_a = 
\begin{bmatrix}
\mathbf{0}_3 & \cdots & \cdots & \mathbf{0}_3 \\
\vdots & \sigma^2_\omega & \ddots & \vdots \\
\vdots & \ddots & \mathbf{0}_3 & \mathbf{0}_3 \\
\mathbf{0}_3 & \cdots & \mathbf{0}_3 & \sigma^2_a
\end{bmatrix}
```

Which transforms into a process uncertainty matrix, $\mathbf{Q}$:

```math
\mathbf{Q} = \mathbf{FQ}_a\mathbf{F}^\mathrm{T}
```

Finally, we assume that because our three sensors—GPS, gyroscope, accelerometer—are all separate devices, even though they’re combined in one smartphone, have no overlapping uncertainties in their larger covariance matrix, $\mathbf{R}$:

```math
\mathbf{R} = 
\begin{bmatrix}
\sigma^2_\mathrm{GPS} & \mathbf{0}_{1\times 3} & \mathbf{0}_{1\times 3} \\
\mathbf{0}_{3\times 1} & \sigma^2_\mathrm{gyro}\mathbf{I}_3 & \mathbf{0}_{3} \\
\mathbf{0}_{3\times 1} & \mathbf{0}_3 & \sigma^2_\mathrm{accel}\mathbf{I}_3
\end{bmatrix}
```

Where we also assume no covariance in measurement within each sensor, as both gyroscopes and accelerometers collect measurements along three orthogonal axes. Even if there is some covariance between sensors or between sensor axes, that will just eventually increase the uncertainty in estimates via covariances that will most likely be quite small.

From here, we can just use the above matrices to march step-by-step through a Kalman filter as measurements come through with the standard formulas:

```math
\mathbf{x}_p = \mathbf{Fx}' \\
\mathbf{P}_p = \mathbf{FP}'\mathbf{F}^\mathrm{T} + \mathbf{Q}
```

Where $\mathbf{x}_p$ is a prediction of the current state (in this case, our car), $\mathbf{P}_p$ a prediction of the uncertainties in that state. Combined with the observation matrix, and noise matrices:

```math
\mathbf{K} = \mathbf{P}_p\mathbf{H}^\mathrm{T}(\mathbf{HP}_p\mathbf{H}^\mathrm{T} + \mathbf{R})^{-1}
```

$\mathbf{K}$ is our Kalman gain, and:

```math
\begin{aligned}
\mathbf{x}_e &= \mathbf{x}_p + \mathbf{K}(\mathbf{z} - \mathbf{Hx}_p) \\
&= (\mathbf{I} - \mathbf{KH})\mathbf{x}_p + \mathbf{Kz} \\
\end{aligned} \\
\\
\mathbf{P}_e = (\mathbf{I-KH})\mathbf{P}_p(\mathbf{I-KH})^\mathrm{T} + \mathbf{KRK}^\mathrm{T}
```

Gives us $\mathbf{x}_e$ as our corrected estimate of the car’s state in this time step, and $\mathbf{P}_e$ the covariance matrix for its uncertainties. The only thing outstanding to make this possible is to figure out what the uncertainties / covariance for our process and measurements are. And in order to get a handle of that, we’ll have to go through a simplified analysis to better understand how various predictions and measurements are combined to create estimates.

## Simplified Analysis

In order to design process or measurement uncertainty models, we first need to understand how, based on our state and measurement prediction models fuse to inform our estimates. Since we’re effectively collecting seven measurements (GPS speed, angular velocity vector, acceleration vector), a full analysis will require us to invert a 7x7 matrix which is untenable by hand. However, if we simplify things, reducing vectors to scalars where appropriate, we’ll only need to invert a 3x3 matrix, which is relatively straightforward to do by hand.

We start with our simplified state transition model, $\mathbf{F}$:

