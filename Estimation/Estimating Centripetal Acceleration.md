<div style="font-size:xx-large">Estimating Centripetal Acceleration</div>
<sub>An Extended Kalman Filter to estimate centripetal acceleration from GNSS and IMU readings
<br>Michael Nix, Allegory Technology, Toronto, Canada, 2021</sub>

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

Now, because of how tires work, when a car is turning (but not slipping, sliding, or drifting), it is always travelling tangent to some curve, which we assume to be a circle.  

<img src="./figures/Circular_motion_vectors.svg" alt="By Jmarini - Own work, CC BY 3.0, https://commons.wikimedia.org/w/index.php?curid=5827902" class="center">


<img src="./figures/car motion.png" class="center">

<style>
    .center {
        display: block;
        margin-left: auto;
        margin-right: auto;
        width: 50%;
    }
</style>
