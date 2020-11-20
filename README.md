# Yeah, But Which Way is Down?
Practical digital filters for eliminating gravity from accelerometer readings.

Drastically slowing things down, you can still get half-decent results when designing simple digital filters with limited requirements if your goals aren’t that ambitious.  Like, trying to figure out where the middle of the Earth is if you’re standing still on top of it.  We analyze a few of the common approaches for this task found around the internet, and then finding them insufficient, figure out a few things that need to be taken into account to actually get pretty good results.

 - Notch and Butterworth filters,
 - And then some Kalman filter for optimal estimation,
 - And then some Extended Kalman fitlers for nonlinear stuff,
 - Maybe particle filters because they're cool; I wonder if one can make them faster.
