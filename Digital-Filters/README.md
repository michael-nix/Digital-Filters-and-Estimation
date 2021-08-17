# Yeah, But Which Way is Down?
Practical digital filters for eliminating gravity from accelerometer readings.

Drastically slowing things down, you can still get half-decent results when designing simple digital filters with limited requirements if your goals aren’t that ambitious.  Like, trying to figure out where the middle of the Earth is if you’re standing still on top of it.  We analyze a few of the common approaches for this task found around the internet, and then finding them insufficient, figure out a few things that need to be taken into account to actually get pretty good results.

# Near Real-Time EMD Filter Without Group Delay
A janky low-pass filter for when your signal is very low frequency.

Butterworth filters are great, except that there will always be group delay, and near the cutoff frequency there can be tons of group delay, shifting your signal in time such that it's no longer useful (i.e. as input to an estimation scheme).  Instead of using a Fourier Transform, Empirical Mode Decomposition is another way of breaking up a signal into its constituent parts, but instead of sines or cosines, EMD gives you Intrinsic Mode Functions (IMFs).  By looking at the instantaneous frequency of each IMF, it's possible to get rid of the ones that are obviously noise, reconstructing a good approximation of your noiseless signal.  While more computationally intensive than standard DSP techniques, you no longer have to worry about group delay, exponential decay due to time constants, or other filter defects.
