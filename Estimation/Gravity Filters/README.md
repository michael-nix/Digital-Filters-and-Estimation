# Further Notes on Anti-Gravity Filtering
Combining DSP and state estimation techniques to remove gravity from accelerometer readings.

For slow moving systems, removing gravity by just straight up filtering out low-frequency components of an accelerometer signal works surprisingly well, but also introduces some defects that—without correction—make acceleration data useless for real-time applications.  Starting out with some basic filters, we then correct for defects using straightforward state estimation via an Extended Kalman Filter.  Examples via MATLAB code included in the appendices. 
