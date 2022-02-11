%EKF_VA   Extended Kalman Filter to estimate velocity and acceleration with
%   a noise model that compensates for defects in low frequency filters.
%
%   [x_est, P_ext] = EKF_VA(z, dt, x_est, P_est) updates the state
%   estimate vector, x_est, and state uncertainty covariance matrix, P_est,
%   based on a simple kinematics model.
%   
%   The first three elements of x_est should be the x-, y-, and 
%   z-components of a velocity vector, and the second three elements should
%   be the x-, y-, and z-components of an acceleration vector.  
%
%   The first element of the measurement vector, z, should be a scalar 
%   speed measurement, and the next three elements should be a vector 
%   acceleration measurement.
%
%   The variable, dt, is the difference in time between this estimate and
%   the previous one.
%   
%   See also EKF_VGA.
function [x_est, P_est] = ekf_va(z, dt, x_est, P_est)

F = state_transition(x_est, dt);

Qa = model_uncertainty(x_est, dt);
Q = F * Qa * F.';
R = measurement_uncertainty(z, dt);

x_prd = state_prediction(x_est, dt);
P_prd = F * P_est * F.' + Q;

y = z - measurement_prediction(x_prd);

H = measurement_observation(x_prd);
S = H * P_prd * H.' + R;
K = P_prd * H.' * (S \ speye(size(S)));

x_est = x_prd + K * y;
Fk = speye(size(P_prd)) - K * H;
P_est = Fk* P_prd * Fk.' + K * R * K.';

    % predicting the current state:
    function fx = state_prediction(x_est, dt)
        
        a = x_est(4:6);
        v = x_est(1:3) + a * dt;
        
        fx = [v; a];
    end

    % predicting the uncertainty of the current state:
    function F = state_transition(x_est, dt)
        F = zeros(length(x_est));
        
        F(1:3, 1:3) = eye(3);
        F(1:3, 4:6) = eye(3) * dt;
        
        F(4:6, 4:6) = eye(3);
    end

    % turning predictions into measurements:
    function hx = measurement_prediction(x_prd)
        v = x_prd(1:3);
        v0 = sqrt(sum(v.^2));
        a = x_prd(4:6);
        
        hx = [v0; a];
    end

    % Determine covariant uncertainty of predictions and measurements:
    function H = measurement_observation(x_prd)
        
        H = zeros(4, 6);
        v = x_prd(1:3);
        v0 = sqrt(sum(v.^2));
        
        if v0 ~= 0
            H(1, 1:3) = [v(1)/v0, v(2)/v0, v(3)/v0];
        else
            H(1, 1:3) = [1, 1, 1];
        end
        
        H(2:4, 4:6) = eye(3);
    end

    % Determine the uncertainty of the state prediction process:
    function Qa = model_uncertainty(x_est, ~)
        Qa = eye(length(x_est));
    end
    
    % Determine the uncertainty of the measurements:
    function R = measurement_uncertainty(z, ~)
        R = eye(length(z));
                
        v0 = z(1);
        if v0 <= 1
            R(2:4, 2:4) = eye(3) * 10000;
        end
    end
end