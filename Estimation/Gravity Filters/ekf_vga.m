%EKF_VGA   Extended Kalman Filter to estimate velocity, acceleration and a
%   gravity vector with a noise model that compensates for defects in low 
%   frequency filters that would otherwise occur.
%
%   [x_est, P_ext] = EKF_VA(z, dt, x_est, P_est) updates the state
%   estimate vector, x_est, and state uncertainty covariance matrix, P_est,
%   based on a simple kinematics model.
%   
%   The first three elements of x_est should be the x-, y-, and 
%   z-components of a velocity vector; the second three elements should
%   be the x-, y-, and z-components of an acceleration vector; and, the
%   final three elements should be the x-, y-, and z-components of a
%   gravity vector.
%
%   The first element of the measurement vector, z, should be a scalar 
%   speed measurement, the second should be a scalar magnitude of gravity, 
%   and the final three elements should be a vector acceleration 
%   measurement.
%
%   The variable, dt, is the difference in time between this estimate and
%   the previous one.
%   
%   See also EKF_VGA.
function [x_est, P_est] = ekf_vga(z, dt, x_est, P_est)

F = state_transition(x_est, dt);

x_prd = state_prediction(x_est, dt);
Qa = model_uncertainty(x_prd, dt);
Q = F * Qa * F.';
R = measurement_uncertainty(z, dt);
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
        g = x_est(7:9);
        v = x_est(1:3) + (a - g) * dt;
        
        fx = [v; a; g];
        
    end

    % predicting the uncertainty of the current state:
    function F = state_transition(x_est, dt)
        F = zeros(length(x_est));
        
        F(1:3,1:3) = eye(3);
        F(1:3,4:6) = eye(3) * dt;
        F(1:3,7:9) = -eye(3) * dt;
        
        F(4:6,4:6) = eye(3);
        F(7:9,7:9) = eye(3);
    end

    % turning predictions into measurements:
    function hx = measurement_prediction(x_prd)
        v = x_prd(1:3);
        v0 = sqrt(sum(v.^2));
        
        g = x_prd(7:9);
        g0 = sqrt(sum(g.^2));
        
        a = x_prd(4:6);
        
        hx = [v0; g0; a];
    end

    % Determine covariant uncertainty of predictions and measurements:
    function H = measurement_observation(x_prd)
        
        H = zeros(5, length(x_prd));
        
        v = x_prd(1:3);
        v0 = sqrt(sum(v.^2));
        if v0 ~= 0
            H(1,1:3) = [v(1)/v0, v(2)/v0, v(3)/v0];
        else
            H(1,1:3) = [1, 1, 1];
        end
        
        g = x_prd(7:9);
        g0 = sqrt(sum(g.^2));
        if g0 ~= 0
            H(2,7:9) = [g(1), g(2), g(3)] / g0;
        else
            H(2,7:9) = [1, 1, 1];
        end
        
        H(3:5,4:6) = eye(3);
    end

    % Determine the uncertainty of the state prediction process:
    function Qa = model_uncertainty(x_prd, ~)
        Qa = zeros(length(x_prd));
        Qa(4:6,4:6) = eye(3) * 0.05;
        
        v = sqrt(sum(x_prd(1:3).^2));
        Qa(7:9,7:9) = eye(3) * 1e-6 * (1 + 2500/(1+v^2));
    end
    
    % Determine the uncertainty of the measurements:
    function R = measurement_uncertainty(z, ~)
        R = eye(length(z));
    end
end