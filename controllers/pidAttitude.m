function torque = ...
    pidAttitude(reference, attitude, angularRate, param)
% =========================================================
% CONTROLADOR PID DE ATITUDE
%
% reference:
% [rollRef pitchRef yawRef]
%
% attitude:
% [roll pitch yaw]
%
% angularRate:
% [p q r]
%
% saída:
% torque = [tauRoll tauPitch tauYaw]
% ==========================================================

%% Variável persistente

persistent integral

if isempty(integral)
    integral = zeros(3,1);
end

%% Erro angular

error = reference - attitude;

%% Normalização do erro de yaw

error(3) = atan2( ...
    sin(error(3)), ...
    cos(error(3)));

%% Termo integral

integral = integral + ...
           error * param.dt;

%% Anti-windup

integral = max( ...
    min(integral, param.maxIntegralAttitude), ...
    -param.maxIntegralAttitude);

%% Termo derivativo

derivative = -angularRate;

%% =========================================================
% PID de Roll
% ==========================================================

torqueRoll = ...
    param.KpRoll * error(1) ...
    + param.KiRoll * integral(1) ...
    + param.KdRoll * derivative(1);

%% =========================================================
% PID de Pitch
% ==========================================================

torquePitch = ...
    param.KpPitch * error(2) ...
    + param.KiPitch * integral(2) ...
    + param.KdPitch * derivative(2);

%% =========================================================
% PID de Yaw
% ==========================================================

torqueYaw = ...
    param.KpYaw * error(3) ...
    + param.KiYaw * integral(3) ...
    + param.KdYaw * derivative(3);

%% Vetor de torque

torque = [ ...
    torqueRoll;
    torquePitch;
    torqueYaw];

end