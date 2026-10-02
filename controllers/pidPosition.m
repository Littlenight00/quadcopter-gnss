function [rollRef, pitchRef, thrust] = ...
    pidPosition(reference, position, velocity, param)
% =========================================================
% CONTROLADOR PID DE POSIÇÃO
%
% Entrada:
% reference -> posição desejada [x y z]
% position  -> posição atual [x y z]
% velocity  -> velocidade atual [vx vy vz]
%
% Saída:
% rollRef
% pitchRef
% thrust
% ==========================================================

%% Variáveis persistentes

persistent integralError

if isempty(integralError)
    integralError = zeros(3,1);
end

%% Erro de posição

error = reference - position;

%% Termo integral

integralError = integralError + ...
                error * param.dt;

%% Anti-windup

integralError = max( ...
    min(integralError, param.maxIntegralPosition), ...
    -param.maxIntegralPosition);

%% Termo derivativo

derivative = -velocity;

%% =========================================================
% Controle horizontal
% ==========================================================

ux = param.KpPos * error(1) ...
   + param.KiPos * integralError(1) ...
   + param.KdPos * derivative(1);

uy = param.KpPos * error(2) ...
   + param.KiPos * integralError(2) ...
   + param.KdPos * derivative(2);

%% =========================================================
% Controle vertical
% ==========================================================

uz = param.KpAlt * error(3) ...
   + param.KiAlt * integralError(3) ...
   + param.KdAlt * derivative(3);

%% =========================================================
% Conversão da aceleração desejada em atitude
% ==========================================================

rollRef = -uy;
pitchRef = ux;

%% Limitação da atitude

rollRef = max( ...
    min(rollRef, param.maxRoll), ...
    -param.maxRoll);

pitchRef = max( ...
    min(pitchRef, param.maxPitch), ...
    -param.maxPitch);

%% =========================================================
% Empuxo
% ==========================================================

thrust = param.m * (param.g + uz);

%% Limitação do empuxo

thrust = max( ...
    min(thrust, param.maxThrust), ...
    param.minThrust);

end