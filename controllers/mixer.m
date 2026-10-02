function motorCommand = mixer(thrust, torque, param)

% ==========================================================
% MIXER PARA QUADRICÓPTERO
%
% Entrada:
%   thrust = empuxo total [N]
%   torque = [tauRoll tauPitch tauYaw]' [N.m]
%
% Saída:
%   motorCommand = [omega1 omega2 omega3 omega4]' [rad/s]
%
% A ordem dos motores deve permanecer compatível com
% o modelo dinâmico utilizado em dynamics.m.
% ==========================================================

%% Parâmetros

L  = param.l;
Kf = param.kf;
Km = param.km;

%% =========================================================
% MATRIZ DE MISTURA
% =========================================================
%
% [T; tauRoll; tauPitch; tauYaw]
%
%       = M * [w1²; w2²; w3²; w4²]
%
% ==========================================================

M = [
     Kf       Kf       Kf       Kf;
     0        L*Kf     0       -L*Kf;
    -L*Kf     0        L*Kf     0;
     Km      -Km       Km      -Km
];

%% Vetor de comandos

U = [
    thrust;
    torque(1);
    torque(2);
    torque(3)
];

%% =========================================================
% RESOLUÇÃO DO SISTEMA
% ==========================================================

omegaSquared = M \ U;

%% =========================================================
% PROTEÇÃO CONTRA VALORES NEGATIVOS
% ==========================================================

omegaSquared = max(omegaSquared, 0);

%% =========================================================
% CONVERSÃO PARA VELOCIDADE ANGULAR
% ==========================================================

motorCommand = sqrt(omegaSquared);

%% =========================================================
% LIMITAÇÃO DA VELOCIDADE DOS MOTORES
% ==========================================================

    if isfield(param, 'maxMotorOmega')
    
        motorCommand = min( ...
            motorCommand, ...
            param.maxMotorOmega);
    
    end

end