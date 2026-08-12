%% =========================================================
%  INITIALIZATION.M
%
%  Inicialização do modelo matemático do quadricóptero.
%
%  Projeto:
%  SIMULAÇÃO DE DRONES E DIRETRIZES PARA IMPLANTAÇÃO DE
%  UM LABORATÓRIO DE VANTs NO IFSP-PEP
%
%  Autor: Evellyn Gomes
%
% ==========================================================

clc;

%% =========================================================
% 0. CARREGAMENTO DOS PARÂMETROS
% ==========================================================

% Garantir que os parâmetros do modelo estejam disponíveis.

if ~exist('param', 'var')

    parameters;

end

%% =========================================================
% 1. ESTADO INICIAL
% ==========================================================

% Vetor de estados:
%
% x(1)  = X          [m]
% x(2)  = Y          [m]
% x(3)  = Z          [m]
% x(4)  = Vx         [m/s]
% x(5)  = Vy         [m/s]
% x(6)  = Vz         [m/s]
% x(7)  = Roll       [rad]
% x(8)  = Pitch      [rad]
% x(9)  = Yaw        [rad]
% x(10) = Roll rate  [rad/s]
% x(11) = Pitch rate [rad/s]
% x(12) = Yaw rate   [rad/s]

x0 = zeros(12,1);


%% =========================================================
% 2. POSIÇÃO INICIAL
% ==========================================================

% O drone inicia na origem do sistema de coordenadas.

x0(1) = 0.0;       % X [m]
x0(2) = 0.0;       % Y [m]
x0(3) = 0.0;       % Z [m]


%% =========================================================
% 3. VELOCIDADE LINEAR INICIAL
% ==========================================================

% O drone inicia em repouso.

x0(4) = 0.0;       % Vx [m/s]
x0(5) = 0.0;       % Vy [m/s]
x0(6) = 0.0;       % Vz [m/s]


%% =========================================================
% 4. ORIENTAÇÃO INICIAL
% ==========================================================

% O drone inicia nivelado e sem rotação.

x0(7) = 0.0;       % Roll [rad]
x0(8) = 0.0;       % Pitch [rad]
x0(9) = 0.0;       % Yaw [rad]


%% =========================================================
% 5. VELOCIDADES ANGULARES INICIAIS
% ==========================================================

% O drone inicia sem velocidade angular.

x0(10) = 0.0;      % Roll rate [rad/s]
x0(11) = 0.0;      % Pitch rate [rad/s]
x0(12) = 0.0;      % Yaw rate [rad/s]


%% =========================================================
% 6. REFERÊNCIA INICIAL
% ==========================================================

% A trajetória será definida posteriormente pelo
% quadcopter_package_select_trajectory.m.
%
% Para o início da simulação:
%
% Xref = 0 m
% Yref = 0 m
% Zref = 1 m
%
% Neste modelo, Z será tratado como altitude positiva.
% A correção da dinâmica do eixo Z será feita no arquivo
% dynamics.m.

Xref = 0.0;        % Referência em X [m]
Yref = 0.0;        % Referência em Y [m]
Zref = 1.0;        % Altitude desejada [m]


%% =========================================================
% 7. VELOCIDADE INICIAL DOS MOTORES
% ==========================================================

% Os quatro motores iniciam na velocidade necessária
% para produzir o empuxo de hover.
%
% A velocidade é calculada em parameters.m através de:
%
% hoverOmega = sqrt((m*g)/(4*kf))
%
% Não utilizar motorOmega_hover proveniente do
% quadcopter_package_parameters.m, pois esse arquivo
% pertence ao modelo Simscape/Multibody.

motorOmega = repmat( ...
    param.hoverOmega, ...
    4, ...
    1);


%% =========================================================
% 8. VALIDAÇÃO DOS MOTORES
% ==========================================================

% Garantir que motorOmega seja um vetor coluna.

motorOmega = motorOmega(:);


% Verificar se existem exatamente quatro motores.

if numel(motorOmega) ~= 4

    error( ...
        'motorOmega deve conter exatamente 4 valores.');

end


% Compatibilidade com arquivos que utilizam motorOmega0.

motorOmega0 = motorOmega;


%% =========================================================
% 9. EXIBIÇÃO DAS CONDIÇÕES INICIAIS
% ==========================================================

fprintf('\n');
fprintf('============================================\n');
fprintf(' CONDIÇÕES INICIAIS\n');
fprintf('============================================\n');

fprintf('\n');

fprintf('X     = %.4f m\n', x0(1));
fprintf('Y     = %.4f m\n', x0(2));
fprintf('Z     = %.4f m\n', x0(3));

fprintf('\n');

fprintf('Vx    = %.4f m/s\n', x0(4));
fprintf('Vy    = %.4f m/s\n', x0(5));
fprintf('Vz    = %.4f m/s\n', x0(6));

fprintf('\n');

fprintf( ...
    'Roll  = %.4f graus\n', ...
    rad2deg(x0(7)));

fprintf( ...
    'Pitch = %.4f graus\n', ...
    rad2deg(x0(8)));

fprintf( ...
    'Yaw   = %.4f graus\n', ...
    rad2deg(x0(9)));

fprintf('\n');

fprintf('Velocidade inicial dos motores:\n');

for i = 1:4

    fprintf( ...
        'Motor %d = %.4f rad/s\n', ...
        i, ...
        motorOmega(i));

end

fprintf('\n');

fprintf('Referência inicial:\n');

fprintf( ...
    'Xref = %.4f m\n', ...
    Xref);

fprintf( ...
    'Yref = %.4f m\n', ...
    Yref);

fprintf( ...
    'Zref = %.4f m\n', ...
    Zref);

fprintf('\n');

fprintf('============================================\n');


%% =========================================================
% 10. DISPONIBILIZAR VARIÁVEIS NO BASE WORKSPACE
% ==========================================================

assignin( ...
    'base', ...
    'x0', ...
    x0);

assignin( ...
    'base', ...
    'Xref', ...
    Xref);

assignin( ...
    'base', ...
    'Yref', ...
    Yref);

assignin( ...
    'base', ...
    'Zref', ...
    Zref);

assignin( ...
    'base', ...
    'motorOmega', ...
    motorOmega);

assignin( ...
    'base', ...
    'motorOmega0', ...
    motorOmega0);