%% ==========================================================
% MAIN.M
%
% Programa principal da simulação do quadricóptero.
%
% Projeto:
% SIMULAÇÃO DE DRONES E DIRETRIZES PARA IMPLANTAÇÃO DE
% UM LABORATÓRIO DE VANTs NO IFSP-PEP
%
% Autor: Evellyn Gomes
% ==========================================================

clearvars;
clc;

fprintf('\n');
fprintf('============================================\n');
fprintf(' SIMULAÇÃO DE QUADRICÓPTERO\n');
fprintf('============================================\n');
fprintf('Autor: Evellyn Gomes\n');
fprintf('============================================\n\n');


%% =========================================================
% 1. LIMPAR TRAJETÓRIAS ANTERIORES
% ==========================================================

evalin('base', ...
    'clear waypoints timespot_spl spline_data spline_yaw wayp_path_vis');

clear waypoints;
clear timespot_spl;
clear spline_data;
clear spline_yaw;
clear wayp_path_vis;

%% =========================================================
% 2. CAMINHO DO PROJETO
% ==========================================================

projectRoot = '/MATLAB Drive/DroneHoverSimulation';

if ~isfolder(projectRoot)

    error( ...
        'Diretório do projeto não encontrado: %s', ...
        projectRoot);

end

fprintf('\n');
fprintf('Caminho do projeto: %s\n', projectRoot);

addpath(projectRoot);


%% =========================================================
% 3. CARREGAR PARÂMETROS
% ==========================================================

fprintf('\n');
fprintf('Carregando parâmetros...\n');

quadcopter_package_parameters;

fprintf('Parâmetros carregados com sucesso.\n');


%% =========================================================
% 4. CARREGAR CONDIÇÕES INICIAIS
% ==========================================================

fprintf('Carregando condições iniciais...\n');

initialization;

fprintf('Condições iniciais carregadas com sucesso.\n');


%% =========================================================
% 5. DURAÇÃO DA SIMULAÇÃO
% ==========================================================

while true

    Tsim = input( ...
        'Digite a duração da simulação em segundos: ');

    if isempty(Tsim) || ...
            ~isnumeric(Tsim) || ...
            ~isscalar(Tsim) || ...
            ~isfinite(Tsim) || ...
            Tsim <= 0

        fprintf('\n');
        fprintf( ...
            'Digite uma duração válida maior que zero.\n\n');

    else

        break;

    end

end

fprintf('\n');
fprintf( ...
    'Duração selecionada: %.2f segundos\n', ...
    Tsim);


%% =========================================================
% 6. SELEÇÃO DA TRAJETÓRIA
% ==========================================================

fprintf('\n');
fprintf('============================================\n');
fprintf(' SELEÇÃO DA TRAJETÓRIA\n');
fprintf('============================================\n\n');

fprintf('1 - H00 - Hover\n');
fprintf('    Subida até 1 m e estabilização em hover.\n\n');

fprintf('2 - H01 - Subida Vertical\n');
fprintf('    Subida vertical até 2 m.\n\n');

fprintf('3 - H02 - Deslocamento em X\n');
fprintf('    Deslocamento horizontal no eixo X.\n\n');

fprintf('4 - H03 - Deslocamento em Y\n');
fprintf('    Deslocamento horizontal no eixo Y.\n\n');

fprintf('5 - H04 - Trajetória Quadrada\n');
fprintf('    Percurso quadrado no plano horizontal.\n\n');

fprintf('6 - H05 - Ida e Volta\n');
fprintf('    Deslocamento e retorno à posição inicial.\n\n');


while true

    trajectoryNumber = input( ...
        'Digite o número da trajetória (1-6): ');

    if isempty(trajectoryNumber) || ...
            ~isnumeric(trajectoryNumber) || ...
            ~isscalar(trajectoryNumber) || ...
            ~isfinite(trajectoryNumber) || ...
            trajectoryNumber ~= floor(trajectoryNumber) || ...
            trajectoryNumber < 1 || ...
            trajectoryNumber > 6

        fprintf('\n');
        fprintf( ...
            'Opção inválida. Escolha de 1 a 6.\n\n');

    else

        break;

    end

end


%% =========================================================
% 7. IDENTIFICAÇÃO DA TRAJETÓRIA
% ==========================================================

trajectoryNames = { ...
    'H00 - Hover'
    'H01 - Subida Vertical'
    'H02 - Deslocamento em X'
    'H03 - Deslocamento em Y'
    'H04 - Trajetória Quadrada'
    'H05 - Ida e Volta'
    };

selectedTrajectory = ...
    trajectoryNames{trajectoryNumber};

fprintf('\n');
fprintf( ...
    'Trajetória selecionada: %s\n', ...
    selectedTrajectory);


%% =========================================================
% 8. CONFIGURAÇÃO DA TRAJETÓRIA
% ==========================================================

trajectoryConfig = struct();


switch trajectoryNumber

    case 1

        trajectoryConfig.name = 'H00';
        trajectoryConfig.maxVelocity = 0.50;
        trajectoryConfig.maxAcceleration = 0.50;
        trajectoryConfig.targetAltitude = 1.0;


    case 2

        trajectoryConfig.name = 'H01';
        trajectoryConfig.maxVelocity = 0.40;
        trajectoryConfig.maxAcceleration = 0.40;
        trajectoryConfig.targetAltitude = 2.0;


    case 3

        trajectoryConfig.name = 'H02';
        trajectoryConfig.maxVelocity = 0.50;
        trajectoryConfig.maxAcceleration = 0.50;
        trajectoryConfig.targetAltitude = 1.0;


    case 4

        trajectoryConfig.name = 'H03';
        trajectoryConfig.maxVelocity = 0.50;
        trajectoryConfig.maxAcceleration = 0.50;
        trajectoryConfig.targetAltitude = 1.0;


    case 5

        trajectoryConfig.name = 'H04';
        trajectoryConfig.maxVelocity = 0.40;
        trajectoryConfig.maxAcceleration = 0.40;
        trajectoryConfig.targetAltitude = 1.0;


    case 6

        trajectoryConfig.name = 'H05';
        trajectoryConfig.maxVelocity = 0.50;
        trajectoryConfig.maxAcceleration = 0.50;
        trajectoryConfig.targetAltitude = 1.0;

end


assignin( ...
    'base', ...
    'trajectoryConfig', ...
    trajectoryConfig);


%% =========================================================
% 9. GERAR TRAJETÓRIA
% ==========================================================

fprintf('\n');
fprintf('Gerando trajetória...\n');

[
    waypoints, ...
    timespot_spl, ...
    spline_data, ...
    spline_yaw, ...
    wayp_path_vis ...
    ] = ...
    quadcopter_package_select_trajectory( ...
    trajectoryNumber, ...
    false);

fprintf('Trajetória gerada com sucesso.\n');


%% =========================================================
% 10. NORMALIZAR WAYPOINTS
% ==========================================================

if isempty(waypoints)

    error( ...
        'A função de trajetória não retornou nenhum waypoint.');

end


waypointsSize = size(waypoints);


if waypointsSize(1) == 3

    % Formato correto: 3 x N

elseif waypointsSize(2) == 3

    % Converter N x 3 para 3 x N

    waypoints = waypoints.';

else

    error([ ...
        'Formato inválido de waypoints: %d x %d. ' ...
        'A matriz deve possuir 3 linhas correspondentes ' ...
        'a X, Y e Z.'], ...
        waypointsSize(1), ...
        waypointsSize(2));

end


%% =========================================================
% 11. REMOVER WAYPOINTS DUPLICADOS
% ==========================================================

waypoints = unique( ...
    waypoints.', ...
    'rows', ...
    'stable').';


%% =========================================================
% 12. NORMALIZAR VETORES
% ==========================================================

timespot_spl = timespot_spl(:);

spline_yaw = spline_yaw(:);


%% =========================================================
% 13. VALIDAR TRAJETÓRIA
% ==========================================================

if size(waypoints,1) ~= 3

    error( ...
        'Erro interno: waypoints não está no formato 3 x N.');

end


if size(waypoints,2) < 1

    error( ...
        'Erro interno: nenhum waypoint válido foi encontrado.');

end


fprintf('\n');
fprintf('============================================\n');
fprintf(' TRAJETÓRIA NORMALIZADA\n');
fprintf('============================================\n');

fprintf( ...
    'Formato de waypoints: %d x %d\n', ...
    size(waypoints,1), ...
    size(waypoints,2));

fprintf( ...
    'X inicial = %.4f m\n', ...
    waypoints(1,1));

fprintf( ...
    'Y inicial = %.4f m\n', ...
    waypoints(2,1));

fprintf( ...
    'Z inicial = %.4f m\n', ...
    waypoints(3,1));

fprintf( ...
    'X final = %.4f m\n', ...
    waypoints(1,end));

fprintf( ...
    'Y final = %.4f m\n', ...
    waypoints(2,end));

fprintf( ...
    'Z final = %.4f m\n', ...
    waypoints(3,end));

fprintf('============================================\n');


%% =========================================================
% 14. DISPONIBILIZAR TRAJETÓRIA NO BASE WORKSPACE
% ==========================================================

assignin('base', 'waypoints', waypoints);
assignin('base', 'timespot_spl', timespot_spl);
assignin('base', 'spline_data', spline_data);
assignin('base', 'spline_yaw', spline_yaw);
assignin('base', 'wayp_path_vis', wayp_path_vis);
assignin('base', 'trajectoryNumber', trajectoryNumber);
assignin('base', 'selectedTrajectory', selectedTrajectory);
assignin('base', 'Tsim', Tsim);


%% =========================================================
% 15. VELOCIDADE DOS MOTORES
% ==========================================================

if ~exist('motorOmega','var')

    if exist('motorOmega_hover','var')

        motorOmega = repmat( ...
            motorOmega_hover, ...
            4, ...
            1);

    elseif exist('param','var') && ...
            isfield(param,'hoverOmega')

        motorOmega = repmat( ...
            param.hoverOmega, ...
            4, ...
            1);

    else

        error([ ...
            'Não foi possível determinar a velocidade ' ...
            'inicial dos motores. Verifique parameters.m.']);

    end

end


motorOmega = motorOmega(:);


if numel(motorOmega) ~= 4

    error( ...
        'motorOmega deve conter exatamente quatro valores.');

end


motorOmega0 = motorOmega;


assignin('base', 'motorOmega', motorOmega);
assignin('base', 'motorOmega0', motorOmega);


fprintf('\n');
fprintf('Velocidade inicial dos motores:\n');

for i = 1:4

    fprintf( ...
        'Motor %d = %.4f rad/s\n', ...
        i, ...
        motorOmega(i));

end


%% =========================================================
% 16. ABRIR PROJETO SIMULINK
% ==========================================================

fprintf('\n');
fprintf('Abrindo projeto Simulink...\n');


% Caminho explícito do projeto

projectRoot = '/MATLAB Drive/DroneHoverSimulation';


% Arquivo do projeto

projectFile = fullfile( ...
    projectRoot, ...
    'Quadcopter_Drone.prj');


% Verificar diretório

if ~isfolder(projectRoot)

    error( ...
        'Diretório do projeto não encontrado: %s', ...
        projectRoot);

end


% Verificar arquivo .prj

if ~isfile(projectFile)

    error( ...
        'Arquivo de projeto não encontrado: %s', ...
        projectFile);

end


fprintf( ...
    'Caminho do projeto: %s\n', ...
    projectRoot);

fprintf( ...
    'Arquivo do projeto: %s\n', ...
    projectFile);


% Abrir projeto

try

    openProject(projectFile);

    curr_proj = currentProject;

    fprintf( ...
        'Projeto %s aberto com sucesso.\n', ...
        curr_proj.Name);

catch ME

    error( ...
        'Não foi possível abrir o projeto Simulink: %s', ...
        ME.message);

end


%% =========================================================
% 17. LOCALIZAR MODELO
% ==========================================================

modelName = ...
    'quadcopter_package_delivery';


modelPath = fullfile( ...
    projectRoot, ...
    'simulink', ...
    'Models', ...
    [modelName '.slx']);


fprintf('\n');
fprintf('Localizando modelo Simulink...\n');


if ~isfile(modelPath)

    error([ ...
        'Modelo Simulink não encontrado:' ...
        newline ...
        '%s'], ...
        modelPath);

end


fprintf( ...
    'Modelo Simulink encontrado:\n%s\n', ...
    modelPath);


%% =========================================================
% 18. CARREGAR MODELO
% ==========================================================

fprintf('\n');
fprintf('Carregando modelo Simulink...\n');

load_system('/MATLAB Drive/DroneHoverSimulation/simulink/Models/quadcopter_package_delivery.slx');

fprintf( ...
    'Modelo Simulink carregado com sucesso.\n');


%% =========================================================
% 19. CONECTAR TRAJETÓRIA AO MODEL WORKSPACE
% ==========================================================

mdlWks = get_param( ...
    modelName, ...
    'ModelWorkspace');


assignin( ...
    mdlWks, ...
    'waypoints', ...
    waypoints);

assignin( ...
    mdlWks, ...
    'timespot_spl', ...
    timespot_spl);

assignin( ...
    mdlWks, ...
    'spline_data', ...
    spline_data);

assignin( ...
    mdlWks, ...
    'spline_yaw', ...
    spline_yaw);

assignin( ...
    mdlWks, ...
    'wayp_path_vis', ...
    wayp_path_vis);

assignin( ...
    mdlWks, ...
    'trajectoryNumber', ...
    trajectoryNumber);

assignin( ...
    mdlWks, ...
    'selectedTrajectory', ...
    selectedTrajectory);

assignin( ...
    mdlWks, ...
    'Tsim', ...
    Tsim);

assignin( ...
    mdlWks, ...
    'trajectoryConfig', ...
    trajectoryConfig);

assignin( ...
    mdlWks, ...
    'motorOmega', ...
    motorOmega);

assignin( ...
    mdlWks, ...
    'motorOmega0', ...
    motorOmega0);


%% =========================================================
% 20. VERIFICAR WAYPOINTS NO MODEL WORKSPACE
% ==========================================================

waypointsModel = getVariable( ...
    mdlWks, ...
    'waypoints');


fprintf('\n');
fprintf('============================================\n');
fprintf(' CONEXÃO TRAJETÓRIA -> SIMULINK\n');
fprintf('============================================\n');

fprintf( ...
    'Dimensão de waypoints: %d x %d\n', ...
    size(waypointsModel,1), ...
    size(waypointsModel,2));


if size(waypointsModel,1) ~= 3

    error([ ...
        'A variável waypoints no Model Workspace ' ...
        'possui formato inválido. Esperado: 3 x N. ' ...
        'Obtido: %d x %d.'], ...
        size(waypointsModel,1), ...
        size(waypointsModel,2));

end


fprintf( ...
    'X inicial = %.4f m\n', ...
    waypointsModel(1,1));

fprintf( ...
    'Y inicial = %.4f m\n', ...
    waypointsModel(2,1));

fprintf( ...
    'Z inicial = %.4f m\n', ...
    waypointsModel(3,1));

fprintf( ...
    'X final = %.4f m\n', ...
    waypointsModel(1,end));

fprintf( ...
    'Y final = %.4f m\n', ...
    waypointsModel(2,end));

fprintf( ...
    'Z final = %.4f m\n', ...
    waypointsModel(3,end));

fprintf( ...
    'Trajetória conectada ao Model Workspace.\n');

fprintf('============================================\n');


%% =========================================================
% 21. CONFIGURAR TEMPO DE SIMULAÇÃO
% ==========================================================

set_param( ...
    modelName, ...
    'StopTime', ...
    num2str(Tsim));


fprintf( ...
    'Tempo de simulação configurado: %.3f s\n', ...
    Tsim);


%% =========================================================
% 22. ABRIR MODELO
% ==========================================================

open_system(modelName);

fprintf( ...
    'Modelo Simulink aberto.\n');


%% =========================================================
% 23. EXECUTAR SIMULAÇÃO
% ==========================================================

fprintf('\n');
fprintf('============================================\n');
fprintf(' INICIANDO SIMULAÇÃO\n');
fprintf('============================================\n');

simOut = sim( ...
    modelName, ...
    'StopTime', ...
    num2str(Tsim));

fprintf('\n');
fprintf('Modelo visual concluído.\n');

%% =========================================================
% 24. GRÁFICO PVO
% ==========================================================

fprintf('\n');
fprintf('Gerando gráfico PVO...\n');

try

    quadcopter_package_delivery_plot1pvo;

    fprintf('\n');
    fprintf('Gráfico PVO gerado com sucesso.\n');

catch ME

    fprintf('\n');
    fprintf('============================================\n');
    fprintf(' AVISO - GRÁFICO PVO\n');
    fprintf('============================================\n');

    fprintf( ...
        'Não foi possível concluir o gráfico PVO.\n');

    fprintf( ...
        'Erro: %s\n', ...
        ME.message);

    fprintf('\n');
    fprintf( ...
        'A simulação do quadricóptero foi concluída.\n');

    fprintf( ...
        'O problema ocorreu somente na visualização.\n');

    fprintf('============================================\n');

end


%% =========================================================
% 25. TRAJETÓRIA 3D
% ==========================================================

fprintf('\n');
fprintf('Gerando trajetória 3D...\n');

try

    quadcopter_package_delivery_plot2xyz;

    fprintf( ...
        'Trajetória 3D gerada com sucesso.\n');

catch ME

    warning( ...
        'Não foi possível gerar a trajetória 3D: %s', ...
        ME.message);

end


%% =========================================================
% 26. FINALIZAÇÃO
% ==========================================================

fprintf('\n');
fprintf('============================================\n');
fprintf(' SIMULAÇÃO CONCLUÍDA\n');
fprintf('============================================\n');

fprintf( ...
    'Trajetória: %s\n', ...
    selectedTrajectory);

fprintf( ...
    'Duração: %.2f segundos\n', ...
    Tsim);

fprintf('============================================\n');
fprintf('\n');