function reportPlots(time, state)
% ==========================================================
% REPORTPLOTS.M
% Geração automática dos gráficos do relatório
% ==========================================================

    x = state(:,1);
    y = state(:,2);
    z = state(:,3);
    
    roll = state(:,7);
    pitch = state(:,8);
    yaw = state(:,9);
    
    figure;
    
    plot(time, x, 'LineWidth', 1.5);
    
    grid on;
    
    xlabel('Tempo (s)');
    
    ylabel('X (m)');
    
    title('Posição no eixo X');
    
    figure;
    
    plot(time, y, 'LineWidth', 1.5);
    
    grid on;
    
    xlabel('Tempo (s)');
    
    ylabel('Y (m)');
    
    title('Posição no eixo Y');
    
    figure;
    
    plot(time, z, 'LineWidth', 1.5);
    
    grid on;
    
    xlabel('Tempo (s)');
    
    ylabel('Z (m)');
    
    title('Posição no eixo Z');
    
    figure;
    
    plot(time, roll, 'LineWidth', 1.5);
    
    grid on;
    
    xlabel('Tempo (s)');
    
    ylabel('Roll (rad)');
    
    title('Ângulo de Roll');
    
    figure;
    
    plot(time, pitch, 'LineWidth', 1.5);
    
    grid on;
    
    xlabel('Tempo (s)');
    
    ylabel('Pitch (rad)');
    
    title('Ângulo de Pitch');
    
    figure;
    
    plot(time, yaw, 'LineWidth', 1.5);
    
    grid on;
    
    xlabel('Tempo (s)');
    
    ylabel('Yaw (rad)');
    
    title('Ângulo de Yaw');
    
    trajectoryPlot(x, y, z);

end