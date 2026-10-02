function trajectoryPlot(x, y, z)
% ==========================================================
% TRAJECTORYPLOT.M
% Trajetória do drone
% ==========================================================

    figure
    
    plot(x, y, 'b', 'LineWidth', 2);
    
    grid on;
    
    axis equal;
    
    xlabel('X (m)');
    
    ylabel('Y (m)');
    
    title('Trajetória Horizontal');
    
    figure
    
    plot3(x, y, z, 'r', 'LineWidth', 2);
    
    grid on;
    
    xlabel('X (m)');
    
    ylabel('Y (m)');
    
    zlabel('Z (m)');
    
    title('Trajetória Tridimensional');

end