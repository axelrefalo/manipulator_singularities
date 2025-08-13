function displayRobot(robot, title, q, showAxes,T0i)
% displayRobot Visualizes the robot in a given configuration

    fig = figure('Name', title);
    axis equal;          

    show(robot, q, 'Frames', 'off');
    
    % Set the axis limits
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-1.5 2]);% singularities outside the workspace also s

    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    grid on
    
    if(showAxes)
        % Plot Z-axes for each body as lines
        hold on;
        scale = 2; % length of each axis line
    
        syms tet1 tet2 tet3 tet4 tet5 tet6 real
        tet = [tet1, tet2, tet3, tet4, tet5, tet6];
    
        for i = 1:6
    
            T = subs(T0i{i}, tet, q);
            origin = T(1:3, 4);     % position of the frame in m
            z_axis = T(1:3, 3);          % Z-axis direction
    
            % Compute line endpoints
            p1 = origin - scale * z_axis;
            p2 = origin + scale * z_axis;
    
            % Draw Z-axis as a red line
            plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'r', 'LineWidth', 1.5);

            label = sprintf('Z{%d}', i);
            text(p2(1), p2(2), p2(3), label, ...
            'FontSize', 10, 'Color', 'r', ...
            'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

        end
    end

    q_deg = rad2deg(q); % convert to degrees
    angle_str = sprintf('θ1 = %.3f°, θ2 = %.3f°, θ3 = %.3f°, θ4 = %.3f°, θ5 = %.3f°, θ6 = %.3f°', q_deg);

    annotation('textbox', [0.1, 0.01, 0.8, 0.05], ...
               'String', angle_str, ...
               'EdgeColor', 'none', ...
               'HorizontalAlignment', 'center', ...
               'FontSize', 10, ...
               'Interpreter', 'none');

    hold off;
    drawnow;

    title_str = char(title);
    savefig(fig, fullfile('/Users/axel/Desktop/Axel/ETS/CoRo/Matlab/figures/', [title_str '.fig']));


end