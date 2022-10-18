function VisualizeTestData(s)
    close all;
    %% payload position
    figure
    subplot(2, 1, 1);
    plot(s.tout, s.pld_abs_pos(1, :), '-.r', 'LineWidth', 1.5);
    hold on
    plot(s.tout, s.pld_abs_pos(2, :), '-g', 'LineWidth', 1);
    plot(s.tout, s.pld_abs_pos(3, :), '-k', 'LineWidth', 0.8);
    legend({'$x_{p,x}$', '$x_{p,y}$', '$x_{p,z}$'}, 'Interpreter', 'latex')
    ylabel('Position (m)', 'Interpreter', 'latex')
    hold on
    grid on;
    xlabel('Time (s)');
    title('Payload Position');

    subplot(2, 1, 2);
    plot(s.tout, s.pld_abs_vel(1, :), '-.r', 'LineWidth', 1.5);
    hold on
    plot(s.tout, s.pld_abs_vel(2, :), '-g', 'LineWidth', 1);
    plot(s.tout, s.pld_abs_vel(3, :), '-k', 'LineWidth', 0.8);
    legend({'$v_{p,x}$', '$v_{p,y}$', '$v_{p,z}$'}, 'Interpreter', 'latex')
    ylabel('Velocity (m/s)', 'Interpreter', 'latex')
    hold on
    grid on;
    xlabel('Time (s)');
    title('Payload Velocity');

    %% path following error
    figure;
    subplot(2, 1, 1);
    plot(s.tout, s.pld_vel_err, 'LineWidth', 2);
    ylabel('Velocity Error $||e_{v,i}||$(m/s)', 'Interpreter', 'latex')
    hold on
    grid on;
    xlabel('Time (s)');
    title('Path Velocity Error');

    subplot(2, 1, 2)
    plot(s.tout, s.pld_pos_err, 'b', 'LineWidth', 2);
    hold on
    grid on;
    xlabel('Time (s)');
    ylabel('Position Error $||e_{x,i}||$ (m)', ...
        'Interpreter', 'latex');
    title('Path Position Error');
end
