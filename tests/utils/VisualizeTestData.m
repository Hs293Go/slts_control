function VisualizeTestData(s)
    fds = fields(s);

    for idx = 1:numel(fds)
        it = s.(fds{idx});
        data = reshape(it.value, it.size);
        s.(fds{idx}) = data;
    end

    %% payload position
    figure
    subplot(2, 1, 1);
    plot(s.tout, s.pld_abs_pos(:, 1), '-.r', 'LineWidth', 1.5);
    hold on
    plot(s.tout, s.pld_abs_pos(:, 2), '-g', 'LineWidth', 1);
    plot(s.tout, s.pld_abs_pos(:, 3), '-k', 'LineWidth', 0.8);
    legend({'$x_{p,x}$', '$x_{p,y}$', '$x_{p,z}$'}, 'Interpreter', 'latex')
    ylabel('Position (m)', 'Interpreter', 'latex')
    hold on
    grid on;
    xlabel('Time (s)');
    title('Payload Position');

    subplot(2, 1, 2);
    plot(s.tout, s.pld_abs_vel(:, 1), '-.r', 'LineWidth', 1.5);
    hold on
    plot(s.tout, s.pld_abs_vel(:, 2), '-g', 'LineWidth', 1);
    plot(s.tout, s.pld_abs_vel(:, 3), '-k', 'LineWidth', 0.8);
    legend({'$v_{p,x}$', '$v_{p,y}$', '$v_{p,z}$'}, 'Interpreter', 'latex')
    ylabel('Velocity (m/s)', 'Interpreter', 'latex')
    hold on
    grid on;
    xlabel('Time (s)');
    title('Payload Velocity');

    %% path following error
    figure;
    subplot(2, 1, 1);
    plot(s.tout, s.vel_err, 'LineWidth', 2);
    ylabel('Velocity Error $||e_{v,i}||$(m/s)', 'Interpreter', 'latex')
    hold on
    grid on;
    xlabel('Time (s)');
    title('Path Velocity Error');

    subplot(2, 1, 2)
    plot(s.tout, s.pos_err, 'b', 'LineWidth', 2);
    hold on
    grid on;
    xlabel('Time (s)');
    ylabel('Position Error $||e_{x,i}||$ (m)', ...
        'Interpreter', 'latex');
    title('Path Position Error');

    %% plot the disturbance estimation results
    figure
    subplot(2, 1, 1)
    plot(s.tout, s.proj_de_est_err, '-b', 'LineWidth', 2);
    hold on
    plot(s.tout, s.proj_de(:, 1), '-.r', 'LineWidth', 1.5);
    plot(s.tout, s.proj_de(:, 2), '-g', 'LineWidth', 1);
    plot(s.tout, s.proj_de(:, 3), '-k', 'LineWidth', 0.8);
    ylabel('Magnitude $(N)$', 'Interpreter', 'latex')
    xlabel('Time (s)'); title('Estimated Disturbance')
    legend({'$||\tilde{\Delta}_{\bot}||$', '$\hat{\Delta}_{\bot,x}$', '$\hat{\Delta}_{\bot,y}$', '$\hat{\Delta}_{\bot,z}$'}, 'Interpreter', 'latex')
    grid on
    title('The Estimated Disturbances on The Payload')

    subplot(2, 1, 2)

    plot(s.tout, s.total_de_est_err, '-b', 'LineWidth', 2);
    hold on
    plot(s.tout, s.total_de(:, 1), '-.r', 'LineWidth', 1.5);
    plot(s.tout, s.total_de(:, 2), '-g', 'LineWidth', 1);
    plot(s.tout, s.total_de(:, 3), '-k', 'LineWidth', 0.8);

    ylabel('Magnitude $(N)$', 'Interpreter', 'latex')
    xlabel('Time (s)'); title('Estimated Disturbance')
    legend({'$||\tilde{\Delta}_{T}||$', '$\hat{\Delta}_{T,x}$', '$\hat{\Delta}_{T,y}$', '$\hat{\Delta}_{T,z}$'}, 'Interpreter', 'latex')
    grid on
    title('The Estimated Disturbances on The Quadrotor')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure
    plot(s.tout, s.pld_swing_est_err(:, 1), '-b', 'LineWidth', 1);
    hold on
    plot(s.tout, s.pld_swing_est_err(:, 2), '-.r', 'LineWidth', 2);
    ylabel('Position (m)', 'Interpreter', 'latex')

    title('Quadrotor Relative Position Error');
    legend({'$\tilde{r}_{x}$', '$\tilde{r}_{y}$'}, 'Interpreter', 'latex')
    xlabel('Time (s)')
    grid on
end
