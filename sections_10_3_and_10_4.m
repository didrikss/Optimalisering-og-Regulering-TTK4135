% ========================================================================
% TTK4135 Helicopter Lab - Sections 10.3 & 10.4
% Complete Implementation: LQ Control and Elevation Control
% ========================================================================
% This script provides complete solutions for:
%   - Section 10.3: Optimal Control of Pitch/Travel with Feedback (LQ)
%   - Section 10.4: Optimal Control with Elevation
%
% Make sure you have completed Section 10.2 before running this!
% ========================================================================

clear; clc; close all;

%% ========================================================================
%  CONFIGURATION - Choose which section to run
% ========================================================================

RUN_SECTION_10_3 = true;   % Set to true to run Section 10.3
RUN_SECTION_10_4 = true;   % Set to true to run Section 10.4

fprintf('========================================================================\n');
fprintf('TTK4135 Helicopter Lab - Sections 10.3 & 10.4\n');
fprintf('========================================================================\n\n');

%% ========================================================================
%  SECTION 10.3: OPTIMAL CONTROL OF PITCH/TRAVEL WITH FEEDBACK (LQ)
% ========================================================================

if RUN_SECTION_10_3
    fprintf('\n');
    fprintf('========================================================================\n');
    fprintf('SECTION 10.3: PITCH/TRAVEL WITH LQ FEEDBACK\n');
    fprintf('========================================================================\n\n');
    
    %% Load the optimal trajectory from Section 10.2
    fprintf('Step 1: Loading optimal trajectory from Section 10.2...\n');
    
    % Choose which q value to use (recommended: q = 1.2)
    q_chosen = 1.2;
    filename_10_2 = sprintf('optimal_trajectory_q%.2f.mat', q_chosen);
    
    if ~exist(filename_10_2, 'file')
        error(['File %s not found!\n' ...
               'You must complete Section 10.2 first and generate the optimal trajectory.'], ...
               filename_10_2);
    end
    
    load(filename_10_2);  % Loads: u_opt, x_opt, A, B, dt, N, etc.
    fprintf('   Loaded: %s\n', filename_10_2);
    fprintf('   Optimal trajectory: %d time steps\n', N);
    fprintf('   Sampling time: %.2f s\n\n', dt);
    
    %% System parameters (should match Section 10.2)
    % IMPORTANT: Load from your init.m file!
    % run('init.m');
    
    % Placeholder values - REPLACE THESE!
    K1 = 1.0;   % REPLACE
    K2 = 1.0;   % REPLACE
    Kpp = 1.0;  % REPLACE
    Kpd = 1.0;  % REPLACE
    
    fprintf('WARNING: Using placeholder K values!\n');
    fprintf('Update K1, K2, Kpp, Kpd from your init.m file!\n\n');
    
    %% Build state space model (same as 10.2)
    fprintf('Step 2: Building state-space model...\n');
    
    % State: x = [lambda, r, p, p_dot]'
    % Input: u = pc
    
    Ac_10_3 = [0,  1,      0,           0;
               0,  0,     -K2,          0;
               0,  0,      0,           1;
               0,  0,  -K1*Kpp,  -K1*Kpd];
    
    Bc_10_3 = [0; 0; 0; K1*Kpp];
    
    % Discretize
    A_10_3 = eye(4) + dt * Ac_10_3;
    B_10_3 = dt * Bc_10_3;
    
    fprintf('   State-space model built (4 states, 1 input)\n\n');
    
    %% Design LQ controller
    fprintf('Step 3: Designing LQ controller using dlqr...\n');
    
    % Task 10.3.1: Calculate K matrix using LQ control
    % The LQ controller minimizes: J = sum(Δx'*Q*Δx + Δu'*R*Δu)
    % where Δx = x - x*, Δu = u - u*
    
    % Design Q and R matrices
    % Q penalizes state deviations, R penalizes control effort
    
    % Try different values - these are starting suggestions
    Q_lambda = 1;      % Penalty on travel angle deviation
    Q_r = 0;           % Penalty on travel rate deviation
    Q_p = 0;           % Penalty on pitch deviation
    Q_pdot = 0;        % Penalty on pitch rate deviation
    
    R_lq = 1;          % Penalty on control input deviation
    
    % Build Q matrix (diagonal)
    Q_10_3 = diag([Q_lambda, Q_r, Q_p, Q_pdot]);
    R_10_3 = R_lq;
    
    fprintf('   Q matrix (state weights):\n');
    fprintf('     Q_lambda = %.2f\n', Q_lambda);
    fprintf('     Q_r      = %.2f\n', Q_r);
    fprintf('     Q_p      = %.2f\n', Q_p);
    fprintf('     Q_pdot   = %.2f\n', Q_pdot);
    fprintf('   R (input weight) = %.2f\n\n', R_lq);
    
    % Calculate optimal feedback gain using dlqr
    [K_lq, S, e] = dlqr(A_10_3, B_10_3, Q_10_3, R_10_3);
    
    fprintf('   LQ gain matrix K calculated:\n');
    fprintf('   K = [%.4f, %.4f, %.4f, %.4f]\n', K_lq(1), K_lq(2), K_lq(3), K_lq(4));
    fprintf('   Closed-loop eigenvalues:\n');
    for i = 1:length(e)
        fprintf('     λ_%d = %.4f\n', i, e(i));
    end
    fprintf('\n');
    
    %% Save results for Simulink (Section 10.3)
    fprintf('Step 4: Preparing data for Simulink implementation...\n');
    
    % Add padding (same as before)
    padding_time = 5;
    padding_steps = round(padding_time / dt);
    
    % Initial state
    lambda_0 = pi;
    x0_10_3 = [lambda_0; 0; 0; 0];
    
    % Padded optimal trajectory
    u_opt_padded_10_3 = [zeros(padding_steps, 1); u_opt; zeros(padding_steps, 1)];
    x_opt_padded_10_3 = [repmat(x0_10_3, 1, padding_steps), x_opt, ...
                         repmat(x_opt(:,end), 1, padding_steps)];
    
    t_padded_10_3 = (0:length(u_opt_padded_10_3)-1) * dt;
    t_x_padded_10_3 = (0:size(x_opt_padded_10_3, 2)-1) * dt;
    
    % Create timeseries for Simulink
    u_opt_timeseries_10_3 = timeseries(u_opt_padded_10_3, t_padded_10_3);
    u_opt_timeseries_10_3.Name = 'u_opt_10_3';
    
    x_opt_timeseries_10_3 = timeseries(x_opt_padded_10_3', t_x_padded_10_3);
    x_opt_timeseries_10_3.Name = 'x_opt_10_3';
    
    % Save to file
    filename_10_3 = 'section_10_3_data.mat';
    save(filename_10_3, 'K_lq', 'Q_10_3', 'R_10_3', ...
         'u_opt_padded_10_3', 'x_opt_padded_10_3', ...
         't_padded_10_3', 't_x_padded_10_3', ...
         'u_opt_timeseries_10_3', 'x_opt_timeseries_10_3', ...
         'A_10_3', 'B_10_3', 'dt');
    
    fprintf('   Saved to: %s\n', filename_10_3);
    fprintf('   Variables for Simulink:\n');
    fprintf('     - K_lq (feedback gain matrix)\n');
    fprintf('     - u_opt_timeseries_10_3 (optimal input trajectory)\n');
    fprintf('     - x_opt_timeseries_10_3 (optimal state trajectory)\n\n');
    
    %% Visualize LQ controller design
    fprintf('Step 5: Visualizing LQ controller...\n\n');
    
    figure('Name', 'Section 10.3: LQ Controller Design', 'Position', [100, 100, 1000, 600]);
    
    subplot(2,2,1)
    bar(K_lq)
    title('LQ Feedback Gains')
    xlabel('State Index')
    ylabel('Gain Value')
    set(gca, 'XTickLabel', {'\lambda', 'r', 'p', '\dotp'})
    grid on
    
    subplot(2,2,2)
    plot(real(e), imag(e), 'bx', 'MarkerSize', 10, 'LineWidth', 2)
    hold on
    theta = linspace(0, 2*pi, 100);
    plot(cos(theta), sin(theta), 'r--')
    grid on
    axis equal
    xlabel('Real')
    ylabel('Imaginary')
    title('Closed-Loop Poles')
    legend('Poles', 'Unit Circle')
    
    subplot(2,2,3)
    stairs(t_padded_10_3, u_opt_padded_10_3*180/pi, 'LineWidth', 1.5)
    grid on
    xlabel('Time [s]')
    ylabel('Optimal Input p_c [deg]')
    title('Optimal Input Trajectory (u*)')
    
    subplot(2,2,4)
    plot(t_x_padded_10_3, x_opt_padded_10_3(1,:)*180/pi, 'LineWidth', 1.5)
    hold on
    plot(t_x_padded_10_3, x_opt_padded_10_3(3,:)*180/pi, 'LineWidth', 1.5)
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    title('Optimal State Trajectory (x*)')
    legend('\lambda*', 'p*')
    
    fprintf('   Section 10.3 complete!\n');
    fprintf('   Next: Implement in Simulink using the feedback law:\n');
    fprintf('         u = u* - K*(x - x*)\n\n');
    
end

%% ========================================================================
%  SECTION 10.4: OPTIMAL CONTROL WITH ELEVATION
% ========================================================================

if RUN_SECTION_10_4
    fprintf('\n');
    fprintf('========================================================================\n');
    fprintf('SECTION 10.4: PITCH/TRAVEL AND ELEVATION WITH FEEDBACK\n');
    fprintf('========================================================================\n\n');
    
    %% System parameters (should match previous sections)
    % IMPORTANT: Load from your init.m file!
    % run('init.m');
    
    % Placeholder values - REPLACE THESE!
    K1_10_4 = 1.0;   % REPLACE
    K2_10_4 = 1.0;   % REPLACE
    K3_10_4 = 1.0;   % REPLACE (for elevation)
    Kpp_10_4 = 1.0;  % REPLACE
    Kpd_10_4 = 1.0;  % REPLACE
    Kep_10_4 = 1.0;  % REPLACE (elevation proportional gain)
    Ked_10_4 = 1.0;  % REPLACE (elevation derivative gain)
    
    fprintf('WARNING: Using placeholder K values!\n');
    fprintf('Update all K values from your init.m file!\n\n');
    
    %% Build extended state space model with elevation
    fprintf('Step 1: Building extended state-space model with elevation...\n');
    
    % State: x = [lambda, r, p, p_dot, e, e_dot]'
    % Input: u = [pc, ec]'
    
    Ac_10_4 = [0,  1,      0,           0,              0,           0;
               0,  0,     -K2_10_4,     0,              0,           0;
               0,  0,      0,           1,              0,           0;
               0,  0,  -K1_10_4*Kpp_10_4,  -K1_10_4*Kpd_10_4,  0,   0;
               0,  0,      0,           0,              0,           1;
               0,  0,      0,           0,  -K3_10_4*Kep_10_4, -K3_10_4*Ked_10_4];
    
    Bc_10_4 = [0,           0;
               0,           0;
               0,           0;
               K1_10_4*Kpp_10_4,  0;
               0,           0;
               0,           K3_10_4*Kep_10_4];
    
    % Discretize
    A_10_4 = eye(6) + dt * Ac_10_4;
    B_10_4 = dt * Bc_10_4;
    
    fprintf('   State-space model built (6 states, 2 inputs)\n');
    fprintf('   States: [lambda, r, p, p_dot, e, e_dot]''\n');
    fprintf('   Inputs: [pc, ec]''\n\n');
    
    %% Optimization parameters
    fprintf('Step 2: Setting up optimization problem...\n');
    
    % Time parameters
    dt_10_4 = 0.25;
    N_10_4 = 40;  % Shorter horizon as suggested in manual
    
    % Initial and final states
    lambda_0_10_4 = pi;
    lambda_f_10_4 = 0;
    
    x0_10_4 = [lambda_0_10_4; 0; 0; 0; 0; 0];
    xf_10_4 = [lambda_f_10_4; 0; 0; 0; 0; 0];
    
    % Constraints
    p_max_10_4 = 60 * pi/180;  % Pitch constraint
    
    % Elevation constraint parameters
    alpha = 0.2;
    beta = 20;
    lambda_t = 2*pi/3;
    
    % Objective function weights
    q1 = 1;  % Weight on pitch reference
    q2 = 1;  % Weight on elevation reference
    
    fprintf('   Time horizon: %.1f seconds (%d steps)\n', N_10_4*dt_10_4, N_10_4);
    fprintf('   Initial state: lambda = %.2f rad (%.1f deg)\n', lambda_0_10_4, lambda_0_10_4*180/pi);
    fprintf('   Final state: lambda = %.2f rad (%.1f deg)\n', lambda_f_10_4, lambda_f_10_4*180/pi);
    fprintf('   Pitch constraint: |p| <= %.1f deg\n', p_max_10_4*180/pi);
    fprintf('   Elevation constraint: e >= %.2f*exp(-%.1f*(lambda-%.2f)^2)\n', alpha, beta, lambda_t);
    fprintf('   Weights: q1 = %.2f, q2 = %.2f\n\n', q1, q2);
    
    %% Formulate optimization problem for fmincon
    fprintf('Step 3: Formulating optimization problem for fmincon...\n');
    
    % Decision variables: z = [x_1, ..., x_N, u_0, ..., u_{N-1}]
    % where x_i is 6D and u_i is 2D
    n_states_10_4 = 6;
    n_inputs_10_4 = 2;
    n_variables_10_4 = n_states_10_4 * N_10_4 + n_inputs_10_4 * N_10_4;
    
    fprintf('   Decision variables: %d total\n', n_variables_10_4);
    fprintf('     - States: %d (6 states × %d steps)\n', n_states_10_4*N_10_4, N_10_4);
    fprintf('     - Inputs: %d (2 inputs × %d steps)\n\n', n_inputs_10_4*N_10_4, N_10_4);
    
    %% Objective function
    fprintf('Step 4: Building objective function...\n');
    
    % Objective: sum (lambda_i - lambda_f)^2 + q1*pc_i^2 + q2*ec_i^2
    
    objective_fun = @(z) objective_10_4(z, N_10_4, n_states_10_4, n_inputs_10_4, ...
                                        lambda_f_10_4, q1, q2);
    
    fprintf('   Objective function defined\n\n');
    
    %% Equality constraints (dynamics)
    fprintf('Step 5: Building equality constraints (dynamics)...\n');
    
    [Aeq_10_4, beq_10_4] = build_equality_constraints_10_4(A_10_4, B_10_4, ...
                                                            N_10_4, n_states_10_4, ...
                                                            n_inputs_10_4, x0_10_4);
    
    fprintf('   Equality constraints built: %d constraints\n\n', size(Aeq_10_4, 1));
    
    %% Inequality constraints
    fprintf('Step 6: Building inequality constraints...\n');
    
    % Linear inequality constraints (pitch bounds)
    [Ain_10_4, bin_10_4] = build_linear_inequality_10_4(N_10_4, n_states_10_4, ...
                                                         n_inputs_10_4, p_max_10_4);
    
    % Nonlinear inequality constraints (elevation)
    nonlcon = @(z) nonlinear_constraints_10_4(z, N_10_4, n_states_10_4, ...
                                               alpha, beta, lambda_t);
    
    fprintf('   Linear inequality constraints: %d (pitch bounds)\n', size(Ain_10_4, 1));
    fprintf('   Nonlinear constraints: %d (elevation obstacle)\n\n', N_10_4);
    
    %% Initial guess
    fprintf('Step 7: Creating initial guess...\n');
    
    % Simple initial guess: linear interpolation for states, zeros for inputs
    z0_10_4 = zeros(n_variables_10_4, 1);
    
    for i = 1:N_10_4
        % Linear interpolation for lambda
        lambda_interp = lambda_0_10_4 + (lambda_f_10_4 - lambda_0_10_4) * i / N_10_4;
        
        % State indices
        state_idx = (i-1)*n_states_10_4 + (1:n_states_10_4);
        z0_10_4(state_idx) = [lambda_interp; 0; 0; 0; 0; 0];
        
        % Input indices - start with zeros
        % (will be filled by optimizer)
    end
    
    fprintf('   Initial guess created\n\n');
    
    %% Solve optimization problem with fmincon
    fprintf('Step 8: Solving optimization problem with fmincon...\n');
    fprintf('   This may take a while...\n\n');
    
    options_fmincon = optimoptions('fmincon', ...
        'Display', 'iter', ...
        'Algorithm', 'sqp', ...
        'MaxIterations', 1000, ...
        'MaxFunctionEvaluations', 10000, ...
        'ConstraintTolerance', 1e-6, ...
        'OptimalityTolerance', 1e-6);
    
    tic;
    [z_opt_10_4, fval_10_4, exitflag_10_4, output_10_4] = fmincon(...
        objective_fun, z0_10_4, Ain_10_4, bin_10_4, Aeq_10_4, beq_10_4, ...
        [], [], nonlcon, options_fmincon);
    solve_time_10_4 = toc;
    
    fprintf('\n   Optimization completed in %.2f seconds\n', solve_time_10_4);
    fprintf('   Exit flag: %d\n', exitflag_10_4);
    
    if exitflag_10_4 <= 0
        warning('Optimization did not converge properly!');
        fprintf('   Message: %s\n', output_10_4.message);
    else
        fprintf('   Optimization successful!\n');
    end
    fprintf('\n');
    
    %% Extract results
    fprintf('Step 9: Extracting optimal trajectory...\n');
    
    % Extract states
    x_opt_10_4 = reshape(z_opt_10_4(1:n_states_10_4*N_10_4), n_states_10_4, N_10_4);
    
    % Extract inputs
    u_opt_10_4 = reshape(z_opt_10_4(n_states_10_4*N_10_4+1:end), n_inputs_10_4, N_10_4);
    
    % Add initial state
    x_traj_10_4 = [x0_10_4, x_opt_10_4];
    
    % Time vectors
    t_10_4 = 0:dt_10_4:(N_10_4*dt_10_4);
    t_u_10_4 = 0:dt_10_4:((N_10_4-1)*dt_10_4);
    
    fprintf('   Trajectory extracted\n');
    fprintf('   Final lambda: %.4f rad (%.2f deg)\n', ...
        x_traj_10_4(1,end), x_traj_10_4(1,end)*180/pi);
    fprintf('   Final elevation: %.4f rad (%.2f deg)\n', ...
        x_traj_10_4(5,end), x_traj_10_4(5,end)*180/pi);
    fprintf('\n');
    
    %% Design LQ controller for Section 10.4
    fprintf('Step 10: Designing LQ controller for elevation system...\n');
    
    % Q matrix (6 states)
    Q_10_4 = diag([1, 0, 0, 0, 0, 0]);  % Penalize travel angle deviation
    
    % R matrix (2 inputs)
    R_10_4 = eye(2);
    
    % Calculate LQ gain
    [K_lq_10_4, S_10_4, e_10_4] = dlqr(A_10_4, B_10_4, Q_10_4, R_10_4);
    
    fprintf('   LQ gain matrix K calculated (2x6)\n');
    fprintf('   Closed-loop eigenvalues: ');
    fprintf('%.4f ', e_10_4);
    fprintf('\n\n');
    
    %% Prepare for Simulink
    fprintf('Step 11: Preparing data for Simulink...\n');
    
    % Add padding
    padding_steps_10_4 = round(5 / dt_10_4);
    
    u_opt_padded_10_4 = [zeros(n_inputs_10_4, padding_steps_10_4), ...
                         u_opt_10_4, ...
                         repmat(u_opt_10_4(:,end), 1, padding_steps_10_4)];
    
    x_opt_padded_10_4 = [repmat(x0_10_4, 1, padding_steps_10_4), ...
                         x_opt_10_4, ...
                         repmat(x_opt_10_4(:,end), 1, padding_steps_10_4)];
    
    t_padded_10_4 = (0:size(u_opt_padded_10_4, 2)-1) * dt_10_4;
    t_x_padded_10_4 = (0:size(x_opt_padded_10_4, 2)-1) * dt_10_4;
    
    % Create timeseries (need to transpose for timeseries format)
    u_opt_timeseries_10_4 = timeseries(u_opt_padded_10_4', t_padded_10_4);
    u_opt_timeseries_10_4.Name = 'u_opt_10_4';
    
    x_opt_timeseries_10_4 = timeseries(x_opt_padded_10_4', t_x_padded_10_4);
    x_opt_timeseries_10_4.Name = 'x_opt_10_4';
    
    % Save results
    filename_10_4 = 'section_10_4_data.mat';
    save(filename_10_4, 'K_lq_10_4', 'Q_10_4', 'R_10_4', ...
         'u_opt_10_4', 'x_opt_10_4', 't_10_4', 't_u_10_4', ...
         'u_opt_padded_10_4', 'x_opt_padded_10_4', ...
         't_padded_10_4', 't_x_padded_10_4', ...
         'u_opt_timeseries_10_4', 'x_opt_timeseries_10_4', ...
         'A_10_4', 'B_10_4', 'dt_10_4', 'N_10_4', ...
         'alpha', 'beta', 'lambda_t');
    
    fprintf('   Saved to: %s\n\n', filename_10_4);
    
    %% Visualize results
    fprintf('Step 12: Visualizing results...\n\n');
    
    figure('Name', 'Section 10.4: Optimal Trajectory with Elevation', ...
           'Position', [150, 100, 1200, 800]);
    
    % Travel angle
    subplot(3,2,1)
    plot(t_10_4, x_traj_10_4(1,:)*180/pi, 'b-', 'LineWidth', 1.5)
    hold on
    plot([0, t_10_4(end)], [lambda_f_10_4, lambda_f_10_4]*180/pi, 'r--')
    grid on
    ylabel('Travel \lambda [deg]')
    title('Travel Angle')
    legend('\lambda', '\lambda_f')
    
    % Elevation
    subplot(3,2,2)
    plot(t_10_4, x_traj_10_4(5,:)*180/pi, 'b-', 'LineWidth', 1.5)
    hold on
    % Plot elevation constraint
    lambda_plot = x_traj_10_4(1,:);
    e_constraint = alpha * exp(-beta * (lambda_plot - lambda_t).^2);
    plot(t_10_4, e_constraint*180/pi, 'r--', 'LineWidth', 1)
    grid on
    ylabel('Elevation e [deg]')
    title('Elevation with Constraint')
    legend('e', 'Constraint')
    
    % Pitch
    subplot(3,2,3)
    plot(t_10_4, x_traj_10_4(3,:)*180/pi, 'b-', 'LineWidth', 1.5)
    hold on
    plot([0, t_10_4(end)], [p_max_10_4, p_max_10_4]*180/pi, 'r--')
    plot([0, t_10_4(end)], [-p_max_10_4, -p_max_10_4]*180/pi, 'r--')
    grid on
    ylabel('Pitch p [deg]')
    title('Pitch Angle')
    legend('p', 'Constraints')
    
    % Travel rate
    subplot(3,2,4)
    plot(t_10_4, x_traj_10_4(2,:)*180/pi, 'b-', 'LineWidth', 1.5)
    grid on
    ylabel('Travel rate r [deg/s]')
    title('Travel Rate')
    
    % Inputs
    subplot(3,2,5)
    stairs(t_u_10_4, u_opt_10_4(1,:)*180/pi, 'b-', 'LineWidth', 1.5)
    grid on
    xlabel('Time [s]')
    ylabel('Pitch ref p_c [deg]')
    title('Pitch Reference Input')
    
    subplot(3,2,6)
    stairs(t_u_10_4, u_opt_10_4(2,:)*180/pi, 'b-', 'LineWidth', 1.5)
    grid on
    xlabel('Time [s]')
    ylabel('Elevation ref e_c [deg]')
    title('Elevation Reference Input')
    
    % 3D trajectory plot
    figure('Name', 'Section 10.4: 3D Trajectory', 'Position', [200, 150, 800, 600]);
    plot3(x_traj_10_4(1,:)*180/pi, ...
          x_traj_10_4(5,:)*180/pi, ...
          t_10_4, 'b-', 'LineWidth', 2)
    hold on
    
    % Plot constraint surface
    lambda_surf = linspace(0, pi, 50);
    e_surf = alpha * exp(-beta * (lambda_surf - lambda_t).^2);
    [Lambda_surf, T_surf] = meshgrid(lambda_surf*180/pi, t_10_4);
    E_surf = repmat(e_surf*180/pi, length(t_10_4), 1);
    surf(Lambda_surf, E_surf, T_surf, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', 'r')
    
    grid on
    xlabel('Travel \lambda [deg]')
    ylabel('Elevation e [deg]')
    zlabel('Time [s]')
    title('3D Trajectory (Travel vs Elevation vs Time)')
    view(45, 30)
    legend('Trajectory', 'Elevation Constraint')
    
    fprintf('   Section 10.4 complete!\n\n');
    
end

%% ========================================================================
%  SUMMARY
% ========================================================================

fprintf('========================================================================\n');
fprintf('SUMMARY\n');
fprintf('========================================================================\n\n');

if RUN_SECTION_10_3
    fprintf('Section 10.3: COMPLETED\n');
    fprintf('  - LQ controller designed\n');
    fprintf('  - Data saved to: section_10_3_data.mat\n');
    fprintf('  - Ready for Simulink implementation\n');
    fprintf('  - Feedback law: u = u* - K*(x - x*)\n\n');
end

if RUN_SECTION_10_4
    fprintf('Section 10.4: COMPLETED\n');
    fprintf('  - Optimal trajectory with elevation calculated\n');
    fprintf('  - Elevation constraint satisfied\n');
    fprintf('  - LQ controller designed (6 states, 2 inputs)\n');
    fprintf('  - Data saved to: section_10_4_data.mat\n');
    fprintf('  - Ready for Simulink implementation\n\n');
end

fprintf('Next steps:\n');
fprintf('1. Implement feedback controllers in Simulink\n');
fprintf('2. Test at the lab\n');
fprintf('3. Compare open-loop (10.2) vs closed-loop (10.3) performance\n');
fprintf('4. Test elevation control (10.4)\n\n');

fprintf('Remember to update K values from your init.m file!\n');
fprintf('========================================================================\n\n');

%% ========================================================================
%  HELPER FUNCTIONS
% ========================================================================

function cost = objective_10_4(z, N, n_states, n_inputs, lambda_f, q1, q2)
    % Objective function for Section 10.4
    % Minimize: sum (lambda_i - lambda_f)^2 + q1*pc_i^2 + q2*ec_i^2
    
    cost = 0;
    
    for i = 1:N
        % Extract lambda_i (first state of x_i)
        lambda_idx = (i-1)*n_states + 1;
        lambda_i = z(lambda_idx);
        
        % Cost on lambda deviation
        cost = cost + (lambda_i - lambda_f)^2;
        
        % Extract inputs u_{i-1}
        u_idx_base = n_states*N + (i-1)*n_inputs;
        pc_i = z(u_idx_base + 1);  % First input
        ec_i = z(u_idx_base + 2);  % Second input
        
        % Cost on inputs
        cost = cost + q1*pc_i^2 + q2*ec_i^2;
    end
end

function [Aeq, beq] = build_equality_constraints_10_4(A, B, N, n_states, n_inputs, x0)
    % Build equality constraints for dynamics: x_{k+1} = A*x_k + B*u_k
    
    n_eq = n_states * N;
    n_vars = n_states*N + n_inputs*N;
    
    Aeq = zeros(n_eq, n_vars);
    beq = zeros(n_eq, 1);
    
    % First constraint: x_1 = A*x_0 + B*u_0
    eq_idx = 1:n_states;
    Aeq(eq_idx, 1:n_states) = eye(n_states);
    Aeq(eq_idx, n_states*N + (1:n_inputs)) = -B;
    beq(eq_idx) = A * x0;
    
    % Remaining constraints
    for k = 1:(N-1)
        eq_idx = k*n_states + (1:n_states);
        x_k_idx = (k-1)*n_states + (1:n_states);
        x_kp1_idx = k*n_states + (1:n_states);
        u_k_idx = n_states*N + k*n_inputs + (1:n_inputs);
        
        Aeq(eq_idx, x_kp1_idx) = eye(n_states);
        Aeq(eq_idx, x_k_idx) = -A;
        Aeq(eq_idx, u_k_idx) = -B;
    end
end

function [Ain, bin] = build_linear_inequality_10_4(N, n_states, n_inputs, p_max)
    % Build linear inequality constraints for pitch: |p| <= p_max
    
    n_ineq = 2 * N;
    n_vars = n_states*N + n_inputs*N;
    
    Ain = zeros(n_ineq, n_vars);
    bin = zeros(n_ineq, 1);
    
    for k = 1:N
        % Pitch is the 3rd state
        p_idx = (k-1)*n_states + 3;
        
        % p <= p_max
        Ain(2*k-1, p_idx) = 1;
        bin(2*k-1) = p_max;
        
        % -p <= p_max
        Ain(2*k, p_idx) = -1;
        bin(2*k) = p_max;
    end
end

function [c, ceq] = nonlinear_constraints_10_4(z, N, n_states, alpha, beta, lambda_t)
    % Nonlinear inequality constraints for elevation
    % Constraint: e >= alpha * exp(-beta*(lambda - lambda_t)^2)
    % Formulated as: -e + alpha*exp(-beta*(lambda-lambda_t)^2) <= 0
    
    c = zeros(N, 1);
    ceq = [];
    
    for k = 1:N
        % Extract lambda_k and e_k
        lambda_idx = (k-1)*n_states + 1;
        e_idx = (k-1)*n_states + 5;
        
        lambda_k = z(lambda_idx);
        e_k = z(e_idx);
        
        % Constraint: alpha*exp(-beta*(lambda-lambda_t)^2) - e <= 0
        c(k) = alpha * exp(-beta * (lambda_k - lambda_t)^2) - e_k;
    end
end
