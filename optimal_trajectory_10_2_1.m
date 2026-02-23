% TTK4135 Helicopter Lab - Section 10.2.1, Task 3
% Optimal Control of Pitch/Travel without Feedback
% Calculate optimal trajectory using quadprog

clear; clc; close all;

%% Parameters
% Time parameters
dt = 0.25;          % Sampling time [s]
N = 100;            % Horizon length

% Initial and final states
lambda_0 = pi;      % Initial travel angle [rad]
lambda_f = 0;       % Final travel angle [rad]

% State: x = [lambda, r, p, p_dot]'
% where lambda = travel, r = travel rate, p = pitch, p_dot = pitch rate

% Initial state
x0 = [lambda_0; 0; 0; 0];

% Final state
xf = [lambda_f; 0; 0; 0];

% Pitch constraint
p_max = 60 * pi/180;  % Maximum pitch angle [rad]

% Weight on input (try different values)
q_values = [0.12, 1.2, 12];

%% System parameters (from Table 1 and equations in Section 8)
% These should match your helicopter - adjust if needed
% You may need to load these from init.m file instead

% From model equations (11b) and (11d):
% p_ddot + K1*Kpd*p_dot + K1*Kpp*p = K1*Kpp*pc
% r_dot = -K2*p
% lambda_dot = r

%% Eksempel verdier fra sim-filer. ER herfra ->
%% Physical constants
m_h = 0.4; % Total mass of the motors.
m_g = 0.03; % Effective mass of the helicopter.
l_a = 0.65; % Distance from elevation axis to helicopter body
l_h = 0.17; % Distance from pitch axis to motor

% Moments of inertia
J_e = 2 * m_h * l_a *l_a;         % Moment of interia for elevation
J_p = 2 * ( m_h/2 * l_h * l_h);   % Moment of interia for pitch
J_t = 2 * m_h * l_a *l_a;         % Moment of interia for travel

% Identified voltage sum and difference
V_s_eq = 7.88;%87;% Identified equilibrium voltage sum.
V_d_eq = 0.1; % % Identified equilibrium voltage difference.

% Model parameters
Kp = m_g*9.81; % Force to lift the helicopter from the ground.
Kf = Kp/V_s_eq; % Force motor constant.
K1 = l_h*Kf/J_p;
K2 = Kp*l_a/J_t;
K3 = Kf*l_a/J_e;
K4 = Kp*l_a/J_e;

%% Pitch closed loop syntesis
% Controller parameters
w_p = 1.8; % Pitch controller bandwidth.
d_p = 1.0; % Pitch controller rel. damping.
Kpp = w_p^2/K1;
Kpd = 2*d_p*sqrt(Kpp/K1);
Vd_ff = V_d_eq;

%% -> Og til hit

% Note: You should load actual values like this:
% run('init.m');  % This loads the actual helicopter parameters
% Then use the actual K1, K2, Kpp, Kpd values

%% Continuous-time state space model
% State: x = [lambda, r, p, p_dot]'
% Input: u = pc (pitch reference)

% From equations (11b), (11c), (11d):
% lambda_dot = r
% r_dot = -K2*p
% p_dot = p_dot
% p_ddot = -K1*Kpd*p_dot - K1*Kpp*p + K1*Kpp*pc

Ac = [0,  1,      0,           0;
      0,  0,     -K2,          0;
      0,  0,      0,           1;
      0,  0,  -K1*Kpp,  -K1*Kpd];

Bc = [0;
      0;
      0;
      K1*Kpp];

%% Discretization using Forward Euler
% x_{k+1} = A*x_k + B*u_k
A = eye(4) + dt * Ac;
B = dt * Bc;

%% Formulate QP problem
% Decision variables: z = [x_1, x_2, ..., x_N, u_0, u_1, ..., u_{N-1}]'
% Total variables: 4*N (states) + N (inputs) = 5*N

n_states = 4;
n_inputs = 1;
n_variables = n_states * N + n_inputs * N;

% Objective function: min 0.5*z'*H*z + f'*z
% phi = sum_{i=0}^{N-1} (lambda_{i+1} - lambda_f)^2 + q*pc_i^2

%% Loop through different q values
for q_idx = 1:length(q_values)
    q = q_values(q_idx);
    
    fprintf('\n=== Running optimization with q = %.2f ===\n', q);
    
    %% Build objective function matrices H and f
    H = zeros(n_variables, n_variables);
    f = zeros(n_variables, 1);
    
    % Quadratic terms for states (lambda deviations)
    for i = 1:N
        % Index for lambda in state x_i
        lambda_idx = (i-1)*n_states + 1;
        H(lambda_idx, lambda_idx) = 2;  % Coefficient for (lambda_i - lambda_f)^2
        f(lambda_idx) = -2*lambda_f;     % Linear term from expanding square
    end
    
    % Quadratic terms for inputs (pc)
    for i = 1:N
        % Index for u_{i-1} = pc_{i-1}
        u_idx = n_states*N + i;
        H(u_idx, u_idx) = 2*q;  % Coefficient for q*pc_i^2
    end
    
    %% Equality constraints: Aeq*z = beq
    % Model dynamics: x_{k+1} = A*x_k + B*u_k
    % Rearranged: -x_{k+1} + A*x_k + B*u_k = 0
    % And: x_1 = A*x_0 + B*u_0 (initial condition)
    
    n_eq = n_states * N;  % N dynamic constraints (each with 4 equations)
    Aeq = zeros(n_eq, n_variables);
    beq = zeros(n_eq, 1);
    
    % First constraint: x_1 = A*x_0 + B*u_0
    eq_idx = 1:n_states;
    Aeq(eq_idx, 1:n_states) = eye(n_states);  % x_1
    Aeq(eq_idx, n_states*N + 1) = -B;         % -B*u_0
    beq(eq_idx) = A * x0;
    
    % Remaining constraints: x_{k+1} = A*x_k + B*u_k for k=1 to N-1
    for k = 1:(N-1)
        eq_idx = k*n_states + (1:n_states);
        state_idx_k = (k-1)*n_states + (1:n_states);
        state_idx_kp1 = k*n_states + (1:n_states);
        u_idx_k = n_states*N + k;
        
        Aeq(eq_idx, state_idx_kp1) = eye(n_states);  % x_{k+1}
        Aeq(eq_idx, state_idx_k) = -A;               % -A*x_k
        Aeq(eq_idx, u_idx_k) = -B;                   % -B*u_k
    end
    
    %% Inequality constraints: Ain*z <= bin
    % Pitch constraint: -p_max <= p_k <= p_max for k=1 to N
    % This gives: p_k <= p_max and -p_k <= p_max
    
    n_ineq = 2 * N;  % Two constraints per time step
    Ain = zeros(n_ineq, n_variables);
    bin = zeros(n_ineq, 1);
    
    for k = 1:N
        % Index for p in state x_k
        p_idx = (k-1)*n_states + 3;  % p is the 3rd state
        
        % Constraint: p_k <= p_max
        Ain(2*k-1, p_idx) = 1;
        bin(2*k-1) = p_max;
        
        % Constraint: -p_k <= p_max (i.e., p_k >= -p_max)
        Ain(2*k, p_idx) = -1;
        bin(2*k) = p_max;
    end
    
    %% Solve QP problem
    options = optimset('Display', 'iter', 'Algorithm', 'interior-point-convex');
    
    [z_opt, fval, exitflag, output] = quadprog(H, f, Ain, bin, Aeq, beq, ...
        [], [], [], options);
    
    if exitflag <= 0
        warning('Optimization did not converge properly for q = %.2f', q);
        continue;
    end
    
    %% Extract optimal trajectory
    % Extract states
    x_opt = reshape(z_opt(1:n_states*N), n_states, N);
    
    % Extract inputs
    u_opt = z_opt(n_states*N+1:end);
    
    % Add initial state at the beginning
    x_traj = [x0, x_opt];
    
    % Create time vector
    t = 0:dt:(N*dt);
    t_u = 0:dt:((N-1)*dt);
    
    %% Plot results
    figure('Name', sprintf('Optimal Trajectory q=%.2f', q));
    
    subplot(3,1,1)
    plot(t, x_traj(1,:)*180/pi, 'LineWidth', 1.5)
    hold on
    plot([0, N*dt], [lambda_f, lambda_f]*180/pi, 'r--', 'LineWidth', 1)
    grid on
    ylabel('Travel \lambda [deg]')
    title(sprintf('Optimal Trajectory (q = %.2f)', q))
    legend('\lambda', '\lambda_f', 'Location', 'best')
    
    subplot(3,1,2)
    plot(t, x_traj(3,:)*180/pi, 'LineWidth', 1.5)
    hold on
    plot([0, N*dt], [p_max, p_max]*180/pi, 'r--', 'LineWidth', 1)
    plot([0, N*dt], [-p_max, -p_max]*180/pi, 'r--', 'LineWidth', 1)
    grid on
    ylabel('Pitch p [deg]')
    legend('p', 'Constraints', 'Location', 'best')
    
    subplot(3,1,3)
    stairs(t_u, u_opt*180/pi, 'LineWidth', 1.5)
    grid on
    xlabel('Time [s]')
    ylabel('Pitch reference p_c [deg]')
    legend('p_c', 'Location', 'best')
    
    %% Display some statistics
    fprintf('\nOptimization results for q = %.2f:\n', q);
    fprintf('  Final lambda: %.4f rad (%.2f deg)\n', ...
        x_traj(1,end), x_traj(1,end)*180/pi);
    fprintf('  Final lambda error: %.4f rad (%.2f deg)\n', ...
        x_traj(1,end)-lambda_f, (x_traj(1,end)-lambda_f)*180/pi);
    fprintf('  Max pitch: %.4f rad (%.2f deg)\n', ...
        max(abs(x_traj(3,:))), max(abs(x_traj(3,:)))*180/pi);
    fprintf('  Max input: %.4f rad (%.2f deg)\n', ...
        max(abs(u_opt)), max(abs(u_opt))*180/pi);
    fprintf('  Objective value: %.4f\n', fval);
    
    %% Save results for use in Simulink
    % Add padding (5 seconds before and after)
    padding_time = 5;  % seconds
    padding_steps = round(padding_time / dt);
    
    % Padded input sequence
    u_padded = [zeros(padding_steps, 1); u_opt; zeros(padding_steps, 1)];
    
    % Padded state trajectory (for feedback in later tasks)
    x_padded = [repmat(x0, 1, padding_steps), x_opt, ...
                repmat(x_opt(:,end), 1, padding_steps)];
    
    % Time vectors
    t_padded = (0:length(u_padded)-1) * dt;
    t_x_padded = (0:size(x_padded,2)-1) * dt;
    
    % Save to file for this q value
    filename = sprintf('optimal_trajectory_q%.2f.mat', q);
    save(filename, 'u_padded', 'x_padded', 't_padded', 't_x_padded', ...
         'dt', 'N', 'q', 'x_opt', 'u_opt', 't', 't_u');
    
    fprintf('  Saved to: %s\n', filename);
end

fprintf('\n=== All optimizations complete ===\n');
fprintf('Remember to adjust K1, K2, Kpp, Kpd based on your init.m file!\n');

%% Comments on the results
fprintf('\n=== Discussion Points ===\n');
fprintf('1. Compare the three plots with different q values\n');
fprintf('2. Small q: Less penalty on input → More aggressive control\n');
fprintf('3. Large q: More penalty on input → Smoother, slower control\n');
fprintf('4. The pitch constraint prevents too aggressive maneuvers\n');
fprintf('5. Trade-off between speed and control effort\n');
