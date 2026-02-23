% ========================================================================
% STEP-BY-STEP GUIDE: Sections 10.3 & 10.4
% TTK4135 Helicopter Lab
% ========================================================================
% This guide walks you through implementing LQ feedback control and
% elevation control for the helicopter lab.
%
% Created: 2025
% ========================================================================

fprintf('╔════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                                                                    ║\n');
fprintf('║     TTK4135 HELICOPTER LAB - SECTIONS 10.3 & 10.4 GUIDE          ║\n');
fprintf('║                                                                    ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%  TABLE OF CONTENTS
% ========================================================================

fprintf('TABLE OF CONTENTS:\n');
fprintf('──────────────────────────────────────────────────────────────────\n');
fprintf('PART A: SECTION 10.3 - LQ FEEDBACK CONTROL\n');
fprintf('  1. Overview and Theory\n');
fprintf('  2. Pre-work Tasks (before lab)\n');
fprintf('  3. MATLAB Implementation\n');
fprintf('  4. Simulink Setup\n');
fprintf('  5. Lab Procedure\n');
fprintf('  6. Analysis and Report\n\n');
fprintf('PART B: SECTION 10.4 - ELEVATION CONTROL\n');
fprintf('  7. Overview and Theory\n');
fprintf('  8. Pre-work Tasks (before lab)\n');
fprintf('  9. MATLAB Implementation\n');
fprintf('  10. Simulink Setup\n');
fprintf('  11. Lab Procedure\n');
fprintf('  12. Analysis and Report\n\n');
fprintf('──────────────────────────────────────────────────────────────────\n\n');

%% ========================================================================
%  PART A: SECTION 10.3 - LQ FEEDBACK CONTROL
% ========================================================================

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════════╗\n');
fprintf('║               PART A: SECTION 10.3 - LQ FEEDBACK                   ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════╝\n\n');

fprintf('========================================================================\n');
fprintf('1. OVERVIEW AND THEORY (Section 10.3)\n');
fprintf('========================================================================\n\n');

fprintf('MOTIVATION:\n');
fprintf('───────────\n');
fprintf('In Section 10.2, you calculated an optimal open-loop trajectory.\n');
fprintf('However, the helicopter did NOT follow it perfectly because:\n');
fprintf('  • Model mismatch (real helicopter ≠ mathematical model)\n');
fprintf('  • Disturbances (air currents, friction)\n');
fprintf('  • Parameter uncertainty\n');
fprintf('  • No feedback to correct deviations!\n\n');

fprintf('SOLUTION: Add LQ Feedback Control\n');
fprintf('──────────────────────────────────\n');
fprintf('The control law becomes:\n');
fprintf('  u_k = u*_k - K(x_k - x*_k)\n');
fprintf('       └─┬──┘   └────┬─────┘\n');
fprintf('   Feedforward  Feedback\n\n');

fprintf('Where:\n');
fprintf('  u*_k  = Optimal input from Section 10.2\n');
fprintf('  x*_k  = Optimal state trajectory from Section 10.2\n');
fprintf('  x_k   = Measured state at time k\n');
fprintf('  K     = LQ feedback gain matrix\n\n');

fprintf('HOW IT WORKS:\n');
fprintf('  • If x_k = x*_k (on track): Apply u*_k (feedforward)\n');
fprintf('  • If x_k ≠ x*_k (deviation): Correct with -K(x_k - x*_k)\n\n');

fprintf('LQ CONTROLLER DESIGN:\n');
fprintf('─────────────────────\n');
fprintf('The gain K minimizes the infinite-horizon cost:\n');
fprintf('  J = Σ (Δx''QΔx + Δu''RΔu)\n');
fprintf('where Δx = x - x*, Δu = u - u*\n\n');

fprintf('Design choices:\n');
fprintf('  Q matrix: Weights on state deviations (4×4 diagonal)\n');
fprintf('    - Large Q_λ: Penalize travel angle errors heavily\n');
fprintf('    - Large Q_p: Penalize pitch errors heavily\n');
fprintf('  R matrix: Weight on control effort (scalar)\n');
fprintf('    - Large R: Gentle feedback (less aggressive)\n');
fprintf('    - Small R: Strong feedback (more aggressive)\n\n');

fprintf('In MATLAB: [K, S, e] = dlqr(A, B, Q, R)\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('2. PRE-WORK TASKS (Before Coming to Lab)\n');
fprintf('========================================================================\n\n');

fprintf('TASK 10.3.1: Calculate the LQ Gain Matrix K\n');
fprintf('────────────────────────────────────────────\n\n');

fprintf('Step 1: Use the same model as Section 10.2\n');
fprintf('───────────────────────────────────────────\n');
fprintf('State: x = [λ, r, p, ṗ]''  (4 states)\n');
fprintf('Input: u = pc             (1 input)\n\n');

fprintf('Continuous model:\n');
fprintf('  ẋ = Ac·x + Bc·u\n\n');

fprintf('Discrete model (Forward Euler, Δt = 0.25s):\n');
fprintf('  x_{k+1} = A·x_k + B·u_k\n');
fprintf('  A = I + Δt·Ac\n');
fprintf('  B = Δt·Bc\n\n');

fprintf('Step 2: Design Q and R matrices\n');
fprintf('───────────────────────────────\n');
fprintf('Start with these values (you can tune them):\n\n');

fprintf('  Q = diag([1, 0, 0, 0])    %% Penalize λ deviation only\n');
fprintf('  R = 1                     %% Moderate control effort\n\n');

fprintf('Try different values and see how they affect:\n');
fprintf('  • Convergence speed\n');
fprintf('  • Smoothness of control\n');
fprintf('  • Robustness to disturbances\n\n');

fprintf('Step 3: Calculate K using dlqr\n');
fprintf('──────────────────────────────\n');
fprintf('  [K, S, e] = dlqr(A, B, Q, R);\n\n');

fprintf('Check:\n');
fprintf('  • K should be a 1×4 matrix (1 input, 4 states)\n');
fprintf('  • All eigenvalues e should be inside unit circle (|e| < 1)\n');
fprintf('  • If unstable, adjust Q and R\n\n');

fprintf('Step 4: Prepare data for Simulink\n');
fprintf('──────────────────────────────────\n');
fprintf('You need:\n');
fprintf('  • K (feedback gain)\n');
fprintf('  • u* (optimal input trajectory from 10.2)\n');
fprintf('  • x* (optimal state trajectory from 10.2)\n\n');

fprintf('Save as timeseries:\n');
fprintf('  u_opt_ts = timeseries(u_padded, t_padded);\n');
fprintf('  x_opt_ts = timeseries(x_padded'', t_x_padded);\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('3. MATLAB IMPLEMENTATION (Section 10.3)\n');
fprintf('========================================================================\n\n');

fprintf('RUNNING THE CODE:\n');
fprintf('─────────────────\n');
fprintf('1. Make sure you completed Section 10.2 and have:\n');
fprintf('   optimal_trajectory_q1.20.mat\n\n');

fprintf('2. Update K values in sections_10_3_and_10_4.m:\n');
fprintf('   Run your init.m and copy the values for:\n');
fprintf('   K1, K2, Kpp, Kpd\n\n');

fprintf('3. Run the script:\n');
fprintf('   >> sections_10_3_and_10_4\n\n');

fprintf('4. Set RUN_SECTION_10_3 = true at the top\n\n');

fprintf('WHAT THE SCRIPT DOES:\n');
fprintf('─────────────────────\n');
fprintf('  1. Loads optimal trajectory from Section 10.2\n');
fprintf('  2. Builds the state-space model\n');
fprintf('  3. Designs LQ controller using dlqr\n');
fprintf('  4. Saves K, u*, x* to section_10_3_data.mat\n');
fprintf('  5. Creates timeseries for Simulink\n');
fprintf('  6. Plots the results\n\n');

fprintf('OUTPUT FILES:\n');
fprintf('─────────────\n');
fprintf('  section_10_3_data.mat containing:\n');
fprintf('    - K_lq: Feedback gain matrix (1×4)\n');
fprintf('    - u_opt_timeseries_10_3: Optimal input u*\n');
fprintf('    - x_opt_timeseries_10_3: Optimal states x*\n');
fprintf('    - Q_10_3, R_10_3: LQ weights\n');
fprintf('    - A_10_3, B_10_3: System matrices\n\n');

fprintf('TUNING Q AND R:\n');
fprintf('───────────────\n');
fprintf('Try different values and observe:\n\n');

fprintf('Example 1: Q = diag([1, 0, 0, 0]), R = 1\n');
fprintf('  → Moderate feedback on travel angle\n\n');

fprintf('Example 2: Q = diag([10, 0, 0, 0]), R = 1\n');
fprintf('  → Stronger feedback, faster correction\n\n');

fprintf('Example 3: Q = diag([1, 0, 0, 0]), R = 10\n');
fprintf('  → Gentler feedback, slower correction\n\n');

fprintf('Example 4: Q = diag([1, 1, 1, 1]), R = 1\n');
fprintf('  → Feedback on all states\n\n');

fprintf('Choose values that give:\n');
fprintf('  ✓ Stable closed-loop (all |eigenvalues| < 1)\n');
fprintf('  ✓ Good tracking of x*\n');
fprintf('  ✓ Reasonable control effort\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('4. SIMULINK SETUP (Section 10.3)\n');
fprintf('========================================================================\n\n');

fprintf('BLOCK DIAGRAM STRUCTURE:\n');
fprintf('────────────────────────\n\n');

fprintf('  [x_opt_ts] ────┐\n');
fprintf('                 │\n');
fprintf('  [x_measured]───┴─► [Subtract] ──► [K] ──┐\n');
fprintf('                       (x - x*)            │\n');
fprintf('                                           │ -K(x-x*)\n');
fprintf('  [u_opt_ts] ────────────────────────┬────┴─► [Sum] ──► pc ──► [PD Controller]\n');
fprintf('                                     │         u* - K(x-x*)\n');
fprintf('                                     └─ u*\n\n');

fprintf('STEP-BY-STEP SIMULINK IMPLEMENTATION:\n');
fprintf('──────────────────────────────────────\n\n');

fprintf('Step 1: Load workspace variables\n');
fprintf('─────────────────────────────────\n');
fprintf('In MATLAB command window:\n');
fprintf('  >> load(''section_10_3_data.mat'')\n\n');

fprintf('Verify variables are loaded:\n');
fprintf('  >> whos K_lq u_opt_timeseries_10_3 x_opt_timeseries_10_3\n\n');

fprintf('Step 2: Open your helicopter.slx from Section 10.2\n');
fprintf('──────────────────────────────────────────────────\n');
fprintf('Start from your working Section 10.2 model.\n');
fprintf('Save a copy as helicopter_10_3.slx\n\n');

fprintf('Step 3: Add blocks for state reference x*\n');
fprintf('──────────────────────────────────────────\n');
fprintf('Add a new "From Workspace" block:\n');
fprintf('  Block: Simulink > Sources > From Workspace\n');
fprintf('  Name it: "x_opt_reference"\n');
fprintf('  Settings:\n');
fprintf('    Data: x_opt_timeseries_10_3\n');
fprintf('    Sample time: 0\n');
fprintf('    Interpolation: Zero-order hold\n\n');

fprintf('This outputs a 4-element vector: [λ*, r*, p*, ṗ*]''\n\n');

fprintf('Step 4: Add Demux for x_opt\n');
fprintf('───────────────────────────\n');
fprintf('Add "Demux" block after x_opt_reference:\n');
fprintf('  Block: Simulink > Signal Routing > Demux\n');
fprintf('  Number of outputs: 4\n\n');

fprintf('Step 5: Build measured state vector x\n');
fprintf('──────────────────────────────────────\n');
fprintf('Use a "Mux" block to combine measurements:\n');
fprintf('  Block: Simulink > Signal Routing > Mux\n');
fprintf('  Number of inputs: 4\n');
fprintf('  Connect:\n');
fprintf('    Input 1: λ (travel) measurement\n');
fprintf('    Input 2: r (travel rate) measurement\n');
fprintf('    Input 3: p (pitch) measurement\n');
fprintf('    Input 4: ṗ (pitch rate) measurement\n\n');

fprintf('IMPORTANT: Make sure units match!\n');
fprintf('  • Measurements might be in degrees\n');
fprintf('  • K expects radians\n');
fprintf('  • Add Gain blocks (π/180) if needed\n\n');

fprintf('Step 6: Calculate deviation Δx = x - x*\n');
fprintf('────────────────────────────────────────\n');
fprintf('Add a "Subtract" block:\n');
fprintf('  Block: Simulink > Math Operations > Subtract\n');
fprintf('  Connect:\n');
fprintf('    Input 1 (+): x_measured (from Mux)\n');
fprintf('    Input 2 (-): x_opt (from Demux)\n');
fprintf('  Output: Δx\n\n');

fprintf('Step 7: Apply feedback gain K\n');
fprintf('──────────────────────────────\n');
fprintf('Add "Matrix Multiply" or "Gain" block:\n');
fprintf('  Block: Simulink > Math Operations > Gain\n');
fprintf('  Gain: K_lq\n');
fprintf('  Connect input to Δx\n');
fprintf('  Output: K·Δx\n\n');

fprintf('Step 8: Negate the feedback\n');
fprintf('───────────────────────────\n');
fprintf('Add a "Gain" block with gain = -1\n');
fprintf('  Output: -K·Δx\n\n');

fprintf('Step 9: Combine feedforward and feedback\n');
fprintf('─────────────────────────────────────────\n');
fprintf('Add a "Sum" block:\n');
fprintf('  Block: Simulink > Math Operations > Sum\n');
fprintf('  List of signs: ++\n');
fprintf('  Connect:\n');
fprintf('    Input 1: u* (from u_opt_timeseries)\n');
fprintf('    Input 2: -K·Δx\n');
fprintf('  Output: u = u* - K·Δx\n\n');

fprintf('Step 10: Connect to pitch controller\n');
fprintf('─────────────────────────────────────\n');
fprintf('Connect the Sum output to pc (pitch reference)\n');
fprintf('This replaces the direct connection from 10.2\n\n');

fprintf('Step 11: Add data recording\n');
fprintf('───────────────────────────\n');
fprintf('Add "To File" blocks to record:\n');
fprintf('  • x_measured (all 4 states)\n');
fprintf('  • x_opt (reference trajectory)\n');
fprintf('  • u (control input)\n');
fprintf('  • u_opt (feedforward input)\n\n');

fprintf('Use Mux to combine signals before "To File"\n');
fprintf('Remember: Format = Array\n\n');

fprintf('Step 12: Build and verify\n');
fprintf('─────────────────────────\n');
fprintf('  1. Save your model\n');
fprintf('  2. QuaRC > Build for Monitoring\n');
fprintf('  3. Check for errors\n');
fprintf('  4. Verify signal dimensions match\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('5. LAB PROCEDURE (Section 10.3)\n');
fprintf('========================================================================\n\n');

fprintf('PREPARATION:\n');
fprintf('────────────\n');
fprintf('  □ Section 10_3_data.mat loaded in workspace\n');
fprintf('  □ Simulink model built successfully\n');
fprintf('  □ Recording blocks configured\n');
fprintf('  □ Safety check complete\n\n');

fprintf('FLIGHT PROCEDURE:\n');
fprintf('─────────────────\n');
fprintf('1. Turn ON power module\n\n');

fprintf('2. Start the system:\n');
fprintf('   QuaRC > Monitor & Tune\n\n');

fprintf('3. Observe the flight:\n');
fprintf('   • 0-5s: Rise and stabilize\n');
fprintf('   • 5-30s: Execute trajectory with feedback\n');
fprintf('   • 30-35s: Stabilize at final position\n\n');

fprintf('4. What to watch for:\n');
fprintf('   ✓ Helicopter should follow x* much better than 10.2\n');
fprintf('   ✓ Final position should be closer to target\n');
fprintf('   ✓ Smoother response, less oscillation\n\n');

fprintf('5. After flight:\n');
fprintf('   • Stop the program\n');
fprintf('   • Turn OFF power module\n');
fprintf('   • Save data immediately!\n\n');

fprintf('EXPERIMENTS TO TRY:\n');
fprintf('───────────────────\n');
fprintf('Task 10.3.2 (At the lab): Try different Q and R values\n\n');

fprintf('Suggestion 1: Q = diag([1,0,0,0]), R = 1 (baseline)\n');
fprintf('Suggestion 2: Q = diag([10,0,0,0]), R = 1 (aggressive)\n');
fprintf('Suggestion 3: Q = diag([1,0,0,0]), R = 10 (gentle)\n\n');

fprintf('For each:\n');
fprintf('  1. Recalculate K in MATLAB\n');
fprintf('  2. Reload workspace in Simulink\n');
fprintf('  3. Rebuild model\n');
fprintf('  4. Fly and record data\n');
fprintf('  5. Compare results\n\n');

fprintf('TROUBLESHOOTING:\n');
fprintf('────────────────\n');
fprintf('Problem: Helicopter oscillates wildly\n');
fprintf('  → K might be too aggressive\n');
fprintf('  → Increase R (reduce feedback gain)\n');
fprintf('  → Check sign of feedback (-K, not +K)\n\n');

fprintf('Problem: Poor tracking of x*\n');
fprintf('  → K might be too weak\n');
fprintf('  → Increase Q (penalize errors more)\n');
fprintf('  → Check units (rad vs deg)\n\n');

fprintf('Problem: "Dimension mismatch" error\n');
fprintf('  → Check Mux has 4 inputs for x\n');
fprintf('  → Check Demux has 4 outputs for x*\n');
fprintf('  → Verify K is 1×4 matrix\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('6. ANALYSIS AND REPORT (Section 10.3)\n');
fprintf('========================================================================\n\n');

fprintf('TASK 10.3.4: Justify your choice of Q and R\n');
fprintf('────────────────────────────────────────────\n\n');

fprintf('In your report, discuss:\n\n');

fprintf('1. Q and R values chosen:\n');
fprintf('   • What values did you use?\n');
fprintf('   • Why these values?\n');
fprintf('   • What other values did you try?\n\n');

fprintf('2. Closed-loop eigenvalues:\n');
fprintf('   • Are they all stable (|e| < 1)?\n');
fprintf('   • How close to unit circle?\n');
fprintf('   • What does this say about response speed?\n\n');

fprintf('3. Effect of Q:\n');
fprintf('   • Large Q → fast tracking, aggressive control\n');
fprintf('   • Small Q → slow tracking, gentle control\n\n');

fprintf('4. Effect of R:\n');
fprintf('   • Large R → gentle feedback, smooth control\n');
fprintf('   • Small R → aggressive feedback, fast response\n\n');

fprintf('TASK 10.3.5: Compare with Section 10.2\n');
fprintf('───────────────────────────────────────\n\n');

fprintf('Create comparison plots:\n\n');

fprintf('Plot 1: Travel angle λ\n');
fprintf('  • x* (predicted)\n');
fprintf('  • Section 10.2 measured (open-loop)\n');
fprintf('  • Section 10.3 measured (LQ feedback)\n');
fprintf('  • Target λf = 0\n\n');

fprintf('Plot 2: Pitch angle p\n');
fprintf('  • Compare open-loop vs closed-loop\n');
fprintf('  • Show constraint boundaries\n\n');

fprintf('Plot 3: Control input u\n');
fprintf('  • u* (feedforward)\n');
fprintf('  • u (total = u* - K·Δx)\n');
fprintf('  • Show feedback contribution\n\n');

fprintf('Discussion points:\n');
fprintf('──────────────────\n');
fprintf('• Final error in λ:\n');
fprintf('  - Section 10.2: ±5-15° typical\n');
fprintf('  - Section 10.3: <1-2° typical\n\n');

fprintf('• Tracking performance:\n');
fprintf('  - Open-loop: Drifts from x*\n');
fprintf('  - Closed-loop: Follows x* closely\n\n');

fprintf('• Robustness:\n');
fprintf('  - Open-loop: Sensitive to disturbances\n');
fprintf('  - Closed-loop: Rejects disturbances\n\n');

fprintf('• Advantages of LQ feedback:\n');
fprintf('  1. Corrects model mismatch\n');
fprintf('  2. Handles disturbances\n');
fprintf('  3. Achieves better final accuracy\n');
fprintf('  4. More robust to parameter uncertainty\n\n');

fprintf('• Disadvantages:\n');
fprintf('  1. Requires state measurements\n');
fprintf('  2. More complex implementation\n');
fprintf('  3. Tuning required (Q, R)\n\n');

fprintf('Press Enter to continue to Section 10.4...\n');
pause;

%% ========================================================================
%  PART B: SECTION 10.4 - ELEVATION CONTROL
% ========================================================================

fprintf('\n\n');
fprintf('╔════════════════════════════════════════════════════════════════════╗\n');
fprintf('║            PART B: SECTION 10.4 - ELEVATION CONTROL                ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════╝\n\n');

fprintf('========================================================================\n');
fprintf('7. OVERVIEW AND THEORY (Section 10.4)\n');
fprintf('========================================================================\n\n');

fprintf('NEW CHALLENGE:\n');
fprintf('──────────────\n');
fprintf('Now we include elevation control!\n');
fprintf('The helicopter must:\n');
fprintf('  1. Move from λ = π to λ = 0\n');
fprintf('  2. Navigate around an obstacle (elevation constraint)\n');
fprintf('  3. Use both pitch AND elevation as control inputs\n\n');

fprintf('ELEVATION CONSTRAINT:\n');
fprintf('─────────────────────\n');
fprintf('The helicopter must fly above the constraint:\n');
fprintf('  e_k ≥ α·exp(-β·(λ_k - λ_t)²)\n\n');

fprintf('where:\n');
fprintf('  α = 0.2     (height of obstacle)\n');
fprintf('  β = 20      (width of obstacle)\n');
fprintf('  λ_t = 2π/3  (location of obstacle)\n\n');

fprintf('This creates a "bump" the helicopter must fly over!\n\n');

fprintf('EXTENDED STATE SPACE:\n');
fprintf('─────────────────────\n');
fprintf('State: x = [λ, r, p, ṗ, e, ė]''  (6 states)\n');
fprintf('Input: u = [pc, ec]''            (2 inputs)\n\n');

fprintf('Additional dynamics (from Section 8):\n');
fprintf('  ë + K3·Ked·ė + K3·Kep·e = K3·Kep·ec\n\n');

fprintf('OPTIMIZATION PROBLEM:\n');
fprintf('─────────────────────\n');
fprintf('Minimize:\n');
fprintf('  φ = Σ (λ_i - λ_f)² + q1·pc²_i + q2·ec²_i\n\n');

fprintf('Subject to:\n');
fprintf('  • Dynamics: x_{k+1} = A·x_k + B·u_k\n');
fprintf('  • Pitch constraint: |p_k| ≤ 60°\n');
fprintf('  • Elevation constraint: e_k ≥ α·exp(-β·(λ_k - λ_t)²)\n\n');

fprintf('SOLVER:\n');
fprintf('───────\n');
fprintf('Use fmincon (not quadprog) because:\n');
fprintf('  • Elevation constraint is NONLINEAR\n');
fprintf('  • fmincon uses SQP algorithm\n');
fprintf('  • Can handle nonlinear constraints\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('8. PRE-WORK TASKS (Section 10.4)\n');
fprintf('========================================================================\n\n');

fprintf('TASK 10.4.1: Build and solve the optimization problem\n');
fprintf('──────────────────────────────────────────────────────\n\n');

fprintf('Step 1: Write extended state-space model\n');
fprintf('─────────────────────────────────────────\n');
fprintf('Continuous model with 6 states, 2 inputs:\n\n');

fprintf('  Ac = [0   1   0    0     0   0  ]\n');
fprintf('       [0   0  -K2   0     0   0  ]\n');
fprintf('       [0   0   0    1     0   0  ]\n');
fprintf('       [0   0  -K1Kpp -K1Kpd 0  0  ]\n');
fprintf('       [0   0   0    0     0   1  ]\n');
fprintf('       [0   0   0    0  -K3Kep -K3Ked]\n\n');

fprintf('  Bc = [0        0     ]\n');
fprintf('       [0        0     ]\n');
fprintf('       [0        0     ]\n');
fprintf('       [K1Kpp    0     ]\n');
fprintf('       [0        0     ]\n');
fprintf('       [0      K3Kep   ]\n\n');

fprintf('Step 2: Discretize (Forward Euler)\n');
fprintf('───────────────────────────────────\n');
fprintf('  A = I + Δt·Ac\n');
fprintf('  B = Δt·Bc\n\n');

fprintf('Step 3: Formulate optimization for fmincon\n');
fprintf('───────────────────────────────────────────\n');
fprintf('Decision variable z has N·6 states + N·2 inputs:\n');
fprintf('  z = [x_1, ..., x_N, u_0, ..., u_{N-1}]''\n\n');

fprintf('Objective function:\n');
fprintf('  cost = Σ(λ_i - λ_f)² + q1·pc²_i + q2·ec²_i\n\n');

fprintf('Equality constraints (dynamics):\n');
fprintf('  Aeq·z = beq\n');
fprintf('  Encodes: x_{k+1} = A·x_k + B·u_k\n\n');

fprintf('Linear inequality (pitch):\n');
fprintf('  Ain·z ≤ bin\n');
fprintf('  Encodes: -60° ≤ p_k ≤ 60°\n\n');

fprintf('Nonlinear inequality (elevation):\n');
fprintf('  c(z) ≤ 0\n');
fprintf('  c_k = α·exp(-β·(λ_k - λ_t)²) - e_k\n\n');

fprintf('Step 4: Solve with fmincon\n');
fprintf('──────────────────────────\n');
fprintf('[z_opt, fval, exitflag] = fmincon(objective, z0, ...\n');
fprintf('    Ain, bin, Aeq, beq, [], [], nonlcon, options);\n\n');

fprintf('This takes longer than quadprog!\n');
fprintf('  • Start with short horizon (N=15-20)\n');
fprintf('  • Then try N=40 as suggested\n\n');

fprintf('Step 5: Design LQ controller\n');
fprintf('────────────────────────────\n');
fprintf('Same as 10.3 but with 6 states, 2 inputs:\n');
fprintf('  Q = diag([1, 0, 0, 0, 0, 0])  (6×6)\n');
fprintf('  R = eye(2)                    (2×2)\n');
fprintf('  [K, S, e] = dlqr(A, B, Q, R);\n\n');

fprintf('K will be 2×6 matrix\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('9. MATLAB IMPLEMENTATION (Section 10.4)\n');
fprintf('========================================================================\n\n');

fprintf('RUNNING THE CODE:\n');
fprintf('─────────────────\n');
fprintf('1. Update K values in sections_10_3_and_10_4.m:\n');
fprintf('   You need: K1, K2, K3, Kpp, Kpd, Kep, Ked\n');
fprintf('   Load from your init.m file\n\n');

fprintf('2. Set RUN_SECTION_10_4 = true\n\n');

fprintf('3. Run the script:\n');
fprintf('   >> sections_10_3_and_10_4\n\n');

fprintf('4. Be patient - fmincon is slower than quadprog!\n');
fprintf('   • N=20: ~30 seconds\n');
fprintf('   • N=40: 1-3 minutes\n\n');

fprintf('WHAT THE SCRIPT DOES:\n');
fprintf('─────────────────────\n');
fprintf('  1. Builds 6-state, 2-input model\n');
fprintf('  2. Sets up optimization problem\n');
fprintf('  3. Creates objective function\n');
fprintf('  4. Builds equality constraints (dynamics)\n');
fprintf('  5. Builds linear inequality (pitch)\n');
fprintf('  6. Builds nonlinear constraint (elevation)\n');
fprintf('  7. Solves with fmincon\n');
fprintf('  8. Designs LQ controller\n');
fprintf('  9. Saves results and creates plots\n\n');

fprintf('OUTPUT FILES:\n');
fprintf('─────────────\n');
fprintf('  section_10_4_data.mat containing:\n');
fprintf('    - K_lq_10_4: Feedback gain (2×6)\n');
fprintf('    - u_opt_timeseries_10_4: Optimal inputs [pc, ec]\n');
fprintf('    - x_opt_timeseries_10_4: Optimal states\n');
fprintf('    - Constraint parameters: alpha, beta, lambda_t\n\n');

fprintf('PLOTS GENERATED:\n');
fprintf('────────────────\n');
fprintf('  Figure 1: 6 subplots showing:\n');
fprintf('    • Travel λ vs time\n');
fprintf('    • Elevation e vs time (with constraint)\n');
fprintf('    • Pitch p vs time\n');
fprintf('    • Travel rate r vs time\n');
fprintf('    • Pitch input pc vs time\n');
fprintf('    • Elevation input ec vs time\n\n');

fprintf('  Figure 2: 3D trajectory\n');
fprintf('    • (λ, e, time) in 3D space\n');
fprintf('    • Elevation constraint surface shown\n\n');

fprintf('VERIFICATION:\n');
fprintf('─────────────\n');
fprintf('Check that:\n');
fprintf('  ✓ e_k stays above constraint curve\n');
fprintf('  ✓ |p_k| ≤ 60° for all k\n');
fprintf('  ✓ λ_0 = π, λ_N ≈ 0\n');
fprintf('  ✓ Final states ≈ 0\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('10. SIMULINK SETUP (Section 10.4)\n');
fprintf('========================================================================\n\n');

fprintf('DIFFERENCES FROM 10.3:\n');
fprintf('──────────────────────\n');
fprintf('  • 6 states instead of 4\n');
fprintf('  • 2 inputs instead of 1\n');
fprintf('  • K is 2×6 instead of 1×4\n');
fprintf('  • Need to control BOTH pc and ec\n\n');

fprintf('BLOCK DIAGRAM:\n');
fprintf('──────────────\n\n');

fprintf('  [x_opt_ts] (6 states) ────┐\n');
fprintf('                            │\n');
fprintf('  [x_measured] (6 states)───┴─► [Subtract] ─► [K] (2×6) ──┐\n');
fprintf('                                 (x - x*)                  │\n');
fprintf('                                                           │ -K(x-x*)\n');
fprintf('  [u_opt_ts] (2 inputs) ─────────────────────────┬────────┴─► [Sum]\n');
fprintf('                                                 │              │\n');
fprintf('                                                 └─ u*          │\n');
fprintf('                                                                │\n');
fprintf('                         u = [pc, ec]'' = u* - K(x-x*) ────────┘\n');
fprintf('                                │\n');
fprintf('                                ├──► pc ──► [PD Controller]\n');
fprintf('                                │\n');
fprintf('                                └──► ec ──► [PID Controller]\n\n');

fprintf('IMPLEMENTATION STEPS:\n');
fprintf('─────────────────────\n\n');

fprintf('Step 1: Load data\n');
fprintf('  >> load(''section_10_4_data.mat'')\n\n');

fprintf('Step 2: Build x_measured vector (6 elements)\n');
fprintf('  Use Mux with 6 inputs:\n');
fprintf('    1. λ (travel)\n');
fprintf('    2. r (travel rate)\n');
fprintf('    3. p (pitch)\n');
fprintf('    4. ṗ (pitch rate)\n');
fprintf('    5. e (elevation)\n');
fprintf('    6. ė (elevation rate)\n\n');

fprintf('Step 3: Load x_opt (6 states)\n');
fprintf('  From Workspace block:\n');
fprintf('    Data: x_opt_timeseries_10_4\n');
fprintf('  Demux with 6 outputs\n\n');

fprintf('Step 4: Calculate Δx = x - x*\n');
fprintf('  Subtract block (6-element vectors)\n\n');

fprintf('Step 5: Apply feedback K\n');
fprintf('  Matrix Multiply:\n');
fprintf('    Gain: K_lq_10_4 (2×6 matrix)\n');
fprintf('    Input: Δx (6×1)\n');
fprintf('    Output: K·Δx (2×1)\n\n');

fprintf('Step 6: Negate feedback\n');
fprintf('  Gain = -1\n\n');

fprintf('Step 7: Load u_opt (2 inputs)\n');
fprintf('  From Workspace block:\n');
fprintf('    Data: u_opt_timeseries_10_4\n');
fprintf('  Demux with 2 outputs: [pc*, ec*]\n\n');

fprintf('Step 8: Add feedforward + feedback\n');
fprintf('  Sum block (vector addition)\n');
fprintf('    Input 1: u* (2×1)\n');
fprintf('    Input 2: -K·Δx (2×1)\n');
fprintf('    Output: u (2×1)\n\n');

fprintf('Step 9: Demux the control input\n');
fprintf('  Demux u into:\n');
fprintf('    Output 1: pc (pitch reference)\n');
fprintf('    Output 2: ec (elevation reference)\n\n');

fprintf('Step 10: Connect to controllers\n');
fprintf('  pc → PD Controller (pitch)\n');
fprintf('  ec → PID Controller (elevation)\n\n');

fprintf('Step 11: Recording\n');
fprintf('  Record all 6 states, 2 inputs\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('11. LAB PROCEDURE (Section 10.4)\n');
fprintf('========================================================================\n\n');

fprintf('FLIGHT PROCEDURE:\n');
fprintf('─────────────────\n');
fprintf('1. Verify section_10_4_data.mat is loaded\n\n');

fprintf('2. Build the model:\n');
fprintf('   QuaRC > Build for Monitoring\n\n');

fprintf('3. Turn ON power module\n\n');

fprintf('4. Start flight:\n');
fprintf('   QuaRC > Monitor & Tune\n\n');

fprintf('5. Observe:\n');
fprintf('   • Helicopter rises to clear obstacle\n');
fprintf('   • Travels around the constraint\n');
fprintf('   • Returns to e ≈ 0 at the end\n\n');

fprintf('6. What you should see:\n');
fprintf('   ✓ Elevation increases at λ ≈ 2π/3\n');
fprintf('   ✓ Helicopter clears obstacle\n');
fprintf('   ✓ Smooth trajectory\n');
fprintf('   ✓ Final position close to target\n\n');

fprintf('7. Save data!\n\n');

fprintf('TASK 10.4.7: Why does it work despite model mismatch?\n');
fprintf('──────────────────────────────────────────────────────\n\n');

fprintf('The model assumes pitch and elevation are decoupled:\n');
fprintf('  • Pitch affects travel only\n');
fprintf('  • Elevation is independent\n\n');

fprintf('But in reality:\n');
fprintf('  • Pitch affects force vector direction\n');
fprintf('  • At p = 90°, no vertical force!\n');
fprintf('  • Connection: ë ∝ Vs·cos(p)\n\n');

fprintf('Two reasons it still works:\n\n');

fprintf('Reason 1: Small pitch angles\n');
fprintf('  • Pitch stays within ±60°\n');
fprintf('  • cos(60°) = 0.5 (still significant vertical force)\n');
fprintf('  • Approximation ë ≈ Vs is reasonable\n\n');

fprintf('Reason 2: LQ feedback compensates!\n');
fprintf('  • Feedback corrects for modeling errors\n');
fprintf('  • If elevation deviates, ec adjusts\n');
fprintf('  • Robust to unmodeled dynamics\n\n');

fprintf('Press Enter to continue...\n');
pause;

fprintf('\n========================================================================\n');
fprintf('12. ANALYSIS AND REPORT (Section 10.4)\n');
fprintf('========================================================================\n\n');

fprintf('REQUIRED PLOTS:\n');
fprintf('───────────────\n');
fprintf('1. All 6 states vs time:\n');
fprintf('   • Travel λ\n');
fprintf('   • Travel rate r\n');
fprintf('   • Pitch p\n');
fprintf('   • Pitch rate ṗ\n');
fprintf('   • Elevation e (with constraint curve)\n');
fprintf('   • Elevation rate ė\n\n');

fprintf('2. Both inputs vs time:\n');
fprintf('   • Pitch reference pc\n');
fprintf('   • Elevation reference ec\n\n');

fprintf('3. Comparison plots:\n');
fprintf('   • Predicted x* vs measured x\n');
fprintf('   • Show feedback is working\n\n');

fprintf('4. 3D visualization:\n');
fprintf('   • (λ, e, time) trajectory\n');
fprintf('   • Constraint surface\n\n');

fprintf('DISCUSSION POINTS:\n');
fprintf('──────────────────\n\n');

fprintf('1. Constraint satisfaction:\n');
fprintf('   • Does e stay above the constraint?\n');
fprintf('   • What is the minimum clearance?\n');
fprintf('   • Safety margin discussion\n\n');

fprintf('2. Effect of weights q1, q2:\n');
fprintf('   • What values did you use?\n');
fprintf('   • How do they affect trajectory?\n');
fprintf('   • Trade-offs?\n\n');

fprintf('3. Computational cost:\n');
fprintf('   • How long did fmincon take?\n');
fprintf('   • Effect of horizon length N\n');
fprintf('   • Practical implications\n\n');

fprintf('4. Model coupling (Task 10.4.7):\n');
fprintf('   • Explain pitch-elevation coupling\n');
fprintf('   • Why does controller work anyway?\n');
fprintf('   • Role of feedback\n\n');

fprintf('5. Comparison with 10.3:\n');
fprintf('   • More complex optimization\n');
fprintf('   • More states/inputs\n');
fprintf('   • Better trajectory planning\n\n');

fprintf('OPTIONAL TASK 10.4.8:\n');
fprintf('─────────────────────\n');
fprintf('For bonus understanding (not required):\n\n');

fprintf('Try implementing suggested improvements:\n\n');

fprintf('Option A: Add cos(p) to model\n');
fprintf('  • Nonlinear model: harder optimization\n');
fprintf('  • More accurate dynamics\n');
fprintf('  • Compare performance\n\n');

fprintf('Option B: Penalty on model mismatch\n');
fprintf('  • Add Σ(ė - ė_predicted)² to objective\n');
fprintf('  • Encourages model consistency\n');
fprintf('  • May improve results\n\n');

fprintf('Option C: Adaptive Q weights\n');
fprintf('  • Q_e depends on p: Q_e(p) = Q_e0 · cos(p)\n');
fprintf('  • Increase elevation penalty at high pitch\n');
fprintf('  • More sophisticated control\n\n');

fprintf('Option D: Additional constraints\n');
fprintf('  • Max velocity: |ė| ≤ ė_max\n');
fprintf('  • Max travel rate: |r| ≤ r_max\n');
fprintf('  • See if feasible and how it affects trajectory\n\n');

fprintf('\n========================================================================\n');
fprintf('FINAL CHECKLIST\n');
fprintf('========================================================================\n\n');

fprintf('SECTION 10.3:\n');
fprintf('  □ LQ gain K calculated\n');
fprintf('  □ Q and R values justified\n');
fprintf('  □ Simulink model implemented\n');
fprintf('  □ Flight data recorded\n');
fprintf('  □ Comparison with 10.2 completed\n');
fprintf('  □ Analysis in report\n\n');

fprintf('SECTION 10.4:\n');
fprintf('  □ Extended model (6 states, 2 inputs) formulated\n');
fprintf('  □ fmincon optimization successful\n');
fprintf('  □ Constraint satisfaction verified\n');
fprintf('  □ LQ controller designed\n');
fprintf('  □ Simulink model implemented\n');
fprintf('  □ Flight data recorded\n');
fprintf('  □ Analysis in report\n');
fprintf('  □ Task 10.4.7 discussion included\n\n');

fprintf('REPORT QUALITY:\n');
fprintf('  □ All plots have titles, labels, units, legends\n');
fprintf('  □ Vector graphics (PDF/EPS)\n');
fprintf('  □ Subplots used to save space\n');
fprintf('  □ Clear discussion of results\n');
fprintf('  □ Code included (only modified parts)\n');
fprintf('  □ Simulink diagrams included\n\n');

fprintf('========================================================================\n');
fprintf('END OF GUIDE - GOOD LUCK!\n');
fprintf('========================================================================\n\n');

fprintf('Questions? Check:\n');
fprintf('  1. This guide\n');
fprintf('  2. Lab manual\n');
fprintf('  3. Course lecture notes\n');
fprintf('  4. Teaching assistant\n');
fprintf('  5. Blackboard hints\n\n');

fprintf('Remember: Each group must do the work independently!\n\n');
