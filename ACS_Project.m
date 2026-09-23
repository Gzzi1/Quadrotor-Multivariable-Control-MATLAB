%% =========================================================================
%% AEROSPACE CONTROL SYSTEMS - EXAM PROJECT AA 25/26
%% Robust Multivariable Control of the ANT-X Quadrotor Dynamics
%% =========================================================================
% Clears the workspace: removes variables, closes figures, and clears the console.
clear
close all
clc

% Definition of the complex Laplace variable 's' for transfer functions.
s = tf('s');

%% =========================================================================
%  1. NOMINAL PARAMETERS 
%% =========================================================================
% Gravity acceleration [m/s^2]
g = 9.81;

% --- Nominal Aerodynamic and Control Derivatives: Lateral Axis ---
Yv_nom = -0.1068;   % Lateral force derivative with respect to velocity v [1/s]
Yp_nom = -0.1192;   % Lateral force derivative with respect to angular velocity p [m/(s*rad)]
Lp_nom =  2.6478;   % Roll moment derivative with respect to p (roll damping) [1/s]
Yd_nom = 10.1647;   % Roll control effectiveness on lateral force [m/s^2]
Ld_nom = 450.7085;  % Roll control effectiveness on roll moment [rad/s^2]

% --- Nominal Aerodynamic and Control Derivatives: Longitudinal Axis (Symmetric) ---
Xu_nom = -0.1068;   % Axial force derivative with respect to velocity u [1/s]
Xq_nom =  0.1192;   % Axial force derivative with respect to angular velocity q [m/(s*rad)]
Mq_nom = -2.6478;   % Pitch moment derivative with respect to q (pitch damping) [1/s]
Xd_nom = -10.1647;  % Pitch control effectiveness on axial force [m/s^2]
Md_nom = 450.7085;  % Pitch control effectiveness on pitch moment [rad/s^2]

%% =========================================================================
%  2. STATE-SPACE NOMINAL MODELS 
%% =========================================================================

% --- State-Space Modeling for the Lateral Axis ---
% State: x_lat = [v (transverse velocity), p (roll rate), \phi (roll angle)]'
A_lat = [Yv_nom  Yp_nom  g;
          0      Lp_nom  0;
          0       1      0];
B_lat = [Yd_nom; Ld_nom; 0];
C_lat = [0 1 0;   % First output: roll angular velocity 'p'
         0 0 1];  % Second output: roll angle '\phi'
D_lat = [0; 0];   % Null direct transmission matrix

% Creation of the nominal lateral State-Space (ss) object and assignment of channel names
sys_lat_nom = ss(A_lat, B_lat, C_lat, D_lat);
sys_lat_nom.u = {'\delta_{lat}'};
sys_lat_nom.y = {'p', '\phi'};

% --- State-Space Modeling for the Longitudinal Axis ---
% State: x_lon = [u (longitudinal velocity), q (pitch rate), \theta (pitch angle)]'
A_lon = [Xu_nom  Xq_nom  -g;
          0      Mq_nom   0;
          0       1       0];
B_lon = [Xd_nom; Md_nom; 0];
C_lon = [0 1 0;   % First output: pitch angular velocity 'q'
         0 0 1];  % Second output: pitch angle '\theta'
D_lon = [0; 0];   % Null direct transmission matrix

% Creation of the nominal longitudinal State-Space (ss) object
sys_lon_nom = ss(A_lon, B_lon, C_lon, D_lon);
sys_lon_nom.u = {'\delta_{lon}'};
sys_lon_nom.y = {'q', '\theta'};

%% =========================================================================
%%   TASK 1 - LATERAL AXIS - NOMINAL MODELLING AND DESIGN
%% =========================================================================

%% --- 1.1 Observability Checks and Minimal Realization ---
% Calculation of the observability matrix with respect to output p only
Ob_lat_1 = obsv(A_lat, C_lat(1,:)); 
fprintf('LATERAL: rank of obsv(A,Cp)   = %d (state dim = %d)\n', rank(Ob_lat_1), size(A_lat,1));

% Calculation of the observability matrix with respect to output phi only
Ob_lat_2 = obsv(A_lat, C_lat(2,:));
fprintf('LATERAL: rank of obsv(A,Cphi) = %d (state dim = %d)\n', rank(Ob_lat_2), size(A_lat,1));

% Elimination of the unobservable state (transverse velocity v) via minimal realization
sys_lat_nom_min = minreal(sys_lat_nom); 

% Extraction of the transfer functions of the individual output channels
G_p_lat   = tf(sys_lat_nom_min(1));   % Transfer from \delta_{lat} to p
G_phi_lat = tf(sys_lat_nom_min(2));   % Transfer from \delta_{lat} to \phi

% Printing open-loop poles and zeros to highlight plant instability
fprintf('\n=== LATERAL: open-loop poles ===\n');
disp(pole(sys_lat_nom_min))
fprintf('Zeros p/delta_lat:   ');   disp(tzero(G_p_lat)')
fprintf('Zeros phi/delta_lat: '); disp(tzero(G_phi_lat)')

%% --- 1.2 Open-Loop Analysis Plots (Lateral Axis) ---
% Open-loop Pole-Zero map
figure('Name','T1_LAT_01_PoleZero');
subplot(2,1,1);
pzmap(sys_lat_nom_min(1)); grid on;
title('Open-Loop Poles/Zeros: p / \delta_{lat}');
subplot(2,1,2);
pzmap(sys_lat_nom_min(2)); grid on;
title('Open-Loop Poles/Zeros: \phi / \delta_{lat}');
sgtitle('Task 1 - Lateral Axis - Open-Loop Pole-Zero Map');

% Open-loop Bode diagram
figure('Name','T1_LAT_02_Bode');
bode(sys_lat_nom_min); grid on;
title('Task 1 - Lateral Axis - Open-Loop Frequency Response');

% Open-loop step response (highlights divergence/instability)
figure('Name','T1_LAT_03_Step');
subplot(2,1,1);
step(G_p_lat); grid on;
title('Open-Loop Step Response: p / \delta_{lat}');
xlabel('Time [s]'); ylabel('p [rad/s]');
subplot(2,1,2);
step(G_phi_lat); grid on;
title('Open-Loop Step Response: \phi / \delta_{lat}');
xlabel('Time [s]'); ylabel('\phi [rad]');
sgtitle('Task 1 - Lateral Axis - Open-Loop Step Response (unstable plant)');

%% --- 1.3 Structure of Tunable Controllers ---
% Outer Loop: Proportional Controller on roll angle error
R_phi = tunablePID('R_phi','P');
R_phi.u = 'e_\phi';
R_phi.y = 'p_0';

% Inner Loop: Filtered PID Controller on roll rate (two degrees of freedom)
R_p = tunablePID2('R_p','PID');
R_p.c.Value = 0;    R_p.c.Free = false;   % Disables setpoint weight on the derivative
R_p.b.Value = 1;    R_p.b.Free = false;   % Proportional on the reference
R_p.Tf.Value = 0.02; R_p.Tf.Free = false; % Fixed time constant of the derivative filter (20 ms)
R_p.u = {'p_0','p'};
R_p.y = {'\delta_{lat}'};

% Summing junction for angle tracking error generation
SumOuter = sumblk('e_\phi = \phi_0 - \phi');

% Tunable nominal interconnection for the lateral axis
CL_tun = connect(SumOuter, R_phi, R_p, sys_lat_nom, '\phi_0', {'\phi','p'});

%% --- 1.4 Definition of Weight Functions for H-infinity Synthesis ---
omega_n = 10;      % Minimum required natural frequency [rad/s]
xi      = 0.8;     % Damping of the reference second-order model

% Desired closed-loop second-order model
F_2 = omega_n^2 / (s^2 + 2*s*xi*omega_n + omega_n^2);
S_2 = 1 - F_2;     % Associated ideal sensitivity

% Plot of the reference model in time and frequency domains
figure('Name','T1_LAT_04_RefModel_Step');
step(F_2); grid on;
title('Task 1 - Lateral Axis - Reference Second-Order Model - Step Response');
xlabel('Time [s]'); ylabel('\phi / \phi_0');

figure('Name','T1_LAT_05_RefModel_Bode');
bodemag(F_2); hold on;
bodemag(S_2); grid on;
title('Task 1 - Lateral Axis - Reference Complementary/Sensitivity Functions');
legend('$F_2$ (reference F)','$S_2$ (reference S)','interpreter','latex');

% Construction of the performance weight function W_p2(s) to model Sensitivity S(s)
M       = 2;                % Limit on resonance peak (M_s)
A       = 1e-3;             % Allowed steady-state error at low frequencies
omega_b = 0.61*omega_n;     % Target bandwidth
W_p2 = ss((s/M + omega_b)/(s + A*omega_b));   % Performance weight in state-space form

figure('Name','T1_LAT_06_PerformanceWeight');
bodemag(F_2); hold on;
bodemag(S_2);
bodemag(1/W_p2); grid on;
title('Task 1 - Lateral Axis - Performance Weight vs Reference Models');
legend('$F_2$','$S_2$','$1/W_{p2}$','interpreter','latex','location','southeast');

% Construction of the control effort weight W_q(s) to avoid saturation
alpha_lat = deg2rad(20)/0.0715; % High-frequency gain to limit deflection
w_tau_lat = 20;                 % Actuator roll-off frequency [rad/s]
W_q = alpha_lat*(s + w_tau_lat*1e-3)/(s + w_tau_lat);

figure('Name','T1_LAT_07_ControlWeight');
bodemag(1/W_q); grid on;
title('Task 1 - Lateral Axis - Control Effort Weight (1/W_q)');

%% --- 1.5 Structured Synthesis with hinfstruct ---
% Assignment of inputs and outputs to weight blocks
W_p2.u = 'e_\phi';       W_p2.y = 'z_1';   % Weighted performance output
W_q.u  = '\delta_{lat}'; W_q.y = 'z_2';   % Weighted control effort output

% Construction of the generalized open-loop plant for hinfstruct
CL0 = connect(R_p, R_phi, sys_lat_nom, SumOuter, W_p2, W_q, ...
    '\phi_0', {'z_1','z_2'}, {'\delta_{lat}','e_\phi','\phi'});

% Options for non-smooth H-infinity optimization (20 random starts to avoid local minima)
opt = hinfstructOptions('Display','final','RandomStart',20);
[CL, gamma, info] = hinfstruct(CL0, opt);
fprintf('\ngamma_lat = %.4f\n', gamma);
showTunable(CL) % Prints tuned gains

% Extraction and visualization of synthesized system responses
F = getIOTransfer(CL,'\phi_0','\phi');
figure('Name','T1_LAT_08_ClosedLoop_Bode');
bodemag(F); grid on;
title('Task 1 - Lateral Axis - Closed-Loop \phi/\phi_0 Frequency Response (hinfstruct)');

figure('Name','T1_LAT_09_ClosedLoop_Step');
step(F); hold on;
step(F_2); grid on;
title('Task 1 - Lateral Axis - Closed-Loop Step Response vs Reference Model');
legend('hinfstruct design','second-order reference','location','southeast');

S = getIOTransfer(CL,'\phi_0','e_\phi');
figure('Name','T1_LAT_10_Sensitivity');
bodemag(S); hold on;
bodemag(1/W_p2);
bodemag(S_2); grid on;
title('Task 1 - Lateral Axis - Sensitivity Function S vs Design Bound');
legend('$S$','$1/W_p$','$S_2$','interpreter','latex','location','southeast');

Q = getIOTransfer(CL,'\phi_0','\delta_{lat}');
figure('Name','T1_LAT_11_ControlSensitivity');
bodemag(Q); hold on;
bodemag(1/W_q); grid on;
title('Task 1 - Lateral Axis - Control Sensitivity Q vs Design Bound');
legend('$Q$','$1/W_q$','interpreter','latex','location','southeast');

% Calculation and plot of open-loop stability margins L(s)
L = (1 - S)/S;
figure('Name','T1_LAT_12_Margins');
margin(L); grid on;
title('Task 1 - Lateral Axis - Open-Loop Gain/Phase Margins');

% Simulation of control effort subjected to a doublet maneuver
t = linspace(0,6,10^4);
u = 0*(t<=1) + 10*(t>1 & t<=3) - 10*(t>3 & t<=5) + 0*(t>5);
u = deg2rad(u); % Conversion to radians
figure('Name','T1_LAT_13_DoubletControlEffort');
lsim(Q, u, t); grid on;
title('Task 1 - Lateral Axis - Control Effort \delta_{lat} for \phi_0 Doublet');
xlabel('Time [s]'); ylabel('\delta_{lat}');

%% --- 1.6 Diagnostics and Requirements Verification (Lateral Axis) ---
% Extraction of tuned control blocks
R_p_tuned   = getBlockValue(CL, 'R_p');
R_phi_tuned = getBlockValue(CL, 'R_phi');
R_p_tuned.u   = {'p_0','p'};
R_p_tuned.y   = {'\delta_{lat}'};
R_phi_tuned.u = {'e_\phi'};
R_phi_tuned.y = {'p_0'};

% Reconstruction of the "clean" nominal closed loop (without synthesis weights)
CL_clean = connect(R_p_tuned, R_phi_tuned, sys_lat_nom, SumOuter, ...
    '\phi_0', {'\phi','p','\delta_{lat}','e_\phi'});
F_clean = CL_clean(1,1);   
Q_clean = CL_clean(3,1);   
S_clean = CL_clean(4,1);   

% Calculation of equivalent wn and xi based on step response (Response-based)
s_lat = stepinfo(F_clean);
OS_lat = s_lat.Overshoot;
tr_lat = s_lat.RiseTime;

if OS_lat > 0
    lo = log(OS_lat/100);
    xi_cl = -lo/sqrt(pi^2+lo^2);
else
    xi_cl = 1;
end

tr_ref_lat = stepinfo(F_2).RiseTime;
wn_cl = omega_n * (tr_ref_lat / tr_lat);

fprintf('\n[LATERAL] wn equivalent = %.4f rad/s (req >= 10)\n', wn_cl)
fprintf('[LATERAL] xi equivalent = %.4f       (req >= 0.9)\n', xi_cl)

% Calculation of the maximum control effort peak during the doublet
delta_resp = lsim(Q_clean, u, t);
fprintf('[LATERAL] max|delta_lat| = %.5f (req <= 0.05)\n', max(abs(delta_resp)))

% step info comparison reference and actual
fprintf('\nClosed-loop stepinfo');
stepinfo(F_clean)
fprintf('\nSecond-order stepinfo');
stepinfo(F_2)

%% =========================================================================
%%   TASK 1 - LONGITUDINAL AXIS - NOMINAL MODELLING AND DESIGN
%% =========================================================================

% Minimal realization for the longitudinal axis (elimination of state u)
sys_lon_nom_min = minreal(sys_lon_nom); 
G_q_lon     = tf(sys_lon_nom_min(1));   % Transfer \delta_{lon} -> q
G_theta_lon = tf(sys_lon_nom_min(2));   % Transfer \delta_{lon} -> \theta

fprintf('\n=== LONGITUDINAL: open-loop poles ===\n');
disp(pole(sys_lon_nom_min))
fprintf('Zeros q/delta_lon:     ');   disp(tzero(G_q_lon)')
fprintf('Zeros theta/delta_lon: '); disp(tzero(G_theta_lon)')

%% --- 2.1 Open-Loop Analysis Plots (Longitudinal Axis) ---
figure('Name','T1_LON_01_PoleZero');
subplot(2,1,1);
pzmap(sys_lon_nom_min(1)); grid on;
title('Open-Loop Poles/Zeros: q / \delta_{lon}');
subplot(2,1,2);
pzmap(sys_lon_nom_min(2)); grid on;
title('Open-Loop Poles/Zeros: \theta / \delta_{lon}');
sgtitle('Task 1 - Longitudinal Axis - Open-Loop Pole-Zero Map');

figure('Name','T1_LON_02_Bode');
bode(sys_lon_nom_min); grid on;
title('Task 1 - Longitudinal Axis - Open-Loop Frequency Response');

figure('Name','T1_LON_03_Step');
subplot(2,1,1);
step(G_q_lon); grid on;
title('Open-Loop Step Response: q / \delta_{lon}');
xlabel('Time [s]'); ylabel('q [rad/s]');
subplot(2,1,2);
step(G_theta_lon); grid on;
title('Open-Loop Step Response: \theta / \delta_{lon}');
xlabel('Time [s]'); ylabel('\theta [rad]');
sgtitle('Task 1 - Longitudinal Axis - Open-Loop Step Response (unstable plant)');

%% --- 2.2 Structure of Tunable Controllers (Longitudinal) ---
R_theta = tunablePID('R_theta','P');
R_theta.u = 'e_\theta';
R_theta.y = 'q_0';

R_q = tunablePID2('R_q','PID');
R_q.c.Value  = 0;    R_q.c.Free  = false;
R_q.b.Value  = 1;    R_q.b.Free  = false;
R_q.Tf.Value = 0.02; R_q.Tf.Free = false;
R_q.u = {'q_0','q'};
R_q.y = {'\delta_{lon}'};

SumOuter_lon = sumblk('e_\theta = \theta_0 - \theta');
CL_tun_lon = connect(SumOuter_lon, R_theta, R_q, sys_lon_nom, ...
    '\theta_0', {'\theta','q'});

%% --- 2.3 Definition of Weight Functions (Longitudinal) ---
omega_n_lon = 10;
xi_lon_des  = 0.8;
F_2_lon = omega_n_lon^2 / (s^2 + 2*xi_lon_des*omega_n_lon*s + omega_n_lon^2);
S_2_lon = 1 - F_2_lon;

figure('Name','T1_LON_04_RefModel_Step');
step(F_2_lon); grid on;
title('Task 1 - Longitudinal Axis - Reference Second-Order Model - Step Response');
xlabel('Time [s]'); ylabel('\theta / \theta_0');

figure('Name','T1_LON_05_RefModel_Bode');
bodemag(F_2_lon); hold on;
bodemag(S_2_lon); grid on;
title('Task 1 - Longitudinal Axis - Reference Complementary/Sensitivity Functions');
legend('$F_2$ (reference F)','$S_2$ (reference S)','interpreter','latex');

M_lon       = 2;
A_w_lon     = 1e-3;              
omega_b_lon = 0.59 * omega_n_lon;
W_p2_lon = ss((s/M_lon + omega_b_lon)/(s + A_w_lon*omega_b_lon));

figure('Name','T1_LON_06_PerformanceWeight');
bodemag(F_2_lon); hold on;
bodemag(S_2_lon);
bodemag(1/W_p2_lon); grid on;
title('Task 1 - Longitudinal Axis - Performance Weight vs Reference Models');
legend('$F_2$','$S_2$','$1/W_{p2}$','interpreter','latex','location','southeast');

alpha_lon = deg2rad(20)/0.0645;
w_tau_lon = 20;
W_q_lon = alpha_lon*(s + w_tau_lon*1e-3)/(s + w_tau_lon);

figure('Name','T1_LON_07_ControlWeight');
bodemag(1/W_q_lon); grid on;
title('Task 1 - Longitudinal Axis - Control Effort Weight (1/W_q)');

%% --- 2.4 hinfstruct Synthesis (Longitudinal) ---
W_p2_lon.u = 'e_\theta';     W_p2_lon.y = 'z_1';
W_q_lon.u  = '\delta_{lon}'; W_q_lon.y  = 'z_2';

CL0_lon = connect(R_q, R_theta, sys_lon_nom, SumOuter_lon, W_p2_lon, W_q_lon, ...
    '\theta_0', {'z_1','z_2'}, {'\delta_{lon}','e_\theta','\theta'});

opt_lon = hinfstructOptions('Display','final','RandomStart',20);
[CL_lon, gamma_lon, ~] = hinfstruct(CL0_lon, opt_lon);
fprintf('\ngamma_lon = %.4f\n', gamma_lon);
showTunable(CL_lon)

F_lon = getIOTransfer(CL_lon,'\theta_0','\theta');
figure('Name','T1_LON_08_ClosedLoop_Bode');
bodemag(F_lon); grid on;
title('Task 1 - Longitudinal Axis - Closed-Loop \theta/\theta_0 Frequency Response (hinfstruct)');

figure('Name','T1_LON_09_ClosedLoop_Step');
step(F_lon,'b'); hold on;
step(F_2_lon,'r--'); grid on;
title('Task 1 - Longitudinal Axis - Closed-Loop Step Response vs Reference Model');
legend('hinfstruct design','second-order reference','location','southeast');

S_lon = getIOTransfer(CL_lon,'\theta_0','e_\theta');
figure('Name','T1_LON_10_Sensitivity');
bodemag(S_lon); hold on;
bodemag(1/W_p2_lon);
bodemag(S_2_lon); grid on;
title('Task 1 - Longitudinal Axis - Sensitivity Function S vs Design Bound');
legend('$S$','$1/W_p$','$S_2$','interpreter','latex','location','southeast');

Q_lon = getIOTransfer(CL_lon,'\theta_0','\delta_{lon}');
figure('Name','T1_LON_11_ControlSensitivity');
bodemag(Q_lon); hold on;
bodemag(1/W_q_lon); grid on;
title('Task 1 - Longitudinal Axis - Control Sensitivity Q vs Design Bound');
legend('$Q$','$1/W_q$','interpreter','latex','location','southeast');

L_lon = (1 - S_lon)/S_lon;
figure('Name','T1_LON_12_Margins');
margin(L_lon); grid on;
title('Task 1 - Longitudinal Axis - Open-Loop Gain/Phase Margins');

t_lon = linspace(0,6,10^4);
u_lon = 0*(t_lon<=1) + 10*(t_lon>1 & t_lon<=3) - 10*(t_lon>3 & t_lon<=5) + 0*(t_lon>5);
u_lon = deg2rad(u_lon);
figure('Name','T1_LON_13_DoubletControlEffort');
lsim(Q_lon, u_lon, t_lon); grid on;
title('Task 1 - Longitudinal Axis - Control Effort \delta_{lon} for \theta_0 Doublet');
xlabel('Time [s]'); ylabel('\delta_{lon}');

%% --- 2.5 Diagnostics and Requirements Verification (Longitudinal Axis) ---
R_q_tuned     = getBlockValue(CL_lon, 'R_q');
R_theta_tuned = getBlockValue(CL_lon, 'R_theta');
R_q_tuned.u     = {'q_0','q'};
R_q_tuned.y     = {'\delta_{lon}'};
R_theta_tuned.u = {'e_\theta'};
R_theta_tuned.y = {'q_0'};

CL_clean_lon = connect(R_q_tuned, R_theta_tuned, sys_lon_nom, SumOuter_lon, ...
    '\theta_0', {'\theta','q','\delta_{lon}','e_\theta'});
F_lon_clean = CL_clean_lon(1,1);
Q_lon_clean = CL_clean_lon(3,1);
S_lon_clean = CL_clean_lon(4,1);

% Calculation of equivalent wn and xi based on step response (Response-based)
s_lon = stepinfo(F_lon_clean);
OS_lon = s_lon.Overshoot;
tr_lon = s_lon.RiseTime;

if OS_lon > 0
    lo = log(OS_lon/100);
    xi_lon = -lo/sqrt(pi^2+lo^2);
else
    xi_lon = 1;
end

tr_ref_lon = stepinfo(F_2_lon).RiseTime;
wn_lon = omega_n_lon * (tr_ref_lon / tr_lon);

fprintf('\n[LONGITUDINAL] wn equivalent = %.4f rad/s (req >= 10)\n', wn_lon)
fprintf('[LONGITUDINAL] xi equivalent = %.4f       (req >= 0.9)\n', xi_lon)

delta_lon_resp = lsim(Q_lon_clean, u_lon, t_lon);
fprintf('[LONGITUDINAL] max|delta_lon| = %.5f (req <= 0.05)\n', max(abs(delta_lon_resp)))

% step info comparison reference and actual
fprintf('\nClosed-loop stepinfo');
stepinfo(F_lon_clean)
fprintf('\nSecond-order stepinfo');
stepinfo(F_2_lon)

%% =========================================================================
%%   TASK 2 - UNCERTAIN MODELLING 
%% =========================================================================

%% --- 3.1 Definition of Uncertain Parameters (Lateral Axis) ---
% Setting the uncertainty range to 3*sigma (99.7% confidence interval)
Yv_unc = ureal('Yv', Yv_nom, 'Perc', 3*4);    % 12% uncertainty (sigma = 4%)
Yp_unc = ureal('Yp', Yp_nom, 'Perc', 3*2);    % 6% uncertainty  (sigma = 2%)
Lp_unc = ureal('Lp', Lp_nom, 'Perc', 3*2);    % 6% uncertainty  (sigma = 2%)
Yd_unc = ureal('Yd', Yd_nom, 'Perc', 3*1.5);  % 4.5% uncertainty (sigma = 1.5%)
Ld_unc = ureal('Ld', Ld_nom, 'Perc', 3*1);    % 3% uncertainty  (sigma = 1%)

% Uncertain state matrices (Lateral)
A_lat_unc = [Yv_unc  Yp_unc  g;
              0      Lp_unc  0;
              0       1      0];
B_lat_unc = [Yd_unc; Ld_unc; 0];

% Creation of the Uncertain State-Space (uss) system
sys_lat_unc = uss(A_lat_unc, B_lat_unc, C_lat, D_lat);
sys_lat_unc.u = {'\delta_{lat}'};
sys_lat_unc.y = {'p', '\phi'};

sys_lat_unc_nom = sys_lat_unc.NominalValue;
sys_lat_unc_nom.u = {'\delta_{lat}'};
sys_lat_unc_nom.y = {'p', '\phi'};

%% --- 3.2 Definition of Uncertain Parameters (Longitudinal Axis) ---
Xu_unc = ureal('Xu', Xu_nom, 'Perc', 3*4);    % 12% uncertainty (sigma = 4%)
Xq_unc = ureal('Xq', Xq_nom, 'Perc', 3*2);    % 6% uncertainty  (sigma = 2%)
Mq_unc = ureal('Mq', Mq_nom, 'Perc', 3*2);    % 6% uncertainty  (sigma = 2%)
Xd_unc = ureal('Xd', Xd_nom, 'Perc', 3*1.5);  % 4.5% uncertainty (sigma = 1.5%)
Md_unc = ureal('Md', Md_nom, 'Perc', 3*1);    % 3% uncertainty  (sigma = 1%)

% Uncertain state matrices (Longitudinal)
A_lon_unc = [Xu_unc  Xq_unc  -g;
              0      Mq_unc   0;
              0       1       0];
B_lon_unc = [Xd_unc; Md_unc; 0];

% Creation of the uss system (Longitudinal)
sys_lon_unc = uss(A_lon_unc, B_lon_unc, C_lon, D_lon);
sys_lon_unc.u = {'\delta_{lon}'};
sys_lon_unc.y = {'q', '\theta'};

sys_lon_unc_nom = sys_lon_unc.NominalValue;
sys_lon_unc_nom.u = {'\delta_{lon}'};
sys_lon_unc_nom.y = {'q', '\theta'};

%% --- 3.3 Analysis Plots of the Uncertain Systems Family ---
figure('Name','T2_LAT_01_PoleZero');
subplot(2,1,1);
pzplot(minreal(sys_lat_unc(1,1))); hold on;
pzplot(minreal(sys_lat_unc_nom(1,1))); grid on;
title('p / \delta_{lat} - uncertain vs nominal');
legend('Uncertain','Nominal','location','best');
subplot(2,1,2);
pzplot(minreal(sys_lat_unc(2,1))); hold on;
pzplot(minreal(sys_lat_unc_nom(2,1))); grid on;
title('\phi / \delta_{lat} - uncertain vs nominal');
legend('Uncertain','Nominal','location','best');
sgtitle('Task 2 - Lateral Axis - Uncertain Pole-Zero Map');

figure('Name','T2_LON_01_PoleZero');
subplot(2,1,1);
pzplot(minreal(sys_lon_unc(1,1))); hold on;
pzplot(minreal(sys_lon_unc_nom(1,1))); grid on;
title('q / \delta_{lon} - uncertain vs nominal');
legend('Uncertain','Nominal','location','best');
subplot(2,1,2);
pzplot(minreal(sys_lon_unc(2,1))); hold on;
pzplot(minreal(sys_lon_unc_nom(2,1))); grid on;
title('\theta / \delta_{lon} - uncertain vs nominal');
legend('Uncertain','Nominal','location','best');
sgtitle('Task 2 - Longitudinal Axis - Uncertain Pole-Zero Map');

% Bode diagrams with parametric dispersion
figure('Name','T2_LAT_02_Bode');
bode(sys_lat_unc, 'b', sys_lat_unc_nom, 'r--'); grid on;
title('Task 2 - Lateral Axis - Uncertain Plant Frequency Response');
legend('Uncertain','Nominal','location','best');

figure('Name','T2_LON_02_Bode');
bode(sys_lon_unc, 'b', sys_lon_unc_nom, 'r--'); grid on;
title('Task 2 - Longitudinal Axis - Uncertain Plant Frequency Response');
legend('Uncertain','Nominal','location','best');

% Step response of the open-loop uncertain plants family
figure('Name','T2_LAT_03_Step');
step(sys_lat_unc, 'b', sys_lat_unc_nom, 'r--'); grid on;
title('Task 2 - Lateral Axis - Uncertain Plant Step Response');
legend('Uncertain','Nominal','location','best');

figure('Name','T2_LON_03_Step');
step(sys_lon_unc, 'b', sys_lon_unc_nom, 'r--'); grid on;
title('Task 2 - Longitudinal Axis - Uncertain Plant Step Response');
legend('Uncertain','Nominal','location','best');

%% =========================================================================
%% TASK 3: ROBUSTNESS ANALYSIS (P-Delta form and mu-Analysis)
%% =========================================================================
% Logarithmically spaced frequency vector for sampling (from 0.01 to 1000 rad/s)
omega = logspace(-2, 3, 500);

%% -------------------------------------------------------------------------
%  1. LATERAL AXIS ROBUSTNESS ANALYSIS
%  -------------------------------------------------------------------------
fprintf('\n================ TASK 3: ROBUSTNESS ANALYSIS ================\n');
fprintf('--- LATERAL AXIS ---\n');

% Construction of the uncertain closed loop connecting the uncertain system and controllers calculated in Task 1
CL_unc_lat = connect(R_p_tuned, R_phi_tuned, sys_lat_unc, SumOuter, ...
    '\phi_0', {'\phi','p'});

% Extraction of the LFT (M-Delta) decomposition via lftdata.
% M_temp_lat is the nominal interconnection matrix, Delta_lat contains the uncertain parameters
[M_temp_lat, Delta_lat, BlkStruct_lat] = lftdata(CL_unc_lat);

% Isolation of the M11 submatrix: the transfer function that directly "sees" the Delta block
[out_delta_lat, in_delta_lat] = size(Delta_lat);
M11_lat = M_temp_lat(1:out_delta_lat, 1:in_delta_lat);

% Conversion of the continuous M11 matrix into a frequency response data (FRD) model
M_lat_freq = frd(M11_lat, omega);

% Calculation of the Structured Singular Value (\mu) providing the exact block structure (BlkStruct_lat)
mu_bounds_lat = mussv(M_lat_freq, BlkStruct_lat, 's');

% Extraction of the maximum peak value of the \mu upper bound
[RS_mu_lat, ~] = max(mu_bounds_lat.ResponseData(1, 1, :));
RS_mu_lat = squeeze(RS_mu_lat);
fprintf('mu-Analysis Peak mu (Lateral): %.4f\n', RS_mu_lat);

% Robust Stability Criterion: mu_max < 1 to guarantee stability with all uncertainties
if RS_mu_lat < 1
    fprintf('Status: VERIFIED (Robust Stability guaranteed for the given uncertainties)\n');
else
    fprintf('Status: UNSTABLE (The system is not robustly stable, redesign in Task 1 is required)\n');
end

% 4. PLOT IN DECIBEL (Small Gain vs mu Comparison)
[sv_lat, wout_lat] = sigma(M11_lat, {1e-2, 1e3}); % Calculates singular values for the Small Gain

figure('Name', 'T3_LAT_01_MuAnalysis', 'Color', 'w');
% Plots sigma_max(M11) in dB (Small Gain Curve)
semilogx(wout_lat, 20*log10(max(sv_lat,[],1)), 'b', 'LineWidth', 1.5); hold on; grid on;
% Plots mu upper bound in dB
semilogx(omega, 20*log10(squeeze(mu_bounds_lat.ResponseData(1,1,:))), 'm', 'LineWidth', 1.5);
% RS Threshold at 0 dB (equivalent to linear 1)
yline(0, 'r--', 'Threshold RS = 0 dB', 'LineWidth', 1.5);

xlabel('Frequency [rad/s]'); ylabel('Magnitude [dB]');
title('Task 3 - Lateral Axis - Structured RS');
legend('\sigma_{max}(M_{11}) (Small Gain)', '\mu Upper Bound', 'Limit RS = 0 dB', 'Location', 'southwest');
%% -------------------------------------------------------------------------
%  2. LONGITUDINAL AXIS ROBUSTNESS ANALYSIS
%  -------------------------------------------------------------------------
fprintf('\n--- LONGITUDINAL AXIS ---\n');

% Construction of the uncertain closed loop for the longitudinal axis
CL_unc_lon = connect(R_q_tuned, R_theta_tuned, sys_lon_unc, SumOuter_lon, ...
    '\theta_0', {'\theta','q'});

% Extraction of the M-Delta form via lftdata
[M_temp_lon, Delta_lon, BlkStruct_lon] = lftdata(CL_unc_lon);

% Isolation of the M11 submatrix
[out_delta_lon, in_delta_lon] = size(Delta_lon);
M11_lon = M_temp_lon(1:out_delta_lon, 1:in_delta_lon);

% Calculation of structured \mu in frequency
M_lon_freq = frd(M11_lon, omega);
mu_bounds_lon = mussv(M_lon_freq, BlkStruct_lon, 's');

% Extraction of the maximum peak of the upper bound
[RS_mu_lon, ~] = max(mu_bounds_lon.ResponseData(1, 1, :));
RS_mu_lon = squeeze(RS_mu_lon);
fprintf('mu-Analysis Peak mu (Longitudinal): %.4f\n', RS_mu_lon);

if RS_mu_lon < 1
    fprintf('Status: VERIFIED (Robust Stability guaranteed for the given uncertainties)\n');
else
    fprintf('Status: UNSTABLE (The system is not robustly stable, redesign in Task 1 is required)\n');
end

% 4. PLOT IN DECIBEL (Small Gain vs mu Comparison)
[sv_lon, wout_lon] = sigma(M11_lon, {1e-2, 1e3}); % Calculates singular values for the Small Gain

figure('Name', 'T3_LON_01_MuAnalysis', 'Color', 'w');
% Plots sigma_max(M11) in dB (Small Gain Curve)
semilogx(wout_lon, 20*log10(max(sv_lon,[],1)), 'b', 'LineWidth', 1.5); hold on; grid on;
% Plots mu upper bound in dB
semilogx(omega, 20*log10(squeeze(mu_bounds_lon.ResponseData(1,1,:))), 'm', 'LineWidth', 1.5);
% RS Threshold at 0 dB (equivalent to linear 1)
yline(0, 'r--', 'Threshold RS = 0 dB', 'LineWidth', 1.5);

xlabel('Frequency [rad/s]'); ylabel('Magnitude [dB]');
title('Task 3 - Longitudinal Axis - Structured RS');
legend('\sigma_{max}(M_{11}) (Small Gain)', '\mu Upper Bound', 'Limit RS = 0 dB', 'Location', 'southwest');

%% =========================================================================
%% TASK 4: ROBUSTNESS TO COUPLING (MIMO mu-Analysis)
%% =========================================================================
fprintf('\n================ TASK 4: ROBUSTNESS TO COUPLING ================\n');

% 1. Definition of the uncertain misalignment angle psi \in [-15, 15] degrees
psi_unc = ureal('psi', 0, 'Range', deg2rad([-15, 15]));

% 2. Second-order Taylor approximation to maintain rational expressions:
% sin(\psi) \approx \psi, cos(\psi) \approx 1 - \psi^2 / 2
sin_psi = psi_unc;
cos_psi = 1 - (psi_unc^2) / 2;

% Uncertain kinematic rotation/coupling matrix
T_couple = [ cos_psi,  sin_psi;
    -sin_psi,  cos_psi];

% Transformation of the coupling matrix into an Uncertain State-Space (uss) block
CouplingBlk = uss(T_couple);
CouplingBlk.InputName = {'\delta_{lon}', '\delta_{lat}'};
CouplingBlk.OutputName = {'u_lon', 'u_lat'};

% 3. Renaming inputs of the two plants to connect the coupling matrix
sys_lon_unc_MIMO = sys_lon_unc;
sys_lon_unc_MIMO.u = {'u_lon'};

sys_lat_unc_MIMO = sys_lat_unc;
sys_lat_unc_MIMO.u = {'u_lat'};

% 4. Construction of the entire multivariable closed-loop system (MIMO)
CL_MIMO_unc = connect(R_q_tuned, R_theta_tuned, R_p_tuned, R_phi_tuned, ...
    CouplingBlk, sys_lon_unc_MIMO, sys_lat_unc_MIMO, ...
    SumOuter_lon, SumOuter, ...
    {'\theta_0', '\phi_0'}, {'\theta', '\phi'});

% 5. Extraction of the overall M11 matrix of the MIMO system via lftdata
[M_temp_MIMO, Delta_MIMO, BlkStruct_MIMO] = lftdata(CL_MIMO_unc);
[out_delta_MIMO, in_delta_MIMO] = size(Delta_MIMO);
M11_MIMO = M_temp_MIMO(1:out_delta_MIMO, 1:in_delta_MIMO);

% --- SMALL GAIN ---
[peak_MIMO, w_peak_MIMO] = norm(M11_MIMO, inf);
fprintf('Small Gain ||M11||_inf (MIMO): %.4f @ %.2f rad/s\n', peak_MIMO, w_peak_MIMO);

% 6. Calculation of the multivariable Structured Singular Value (\mu)
M_MIMO_freq = frd(M11_MIMO, omega);
mu_bounds_MIMO = mussv(M_MIMO_freq, BlkStruct_MIMO, 's');

% Extraction of the maximum peak of the global \mu upper bound
[RS_mu_MIMO, ~] = max(mu_bounds_MIMO.ResponseData(1, 1, :));
RS_mu_MIMO = squeeze(RS_mu_MIMO);
fprintf('mu-Analysis Peak mu (MIMO with coupling): %.4f\n', RS_mu_MIMO);

% Final check of Multivariable Robust Stability (MIMO)
if RS_mu_MIMO < 1
    fprintf('Task 4 Status: VERIFIED (Robust Stability guaranteed even with coupling)\n');
else
    fprintf('Task 4 Status: UNSTABLE (Coupling destabilizes the system. Controller redesign in Task 1 is required)\n');
end

% Plot in DECIBEL (Small Gain vs mu Comparison)
[sv_MIMO, wout_MIMO] = sigma(M11_MIMO, {1e-2, 1e3});

figure('Name', 'T4_MIMO_01_MuAnalysis', 'Color', 'w');
semilogx(wout_MIMO, 20*log10(max(sv_MIMO,[],1)), 'b', 'LineWidth', 1.5); hold on; grid on;
semilogx(omega, 20*log10(squeeze(mu_bounds_MIMO.ResponseData(1,1,:))), 'm', 'LineWidth', 1.5);
yline(0, 'r--', 'Threshold RS = 0 dB', 'LineWidth', 1.5);
xlabel('Frequency [rad/s]'); ylabel('Magnitude [dB]');
title('Task 4 - MIMO Robust Stability with \psi Coupling');
legend('\sigma_{max}(M_{11}) (Small Gain)', '\mu Upper Bound', 'Limit RS = 0 dB', 'Location', 'southwest');

%% ================================================================
%  TASK 5 : CONTROL SYSTEM VALIDATION (MONTE CARLO)
% ================================================================
%  Combined uncertainty: Gaussian distribution on the parameters of the
%  two axes + uniform coupling psi in [-15,15] deg (11 sources). The
%  number of samples N follows from the Chernoff bound. With the FIXED
%  Task-1 controllers, the fraction of the population meeting the
%  requirements (stability, Req A on wn/xi, Req B on |delta|) is
%  estimated, with a convergence plot and metric histograms.
% ================================================================

fprintf('\n################################################################\n');
fprintf('  TASK 5 : MONTE CARLO ANALYSIS \n');
fprintf('################################################################\n');

rng(0);   % reproducibility of the random samples

% --- number of iterations: Chernoff bound ---
%   |p_hat - p| <= eps with confidence >= 1-delta  =>  N >= ln(2/delta)/(2 eps^2)
eps_acc = 0.02;      % accuracy on the probability (+/- 2%)
delta_c = 0.05;      % 1-delta = 95% confidence
Nmin = ceil(log(2/delta_c)/(2*eps_acc^2)); % Chernoff bound, minimum n. of iterations
N    = Nmin+200;                           % simulation: Nmin + margin
fprintf('   Chernoff: eps=%.3f, conf=%.0f%%  ->  Nmin=%d ; using N=%d\n', ...
        eps_acc,100*(1-delta_c),Nmin,N);

% --- Requirements struct (Req A and Req B) ---
req.wn   = 10;     % [rad/s]
req.xi   = 0.8;    % [-]
req.dmax = 0.05;   % [-]

% --- parameter densities (mean=nominal, sigma=%*|nom|) ---
nom_lat = [Yv_nom Yp_nom Lp_nom Yd_nom Ld_nom];
pct_lat = [4 2 2 1.5 1]/100;
sig_lat = pct_lat.*abs(nom_lat);

nom_lon = [Xu_nom Xq_nom Mq_nom Xd_nom Md_nom];
pct_lon = [4 2 2 1.5 1]/100;
sig_lon = pct_lon.*abs(nom_lon);

% NOTE: The FIXED controllers are already defined and extracted in Task 1/2: 
% R_q_tuned, R_theta_tuned, R_p_tuned, R_phi_tuned.
% The error blocks (SumOuter, SumOuter_lon) are also in memory.

% --- test signals and requirement-check parameters ---
tt = linspace(0,1.5,1500);                        % step response
td = 0:1e-3:6;  amp = deg2rad(10);                % doublet, requirement B
ud = amp*((td>=1 & td<3) - (td>=3 & td<5));
ref2   = tf(req.wn^2,[1 2*req.xi*req.wn req.wn^2]);  % 2nd-order prototype
tr_ref = stepinfo(ref2).RiseTime;

% --- storage (vectors with wn, xi, etc. for each simulation) ---
wn_phi = nan(N,1); xi_phi = nan(N,1); d_lat  = nan(N,1);
wn_th  = nan(N,1); xi_th  = nan(N,1); d_lon  = nan(N,1);
stab   = false(N,1);
okA_lat = false(N,1); okB_lat = false(N,1);
okA_lon = false(N,1); okB_lon = false(N,1);

% --- Monte Carlo loop ---
hw = waitbar(0, 'Running Monte Carlo simulations...');
for i = 1:N
    if mod(i,50)==0, waitbar(i/N, hw); end
    
    % sample parameters (Gaussian) and psi (Uniform)
    pl = nom_lat + sig_lat.*randn(1,5);
    Yv = pl(1);  Yp = pl(2);  Lp = pl(3);  Yd = pl(4);  Ld = pl(5);
    
    po = nom_lon + sig_lon.*randn(1,5);
    Xu = po(1);  Xq = po(2);  Mq = po(3);  Xd = po(4);  Md = po(5);
    
    psi = -15 + 30*rand;   % deg, uniform in [-15,15]

    % sampled plants 
    Glat = ss([Yv Yp g; 0 Lp 0; 0 1 0], [Yd;Ld;0], C_lat, D_lat);
    Glat.u = {'u_lat'}; Glat.y = {'p','\phi'};
    
    Glon = ss([Xu Xq -g; 0 Mq 0; 0 1 0], [Xd;Md;0], C_lon, D_lon);
    Glon.u = {'u_lon'}; Glon.y = {'q','\theta'};

    % coupling R(psi): [u_lon; u_lat] = R(psi)[delta_lon; delta_lat]
    Rc = ss([cosd(psi), sind(psi); -sind(psi), cosd(psi)]);
    Rc.u = {'\delta_{lon}', '\delta_{lat}'};
    Rc.y = {'u_lon', 'u_lat'};

    % full closed loop (2 refs in, 4 outputs of interest)
    CL_mc = connect(Glon, Glat, R_q_tuned, R_theta_tuned, R_p_tuned, R_phi_tuned, ...
                    Rc, SumOuter_lon, SumOuter, ...
                    {'\theta_0','\phi_0'}, {'\theta','\phi','\delta_{lon}','\delta_{lat}'});

    % stability (if unstable: requirements failed, stays NaN/false)
    stab(i) = isstable(CL_mc);
    if ~stab(i)
        continue;
    end

    % --- step response on the main axes ---
    Fphi = CL_mc('\phi', '\phi_0');
    Fth  = CL_mc('\theta', '\theta_0');

    s_lat  = stepinfo(Fphi);
    OS_lat = s_lat.Overshoot;
    tr_lat = s_lat.RiseTime;      % lateral
    if OS_lat > 0
        lo = log(OS_lat/100);                    % overshoot in percentage
        xi_phi(i) = -lo/sqrt(pi^2+lo^2);         % damping from the overshoot
    else
        xi_phi(i) = 1;
    end
    wn_phi(i) = req.wn*tr_ref/tr_lat;            % omega_n from the rise time
    okA_lat(i) = (wn_phi(i) >= req.wn) && (xi_phi(i) >= req.xi);

    s_lon  = stepinfo(Fth);
    OS_lon = s_lon.Overshoot;
    tr_lon = s_lon.RiseTime;      % longitudinal
    if OS_lon > 0
        lo = log(OS_lon/100);
        xi_th(i) = -lo/sqrt(pi^2+lo^2);
    else
        xi_th(i) = 1;
    end
    wn_th(i) = req.wn*tr_ref/tr_lon;
    okA_lon(i) = (wn_th(i) >= req.wn) && (xi_th(i) >= req.xi);

    % --- REQUIREMENT B - control effort at the doublet ---
    Klat = CL_mc('\delta_{lat}', '\phi_0');
    Klon = CL_mc('\delta_{lon}', '\theta_0');
    d_lat(i) = max(abs(lsim(Klat, ud, td)));
    d_lon(i) = max(abs(lsim(Klon, ud, td)));
    okB_lat(i) = d_lat(i) <= req.dmax;
    okB_lon(i) = d_lon(i) <= req.dmax;

    % quick diagnostic on the first iterations
    if i <= 5
        fprintf('i=%d  d_lat=%.6f  d_lon=%.6f\n', i, d_lat(i), d_lon(i));
    end
end
close(hw);

% ================================================================
%  SANITY CHECK on the delta plot
% ================================================================
fprintf('\n   --- delta diagnostics ---\n');
fprintf('   lateral :  mean=%.4f  std=%.2e  min=%.4f  max=%.4f  (valid %d/%d)\n', ...
    mean(d_lat,'omitnan'), std(d_lat,'omitnan'), min(d_lat), max(d_lat), sum(~isnan(d_lat)), N);
fprintf('   longit. :  mean=%.4f  std=%.2e  min=%.4f  max=%.4f  (valid %d/%d)\n', ...
    mean(d_lon,'omitnan'), std(d_lon,'omitnan'), min(d_lon), max(d_lon), sum(~isnan(d_lon)), N);

% coefficient of variation = relative spread (std/mean)
fprintf('   CV lateral = %.2f%%    CV longit. = %.2f%%\n', ...
    100*std(d_lat,'omitnan')/mean(d_lat,'omitnan'), ...
    100*std(d_lon,'omitnan')/mean(d_lon,'omitnan'));

% how many distinct values (if ~1, delta is constant: suspicious)
fprintf('   distinct values: lateral=%d  longit.=%d\n', ...
    numel(unique(round(d_lat(~isnan(d_lat)),6))), ...
    numel(unique(round(d_lon(~isnan(d_lon)),6))));

% --- compliance ---
okAll = stab & okA_lat & okA_lon & okB_lat & okB_lon;
p_hat = mean(okAll);   % drones passing the test / total drones
fprintf('\n   --- Compliance over N=%d realizations ---\n',N);
fprintf('   Closed-loop stability ......... %6.2f %%\n',100*mean(stab));
fprintf('   Req A (perf) lateral .......... %6.2f %%\n',100*mean(okA_lat));
fprintf('   Req A (perf) longitudinal ..... %6.2f %%\n',100*mean(okA_lon));
fprintf('   Req B (effort) lateral ........ %6.2f %%\n',100*mean(okB_lat));
fprintf('   Req B (effort) longitudinal ... %6.2f %%\n',100*mean(okB_lon));
fprintf('   ----------------------------------------\n');
fprintf('   GLOBAL COMPLIANCE p_hat ....... %6.2f %%   (95%% CI: +/- %.1f %%)\n', ...
        100*p_hat,100*eps_acc);

% --- convergence of the estimate (justifies N) ---
run_p = cumsum(okAll)./(1:N)';
figure('Name','Task5 - MC convergence','Color','w');
plot(1:N,100*run_p,'b','LineWidth',1.3);
hold on; grid on; box on;
yline(100*p_hat,'k--');
yline(100*(p_hat+eps_acc),'r:');
yline(100*(p_hat-eps_acc),'r:');
xline(Nmin,'m--');
xlabel('iteration n'); ylabel('compliance estimate [%]');
title('Task 5: convergence of p_{hat}(n) and Chernoff band \pm\epsilon');
legend('p_{hat}(n)','p_{hat} final','\pm\epsilon','N_{min}','Location','best');

% ================================================================
%  TASK 5 - HISTOGRAMS
% ================================================================
nb   = 40;                      % number of bins
cLat = [0.00 0.45 0.74];        % blue   -> lateral
cLon = [0.85 0.33 0.10];        % orange -> longitudinal

% helper: common bin edges for the two axes (to align the histograms)
edges = @(a,b,n) linspace( min([a;b],[],'omitnan'), ...
                           max([a;b],[],'omitnan'), n+1 );

figure('Name','Task5 - histograms','Color','w','Position',[80 80 1500 430]);

% ---- (1) omega_n ----
subplot(1,3,1); hold on; box on; grid on;
e = edges(wn_phi,wn_th,nb);
h1 = histogram(wn_phi,e,'FaceColor',cLat,'FaceAlpha',0.55,'EdgeColor','none');
h2 = histogram(wn_th ,e,'FaceColor',cLon,'FaceAlpha',0.55,'EdgeColor','none');
hr = xline(req.wn,'r--','LineWidth',2);
xlabel('\omega_n  [rad/s]'); ylabel('count');
title('\omega_n distribution');
legend([h1 h2 hr],{'lateral','longitudinal','\omega_n \geq 10'},'Location','best');

% ---- (2) xi ----
subplot(1,3,2); hold on; box on; grid on;
e = edges(xi_phi,xi_th,nb);
h1 = histogram(xi_phi,e,'Normalization','probability','FaceColor',cLat,'FaceAlpha',0.55,'EdgeColor','none');
h2 = histogram(xi_th ,e,'Normalization','probability','FaceColor',cLon,'FaceAlpha',0.55,'EdgeColor','none');
hr = xline(req.xi,'r--','LineWidth',2);
ylim([0 0.20]);                      
xlabel('\xi  [-]'); ylabel('fraction');
title('\xi distribution');
legend([h1 h2 hr],{'lateral','longitudinal','\xi \geq 0.9'},'Location','northwest');

% ---- (3) delta ----
subplot(1,3,3); hold on; box on; grid on;
e = edges(d_lat,d_lon,nb);
h1 = histogram(d_lat,e,'FaceColor',cLat,'FaceAlpha',0.55,'EdgeColor','none');
h2 = histogram(d_lon,e,'FaceColor',cLon,'FaceAlpha',0.55,'EdgeColor','none');
hr = xline(req.dmax,'r--','LineWidth',2);
xlabel('|\delta|_{max}  [-]'); ylabel('count');
title('|\delta| distribution');
legend([h1 h2 hr],{'lateral','longitudinal','|\delta| \leq 0.05'},'Location','best');

sgtitle('Task 5: metric distributions (lateral vs longitudinal) — red = requirement');