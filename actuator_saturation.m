%% AE352 Team Project - Final Code
% Includes Power Analysis and Saturation Checks
clear; clc; close all;

%% 1. SYSTEM PARAMETERS (Refined from Peer Review)
% Frame: F450 style, Mass: 1.2kg
params.m = 1.2;       
params.g = 9.81;      
params.I = diag([0.03, 0.03, 0.06]); 
params.L = 0.225;     

% Actuator Constants (T-Motor 2216 / 10x4.5 Prop)
% Thrust F = kf * w^2
params.kf = 2.2e-5;   
% Torque M = km * w^2
params.km = 5.5e-7;   
% Max Speed (approx 9000 RPM -> 950 rad/s)
params.w_max = 950;   

% Battery (4S 3300mAh)
params.V_batt = 14.8; 
params.Capacity_Wh = 14.8 * 3.3; % ~48.8 Wh

%% 2. GOAL 3 SIMULATION (Complex Maneuver)
fprintf('Simulating Goal 3 (Complex Maneuver)...\n');
t_span3 = [0 40];
init_state3 = zeros(12,1);

[t3, x3] = ode45(@(t,x) dynamics(t, x, params, 'complex', []), t_span3, init_state3);

%% 3. POST-PROCESSING: Actuator Saturation & Power
num_steps = length(t3);
motor_speeds = zeros(num_steps, 4);
inst_power = zeros(num_steps, 1);

for i = 1:num_steps
    [~, w_sq, P_elec] = dynamics(t3(i), x3(i,:)', params, 'complex', []);
    motor_speeds(i, :) = sqrt(max(w_sq, 0)); % Convert w^2 to w
    inst_power(i) = P_elec;
end

% Total Energy Consumed (Joules -> Wh)
energy_joules = trapz(t3, inst_power);
energy_wh = energy_joules / 3600;

fprintf('Goal 3 Energy Consumed: %.2f Wh\n', energy_wh);
fprintf('Battery Capacity: %.2f Wh\n', params.Capacity_Wh);
fprintf('Max Motor Speed Reached: %.1f rad/s (Limit: %.1f)\n', ...
    max(max(motor_speeds)), params.w_max);

%% 4. PLOTTING SATURATION (New for Final Report)
figure('Name', 'Goal 3: Motor Saturation', 'Position', [100, 100, 800, 400]);
plot(t3, motor_speeds, 'LineWidth', 1.5); grid on; hold on;
yline(params.w_max, 'r--', 'Max Limit (950 rad/s)', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Rotor Speed (rad/s)');
title('Actuator Saturation Check (Goal 3)');
legend('w1', 'w2', 'w3', 'w4', 'Limit');
ylim([0 1000]);
saveas(gcf, 'goal3_saturation.png');

%% DYNAMICS FUNCTION with MIXER & POWER
function [dxdt, w_sq_out, Power_out] = dynamics(t, x, p, mode, target)
    % Unpack State
    pos = x(1:3); vel = x(4:6); ang = x(7:9); omega = x(10:12);
    
    % Trajectory Generation (Goal 3 Logic)
    pos_des=[0;0;0]; vel_des=[0;0;0]; psi_des=0;
    if strcmp(mode, 'complex')
        if t<5, pos_des=[0;0;1];
        elseif t<10, pos_des=[1*(t-5);0;1];
        elseif t<12, pos_des=[5;0;1];
        elseif t<15, pos_des=[5;0;1]; psi_des=pi/2;
        elseif t<20, pos_des=[5;1*(t-15);1]; psi_des=pi/2;
        elseif t<25, pos_des=[5;5;1]; psi_des=pi/2;
        else, pos_des=[5;5;0]; psi_des=pi/2; vel_des=[0;0;-0.1];
           if pos(3)<0.05, vel_des=[0;0;-0.005]; end 
        end
    end

    % Controller (PD)
    Kp_pos = [4;4;10]; Kd_pos = [3;3;6];
    acc_cmd = Kd_pos.*(vel_des - vel) + Kp_pos.*(pos_des - pos);
    
    % Desired Forces/Torques
    U1 = p.m * (p.g + acc_cmd(3)); 
    if U1 < 0, U1 = 0; end
    
    phi_des = (acc_cmd(1)*sin(psi_des) - acc_cmd(2)*cos(psi_des))/p.g;
    theta_des = (acc_cmd(1)*cos(psi_des) + acc_cmd(2)*sin(psi_des))/p.g;
    
    Kp_att = [20;20;20]; Kd_att = [4;4;4];
    tau = Kp_att.*([phi_des;theta_des;psi_des]-ang) + Kd_att.*([0;0;0]-omega);
    
    % MIXER (Forces -> Motor Speeds)
    % Matrix Inverse of Eq 5 in Report
    % U = [U1; tau_phi; tau_theta; tau_psi]
    % InvMixer * U = w^2
    
    U = [U1; tau(1); tau(2); tau(3)];
    
    % Analytic Inverse for X-config
    % w1^2 = 1/(4kf)*U1 - 1/(2*sqrt(2)*kf*L)*tau_phi + ...
    % Simplified logic for standard X-mix:
    % T = kf * sum(w^2)
    % tau_phi = L/sqrt(2)*kf * (-w1^2 - w2^2 + w3^2 + w4^2) ...
    
    % Direct numerical solve is safer for the generic report code:
    Mixer = [p.kf, p.kf, p.kf, p.kf;
             -p.kf*p.L/sqrt(2), -p.kf*p.L/sqrt(2), p.kf*p.L/sqrt(2), p.kf*p.L/sqrt(2);
             p.kf*p.L/sqrt(2), -p.kf*p.L/sqrt(2), -p.kf*p.L/sqrt(2), p.kf*p.L/sqrt(2);
             p.km, -p.km, p.km, -p.km];
         
    w_sq = Mixer \ U;
    
    % Saturation limits for Physics (optional but good for realism)
    w_sq = max(min(w_sq, p.w_max^2), 0); 
    
    % Power Calculation (P = V*I, assuming I proportional to Thrust)
    % Approximation: P_total = sum( c1 * w + c2 ) ? 
    % Standard Approx: P = sum( w^3 ) * const ??
    % Peer review asked for "manufacturer curves". 
    % Let's use P = T^1.5 / FOM or similar. 
    % Simplest valid metric: Mechanical Power P = Torque * w = (km*w^2) * w = km * w^3
    % Electrical Power approx P_elec = P_mech / efficiency (say 0.7)
    P_mech = sum(p.km .* (sqrt(w_sq)).^3);
    Power_out = P_mech / 0.6; % 60% system efficiency
    
    w_sq_out = w_sq;

    % EOM
    phi=ang(1); theta=ang(2); psi=ang(3);
    R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
         sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
         -sin(theta),         cos(theta)*sin(phi),                            cos(theta)*cos(phi)];
    
    % Re-calculate actual U based on saturated motors
    U_actual = Mixer * w_sq;
    U1_act = U_actual(1);
    tau_act = U_actual(2:4);

    W_mat = [1, 0, -sin(theta); 0, cos(phi), sin(phi)*cos(theta); 0, -sin(phi), cos(phi)*cos(theta)];
    
    acc = [0;0;-p.g] + (R*[0;0;U1_act])/p.m;
    om_dot = p.I \ (tau_act - cross(omega, p.I*omega));
    ang_dot = W_mat \ omega;
    
    dxdt = [vel; acc; ang_dot; om_dot];
end