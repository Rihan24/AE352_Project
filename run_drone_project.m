%% AE352 Team Project - Fall 2025
% Drone Dynamics and Control Simulation
% Generates plots for Goals 1, 2, and 3

clear; clc; close all;

%% 1. SYSTEM PARAMETERS (Realistic / Off-the-shelf)
params.m = 1.2;       % kg
params.g = 9.81;      % m/s^2
params.I = diag([0.03, 0.03, 0.06]); % Inertia [Ix Iy Iz]
params.L = 0.225;     % Arm length (m)

%% 2. GOAL 1: HOVER (2 Minutes)
fprintf('Running Goal 1: Hover...\n');
t_span1 = [0 120];
init_state1 = zeros(12,1); % Start at ground
target1 = [0; 0; 1; 0];    % x, y, z, yaw

[t1, x1] = ode45(@(t,x) dynamics(t, x, params, 'hover', target1), t_span1, init_state1);

% Plot Goal 1
figure('Name','Goal 1','Position',[100 100 800 600]);
subplot(2,1,1); plot(t1, x1(:,3), 'LineWidth',2); grid on;
ylabel('Altitude (m)'); title('Goal 1: Hover Stability (1m)');
subplot(2,1,2); plot(t1, x1(:,7:9)*180/pi); grid on;
ylabel('Euler Angles (deg)'); legend('Roll','Pitch','Yaw');
xlabel('Time (s)');
saveas(gcf, 'goal1_hover.png');

%% 3. GOAL 2: CIRCLE TRACKING
fprintf('Running Goal 2: Circle...\n');
t_span2 = [0 60];
% Start on the circle to avoid transient jumps
init_state2 = [2; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]; 

[t2, x2] = ode45(@(t,x) dynamics(t, x, params, 'circle', []), t_span2, init_state2);

% Plot Goal 2
figure('Name','Goal 2','Position',[100 100 800 600]);
plot3(x2(:,1), x2(:,2), x2(:,3), 'b', 'LineWidth', 1.5); grid on; hold on;
t_circ = linspace(0, 2*pi*3, 100);
plot3(2*cos(t_circ), 2*sin(t_circ), ones(size(t_circ)), 'r--');
legend('Drone Path', 'Reference'); title('Goal 2: Circular Trajectory');
axis equal; xlabel('X'); ylabel('Y'); zlabel('Z');
saveas(gcf, 'goal2_circle.png');

%% 4. GOAL 3: COMPLEX MANEUVER
fprintf('Running Goal 3: Complex Maneuver...\n');
t_span3 = [0 40];
init_state3 = zeros(12,1);

[t3, x3] = ode45(@(t,x) dynamics(t, x, params, 'complex', []), t_span3, init_state3);

% Calculate Velocity Magnitude
vel_mag = sqrt(x3(:,4).^2 + x3(:,5).^2 + x3(:,6).^2);
landing_v = abs(x3(end,6)); % Vertical landing speed

% Plot Goal 3 (Detailed: Path, Altitude, Yaw, Velocity)
figure('Name','Goal 3','Position',[100 100 800 800]);

subplot(4,1,1); plot(x3(:,1), x3(:,2), 'b-', 'LineWidth',2); grid on;
ylabel('Y (m)'); title('Top-Down Path'); axis equal;

subplot(4,1,2); plot(t3, x3(:,3), 'r-'); grid on;
ylabel('Altitude (m)');

subplot(4,1,3); plot(t3, x3(:,9)*180/pi, 'k-'); grid on;
ylabel('Yaw (deg)'); yline(90, '--b', 'Target');

subplot(4,1,4); plot(t3, vel_mag, 'm-', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('|V| (m/s)');
title(['Final Landing Velocity: ' num2str(landing_v, '%.4f') ' m/s']);

saveas(gcf, 'goal3_complex.png');

%% DYNAMICS FUNCTION
function dxdt = dynamics(t, x, p, mode, target)
    % Unpack State
    pos = x(1:3); vel = x(4:6); ang = x(7:9); omega = x(10:12);
    phi = ang(1); theta = ang(2); psi = ang(3);

    % Trajectory Generation
    pos_des=[0;0;0]; vel_des=[0;0;0]; psi_des=0;
    
    if strcmp(mode,'hover')
        pos_des = target(1:3); psi_des = target(4);
        
    elseif strcmp(mode,'circle')
        w = 0.25; % 0.5m/s / 2m radius
        pos_des = [2*cos(w*t); 2*sin(w*t); 1];
        vel_des = [-0.5*sin(w*t); 0.5*cos(w*t); 0];
        psi_des = w*t + pi/2; 
        
    elseif strcmp(mode,'complex')
        if t<5, pos_des=[0;0;1];                          % Launch
        elseif t<10, pos_des=[1*(t-5);0;1];               % Forward 5m
        elseif t<12, pos_des=[5;0;1];                     % Stop
        elseif t<15, pos_des=[5;0;1]; psi_des=pi/2;       % Yaw 90
        elseif t<20, pos_des=[5;1*(t-15);1]; psi_des=pi/2;% Lateral 5m
        elseif t<25, pos_des=[5;5;1]; psi_des=pi/2;       % Stop
        else, pos_des=[5;5;0]; psi_des=pi/2; vel_des=[0;0;-0.1]; % Land
           if pos(3)<0.05, vel_des=[0;0;-0.005]; end % Soften landing
        end
    end

    % Controller (PD)
    Kp_pos = [4;4;10]; Kd_pos = [3;3;6];
    acc_cmd = Kd_pos.*(vel_des - vel) + Kp_pos.*(pos_des - pos);
    
    % Mixing
    U1 = p.m * (p.g + acc_cmd(3)); 
    if U1<0, U1=0; end
    
    phi_des = (acc_cmd(1)*sin(psi_des) - acc_cmd(2)*cos(psi_des))/p.g;
    theta_des = (acc_cmd(1)*cos(psi_des) + acc_cmd(2)*sin(psi_des))/p.g;
    
    Kp_att = [20;20;20]; Kd_att = [4;4;4];
    tau = Kp_att.*([phi_des;theta_des;psi_des]-ang) + Kd_att.*([0;0;0]-omega);
    
    % EOM
    R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
         sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
         -sin(theta),         cos(theta)*sin(phi),                            cos(theta)*cos(phi)];
     
    W = [1, 0, -sin(theta); 0, cos(phi), sin(phi)*cos(theta); 0, -sin(phi), cos(phi)*cos(theta)];
    
    acc = [0;0;-p.g] + (R*[0;0;U1])/p.m;
    om_dot = p.I \ (tau - cross(omega, p.I*omega));
    ang_dot = W \ omega;
    
    dxdt = [vel; acc; ang_dot; om_dot];
end