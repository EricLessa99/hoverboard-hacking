%% Testes com modelo do motor
%% Limpando workspace
clear; clc; close all;

%% Carrega struct do hover
load('hover.mat')
motor = hover.motor;
eps_i = motor.noload_current*1e-3;

%% Criacao das matrizes de espaco de estados
% x = [ia ib]'
Rm = motor.Rm; Lm = motor.Lm;
A = diag([-Rm/Lm -Rm/Lm]);
B = [ 2/(3*Lm) 1/(3*Lm);
     -1/(3*Lm) 1/(3*Lm)];

% y = [ia ib ic]'
C = [ 1  0;
      0  1;
     -1 -1];
D = zeros(size(C, 1), size(B, 2));

%% Carrega dados do motor
Vs = motor.Vs; % V
p = motor.p; % Numero de polos
stall_torque = motor.stall_torque; % Nm
noload_speed = motor.noload_speed; % rad/s
noload_current = motor.noload_current; % H
Ke = motor.Ke; % V/rad/s
Kt = motor.Kt; % Nm/A;
Kf = motor.Kf; % Nm/rad/s
J_w = 0.00701; % kgm²
motor.Vs = Vs; motor.p = p; motor.Rm = Rm; motor.Lm = Lm;
motor.J_w = J_w; motor.Kt = Kt; motor.Ke = Ke; motor.Kf = Kf;
motor.stall_torque = stall_torque; motor.noload_speed = noload_speed;
motor.noload_current = noload_current;

%% Teste 1 - Commutation Logic
Tend = 0.3;
f = 200e3; % [Hz] Model sampling frequency (200 kHz)
Ts = 1/f; % [s] Model sampling time
f_ctrl = 16e3; % [Hz] Controller frequency
Ts_ctrl = 1/f_ctrl; % [s] Controller sampling time
vec_hallToPos       = [0 2 0 1 4 3 5 0];  % [-] Mapping Hall signal to position
t = (0:0.01:Tend)';
path = trajectorygeneration([0 0.1*noload_speed 0.25*noload_speed 0.25*noload_speed]', [0.05 0.1 0.15]', 0.01);
omega_ref = [t, 1*noload_speed*ones(size(t))];
load_torque = [t, zeros(size(t))];
J_m = J_w;
ControlMode = 1;

myoptions = simset('SrcWorkspace','current',...
                       'DstWorkspace','current',...
                       'ReturnWorkspaceOutputs', 'on');
noload_simOut = sim('BLDCSym.slx', Tend, myoptions);

rs_noload_SimOut = resizeSimFields(noload_simOut);

plotSimData(rs_noload_SimOut, 'Omega_ref',omega_ref)