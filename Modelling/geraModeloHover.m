%% Inicializacao e configuracao
clear; close all; clc;

%% Declarações de símbolos e variáveis
% Tempo
syms t real

% Inicialização do ângulo de tombamento e angulos das rodas
syms theta(t) theta_w0(t) theta_w1(t)

% Parametros do hover
syms r d mb mw Iw Ixx Iyy Izz g L alpha real

I = sym(zeros(3, 3));
I(1, 1) = Ixx; I(2, 2) = Iyy; I(3, 3) = Izz;

% Definição das velocidades e acelerações
theta = theta(t);
theta_w0 = theta_w0(t);
theta_w1 = theta_w1(t);
theta_dot = diff(theta, t);
omega_w0 = diff(theta_w0);
omega_w1 = diff(theta_w1);
theta_ddot = diff(theta_dot, t);
omega_dot_w0 = diff(omega_w0);
omega_dot_w1 = diff(omega_w1);

% Matrizes de rotação corpo->chassi e chassi->corpo
R_brelc = sym(zeros(3, 3));
R_brelc(1, :) = [ cos(theta) 0 sin(theta)];
R_brelc(2, :) = [     0      1    0      ];
R_brelc(3, :) = [-sin(theta) 0 cos(theta)];

R_crelb = transpose(R_brelc);

% Velocidade translacional das rodas
V_w0 = omega_w0*r;
V_w1 = omega_w1*r;

% Velocidade do sistema do chassi
CVOc = (V_w0 + V_w1)/2;
Comegaz = (V_w0 - V_w1)/d;

% Velocidade angular do corpo no refererencial do corpo e chassi
ComegaB = sym(zeros(3, 1));
ComegaB(2) = theta_dot; ComegaB(3) = Comegaz;
BomegaB = R_crelb*ComegaB;

% Posição do CG relativa ao corpo e chassi
pCG_B = [L*sin(alpha); 0; L*cos(alpha)];
pCG_C = simplify(R_brelc*pCG_B);

% Velocidade do CG em relação ao chassi
CVcg = [CVOc; 0; 0] + cross(ComegaB, pCG_C);
CVcg = simplify(expand(CVcg));

% Energia cinética das rodas
K_w0 = simplify(expand(0.5*(transpose(V_w0)*mw*V_w0 + omega_w0*Iw*omega_w0)));
K_w1 = simplify(expand(0.5*(transpose(V_w1)*mw*V_w1 + omega_w1*Iw*omega_w1)));

% Energia potencial das rodas
U_w0 = sym(0);
U_w1 = sym(0);

% Energia cinética do corpo
K_Bt = simplify(expand(0.5*transpose(CVcg)*mb*CVcg));
K_Br = simplify(expand(0.5*transpose(BomegaB)*I*BomegaB));

% Energia potencial do corpo
U_B = mb*g*pCG_C(3);

% Coordenadas Generalizadas
genCoord = [theta_w0 theta_w1 theta];
n_coord = length(genCoord);

% Contribuições 
LHS_K_w0 = sym(zeros(3, 1));
LHS_K_w1 = sym(zeros(3, 1));
LHS_K_Bt = sym(zeros(3, 1));
LHS_K_Br = sym(zeros(3, 1));
LHS_U_w0 = sym(zeros(3, 1));
LHS_U_w1 = sym(zeros(3, 1));
LHS_U_B = sym(zeros(3, 1));
LHS = sym(zeros(3, 1));
for i = 1:n_coord
    % LHS_K_w0(i) = symLagr(K_w0, genCoord(i), t);
    % LHS_K_w1(i) = symLagr(K_w1, genCoord(i), t);
    LHS_K_Bt(i) = symLagr(K_Bt, genCoord(i), t);
    LHS_K_Br(i) = symLagr(K_Br, genCoord(i), t);
    LHS_U_w0(i) = symLagr(U_w0, genCoord(i), t);
    LHS_U_w1(i) = symLagr(U_w1, genCoord(i), t);
    LHS_U_B(i)  = symLagr(U_B, genCoord(i), t);
    LHS(i) = LHS_K_w0(i) + LHS_K_w1(i) + LHS_K_Bt(i) + LHS_K_Br(i)...
        - (LHS_U_w0(i) + LHS_U_w1(i) + LHS_U_B(i));
end
LHS = simplify(expand(LHS));

% Define simbolos que não dependem do tempo
syms Theta Theta_w0 Theta_w1 real
syms Theta_dot Omega_w0 Omega_w1 real
syms Theta_ddot Omega_dot_w0 Omega_dot_w1 real

% Substitui angulos e derivadas funções do tempo por angulos e derivadas
% quaisquer

old_vars = [omega_dot_w0 omega_dot_w1 theta_ddot ...
    omega_w0 omega_w1 theta_dot ...
    theta_w0 theta_w1 theta];
new_vars = [Omega_dot_w0 Omega_dot_w1 Theta_ddot...
    Omega_w0 Omega_w1 Theta_dot...
    Theta_w0 Theta_w1 Theta];

LHS = subsExpression(LHS, old_vars, new_vars);

%% Matrizes do LHS do modelo não linear
thetas_ddot = [Omega_dot_w0 Omega_dot_w1 Theta_ddot]'; % Vetor de acelerações
M = simplify(jacobian(LHS, thetas_ddot)); %Matriz de Inércia
invM = simplify(inv(M), 1000); % Matriz inversa de inércia
G = simplify(expand(jacobian(LHS - M*thetas_ddot, g)*g)); %vetor de termos gravitacionais
V = simplify(LHS - G - M*thetas_ddot, 1000); % Matriz de efeitos Coriolis e Centrifuga

%% Exportacao das matrizes M, V e G para serem importadas no modelo não linear do simulink
matlabFunction(invM, 'File', 'function_invM_hover');
matlabFunction(V, 'File', 'function_V_hover');
matlabFunction(G, 'File', 'function_G_hover');

%% Lineariza o modelo em torno de um estado qualquer
% vetor de estados x = [Theta Omega_w0 Omega_w1 Theta_dot]
% x_dot = [Theta_dot Omega_dot_w0 Omega_dot_w1 Theta_ddot]
% vetor de controle u = [lbd_0 lbd_1];

x = [Theta Omega_w0 Omega_w1 Theta_dot]; % Define vetor de estado
syms lbd_0 lbd_1 real % Define variaveis de controle
syms stall_torque noload_speed real
T_w0 = lbd_0*((-stall_torque/noload_speed)*Omega_w0 + stall_torque); % Define tracao da roda 0
T_w1 = lbd_1*((-stall_torque/noload_speed)*Omega_w1 + stall_torque); % Define tracao da roda 1
u = [lbd_0 lbd_1]'; % Define vetor com variaveis de controle

x_dot = [Theta_dot; invM*([T_w0 T_w1 0]' - G - V)]; % equação não linear
A_sym = jacobian(x_dot, x);
B_sym = jacobian(x_dot, u);

%% Linearizando em torno do ponto x0 = [-alpha Omega_w0 Omega_w1 0]' e u0 = [0 0]'
Theta = -alpha;
Theta_dot = 0;
lbd_0 = 0;
lbd_1 = 0;

A = subs(A_sym);
B = subs(B_sym);
%C = eye(length(x));
%D = zeros(length(u), length(x));

matlabFunction(A, 'File', 'function_A_hover');
matlabFunction(B, 'File', 'function_B_hover');

%% Carrega as geometrias dos modelos em CAD
folderPath = "CAD Files";

gm{1} = importGeometry(fullfile(pwd, folderPath, "Rotor 0.stl"));
gm{2} = importGeometry(fullfile(pwd, folderPath, "Rotor 1.stl"));
gm{3} = importGeometry(fullfile(pwd, folderPath, "Estrutura.stl"));
hover.gm = gm;

%% Parametros geometricos
hover.r = 6.5*25.4/(2*1000);
hover.d = 300/1000;

%% Motor
Vs = 36; % V
p = 30; % Numero de polos
max_current = 13.22; % A
stall_torque = 13.21; % Nm
noload_speed = (pi/30)*624.4; % rad/s
noload_current = 0.987; % A
Rm = Vs/(2*max_current); % ohm - valor de uma única fase
Lm = 391e-6; % H
Ke = (Vs - 2*Rm*noload_current)/noload_speed; % V/rad/s
Kt = stall_torque*2*Rm/Vs; % Nm/A;
Kf = Kt*noload_current/noload_speed; % Nm/rad/s
J_w = 0.00701; % kgm²
motor.Vs = Vs; motor.p = p; motor.Rm = Rm; motor.Lm = Lm;
motor.J_w = J_w; motor.Kt = Kt; motor.Ke = Ke; motor.Kf = Kf;
motor.stall_torque = stall_torque; motor.noload_speed = noload_speed;
motor.noload_current = noload_current;
hover.motor = motor;

%% Salva struct
save('hover.mat', 'hover');