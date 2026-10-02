clear; clc;

%% ===== 1. 物理参数（占位，后续自行修改）=====
M = 0.4011 - 2 * 0.0594;;          % 机身质量 kg
m = 2 * 0.0594;        % 单轮质量 kg
R = 0.0526 / 2;        % 轮半径 m
L = 0.03;        % 质心高度 m
Iw = 4.746798874999999e-06;      % 轮转动惯量
Itheta = 0.2;   % 机身转动惯量
Idelta = 2.924000000000000e-04;  % 转向惯量
D = 0.1142 / 2;        % 轮距
g = 9.81;

%% ===== 2. 公共分母 =====
Den = 2*Itheta*Iw + 2*Iw*L^2*M + Itheta*M*R^2 ...
    + 2*Itheta*R^2*m + 2*L^2*M*R^2*m;

%% ===== 3. A矩阵 =====
A = zeros(6);

A(1,2) = 1;

A(2,1) = (L*M*g*(2*Iw + M*R^2 + 2*R^2*m)) / Den;

A(3,4) = 1;

A(4,1) = -(L^2*M^2*R^2*g) / Den;

A(5,6) = 1;

%% ===== 4. B矩阵 =====
B = zeros(6,2);

B(2,1) = -(2*Iw + M*R^2 + 2*R^2*m + L*M*R) / Den;
B(2,2) = B(2,1);

B(4,1) = (R*(M*L^2 + M*R*L + Itheta)) / Den;
B(4,2) = B(4,1);

B(6,1) =  (D*R)/(m*D^2*R^2 + Iw*D^2 + 2*Idelta*R^2);
B(6,2) = -(D*R)/(m*D^2*R^2 + Iw*D^2 + 2*Idelta*R^2);

%% ===== 5. LQR 权重矩阵 =====
Q = diag([
    100,   ... % theta（最重要）
    10,    ... % dtheta
    10,    ... % x
    1,     ... % dx
    50,    ... % delta
    1      ... % ddelta
]);

R_mat = diag([1, 1]);   % 左右轮控制代价

%% ===== 6. 可控性检查 =====
Co = ctrb(A,B);
rank_Co = rank(Co);

fprintf('可控性矩阵秩 = %d\n', rank_Co);

if rank_Co < size(A,1)
    warning('系统不可控，请检查模型！');
end

%% ===== 7. LQR 求解 =====
K = lqr(A, B, Q, R_mat);

disp('===== LQR 增益 K =====');
disp(K);

%% ===== 8. 闭环特征值 =====
Acl = A - B*K;
eig_vals = eig(Acl);

disp('===== 闭环极点 =====');
disp(eig_vals);