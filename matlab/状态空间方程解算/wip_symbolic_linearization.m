clear; clc;

%% ===== 1. 定义符号变量 =====
syms theta dtheta x dx delta ddelta real
syms ddx ddtheta dddelta real
syms TL TR real

syms M m R L Iw Itheta Idelta D g real

%% ===== 2. 中间参数 =====
A = R*(M + 2*m + 2*Iw/R^2);
B = Itheta + M*L^2;

%% ===== 3. 动力学方程 =====
eq1 = A*ddx == (TL+TR) ...
    - M*R*L*ddtheta*cos(theta) ...
    + M*L*dtheta^2*sin(theta);

eq2 = B*ddtheta == M*g*L*sin(theta) ...
    - M*L*ddx*cos(theta) ...
    - (TL+TR);

%% ===== 4. 求解 ddx 和 ddtheta =====
sol = solve([eq1, eq2], [ddx, ddtheta]);

ddx_expr = simplify(sol.ddx);
ddtheta_expr = simplify(sol.ddtheta);

%% ===== 5. 转向动力学 =====
den_delta = R*(m*D + Iw*D/R^2 + 2*Idelta/D);
dddelta_expr = (TL - TR)/den_delta;

%% ===== 6. 状态向量（新的顺序！）=====
X = [theta; dtheta; x; dx; delta; ddelta];

%% ===== 7. 状态方程 f(X,U) =====
f = sym(zeros(6,1));

f(1) = dtheta;          % theta_dot
f(2) = ddtheta_expr;    % theta_ddot
f(3) = dx;              % x_dot
f(4) = ddx_expr;        % x_ddot
f(5) = ddelta;          % delta_dot
f(6) = dddelta_expr;    % delta_ddot

U = [TL; TR];

%% ===== 8. Jacobian =====
A_sym = jacobian(f, X);
B_sym = jacobian(f, U);

%% ===== 9. 平衡点（直立）=====
eq_point = [theta, dtheta, x, dx, delta, ddelta];
eq_value = [0, 0, 0, 0, 0, 0];

A_lin = simplify(subs(A_sym, eq_point, eq_value));
B_lin = simplify(subs(B_sym, eq_point, eq_value));

%% ===== 10. 输出 =====
disp('===== A矩阵 =====');
disp(A_lin)

disp('===== B矩阵 =====');
disp(B_lin)

%% ===== 11. 导出函数 =====
% matlabFunction(A_lin, B_lin, ...
%     'File','wip_linear_model_reordered', ...
%     'Vars',{M,m,R,L,Iw,Itheta,Idelta,D,g});