function result = wip_gain_export(output_file)
% 为 esp32/test 的实际反馈状态生成离散 LQR 高度调度参数。
% 需要 Control System Toolbox。输出可直接导入参数网页。
% 左右轮输入力矩均以前进为正；右正/左负力矩对应正 IMU 偏航。
% 固件通过左右轮方向配置转换到 FOC 坐标，IMU 坐标仍需实机确认。
if nargin == 0
    output_file = fullfile(fileparts(mfilename('fullpath')), 'gain_poly_candidate.json');
end

% 固定硬件物理参数：m 是单轮质量，不是两轮质量之和。
M = 0.4011 - 2 * 0.0594;
m = 0.0594;
r = 0.0263;
D = 0.0571;
Iw = 4.746798875e-6;
Iyaw = 2.924e-4;
g = 9.81;
Ts = 0.001;
L_vec = linspace(0.031, 0.082, 200);

% CAD 数据中的质心惯量 Jc，不能把轮轴惯量 Jo 再加 M*L^2。
% 来源：高度与绕轮轴转动惯量拟合/test.m。质量或结构调整后应重新导出 CAD 数据。
yz_mm = [-2.062074067,30.581185585; -2.344496804,35.948556341; ...
    -2.644809395,41.285729291; -2.95266012,47.122124913; ...
    -3.206381809,52.954859608; -3.436791379,60.726054458; ...
    -3.592282470,68.957943589; -3.870786882,76.847155053; ...
    -4.338690063,81.641435889];
Jc_data = [310700;315200;320500;327000;334300;345200;358300;372200;381300] * 1e-9;
Jc_poly = polyfit(vecnorm(yz_mm,2,2) * 1e-3, Jc_data, 3);

% 反馈顺序：[theta, theta_dot, v-v_ref, yaw_dot-yaw_ref, Iv, Iyaw]。
% Iv_dot=v_ref-v，Iyaw_dot=yaw_ref-yaw_dot。
% Bryson 权重表示允许偏差；单轮力矩按当前电机的可用物理力矩设置。
allowable_state = [deg2rad(5), deg2rad(150), 0.30, 2.0, 0.08, 0.30];
allowable_torque_Nm = 0.020;
Q = diag(1 ./ allowable_state.^2);
Ru = eye(2) / allowable_torque_Nm^2;
G = zeros(2,6,numel(L_vec));
models = cell(numel(L_vec),2);
for i = 1:numel(L_vec)
    L = L_vec(i);
    Jc = polyval(Jc_poly,L);
    den = 2*Jc*Iw + 2*Iw*L^2*M + Jc*M*r^2 + 2*Jc*r^2*m + 2*L^2*M*r^2*m;
    A = zeros(6);
    A(1,2) = 1;
    A(2,1) = L*M*g*(2*Iw + M*r^2 + 2*r^2*m) / den;
    A(3,1) = -L^2*M^2*r^2*g / den;
    A(5,3) = -1;
    A(6,4) = -1;
    B = zeros(6,2);
    % 输入为前进轮力矩，俯仰直接采用固件 IMU Y 轴。
    B(2,:) = -(2*Iw + M*r^2 + 2*r^2*m + L*M*r) / den;
    B(3,:) = r*(M*L^2 + M*r*L + Jc) / den;
    yaw_gain = D*r / (m*D^2*r^2 + Iw*D^2 + 2*Iyaw*r^2);
    B(4,:) = [-yaw_gain, yaw_gain];
    discrete = c2d(ss(A,B,eye(6),zeros(6,2)), Ts, 'zoh');
    K = dlqr(discrete.A,discrete.B,Q,Ru);
    G(:,:,i) = -K; % 固件直接执行 torque = gain * feedback。
    models{i,1} = discrete.A;
    models{i,2} = discrete.B;
end

poly = zeros(2,6,4);
max_fit_error = 0;
max_pole_radius = 0;
for side = 1:2
    for state = 1:6
        y = squeeze(G(side,state,:))';
        p = polyfit(L_vec,y,3);
        % 数值接近常数的项归零高阶系数，便于审阅。
        if(max(y)-min(y) < 1e-9), p = [0,0,0,mean(y)]; end
        poly(side,state,:) = p;
        max_fit_error = max(max_fit_error,max(abs(polyval(p,L_vec)-y)));
    end
end
% 必须检查 float32 系数和高度在整个调度范围的离散闭环稳定性。
poly = double(single(poly));
for i = 1:numel(L_vec)
    fitted = zeros(2,6);
    for side = 1:2
        for state = 1:6
            fitted(side,state) = polyval(squeeze(poly(side,state,:))',single(L_vec(i)));
        end
    end
    max_pole_radius = max(max_pole_radius,max(abs(eig(models{i,1}+models{i,2}*fitted))));
end
assert(max_pole_radius < 1,'三次拟合后的闭环不稳定，请缩小范围或调整权重。');
result.version = 1;
% 显式嵌套向量，保证 JSON 次序为 [左/右][六状态][四系数]。
result.balance.gain_poly = cell(1,2);
for side = 1:2
    rows = cell(1,6);
    for state = 1:6
        rows{state} = reshape(poly(side,state,:),1,4);
    end
    result.balance.gain_poly{side} = rows;
end
result.balance.height_min_m = min(L_vec);
result.balance.height_max_m = max(L_vec);
result.balance.model_height_m = 0.048;
result.balance.torque_scale = 1;
result.motor.torque_limit_Nm = 0.025;
fid = fopen(output_file,'w');
assert(fid >= 0,'无法打开输出文件。');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));
fprintf('已导出 %s\n最大拟合误差 %.6g，最大离散极点模 %.9f\n', ...
    output_file,max_fit_error,max_pole_radius);
fprintf('这是物理模型候选值，需要验证转矩常数、坐标方向、质心高度和实机效果。\n');
end
