clc; clear; close all;

%% ================== 0. 论文绘图参数 ==================
exportDir = fullfile(pwd, 'paper_figures_pdf');
if ~exist(exportDir, 'dir')
    mkdir(exportDir);
end

%% ================== 1. 原始数据 ==================
% 单位：
% 坐标：mm
% 惯量：g*mm^2

m = 0.3;  % kg

xyz_mm = [
0.130098967, -2.062074067, 30.581185585;
0.130098968, -2.344496804, 35.948556341;
0.130098967, -2.644809395, 41.285729291;
0.130098967, -2.95266012 , 47.122124913;
0.130098968, -3.206381809, 52.954859608;
0.130098968, -3.436791379, 60.726054458;
0.130098967, -3.59228247 , 68.957943589;
0.130110483, -3.870786882, 76.847155053;
0.130098967, -4.338690063, 81.641435889;
];

Jc_gmm2 = [
310700; 315200; 320500; 327000; 334300;
345200; 358300; 372200; 381300
];

Jo_gmm2 = [
600600; 715700; 848600; 1015000; 1203000;
1487000; 1830000; 2199000; 2444000
];

%% ================== 2. 单位转换 ==================
xyz = xyz_mm * 1e-3;     % m
Jc = Jc_gmm2 * 1e-9;     % kg*m^2
Jo = Jo_gmm2 * 1e-9;     % kg*m^2

x = xyz(:,1);
y = xyz(:,2);
z = xyz(:,3);

%% ================== 3. 计算质心到轮轴距离 ==================
% 假设轮轴沿 x 轴
d = sqrt(y.^2 + z.^2);

%% ================== 4. 一致性检查 ==================
Jo_check = Jc + m * d.^2;
err = Jo - Jo_check;

fprintf('=== 一致性检查 ===\n');
fprintf('最大误差: %.3e\n', max(abs(err)));

figure;
plot(d, err, 'o-','LineWidth',1.5);
xlabel('质心到轮轴距离 L (m)');
ylabel('惯量误差');
title('平行轴定理一致性验证');
grid on;
prepareFigureForPaper(gcf, exportDir);

%% ================== 5. 分离刚性项 ==================
Jc_from_Jo = Jo - m * d.^2;

figure;
plot(d, Jc, 'o','LineWidth',1.5); hold on;
plot(d, Jc_from_Jo, 'x');
legend('原始Jc','Jo反推Jc', 'Location', 'southeast');
title('Jc一致性验证');
grid on;
prepareFigureForPaper(gcf, exportDir);

%% ================== 6. 拟合 ==================
% 二阶 & 三阶对比
p2 = polyfit(d, Jc_from_Jo, 2);
p3 = polyfit(d, Jc_from_Jo, 3);

Jc_fit2 = @(d) polyval(p2, d);
Jc_fit3 = @(d) polyval(p3, d);

Jo_fit2 = @(d) Jc_fit2(d) + m*d.^2;
Jo_fit3 = @(d) Jc_fit3(d) + m*d.^2;

%% ================== 7. 拟合效果 ==================
d_dense = linspace(min(d), max(d), 200);

figure;
plot(d, Jo, 'ko','MarkerSize',8,'LineWidth',1.5); hold on;
% plot(d_dense, Jo_fit2(d_dense), 'b--','LineWidth',2);
plot(d_dense, Jo_fit3(d_dense), 'r-','LineWidth',2);
plot(d_dense, m*d_dense.^2, 'k:','LineWidth',2);

% legend('实测数据','二阶拟合','三阶拟合','m d^2项','Location','northwest');
legend('实测数据','三阶拟合','M·L^2项','Location','southeast');
xlabel('质心到轮轴距离 L (m)');
ylabel('转动惯量 J (kg·m^2)');
title('整机转动惯量随质心位置变化关系');
grid on;
prepareFigureForPaper(gcf, exportDir);

%% ================== 8. 非线性项 ==================
figure;
plot(d_dense, Jo_fit3(d_dense) - m*d_dense.^2, 'LineWidth',2);
xlabel('质心到轮轴距离 L (m)');
ylabel('构型相关惯量 J_c(L) (kg·m^2)');
title('腿部结构引起的非线性惯量变化');
grid on;
prepareFigureForPaper(gcf, exportDir);

%% ================== 9. 归一化分析 ==================
figure;
plot(d_dense, Jo_fit3(d_dense)./(m*d_dense.^2), 'LineWidth',2);
xlabel('质心到轮轴距离 d (m)');
ylabel('归一化惯量 J / (m d^2)');
title('归一化转动惯量分析');
grid on;
prepareFigureForPaper(gcf, exportDir);

%% ================== 10. 质心轨迹 ==================
figure;
plot(z*1e3, y*1e3, 'o-','LineWidth',1.5);
xlabel('z 方向坐标 (mm)');
ylabel('y 方向坐标 (mm)');
title('质心空间运动轨迹');
grid on;
axis equal;
prepareFigureForPaper(gcf, exportDir);

%% ================== 11. 单调性检查 ==================
dJo = gradient(Jo_fit3(d_dense), d_dense);

if any(dJo < 0)
    warning('存在非单调区间！');
else
    fprintf('单调性OK ✔\n');
end

%% ================== 12. 输出表达式 ==================
fprintf('\n=== 三阶 Jc(d) ===\n');
fprintf('%.6e*d^3 + %.6e*d^2 + %.6e*d + %.6e\n', ...
    p3(1), p3(2), p3(3), p3(4));

fprintf('\n=== 最终 Jo(d) ===\n');
fprintf('J(d) = %.6e*d^3 + %.6e*d^2 + %.6e*d + %.6e\n', ...
    p3(1), p3(2)+m, p3(3), p3(4));

%% ================== 13. 保存 ==================
save('inertia_model.mat','p2','p3','m');

function prepareFigureForPaper(figHandle, exportDir)
figWidthCm = 16;
figHeightCm = 10;
axisFontSize = 14;
labelFontSize = 16;
titleFontSize = 16;
legendFontSize = 13;

set(figHandle, ...
    'Color', 'w', ...
    'Units', 'centimeters', ...
    'Position', [2, 2, figWidthCm, figHeightCm], ...
    'PaperPositionMode', 'auto');

axList = findall(figHandle, 'Type', 'axes');
for i = 1:numel(axList)
    set(axList(i), ...
        'FontName', 'Times New Roman', ...
        'FontSize', axisFontSize, ...
        'LineWidth', 1.1);
    if isprop(axList(i), 'Toolbar') && ~isempty(axList(i).Toolbar)
        axList(i).Toolbar.Visible = 'off';
    end

    xlabelHandle = get(axList(i), 'XLabel');
    ylabelHandle = get(axList(i), 'YLabel');
    titleHandle = get(axList(i), 'Title');

    set(xlabelHandle, 'FontName', 'Times New Roman', 'FontSize', labelFontSize);
    set(ylabelHandle, 'FontName', 'Times New Roman', 'FontSize', labelFontSize);
    set(titleHandle, 'FontName', 'Times New Roman', 'FontSize', titleFontSize, 'FontWeight', 'bold');
end

legendList = findall(figHandle, 'Type', 'legend');
for i = 1:numel(legendList)
    set(legendList(i), ...
        'FontName', 'Times New Roman', ...
        'FontSize', legendFontSize, ...
        'Location', 'southeast', ...
        'Box', 'off');
end

lineList = findall(figHandle, 'Type', 'line');
for i = 1:numel(lineList)
    if strcmp(get(lineList(i), 'Marker'), 'none')
        continue;
    end
    currentMarkerSize = get(lineList(i), 'MarkerSize');
    if currentMarkerSize < 9
        set(lineList(i), 'MarkerSize', 9);
    end
end

drawnow;
pdfPath = buildPdfPathFromTitle(figHandle, exportDir);
exportgraphics(figHandle, pdfPath, 'ContentType', 'vector');
end

function pdfPath = buildPdfPathFromTitle(figHandle, exportDir)
axList = findall(figHandle, 'Type', 'axes');
titleText = 'figure';
for i = 1:numel(axList)
    rawTitle = get(get(axList(i), 'Title'), 'String');
    if iscell(rawTitle)
        rawTitle = strjoin(string(rawTitle), ' ');
    end
    if isstring(rawTitle)
        rawTitle = join(rawTitle, " ");
    end
    rawTitle = char(string(rawTitle));
    rawTitle = strtrim(strrep(rawTitle, newline, ' '));
    if ~isempty(rawTitle)
        titleText = rawTitle;
        break;
    end
end

safeTitle = regexprep(titleText, '[<>:"/\\|?*]', '_');
pdfPath = fullfile(exportDir, [safeTitle, '.pdf']);
end
