/***********************
转动惯量 J / 粘性阻尼 B 辨识说明（有感FOC，电流环法）

一、这套方法是干什么的
目标：
1) 采集电机在“已知Iq激励”下的动态数据
2) 得到一组可用于辨识的时序数据：t, Iq, omega, alpha
3) 后续在 MATLAB 中做最小二乘，得到 J、B（可选还可得到常值扰动Tl）

适用条件：
1) 电机空载或轻载（你当前场景符合）
2) 已有稳定电流环（你现在是10kHz）
3) 能实时读到 Iq 和速度（你工程有 MyFoc.Iq、MyFoc.speed）

方法核心：
1) 给定正负小电流脉冲（+Iq / -Iq）
2) 记录速度变化，换算角速度omega与角加速度alpha
3) 利用动力学方程建模：
   J * alpha = Kt * Iq - B * omega - Tl

二、为什么要这样做
你现在知道：
- ReadMe里有额定参数（24V、4.5A、4000rpm、扭矩0.19Nm等）
- 但 J、B 是“动态参数”，不是静态铭牌参数，不能直接靠额定点算出来

所以必须做“激励 + 采样 + 回归”。

三、模块文件与接口
文件：
1) `Identify_J_B.h`
2) `Identify_J_B.c`

主要接口：
1) `Id_J_B_DefaultConfig(...)` 载入默认安全参数
2) `Id_J_B_Init(...)` 初始化模块
3) `Id_J_B_Start(...)` 启动辨识采样
4) `Id_J_B_Update_1kHz(...)` 每1ms调用一次（核心）
5) `Id_J_B_Stop(...)` 手动停止
6) `Id_J_B_IsRunning(...)` 是否运行中
7) `Id_J_B_IsFinished(...)` 是否结束
8) `Id_J_B_IsAborted(...)` 是否异常中止

四、模块输入和输出是什么
输入（每1ms喂给模块）：
1) `iq_meas_a`：实际测得Iq（A），建议 `MyFoc.Iq`
2) `speed_rpm`：机械转速（rpm），建议 `MyFoc.speed`
3) `run_flag`：系统运行标志，建议 `Run_Flag`

输出（模块给控制层）：
1) `iq_cmd_a`：辨识阶段输出的Iq命令（A）
2) `takeover`：是否接管Iq命令（1接管 / 0不接管）

采样日志（模块内部自动记录）：
1) `t_s[]`：时间(s)
2) `iq_cmd_a[]`：命令电流(A)
3) `iq_meas_a[]`：实际电流(A)
4) `speed_rpm[]`：机械转速(rpm)
5) `omega_rad_s[]`：机械角速度(rad/s)
6) `alpha_rad_s2[]`：机械角加速度(rad/s^2)
7) `state[]`：状态机状态

五、状态机怎么工作（小白版）
流程是固定的：
1) PREPARE：先给Iq=0，等待电机平稳
2) PULSE_POS：给 +Iq_test 持续一段时间
3) COAST_1：给 0A 滑行
4) PULSE_NEG：给 -Iq_test 持续一段时间
5) COAST_2：给 0A 滑行
6) 循环若干次后 DONE

这么做的好处：
1) 正负对称激励可降低摩擦偏置影响
2) 多次循环可提高数据质量
3) 带限流、限速、run_flag保护更安全

六、如何在你现有工程里使用
步骤：
1) 把 `Identify_J_B.h/.c` 加进工程并参与编译
2) 在 `main.c` 全局定义句柄和输出变量
3) 在初始化阶段调用 `Id_J_B_DefaultConfig` + `Id_J_B_Init`
4) 在 TIM7 1kHz 回调中调用 `Id_J_B_Update_1kHz`
5) 在调用 `IF_OpenLoop(...)` 前决定Iq命令来源：
   - 正常时用原 `Iqref`
   - 若 `takeover=1` 用 `iq_cmd_a`
6) 按键触发 `Id_J_B_Start(...)`
7) `Id_J_B_IsFinished(...)==1` 后，用JScope导出CSV

七、JScope 导出CSV后要做什么
你先不在板子里算J/B，先在 MATLAB 算，流程更直观：
1) 从CSV读取 `iq_meas_a, omega_rad_s, alpha_rad_s2`
2) 用最小二乘拟合：
   alpha = a * Iq + b * omega + c
3) 由系数换算：
   J = Kt / a
   B = -b * J
   Tl = -c * J （可选）

注意：
- Kt口径必须和Iq口径一致
- 你当前项目建议先用 ReadMe 口径：Kt = 0.0422 Nm/A
- 若后续统一为峰值口径再一起换

八、参数建议（按你当前ReadMe和安全优先）
默认建议：
1) `iq_test_a = 1.2A`（可先1.0A试车）
2) `iq_abs_limit_a = 1.8A`（远低于额定4.5A）
3) `speed_safe_rpm = 1200rpm`（约额定30%）
4) `pulse_ms = 220ms`
5) `coast_ms = 350ms`
6) `cycles = 4`

九、常见问题与排查
1) 现象：加速度很小、数据噪声大
   处理：把 `iq_test_a` 从1.0A提高到1.2A或1.5A
2) 现象：经常ABORT
   处理：检查 run_flag 是否掉线；检查 speed_safe_rpm 是否太低
3) 现象：正负脉冲响应不对称很严重
   处理：检查机械摩擦、编码器方向、相序、Iq极性
4) 现象：CSV有数据但回归结果离谱
   处理：先看 `state[]` 是否按状态机正常切换，再剔除瞬态段做回归

十、你最终得到什么
板子阶段输出：
1) 一份高质量CSV（包含 Iq、omega、alpha 时间序列）

MATLAB阶段输出：
1) J（转动惯量）
2) B（粘性阻尼）
3) 可选 Tl（等效常值扰动）

这些参数后续可用于：
1) DOB（扰动观测器）
2) 更精确速度环模型整定
3) 仿真与控制器对比验证
**********************/
 







J B 的 matlab 最小二乘法拟合代码如下：

%% ===== 用户参数 =====
tblName = 'JIdentify';   % 你的table变量名
Kt = 0.07;               % Nm/A（按你当前工程口径）

% 数据清洗参数（可按数据调）
iqCmdMinActive = 0.05;   % 激励判定阈值(A)
speedMinActive = 30;     % 激励判定阈值(rpm)
headDrop = 20;           % 有效段前再丢N点
tailDrop = 20;           % 有效段后再丢N点
stepDetectThr = 0.05;    % iq_cmd跳变检测阈值(A)
transientDrop = 20;      % 每次跳变前后各丢N点
minAbsIqForFit = 0.05;   % 回归前最小|Iq|
minAbsOmegaForFit = 5;   % 回归前最小|omega|(rad/s)

%% ===== 读取工作区table =====
if ~evalin('base', sprintf("exist('%s','var')", tblName))
    error('工作区中不存在变量 %s', tblName);
end

T = evalin('base', tblName);

if ~istable(T)
    error('%s 不是 table 类型', tblName);
end

% 列名标准化（防止大小写和空格问题）
vn = lower(string(T.Properties.VariableNames));
vn = replace(vn, " ", "");
T.Properties.VariableNames = cellstr(vn);

% 必要列检查
needCols = ["t_s","speed_rpm","sample_idx","omega_rad_s","iq_meas_a","iq_cmd_a","alpha_rad_s2"];
for i = 1:numel(needCols)
    if ~any(strcmpi(T.Properties.VariableNames, needCols(i)))
        error('缺少列: %s', needCols(i));
    end
end

%% ===== 取列 =====
t_s       = T.t_s;
speed_rpm = T.speed_rpm;
sample_idx= T.sample_idx;
omega     = T.omega_rad_s;
iq_meas   = T.iq_meas_a;
iq_cmd    = T.iq_cmd_a;
alpha     = T.alpha_rad_s2;

%% ===== 基础有效掩码 =====
baseValid = isfinite(t_s) & isfinite(omega) & isfinite(alpha) & isfinite(iq_meas) & isfinite(iq_cmd);

%% ===== 自动找中间有效段（剔除开头/结尾无效） =====
active = baseValid & (abs(iq_cmd) > iqCmdMinActive | abs(omega) > speedMinActive * 2 * pi / 60);

i1 = find(active, 1, 'first');
i2 = find(active, 1, 'last');

if isempty(i1) || isempty(i2) || i2 <= i1
    error('未找到有效激励段，请检查数据或阈值');
end

i1 = min(i1 + headDrop, i2);
i2 = max(i2 - tailDrop, i1);

valid = false(size(baseValid));
valid(i1:i2) = true;
valid = valid & baseValid;

%% ===== 剔除 iq_cmd 跳变瞬态 =====
idx = find(valid);
iq_seg = iq_cmd(idx);

jumpPos = find(abs(diff(iq_seg)) > stepDetectThr) + 1; % 在seg内部的跳变点

dropMask = false(size(valid));
for k = 1:numel(jumpPos)
    c = idx(jumpPos(k));      % 映射回原始索引
    L = max(1, c - transientDrop);
    R = min(numel(valid), c + transientDrop);
    dropMask(L:R) = true;
end

valid = valid & ~dropMask;

%% ===== 回归前二次筛选 =====
valid = valid & (abs(iq_meas) >= minAbsIqForFit) & (abs(omega) >= minAbsOmegaForFit);

N = nnz(valid);
if N < 50
    error('有效样本太少: %d，请放宽筛选阈值或重新采集', N);
end

%% ===== 最小二乘回归 =====
% 模型：alpha = a*Iq + b*omega + c
X = [iq_meas(valid), omega(valid), ones(N,1)];
y = alpha(valid);

theta = X \ y;
a = theta(1);
b = theta(2);
c = theta(3);

% 参数换算
J  = Kt / a;
B  = -b * J;
Tl = -c * J;

% 拟合优度
yhat = X * theta;
R2 = 1 - sum((y - yhat).^2) / sum((y - mean(y)).^2);

%% ===== 打印结果 =====
fprintf('\n===== 辨识结果 =====\n');
fprintf('table变量: %s\n', tblName);
fprintf('Kt = %.6f Nm/A\n', Kt);
fprintf('N = %d\n', N);
fprintf('a = %.6g, b = %.6g, c = %.6g\n', a, b, c);
fprintf('J  = %.6e kg*m^2\n', J);
fprintf('B  = %.6e N*m*s/rad\n', B);
fprintf('Tl = %.6e N*m\n', Tl);
fprintf('R^2= %.5f\n', R2);

if J <= 0
    warning('J<=0（不物理），请检查Kt口径、筛选阈值和列数据。');
end

%% ===== 画图验证 =====
t_all = t_s;
t_use = t_s(valid);

iq_cmd_all = iq_cmd;
iq_cmd_use = iq_cmd(valid);

iq_meas_all = iq_meas;
iq_meas_use = iq_meas(valid);

omega_all = omega;
omega_use = omega(valid);

alpha_all = alpha;
alpha_use = alpha(valid);

% 1) 数据筛选总览
figure('Name','Data Cleaning Overview','Color','w');
subplot(4,1,1);
plot(t_all, iq_cmd_all, 'c-', 'LineWidth', 1); hold on;
plot(t_use, iq_cmd_use, 'b.', 'MarkerSize', 6);
grid on; ylabel('iq\_cmd (A)');
legend('raw','used','Location','best');
title(sprintf('Data Cleaning (N used = %d)', N));

subplot(4,1,2);
plot(t_all, iq_meas_all, 'k-', 'LineWidth', 1); hold on;
plot(t_use, iq_meas_use, 'g.', 'MarkerSize', 6);
grid on; ylabel('iq\_meas (A)');
legend('raw','used','Location','best');

subplot(4,1,3);
plot(t_all, omega_all, 'm-', 'LineWidth', 1); hold on;
plot(t_use, omega_use, 'r.', 'MarkerSize', 6);
grid on; ylabel('\omega (rad/s)');
legend('raw','used','Location','best');

subplot(4,1,4);
plot(t_all, alpha_all, 'Color',[0.5 0.5 0.5], 'LineWidth', 1); hold on;
plot(t_use, alpha_use, 'b.', 'MarkerSize', 6);
grid on; ylabel('\alpha (rad/s^2)');
xlabel('t (s)');
legend('raw','used','Location','best');

% 2) 拟合对比
figure('Name','Regression Fit','Color','w');
plot(t_use, y, 'k-', 'LineWidth', 1); hold on;
plot(t_use, yhat, 'r--', 'LineWidth', 1.2);
grid on;
xlabel('t (s)');
ylabel('\alpha (rad/s^2)');
legend('measured \alpha','fitted \alpha','Location','best');
title(sprintf('Fit Result: R^2 = %.4f', R2));

% 3) 残差分析
res = y - yhat;
figure('Name','Residual Analysis','Color','w');
subplot(2,1,1);
plot(t_use, res, 'b-');
grid on;
xlabel('t (s)');
ylabel('residual');
title(sprintf('Residual (mean=%.3e, std=%.3e)', mean(res), std(res)));

subplot(2,1,2);
histogram(res, 50);
grid on;
xlabel('residual');
ylabel('count');
title('Residual Histogram');

% 4) alpha散点验证
figure('Name','Alpha Scatter Check','Color','w');
scatter(y, yhat, 8, 'filled'); hold on; grid on;
mn = min([y; yhat]);
mx = max([y; yhat]);
plot([mn mx], [mn mx], 'r--', 'LineWidth', 1.2);
xlabel('Measured \alpha');
ylabel('Fitted \alpha');
title('Scatter Check (Closer to y=x is better)');


