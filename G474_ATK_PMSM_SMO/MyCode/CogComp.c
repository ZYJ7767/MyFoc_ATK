/*

%% Cogging / periodic disturbance LUT generator
clear; clc; close all;

%% ===== User config =====
csvFile = "your_jscope_export.csv";   % 改成你的CSV文件名

LUT_SIZE = 256;
ENCODER_PPR = 4000;

targetSpeedRpm = 1000;       % 你采集时的目标速度
speedTolRpm = 100;           % 稳态速度筛选范围
dropHeadRatio = 0.10;        % 丢弃前10%
dropTailRatio = 0.05;        % 丢弃后5%

smoothWindow = 7;            % 周期平滑窗口
compSign = 1.0;              % 补偿变差就改成 -1.0

%% ===== Read CSV =====
T = readtable(csvFile);

names = lower(string(T.Properties.VariableNames));
names = replace(names, " ", "");
names = replace(names, "_", "");
names = replace(names, "-", "");
T.Properties.VariableNames = cellstr(names);

disp("Detected columns:");
disp(string(T.Properties.VariableNames).');

%% ===== Use your columns directly =====
raw = double(T.rawvalue);
theta = double(T.mechangle);
encSpeed = double(T.encspeed);
iqref = double(T.iqref);
iq = double(T.iq);
id = double(T.id);
speedref = double(T.speedref);

raw = mod(round(raw), ENCODER_PPR);
theta = mod(theta, 2*pi);

%% ===== Select stable samples =====
N = height(T);

valid = isfinite(raw) & isfinite(iqref) & isfinite(encSpeed);

i1 = floor(N * dropHeadRatio) + 1;
i2 = ceil(N * (1.0 - dropTailRatio));

idxRange = false(N, 1);
idxRange(i1:i2) = true;
valid = valid & idxRange;

% 稳态速度筛选
valid = valid & (abs(abs(encSpeed) - abs(targetSpeedRpm)) <= speedTolRpm);

if nnz(valid) < LUT_SIZE * 5
    warning("有效样本偏少: %d。建议采更长时间或放宽 speedTolRpm。", nnz(valid));
end

%% ===== Angle bin average =====
posForBin = raw * LUT_SIZE / ENCODER_PPR;
bin = floor(posForBin) + 1;
bin(bin < 1) = 1;
bin(bin > LUT_SIZE) = LUT_SIZE;

binValid = bin(valid);
iqValid = iqref(valid);

sumIq = accumarray(binValid, iqValid, [LUT_SIZE 1], @sum, 0);
cntIq = accumarray(binValid, 1,       [LUT_SIZE 1], @sum, 0);

tableRaw = nan(LUT_SIZE, 1);
for k = 1:LUT_SIZE
    if cntIq(k) > 0
        tableRaw(k) = sumIq(k) / cntIq(k);
    end
end

%% ===== Fill empty bins periodically =====
good = isfinite(tableRaw);
if nnz(good) < 2
    error("有效角度bin太少，无法插值。请采集更长时间。");
end

xi = (1:LUT_SIZE).';
xGood = xi(good);
yGood = tableRaw(good);

xExt = [xGood - LUT_SIZE; xGood; xGood + LUT_SIZE];
yExt = [yGood; yGood; yGood];

tableFilled = interp1(xExt, yExt, xi, "linear");

%% ===== Remove DC and smooth periodically =====
tableZeroMean = tableFilled - mean(tableFilled, "omitnan");

if mod(smoothWindow, 2) == 0
    smoothWindow = smoothWindow + 1;
end

half = floor(smoothWindow / 2);
xSmoothExt = [tableZeroMean(end-half+1:end); tableZeroMean; tableZeroMean(1:half)];
ySmoothExt = movmean(xSmoothExt, smoothWindow);
tableSmooth = ySmoothExt(half+1:half+LUT_SIZE);

tableFinal = compSign * tableSmooth;

%% ===== Summary =====
fprintf("\n===== LUT generation summary =====\n");
fprintf("CSV file: %s\n", csvFile);
fprintf("LUT size: %d\n", LUT_SIZE);
fprintf("Total samples: %d\n", N);
fprintf("Valid samples: %d\n", nnz(valid));
fprintf("Iqref mean before zero-mean: %.6f A\n", mean(tableFilled, "omitnan"));
fprintf("LUT peak-to-peak: %.6f A\n", max(tableFinal) - min(tableFinal));
fprintf("LUT max abs: %.6f A\n", max(abs(tableFinal)));

%% ===== Plots =====
xDeg = (0:LUT_SIZE-1).' * 360 / LUT_SIZE;

figure("Name", "Raw Data Overview", "Color", "w");

subplot(4,1,1);
plot(iqref, "Color", [0.2 0.2 0.2]); hold on;
plot(find(valid), iqref(valid), ".", "MarkerSize", 4);
grid on; ylabel("Iqref A");
title("Raw data and selected valid samples");

subplot(4,1,2);
plot(encSpeed, "b"); hold on;
plot(find(valid), encSpeed(valid), ".", "MarkerSize", 4);
grid on; ylabel("EncSpeed rpm");

subplot(4,1,3);
plot(speedref, "m");
grid on; ylabel("Speedref rpm");

subplot(4,1,4);
plot(theta, "g");
grid on; ylabel("MechAngle rad"); xlabel("sample");

figure("Name", "Cogging LUT", "Color", "w");
plot(xDeg, tableZeroMean, "Color", [0.6 0.6 0.6], "LineWidth", 1); hold on;
plot(xDeg, tableFinal, "r", "LineWidth", 1.5);
grid on;
xlabel("Mechanical angle deg");
ylabel("Iq compensation A");
legend("zero-mean raw LUT", "smoothed final LUT");
title("Periodic Iq compensation table");

figure("Name", "Bin Counts", "Color", "w");
bar(xDeg, cntIq);
grid on;
xlabel("Mechanical angle deg");
ylabel("samples per bin");
title("Samples per angle bin");

%% ===== Print C array =====
fprintf("\n===== C array =====\n");
fprintf("const float CogComp_Table[%d] = {\n", LUT_SIZE);

for i = 1:LUT_SIZE
    if mod(i-1, 8) == 0
        fprintf("    ");
    end

    if i < LUT_SIZE
        fprintf("% .7ff, ", tableFinal(i));
    else
        fprintf("% .7ff", tableFinal(i));
    end

    if mod(i, 8) == 0 || i == LUT_SIZE
        fprintf("\n");
    end
end

fprintf("};\n");

%% ===== Save result =====
outTable = table((0:LUT_SIZE-1).', xDeg, tableRaw, tableZeroMean, tableFinal, cntIq, ...
    'VariableNames', {'bin', 'angle_deg', 'iq_bin_raw', 'iq_zero_mean', 'iq_lut_final', 'sample_count'});

writetable(outTable, "cogging_lut_result.csv");
save("cogging_lut_result.mat", "tableFinal", "tableZeroMean", "tableRaw", "cntIq", "LUT_SIZE", "ENCODER_PPR");

fprintf("\nSaved: cogging_lut_result.csv\n");
fprintf("Saved: cogging_lut_result.mat\n");

*/