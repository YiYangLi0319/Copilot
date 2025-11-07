%% =====================================================================
%  性能指标计算工具
%  Metrics Computation Utilities
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  功能：
%  1. RMSE、MAE等回归指标
%  2. 准确率、精确率、召回率、F1等分类指标
%  3. 混淆矩阵
%  4. 跟踪性能指标（OSPA、GOSPA等）
%  =====================================================================

%% ================ RMSE (Root Mean Square Error) ================

function rmse = compute_rmse(predictions, ground_truth)
    % 计算均方根误差
    %
    % 输入:
    %   predictions - 预测值 [N x D]
    %   ground_truth - 真实值 [N x D]
    %
    % 输出:
    %   rmse - 均方根误差
    
    if size(predictions) ~= size(ground_truth)
        error('预测值和真实值的尺寸必须相同');
    end
    
    squared_errors = (predictions - ground_truth).^2;
    rmse = sqrt(mean(squared_errors, 'all'));
end

%% ================ MAE (Mean Absolute Error) ================

function mae = compute_mae(predictions, ground_truth)
    % 计算平均绝对误差
    %
    % 输入:
    %   predictions - 预测值 [N x D]
    %   ground_truth - 真实值 [N x D]
    %
    % 输出:
    %   mae - 平均绝对误差
    
    if size(predictions) ~= size(ground_truth)
        error('预测值和真实值的尺寸必须相同');
    end
    
    absolute_errors = abs(predictions - ground_truth);
    mae = mean(absolute_errors, 'all');
end

%% ================ MAPE (Mean Absolute Percentage Error) ================

function mape = compute_mape(predictions, ground_truth)
    % 计算平均绝对百分比误差
    %
    % 输入:
    %   predictions - 预测值 [N x D]
    %   ground_truth - 真实值 [N x D]
    %
    % 输出:
    %   mape - 平均绝对百分比误差 (%)
    
    if size(predictions) ~= size(ground_truth)
        error('预测值和真实值的尺寸必须相同');
    end
    
    % 避免除零
    mask = abs(ground_truth) > 1e-10;
    percentage_errors = abs((predictions(mask) - ground_truth(mask)) ./ ground_truth(mask));
    mape = mean(percentage_errors) * 100;
end

%% ================ R² Score (Coefficient of Determination) ================

function r2 = compute_r2_score(predictions, ground_truth)
    % 计算R²决定系数
    %
    % 输入:
    %   predictions - 预测值 [N x D]
    %   ground_truth - 真实值 [N x D]
    %
    % 输出:
    %   r2 - R²决定系数
    
    if size(predictions) ~= size(ground_truth)
        error('预测值和真实值的尺寸必须相同');
    end
    
    ss_res = sum((ground_truth - predictions).^2, 'all');
    ss_tot = sum((ground_truth - mean(ground_truth, 'all')).^2, 'all');
    
    r2 = 1 - ss_res / ss_tot;
end

%% ================ 分类准确率 ================

function accuracy = compute_accuracy(predictions, ground_truth)
    % 计算分类准确率
    %
    % 输入:
    %   predictions - 预测类别 [N x 1]
    %   ground_truth - 真实类别 [N x 1]
    %
    % 输出:
    %   accuracy - 准确率 (0-1)
    
    if length(predictions) ~= length(ground_truth)
        error('预测值和真实值的长度必须相同');
    end
    
    correct = sum(predictions == ground_truth);
    total = length(ground_truth);
    accuracy = correct / total;
end

%% ================ 混淆矩阵 ================

function conf_mat = compute_confusion_matrix(predictions, ground_truth, num_classes)
    % 计算混淆矩阵
    %
    % 输入:
    %   predictions - 预测类别 [N x 1]
    %   ground_truth - 真实类别 [N x 1]
    %   num_classes - 类别数量
    %
    % 输出:
    %   conf_mat - 混淆矩阵 [num_classes x num_classes]
    
    if nargin < 3
        num_classes = max(max(predictions), max(ground_truth));
    end
    
    conf_mat = zeros(num_classes, num_classes);
    
    for i = 1:length(predictions)
        true_class = ground_truth(i);
        pred_class = predictions(i);
        
        if true_class >= 1 && true_class <= num_classes && ...
           pred_class >= 1 && pred_class <= num_classes
            conf_mat(true_class, pred_class) = conf_mat(true_class, pred_class) + 1;
        end
    end
end

%% ================ 精确率、召回率、F1分数 ================

function [precision, recall, f1] = compute_precision_recall_f1(predictions, ...
    ground_truth, positive_class)
    % 计算精确率、召回率和F1分数
    %
    % 输入:
    %   predictions - 预测类别 [N x 1]
    %   ground_truth - 真实类别 [N x 1]
    %   positive_class - 正类标签（默认为1）
    %
    % 输出:
    %   precision - 精确率
    %   recall - 召回率
    %   f1 - F1分数
    
    if nargin < 3
        positive_class = 1;
    end
    
    % 真正例 (True Positive)
    tp = sum((predictions == positive_class) & (ground_truth == positive_class));
    
    % 假正例 (False Positive)
    fp = sum((predictions == positive_class) & (ground_truth ~= positive_class));
    
    % 假负例 (False Negative)
    fn = sum((predictions ~= positive_class) & (ground_truth == positive_class));
    
    % 计算指标
    precision = tp / (tp + fp + 1e-10);
    recall = tp / (tp + fn + 1e-10);
    f1 = 2 * precision * recall / (precision + recall + 1e-10);
end

%% ================ 多类别精确率、召回率、F1分数 ================

function [precisions, recalls, f1s] = compute_multiclass_metrics(...
    predictions, ground_truth, num_classes)
    % 计算多类别的精确率、召回率和F1分数
    %
    % 输入:
    %   predictions - 预测类别 [N x 1]
    %   ground_truth - 真实类别 [N x 1]
    %   num_classes - 类别数量
    %
    % 输出:
    %   precisions - 各类别精确率 [num_classes x 1]
    %   recalls - 各类别召回率 [num_classes x 1]
    %   f1s - 各类别F1分数 [num_classes x 1]
    
    if nargin < 3
        num_classes = max(max(predictions), max(ground_truth));
    end
    
    precisions = zeros(num_classes, 1);
    recalls = zeros(num_classes, 1);
    f1s = zeros(num_classes, 1);
    
    for c = 1:num_classes
        [precisions(c), recalls(c), f1s(c)] = compute_precision_recall_f1(...
            predictions, ground_truth, c);
    end
end

%% ================ 宏平均和微平均 ================

function [macro_avg, micro_avg] = compute_macro_micro_average(...
    predictions, ground_truth, num_classes)
    % 计算宏平均和微平均指标
    %
    % 输入:
    %   predictions - 预测类别 [N x 1]
    %   ground_truth - 真实类别 [N x 1]
    %   num_classes - 类别数量
    %
    % 输出:
    %   macro_avg - 宏平均指标结构体
    %   micro_avg - 微平均指标结构体
    
    if nargin < 3
        num_classes = max(max(predictions), max(ground_truth));
    end
    
    % 计算每个类别的指标
    [precisions, recalls, f1s] = compute_multiclass_metrics(...
        predictions, ground_truth, num_classes);
    
    % 宏平均：简单平均
    macro_avg = struct();
    macro_avg.precision = mean(precisions);
    macro_avg.recall = mean(recalls);
    macro_avg.f1 = mean(f1s);
    
    % 微平均：基于总体TP、FP、FN计算
    total_tp = 0;
    total_fp = 0;
    total_fn = 0;
    
    for c = 1:num_classes
        tp = sum((predictions == c) & (ground_truth == c));
        fp = sum((predictions == c) & (ground_truth ~= c));
        fn = sum((predictions ~= c) & (ground_truth == c));
        
        total_tp = total_tp + tp;
        total_fp = total_fp + fp;
        total_fn = total_fn + fn;
    end
    
    micro_avg = struct();
    micro_avg.precision = total_tp / (total_tp + total_fp + 1e-10);
    micro_avg.recall = total_tp / (total_tp + total_fn + 1e-10);
    micro_avg.f1 = 2 * micro_avg.precision * micro_avg.recall / ...
        (micro_avg.precision + micro_avg.recall + 1e-10);
end

%% ================ 位置跟踪误差（ATE） ================

function ate = compute_ate(pred_trajectory, true_trajectory)
    % 计算绝对轨迹误差 (Absolute Trajectory Error)
    %
    % 输入:
    %   pred_trajectory - 预测轨迹 [N x 3]
    %   true_trajectory - 真实轨迹 [N x 3]
    %
    % 输出:
    %   ate - 绝对轨迹误差
    
    if size(pred_trajectory) ~= size(true_trajectory)
        error('预测轨迹和真实轨迹的尺寸必须相同');
    end
    
    errors = sqrt(sum((pred_trajectory - true_trajectory).^2, 2));
    ate = mean(errors);
end

%% ================ 相对位置误差（RTE） ================

function rte = compute_rte(pred_trajectory, true_trajectory, delta)
    % 计算相对轨迹误差 (Relative Trajectory Error)
    %
    % 输入:
    %   pred_trajectory - 预测轨迹 [N x 3]
    %   true_trajectory - 真实轨迹 [N x 3]
    %   delta - 时间间隔（采样点数）
    %
    % 输出:
    %   rte - 相对轨迹误差
    
    if nargin < 3
        delta = 1;
    end
    
    N = size(pred_trajectory, 1);
    relative_errors = [];
    
    for i = 1:(N - delta)
        % 计算相对位移
        pred_displacement = pred_trajectory(i + delta, :) - pred_trajectory(i, :);
        true_displacement = true_trajectory(i + delta, :) - true_trajectory(i, :);
        
        % 相对误差
        rel_error = norm(pred_displacement - true_displacement);
        relative_errors = [relative_errors; rel_error];
    end
    
    rte = mean(relative_errors);
end

%% ================ OSPA距离（简化版）================

function ospa = compute_ospa_distance(pred_positions, true_positions, c, p)
    % 计算OSPA (Optimal Subpattern Assignment) 距离
    % 这是简化版本，适用于单目标跟踪
    %
    % 输入:
    %   pred_positions - 预测位置 [N x 3]
    %   true_positions - 真实位置 [N x 3]
    %   c - 截断参数（默认100）
    %   p - 阶数（默认2）
    %
    % 输出:
    %   ospa - OSPA距离
    
    if nargin < 3
        c = 100;
    end
    if nargin < 4
        p = 2;
    end
    
    % 计算欧氏距离
    distances = sqrt(sum((pred_positions - true_positions).^2, 2));
    
    % 应用截断
    distances = min(distances, c);
    
    % 计算OSPA
    ospa = mean(distances.^p)^(1/p);
end

%% ================ 速度估计误差 ================

function [vmse, vmae] = compute_velocity_errors(pred_velocities, true_velocities)
    % 计算速度估计误差
    %
    % 输入:
    %   pred_velocities - 预测速度 [N x 3]
    %   true_velocities - 真实速度 [N x 3]
    %
    % 输出:
    %   vmse - 速度均方误差
    %   vmae - 速度平均绝对误差
    
    velocity_errors = sqrt(sum((pred_velocities - true_velocities).^2, 2));
    vmse = sqrt(mean(velocity_errors.^2));
    vmae = mean(velocity_errors);
end

%% ================ 机动检测延迟 ================

function [detection_delays, avg_delay] = compute_maneuver_detection_delay(...
    pred_maneuver, true_maneuver, window_size)
    % 计算机动检测延迟
    %
    % 输入:
    %   pred_maneuver - 预测的机动类别序列 [N x 1]
    %   true_maneuver - 真实的机动类别序列 [N x 1]
    %   window_size - 检测窗口大小（默认5）
    %
    % 输出:
    %   detection_delays - 各次机动切换的检测延迟
    %   avg_delay - 平均检测延迟
    
    if nargin < 3
        window_size = 5;
    end
    
    detection_delays = [];
    
    % 找到真实机动切换点
    true_switches = find(diff(true_maneuver) ~= 0);
    
    for switch_idx = true_switches'
        % 在切换点后的窗口内寻找预测的切换
        search_end = min(switch_idx + window_size, length(pred_maneuver) - 1);
        pred_switches = find(diff(pred_maneuver(switch_idx:search_end)) ~= 0);
        
        if ~isempty(pred_switches)
            % 找到最早的预测切换
            delay = pred_switches(1);
            detection_delays = [detection_delays; delay];
        else
            % 未检测到切换，记为最大延迟
            detection_delays = [detection_delays; window_size];
        end
    end
    
    if ~isempty(detection_delays)
        avg_delay = mean(detection_delays);
    else
        avg_delay = NaN;
    end
end

%% ================ 综合跟踪性能评分 ================

function score = compute_tracking_score(position_rmse, velocity_rmse, ...
    maneuver_accuracy, weights)
    % 计算综合跟踪性能评分
    %
    % 输入:
    %   position_rmse - 位置RMSE
    %   velocity_rmse - 速度RMSE
    %   maneuver_accuracy - 机动分类准确率
    %   weights - 权重 [w_pos, w_vel, w_maneuver]（默认[0.5, 0.3, 0.2]）
    %
    % 输出:
    %   score - 综合评分（0-100）
    
    if nargin < 4
        weights = [0.5, 0.3, 0.2];
    end
    
    % 归一化各项指标（使用sigmoid函数）
    pos_score = 100 / (1 + position_rmse / 10);  % 假设10m为参考值
    vel_score = 100 / (1 + velocity_rmse / 5);   % 假设5m/s为参考值
    man_score = maneuver_accuracy * 100;
    
    % 加权平均
    score = weights(1) * pos_score + weights(2) * vel_score + weights(3) * man_score;
end

%% ================ 测试函数 ================

function test_metrics_computation()
    % 测试指标计算函数
    
    fprintf('测试性能指标计算...\n\n');
    
    % 生成测试数据
    N = 100;
    D = 3;
    
    % 回归测试
    fprintf('1. 回归指标测试\n');
    predictions = randn(N, D) * 10;
    ground_truth = predictions + randn(N, D) * 2;  % 添加噪声
    
    rmse = compute_rmse(predictions, ground_truth);
    mae = compute_mae(predictions, ground_truth);
    r2 = compute_r2_score(predictions, ground_truth);
    
    fprintf('   RMSE: %.4f\n', rmse);
    fprintf('   MAE: %.4f\n', mae);
    fprintf('   R²: %.4f\n\n', r2);
    
    % 分类测试
    fprintf('2. 分类指标测试\n');
    num_classes = 4;
    pred_classes = randi(num_classes, N, 1);
    true_classes = randi(num_classes, N, 1);
    
    accuracy = compute_accuracy(pred_classes, true_classes);
    conf_mat = compute_confusion_matrix(pred_classes, true_classes, num_classes);
    [precisions, recalls, f1s] = compute_multiclass_metrics(...
        pred_classes, true_classes, num_classes);
    
    fprintf('   准确率: %.2f%%\n', accuracy * 100);
    fprintf('   混淆矩阵:\n');
    disp(conf_mat);
    fprintf('   各类别F1分数:\n');
    for i = 1:num_classes
        fprintf('     类别%d: %.4f\n', i, f1s(i));
    end
    fprintf('\n');
    
    % 轨迹误差测试
    fprintf('3. 轨迹误差测试\n');
    pred_traj = cumsum(randn(N, 3), 1);
    true_traj = pred_traj + randn(N, 3) * 0.5;
    
    ate = compute_ate(pred_traj, true_traj);
    rte = compute_rte(pred_traj, true_traj, 5);
    ospa = compute_ospa_distance(pred_traj, true_traj);
    
    fprintf('   绝对轨迹误差(ATE): %.4f\n', ate);
    fprintf('   相对轨迹误差(RTE): %.4f\n', rte);
    fprintf('   OSPA距离: %.4f\n\n', ospa);
    
    % 综合评分测试
    fprintf('4. 综合评分测试\n');
    score = compute_tracking_score(5.0, 2.0, 0.85);
    fprintf('   综合跟踪性能评分: %.2f/100\n\n', score);
    
    fprintf('所有测试完成！\n');
end
