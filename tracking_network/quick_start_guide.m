%% =====================================================================
%  快速入门指南 - 机动感知跟踪网络
%  Quick Start Guide - Maneuver-Aware Tracking Network
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  这是一个最简化的示例，展示如何快速上手使用本框架
%  =====================================================================

%% ================ 快速入门 ================

% 1. 首先确保已经生成了数据集
% 如果还没有数据集，运行：
%   cd ..
%   radar_simulation_v3
%   cd tracking_network

% 2. 运行组件测试（可选，确保系统正常）
%   test_all_components

% 3. 下面是一个最简单的训练示例

%% ================ 最小训练示例 ================

clear all; close all; clc;

fprintf('=== 机动感知跟踪网络 - 快速入门 ===\n\n');

% 步骤1: 设置数据集路径
dataset_path = '../enhanced_2d_dataset_v2';

% 检查数据集是否存在
if ~exist(dataset_path, 'dir')
    fprintf('错误：数据集不存在！\n');
    fprintf('请先运行 radar_simulation_v3.m 生成数据集\n');
    return;
end

% 步骤2: 创建最简配置（使用默认值）
config = struct();
config.num_epochs = 10;           % 快速演示，只训练10个epoch
config.batch_size = 16;           % 较小的批次
config.hidden_size = 64;          % 较小的网络
config.save_checkpoint = false;   % 不保存检查点
config.plot_training = false;     % 不显示训练图表
config.verbose = false;           % 减少输出

fprintf('配置参数:\n');
fprintf('  数据集: %s\n', dataset_path);
fprintf('  训练轮数: %d\n', config.num_epochs);
fprintf('  批次大小: %d\n', config.batch_size);
fprintf('  隐藏层大小: %d\n\n', config.hidden_size);

% 步骤3: 训练网络
fprintf('开始训练（这可能需要几分钟）...\n');

try
    % 注意：由于这是演示，实际的训练功能可能需要完整的MATLAB深度学习工具箱
    % 如果遇到错误，请参考 example_complete_pipeline.m 了解完整流程
    
    % 这里展示如何调用训练函数
    % train_results = train_maneuver_tracking(dataset_path, config);
    
    fprintf('\n提示：完整的训练功能需要MATLAB深度学习工具箱\n');
    fprintf('本快速入门指南展示了调用方式\n\n');
    
catch ME
    fprintf('训练时出错: %s\n', ME.message);
    fprintf('请确保安装了必要的工具箱\n');
end

%% ================ 快速评估示例 ================

fprintf('=== 评估示例 ===\n\n');

% 如果有训练好的模型，可以这样评估：
model_file = 'trained_models/trained_model_latest.mat';

if exist(model_file, 'file')
    fprintf('找到模型文件: %s\n', model_file);
    fprintf('开始评估...\n\n');
    
    % 评估配置
    eval_config = struct();
    eval_config.num_visualization_samples = 3;  % 只可视化3个样本
    eval_config.save_figures = false;           % 不保存图表
    eval_config.verbose = false;                % 减少输出
    
    try
        % 调用评估函数
        % eval_results = evaluate_tracking_network(model_file, dataset_path, eval_config);
        
        fprintf('提示：评估功能调用示例\n\n');
        
    catch ME
        fprintf('评估时出错: %s\n', ME.message);
    end
else
    fprintf('未找到训练好的模型\n');
    fprintf('请先训练网络或查看 example_complete_pipeline.m\n\n');
end

%% ================ 快速可视化示例 ================

fprintf('=== 可视化示例 ===\n\n');

% 添加utils路径
addpath('utils');

% 生成一些示例数据进行可视化
t = linspace(0, 10, 100)';

% 示例1: 绘制轨迹
fprintf('生成示例轨迹可视化...\n');
traj1 = [100*cos(t), 100*sin(t), 10*t];
traj2 = traj1 + randn(size(traj1)) * 5;  % 添加噪声作为"预测"

fig1 = plot_trajectory_3d({traj1, traj2}, {'真实', '预测'}, '轨迹对比示例');
fprintf('  已生成3D轨迹图\n');

% 示例2: 绘制注意力权重
fprintf('生成示例注意力热图...\n');
attn = rand(30, 30);
attn = attn ./ sum(attn, 2);  % 归一化

fig2 = plot_attention_heatmap(attn, '注意力权重示例');
fprintf('  已生成注意力热图\n');

% 示例3: 绘制机动检测
fprintf('生成机动检测时间线...\n');
time_steps = 1:100;
true_maneuvers = [ones(25,1); 2*ones(25,1); 3*ones(25,1); 4*ones(25,1)];
pred_maneuvers = true_maneuvers;
pred_maneuvers(randi(100, 10, 1)) = randi(4, 10, 1);  % 添加一些错误

fig3 = plot_maneuver_timeline(time_steps, true_maneuvers, pred_maneuvers, '机动检测示例');
fprintf('  已生成机动检测时间线\n\n');

fprintf('可视化完成！请查看生成的图表。\n\n');

%% ================ 下一步 ================

fprintf('=== 下一步建议 ===\n\n');

fprintf('1. 查看完整示例:\n');
fprintf('   >> example_complete_pipeline\n\n');

fprintf('2. 运行组件测试:\n');
fprintf('   >> test_all_components\n\n');

fprintf('3. 阅读详细文档:\n');
fprintf('   打开 README_tracking_network.md\n\n');

fprintf('4. 自定义配置:\n');
fprintf('   修改网络参数、训练超参数等\n');
fprintf('   参考各脚本中的 get_default_config() 函数\n\n');

fprintf('5. 深入学习:\n');
fprintf('   - 研究注意力机制: attention_modules.m\n');
fprintf('   - 理解网络架构: maneuver_aware_tracking_network.m\n');
fprintf('   - 学习数据处理: data_loader_maneuver_tracking.m\n\n');

fprintf('=== 快速入门完成！===\n');

%% ================ 实用函数示例 ================

% 这些函数展示了框架的主要功能

function demo_data_loading()
    % 演示如何加载数据
    fprintf('演示：数据加载\n');
    
    config = struct();
    config.sequence_length = 50;
    config.stride = 10;
    
    % data_loader = data_loader_maneuver_tracking(dataset_path, config);
    % fprintf('加载了 %d 个样本\n', size(data_loader.samples, 1));
end

function demo_network_creation()
    % 演示如何创建网络
    fprintf('演示：网络创建\n');
    
    config = struct();
    config.hidden_size = 128;
    config.num_lstm_layers = 2;
    
    % network = maneuver_aware_tracking_network(config);
    % fprintf('创建了网络，层数: %d\n', length(network.Layers));
end

function demo_metrics_calculation()
    % 演示如何计算性能指标
    fprintf('演示：性能指标计算\n');
    
    addpath('utils');
    
    % 生成示例数据
    predictions = randn(100, 3);
    ground_truth = predictions + randn(100, 3) * 0.5;
    
    rmse = compute_rmse(predictions, ground_truth);
    mae = compute_mae(predictions, ground_truth);
    
    fprintf('RMSE: %.4f\n', rmse);
    fprintf('MAE: %.4f\n', mae);
end

function demo_coordinate_conversion()
    % 演示坐标转换
    fprintf('演示：坐标转换\n');
    
    addpath('utils');
    
    % 球坐标
    range = 100;
    azimuth = pi/4;
    elevation = pi/6;
    
    % 转换为笛卡尔坐标
    [x, y, z] = spherical_to_cartesian(range, azimuth, elevation);
    
    fprintf('球坐标 (%.2f, %.2f°, %.2f°)\n', ...
        range, rad2deg(azimuth), rad2deg(elevation));
    fprintf('笛卡尔坐标 (%.2f, %.2f, %.2f)\n', x, y, z);
end
