%% =====================================================================
%  示例脚本 - 完整的训练和评估流程
%  Example Script - Complete Training and Evaluation Pipeline
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  本脚本展示如何：
%  1. 生成数据集（如果还没有）
%  2. 加载和预处理数据
%  3. 训练网络
%  4. 评估性能
%  5. 可视化结果
%  =====================================================================

clear all; close all; clc;

fprintf('=====================================================\n');
fprintf('  机动感知跟踪网络 - 完整示例\n');
fprintf('  日期: %s\n', datestr(now));
fprintf('=====================================================\n\n');

%% ================ 步骤1: 检查数据集 ================

fprintf('步骤 1/5: 检查数据集...\n');

dataset_2d_path = '../enhanced_2d_dataset_v2';
dataset_3d_path = '../enhanced_3d_dataset_v2';

% 检查2D数据集是否存在
if ~exist(dataset_2d_path, 'dir')
    fprintf('  2D数据集不存在，需要先运行 radar_simulation_v3.m 生成数据集\n');
    fprintf('  请先运行以下命令生成数据集：\n');
    fprintf('  >> cd ..\n');
    fprintf('  >> radar_simulation_v3\n\n');
    return;
else
    fprintf('  2D数据集已找到: %s\n', dataset_2d_path);
end

% 检查3D数据集
if exist(dataset_3d_path, 'dir')
    fprintf('  3D数据集已找到: %s\n', dataset_3d_path);
    use_3d = true;
else
    fprintf('  3D数据集不存在，将仅使用2D数据集\n');
    use_3d = false;
end

fprintf('数据集检查完成！\n\n');

%% ================ 步骤2: 配置参数 ================

fprintf('步骤 2/5: 配置参数...\n');

% 选择使用哪个数据集
dataset_path = dataset_2d_path;  % 或者使用 dataset_3d_path

% 数据加载配置
data_config = struct();
data_config.sequence_length = 50;      % 输入序列长度
data_config.stride = 10;                % 滑动窗口步长
data_config.train_ratio = 0.7;         % 训练集比例
data_config.val_ratio = 0.15;          % 验证集比例
data_config.test_ratio = 0.15;         % 测试集比例
data_config.normalize_method = 'minmax'; % 归一化方法
data_config.random_seed = 42;          % 随机种子

% 训练配置
train_config = struct();
train_config.sequence_length = data_config.sequence_length;
train_config.stride = data_config.stride;
train_config.train_ratio = data_config.train_ratio;
train_config.val_ratio = data_config.val_ratio;
train_config.test_ratio = data_config.test_ratio;
train_config.normalize_method = data_config.normalize_method;
train_config.random_seed = data_config.random_seed;

% 网络配置
train_config.num_receivers = 3;
train_config.num_features = 3;
train_config.hidden_size = 128;
train_config.num_lstm_layers = 2;
train_config.lstm_type = 'lstm';
train_config.num_attention_heads = 8;
train_config.attention_key_dim = 64;
train_config.attention_value_dim = 64;
train_config.use_self_attention = true;
train_config.use_maneuver_attention = true;
train_config.dropout_rate = 0.2;
train_config.state_output_size = 9;
train_config.num_maneuver_classes = 4;

% 训练超参数
train_config.batch_size = 32;
train_config.num_epochs = 50;  % 示例中使用较少的epoch
train_config.learning_rate = 0.001;
train_config.state_loss_weight = 1.0;
train_config.maneuver_loss_weight = 0.5;

% 其他配置
train_config.verbose = true;
train_config.plot_training = true;
train_config.save_checkpoint = true;
train_config.checkpoint_frequency = 10;
train_config.output_dir = 'trained_models';

fprintf('配置完成！\n');
fprintf('  数据集: %s\n', dataset_path);
fprintf('  序列长度: %d\n', train_config.sequence_length);
fprintf('  隐藏层大小: %d\n', train_config.hidden_size);
fprintf('  批次大小: %d\n', train_config.batch_size);
fprintf('  训练轮数: %d\n\n', train_config.num_epochs);

%% ================ 步骤3: 加载数据 ================

fprintf('步骤 3/5: 加载和预处理数据...\n');

try
    data_loader = data_loader_maneuver_tracking(dataset_path, data_config);
    fprintf('数据加载成功！\n');
    fprintf('  总样本数: %d\n', size(data_loader.normalized_samples, 1));
    fprintf('  特征形状: [%d x %d x %d x %d]\n', ...
        size(data_loader.normalized_samples));
    fprintf('  状态标签形状: [%d x %d]\n', size(data_loader.labels_state));
    fprintf('  机动标签形状: [%d x %d]\n\n', size(data_loader.labels_maneuver));
catch ME
    fprintf('数据加载失败: %s\n', ME.message);
    fprintf('请确保数据集存在并且格式正确\n');
    return;
end

%% ================ 步骤4: 训练网络 ================

fprintf('步骤 4/5: 训练网络...\n');
fprintf('注意：这可能需要较长时间，取决于数据集大小和硬件配置\n\n');

% 询问用户是否要进行训练
user_input = input('是否开始训练？(y/n): ', 's');

if strcmpi(user_input, 'y')
    try
        train_results = train_maneuver_tracking(dataset_path, train_config);
        fprintf('\n训练完成！\n');
        fprintf('  最终训练损失: %.4f\n', train_results.train_info.train_loss(end));
        fprintf('  最终验证损失: %.4f\n', train_results.train_info.val_loss(end));
        
        % 保存训练结果的引用
        model_file = fullfile(train_config.output_dir, ...
            sprintf('trained_model_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
        fprintf('  模型已保存: %s\n\n', model_file);
        
    catch ME
        fprintf('训练失败: %s\n', ME.message);
        fprintf('堆栈跟踪:\n');
        disp(ME.stack);
        return;
    end
else
    fprintf('跳过训练步骤\n');
    fprintf('如果已有训练好的模型，可以直接进行评估\n\n');
    
    % 列出可用的模型文件
    if exist(train_config.output_dir, 'dir')
        model_files = dir(fullfile(train_config.output_dir, 'trained_model_*.mat'));
        if ~isempty(model_files)
            fprintf('找到以下训练好的模型：\n');
            for i = 1:length(model_files)
                fprintf('  %d. %s\n', i, model_files(i).name);
            end
            model_idx = input(sprintf('选择模型编号 (1-%d): ', length(model_files)));
            if model_idx >= 1 && model_idx <= length(model_files)
                model_file = fullfile(train_config.output_dir, model_files(model_idx).name);
                fprintf('已选择模型: %s\n\n', model_file);
            else
                fprintf('无效的模型编号，退出\n');
                return;
            end
        else
            fprintf('没有找到训练好的模型，请先训练网络\n');
            return;
        end
    else
        fprintf('没有找到训练好的模型，请先训练网络\n');
        return;
    end
end

%% ================ 步骤5: 评估网络 ================

fprintf('步骤 5/5: 评估网络性能...\n');

% 评估配置
eval_config = struct();
eval_config.test_ratio = 0.15;
eval_config.num_test_samples = -1;  % 使用所有测试样本
eval_config.num_visualization_samples = 5;
eval_config.plot_trajectories = true;
eval_config.plot_attention_weights = true;
eval_config.plot_maneuver_detection = true;
eval_config.plot_error_distribution = true;
eval_config.save_figures = true;
eval_config.output_dir = 'evaluation_results';
eval_config.generate_report = true;
eval_config.verbose = true;

% 询问用户是否要进行评估
user_input = input('是否开始评估？(y/n): ', 's');

if strcmpi(user_input, 'y')
    try
        eval_results = evaluate_tracking_network(model_file, dataset_path, eval_config);
        
        fprintf('\n评估完成！\n');
        fprintf('主要性能指标：\n');
        fprintf('  位置RMSE: %.4f m\n', eval_results.metrics.state.position_rmse);
        fprintf('  速度RMSE: %.4f m/s\n', eval_results.metrics.state.velocity_rmse);
        fprintf('  加速度RMSE: %.4f m/s²\n', eval_results.metrics.state.acceleration_rmse);
        fprintf('  机动分类准确率: %.2f%%\n', eval_results.metrics.maneuver.accuracy * 100);
        fprintf('  综合评分: %.4f\n', eval_results.metrics.overall.combined_score);
        
    catch ME
        fprintf('评估失败: %s\n', ME.message);
        fprintf('堆栈跟踪:\n');
        disp(ME.stack);
        return;
    end
else
    fprintf('跳过评估步骤\n\n');
end

%% ================ 总结 ================

fprintf('\n=====================================================\n');
fprintf('  示例脚本完成！\n');
fprintf('=====================================================\n\n');

fprintf('下一步建议：\n');
fprintf('1. 查看评估结果图表\n');
fprintf('2. 调整网络配置和训练参数以提升性能\n');
fprintf('3. 尝试在3D数据集上训练\n');
fprintf('4. 使用迁移学习在新数据集上微调\n');
fprintf('5. 实施集成学习提高鲁棒性\n\n');

fprintf('更多信息请参考 README_tracking_network.md\n\n');
