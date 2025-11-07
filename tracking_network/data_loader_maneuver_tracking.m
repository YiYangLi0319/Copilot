%% =====================================================================
%  数据加载器 - 机动感知跟踪网络
%  Data Loader for Maneuver-Aware Tracking Network
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  功能：
%  1. 加载 radar_simulation_v3.m 生成的场景数据
%  2. 使用滑动窗口构建训练样本
%  3. 数据归一化和预处理
%  4. 训练/验证/测试集划分
%  =====================================================================

function data_loader = data_loader_maneuver_tracking(dataset_path, config)
    % 创建数据加载器对象
    %
    % 输入:
    %   dataset_path - 数据集路径（如 'enhanced_2d_dataset_v2'）
    %   config - 配置结构体
    %
    % 输出:
    %   data_loader - 数据加载器结构体
    
    if nargin < 2
        config = get_default_config();
    end
    
    fprintf('=====================================================\n');
    fprintf('  数据加载器初始化\n');
    fprintf('  数据集路径: %s\n', dataset_path);
    fprintf('=====================================================\n\n');
    
    % 初始化数据加载器
    data_loader = struct();
    data_loader.dataset_path = dataset_path;
    data_loader.config = config;
    
    % 加载数据集索引
    index_file = fullfile(dataset_path, 'index.json');
    if exist(index_file, 'file')
        data_loader.index = load_json_file(index_file);
    else
        error('数据集索引文件不存在: %s', index_file);
    end
    
    % 加载数据集信息
    dataset_info_file = fullfile(dataset_path, 'dataset_info.json');
    if exist(dataset_info_file, 'file')
        data_loader.dataset_info = load_json_file(dataset_info_file);
    else
        warning('数据集信息文件不存在: %s', dataset_info_file);
        data_loader.dataset_info = struct();
    end
    
    % 加载所有场景
    fprintf('加载场景数据...\n');
    data_loader.scenarios = load_all_scenarios(dataset_path, data_loader.index);
    fprintf('已加载 %d 个场景\n\n', length(data_loader.scenarios));
    
    % 构建训练样本
    fprintf('构建训练样本（滑动窗口）...\n');
    [samples, labels_state, labels_maneuver] = build_training_samples(...
        data_loader.scenarios, config);
    
    data_loader.samples = samples;
    data_loader.labels_state = labels_state;
    data_loader.labels_maneuver = labels_maneuver;
    fprintf('生成 %d 个训练样本\n\n', size(samples, 1));
    
    % 数据归一化
    fprintf('数据归一化...\n');
    [normalized_samples, normalization_params] = normalize_data(samples, config);
    data_loader.normalized_samples = normalized_samples;
    data_loader.normalization_params = normalization_params;
    fprintf('归一化完成\n\n');
    
    % 划分训练/验证/测试集
    fprintf('划分数据集...\n');
    [train_idx, val_idx, test_idx] = split_dataset(length(data_loader.scenarios), config);
    data_loader.train_idx = train_idx;
    data_loader.val_idx = val_idx;
    data_loader.test_idx = test_idx;
    
    fprintf('训练集: %d 个样本\n', length(train_idx));
    fprintf('验证集: %d 个样本\n', length(val_idx));
    fprintf('测试集: %d 个样本\n\n', length(test_idx));
    
    % 统计信息
    data_loader.statistics = compute_dataset_statistics(data_loader);
    
    fprintf('数据加载完成！\n');
    fprintf('=====================================================\n\n');
end

%% ================ 辅助函数 ================

function config = get_default_config()
    % 获取默认配置
    config = struct();
    config.sequence_length = 50;          % 输入序列长度
    config.prediction_horizon = 1;        % 预测时间步数
    config.stride = 10;                   % 滑动窗口步长
    config.train_ratio = 0.7;             % 训练集比例
    config.val_ratio = 0.15;              % 验证集比例
    config.test_ratio = 0.15;             % 测试集比例
    config.normalize_method = 'minmax';   % 归一化方法
    config.random_seed = 42;              % 随机种子
end

function scenarios = load_all_scenarios(dataset_path, index)
    % 加载所有场景数据
    num_scenarios = index.num_scenarios;
    scenarios = cell(1, num_scenarios);
    
    for i = 1:num_scenarios
        scenario_file = fullfile(dataset_path, sprintf('scenario_%04d.mat', i-1));
        if exist(scenario_file, 'file')
            data = load(scenario_file);
            scenarios{i} = data.scenario;
            
            if mod(i, 10) == 0 || i == num_scenarios
                fprintf('  进度: %d/%d (%.1f%%)\n', i, num_scenarios, 100*i/num_scenarios);
            end
        else
            warning('场景文件不存在: %s', scenario_file);
        end
    end
end

function [samples, labels_state, labels_maneuver] = build_training_samples(scenarios, config)
    % 使用滑动窗口构建训练样本
    %
    % 输出:
    %   samples - [N x num_receivers x 3 x sequence_length]
    %   labels_state - [N x state_dim] (位置、速度、加速度)
    %   labels_maneuver - [N x 1] (机动模式: 1=CV, 2=CA, 3=CT, 4=Singer)
    
    sequence_length = config.sequence_length;
    stride = config.stride;
    
    % 预估样本数量
    total_samples = 0;
    for i = 1:length(scenarios)
        if ~isempty(scenarios{i})
            num_timesteps = length(scenarios{i}.measurements);
            total_samples = total_samples + floor((num_timesteps - sequence_length) / stride) + 1;
        end
    end
    
    % 预分配数组
    % 假设3个接收机，3个量测维度（距离、方位角、仰角）
    num_receivers = 3;
    num_features = 3;
    samples = zeros(total_samples, num_receivers, num_features, sequence_length);
    
    % 状态标签：位置(3) + 速度(3) + 加速度(3) = 9维
    state_dim = 9;
    labels_state = zeros(total_samples, state_dim);
    labels_maneuver = zeros(total_samples, 1);
    
    sample_idx = 1;
    
    for scenario_idx = 1:length(scenarios)
        scenario = scenarios{scenario_idx};
        if isempty(scenario)
            continue;
        end
        
        measurements = scenario.measurements;
        trajectory = scenario.target_info.trajectory;
        num_timesteps = length(measurements);
        
        % 滑动窗口提取样本
        for start_idx = 1:stride:(num_timesteps - sequence_length)
            end_idx = start_idx + sequence_length - 1;
            
            % 提取量测序列
            for t = 1:sequence_length
                meas_idx = start_idx + t - 1;
                meas = measurements{meas_idx}.measurements;
                
                % 提取每个接收机的量测
                for rx = 1:min(num_receivers, length(meas))
                    if length(meas{rx}) >= 3
                        % [距离, 方位角, 仰角]
                        samples(sample_idx, rx, 1, t) = meas{rx}(1);  % 距离
                        samples(sample_idx, rx, 2, t) = meas{rx}(2);  % 方位角
                        samples(sample_idx, rx, 3, t) = meas{rx}(3);  % 仰角
                    end
                end
            end
            
            % 提取标签（预测序列末尾的状态）
            target_traj_point = trajectory{end_idx};
            
            % 状态标签：位置 + 速度 + 加速度
            position = target_traj_point.position;
            velocity = target_traj_point.velocity;
            acceleration = target_traj_point.acceleration;
            
            labels_state(sample_idx, 1:3) = position(1:3);
            labels_state(sample_idx, 4:6) = velocity(1:3);
            labels_state(sample_idx, 7:9) = acceleration(1:3);
            
            % 机动模式标签
            model_name = target_traj_point.motion_model;
            labels_maneuver(sample_idx) = encode_maneuver_model(model_name);
            
            sample_idx = sample_idx + 1;
        end
    end
    
    % 截断到实际样本数
    samples = samples(1:sample_idx-1, :, :, :);
    labels_state = labels_state(1:sample_idx-1, :);
    labels_maneuver = labels_maneuver(1:sample_idx-1);
end

function model_id = encode_maneuver_model(model_name)
    % 将模型名称编码为数字标签
    % 1=CV, 2=CA, 3=CT, 4=Singer
    
    switch lower(model_name)
        case 'cv'
            model_id = 1;
        case 'ca'
            model_id = 2;
        case 'ct'
            model_id = 3;
        case 'singer'
            model_id = 4;
        otherwise
            model_id = 1;  % 默认为CV
    end
end

function [normalized_samples, params] = normalize_data(samples, config)
    % 数据归一化
    %
    % 输入:
    %   samples - [N x num_receivers x 3 x sequence_length]
    %
    % 输出:
    %   normalized_samples - 归一化后的数据
    %   params - 归一化参数（用于反归一化）
    
    [N, num_receivers, num_features, sequence_length] = size(samples);
    normalized_samples = zeros(size(samples));
    params = struct();
    
    % 对每个特征维度独立归一化
    for rx = 1:num_receivers
        for feat = 1:num_features
            % 提取当前特征的所有数据
            feature_data = squeeze(samples(:, rx, feat, :));
            feature_data = feature_data(:);
            
            if strcmp(config.normalize_method, 'minmax')
                % Min-Max归一化到[0, 1]
                min_val = min(feature_data);
                max_val = max(feature_data);
                
                if max_val - min_val > 1e-10
                    normalized_feature = (samples(:, rx, feat, :) - min_val) / (max_val - min_val);
                else
                    normalized_feature = samples(:, rx, feat, :);
                end
                
                params.(sprintf('rx%d_feat%d_min', rx, feat)) = min_val;
                params.(sprintf('rx%d_feat%d_max', rx, feat)) = max_val;
                
            elseif strcmp(config.normalize_method, 'zscore')
                % Z-score标准化
                mean_val = mean(feature_data);
                std_val = std(feature_data);
                
                if std_val > 1e-10
                    normalized_feature = (samples(:, rx, feat, :) - mean_val) / std_val;
                else
                    normalized_feature = samples(:, rx, feat, :);
                end
                
                params.(sprintf('rx%d_feat%d_mean', rx, feat)) = mean_val;
                params.(sprintf('rx%d_feat%d_std', rx, feat)) = std_val;
            else
                normalized_feature = samples(:, rx, feat, :);
            end
            
            normalized_samples(:, rx, feat, :) = normalized_feature;
        end
    end
    
    params.method = config.normalize_method;
end

function [train_idx, val_idx, test_idx] = split_dataset(num_scenarios, config)
    % 划分数据集为训练/验证/测试集
    
    rng(config.random_seed);
    
    % 随机打乱索引
    indices = randperm(num_scenarios);
    
    % 计算划分点
    train_end = floor(num_scenarios * config.train_ratio);
    val_end = train_end + floor(num_scenarios * config.val_ratio);
    
    % 划分索引
    train_idx = indices(1:train_end);
    val_idx = indices(train_end+1:val_end);
    test_idx = indices(val_end+1:end);
end

function stats = compute_dataset_statistics(data_loader)
    % 计算数据集统计信息
    
    stats = struct();
    
    % 机动模式分布
    maneuver_counts = histcounts(data_loader.labels_maneuver, 1:5);
    stats.maneuver_distribution = maneuver_counts;
    stats.maneuver_labels = {'CV', 'CA', 'CT', 'Singer'};
    
    % 状态统计
    stats.state_mean = mean(data_loader.labels_state, 1);
    stats.state_std = std(data_loader.labels_state, 0, 1);
    stats.state_min = min(data_loader.labels_state, [], 1);
    stats.state_max = max(data_loader.labels_state, [], 1);
    
    % 量测统计
    samples_flat = reshape(data_loader.normalized_samples, [], ...
        size(data_loader.normalized_samples, 2) * ...
        size(data_loader.normalized_samples, 3) * ...
        size(data_loader.normalized_samples, 4));
    stats.measurement_mean = mean(samples_flat, 1);
    stats.measurement_std = std(samples_flat, 0, 1);
    
    fprintf('数据集统计信息:\n');
    fprintf('  机动模式分布:\n');
    for i = 1:length(stats.maneuver_labels)
        fprintf('    %s: %d (%.1f%%)\n', stats.maneuver_labels{i}, ...
            maneuver_counts(i), 100*maneuver_counts(i)/sum(maneuver_counts));
    end
    fprintf('\n');
end

function data = load_json_file(filename)
    % 简化的JSON加载函数
    % 注意：实际使用时可能需要更复杂的JSON解析
    
    try
        fid = fopen(filename, 'r');
        raw = fread(fid, inf);
        str = char(raw');
        fclose(fid);
        data = jsondecode(str);
    catch
        warning('无法加载JSON文件: %s', filename);
        data = struct();
    end
end

%% ================ 批次数据获取函数 ================

function batch = get_training_batch(data_loader, batch_indices)
    % 获取训练批次
    %
    % 输入:
    %   data_loader - 数据加载器
    %   batch_indices - 批次样本索引
    %
    % 输出:
    %   batch - 批次数据结构体
    
    batch = struct();
    batch.samples = data_loader.normalized_samples(batch_indices, :, :, :);
    batch.labels_state = data_loader.labels_state(batch_indices, :);
    batch.labels_maneuver = data_loader.labels_maneuver(batch_indices);
end

function batches = create_batch_iterator(data_loader, batch_size, shuffle)
    % 创建批次迭代器
    %
    % 输入:
    %   data_loader - 数据加载器
    %   batch_size - 批次大小
    %   shuffle - 是否打乱数据
    %
    % 输出:
    %   batches - 批次索引数组
    
    if nargin < 3
        shuffle = true;
    end
    
    num_samples = size(data_loader.normalized_samples, 1);
    indices = 1:num_samples;
    
    if shuffle
        indices = indices(randperm(num_samples));
    end
    
    num_batches = floor(num_samples / batch_size);
    batches = cell(1, num_batches);
    
    for i = 1:num_batches
        start_idx = (i-1) * batch_size + 1;
        end_idx = i * batch_size;
        batches{i} = indices(start_idx:end_idx);
    end
end
