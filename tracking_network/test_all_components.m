%% =====================================================================
%  测试脚本 - 验证所有组件
%  Test Script - Validate All Components
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  本脚本测试所有模块的基本功能：
%  1. 数据加载器
%  2. 注意力机制
%  3. 网络架构
%  4. 坐标转换工具
%  5. 性能指标计算
%  6. 可视化工具
%  =====================================================================

clear all; close all; clc;

fprintf('=====================================================\n');
fprintf('  机动感知跟踪网络 - 组件测试\n');
fprintf('  日期: %s\n', datestr(now));
fprintf('=====================================================\n\n');

% 添加utils路径
addpath('utils');

test_results = struct();
test_results.passed = [];
test_results.failed = [];

%% ================ 测试1: 坐标转换工具 ================

fprintf('测试 1/6: 坐标转换工具...\n');
try
    % 测试球坐标与笛卡尔坐标转换
    range = 100;
    azimuth = pi/4;
    elevation = pi/6;
    
    [x, y, z] = spherical_to_cartesian(range, azimuth, elevation);
    [range2, azimuth2, elevation2] = cartesian_to_spherical(x, y, z);
    
    error_norm = norm([range-range2, azimuth-azimuth2, elevation-elevation2]);
    
    if error_norm < 1e-10
        fprintf('  ✓ 球坐标转换测试通过（误差: %.2e）\n', error_norm);
        test_results.passed = [test_results.passed, 1];
    else
        fprintf('  ✗ 球坐标转换测试失败（误差: %.2e）\n', error_norm);
        test_results.failed = [test_results.failed, 1];
    end
    
    % 测试双基地量测计算
    target_pos = [1000, 2000, 500];
    tx_pos = [0, 0, 0];
    rx_pos = [5000, 0, 0];
    
    [bistatic_range, az, el] = compute_bistatic_measurement(...
        target_pos, tx_pos, rx_pos, false);
    
    if bistatic_range > 0 && ~isnan(az) && ~isnan(el)
        fprintf('  ✓ 双基地量测计算测试通过\n');
        test_results.passed = [test_results.passed, 1];
    else
        fprintf('  ✗ 双基地量测计算测试失败\n');
        test_results.failed = [test_results.failed, 1];
    end
    
    % 测试角度归一化
    test_angles = [0, pi/2, pi, 3*pi/2, 2*pi, 5*pi/2];
    all_normalized = true;
    for angle = test_angles
        norm_angle = normalize_angle(angle);
        if abs(norm_angle) > pi
            all_normalized = false;
            break;
        end
    end
    
    if all_normalized
        fprintf('  ✓ 角度归一化测试通过\n');
        test_results.passed = [test_results.passed, 1];
    else
        fprintf('  ✗ 角度归一化测试失败\n');
        test_results.failed = [test_results.failed, 1];
    end
    
    fprintf('坐标转换工具测试完成\n\n');
    
catch ME
    fprintf('  ✗ 坐标转换工具测试异常: %s\n\n', ME.message);
    test_results.failed = [test_results.failed, 1];
end

%% ================ 测试2: 性能指标计算 ================

fprintf('测试 2/6: 性能指标计算...\n');
try
    % 生成测试数据
    N = 100;
    predictions = randn(N, 3) * 10;
    ground_truth = predictions + randn(N, 3) * 2;
    
    % 测试RMSE
    rmse = compute_rmse(predictions, ground_truth);
    if rmse > 0 && rmse < 100
        fprintf('  ✓ RMSE计算测试通过 (RMSE=%.4f)\n', rmse);
        test_results.passed = [test_results.passed, 2];
    else
        fprintf('  ✗ RMSE计算测试失败\n');
        test_results.failed = [test_results.failed, 2];
    end
    
    % 测试MAE
    mae = compute_mae(predictions, ground_truth);
    if mae > 0 && mae < 100
        fprintf('  ✓ MAE计算测试通过 (MAE=%.4f)\n', mae);
        test_results.passed = [test_results.passed, 2];
    else
        fprintf('  ✗ MAE计算测试失败\n');
        test_results.failed = [test_results.failed, 2];
    end
    
    % 测试分类指标
    num_classes = 4;
    pred_classes = randi(num_classes, N, 1);
    true_classes = randi(num_classes, N, 1);
    
    accuracy = compute_accuracy(pred_classes, true_classes);
    conf_mat = compute_confusion_matrix(pred_classes, true_classes, num_classes);
    [precisions, recalls, f1s] = compute_multiclass_metrics(...
        pred_classes, true_classes, num_classes);
    
    if accuracy >= 0 && accuracy <= 1 && all(size(conf_mat) == [num_classes, num_classes])
        fprintf('  ✓ 分类指标计算测试通过 (准确率=%.2f%%)\n', accuracy*100);
        test_results.passed = [test_results.passed, 2];
    else
        fprintf('  ✗ 分类指标计算测试失败\n');
        test_results.failed = [test_results.failed, 2];
    end
    
    fprintf('性能指标计算测试完成\n\n');
    
catch ME
    fprintf('  ✗ 性能指标计算测试异常: %s\n\n', ME.message);
    test_results.failed = [test_results.failed, 2];
end

%% ================ 测试3: 注意力机制 ================

fprintf('测试 3/6: 注意力机制...\n');
try
    % 测试参数
    batch_size = 4;
    seq_len = 50;
    d_model = 128;
    
    % 生成随机输入
    input_seq = randn(batch_size, seq_len, d_model);
    
    % 测试位置编码
    encoded_input = add_positional_encoding(input_seq, d_model);
    if all(size(encoded_input) == size(input_seq))
        fprintf('  ✓ 位置编码测试通过\n');
        test_results.passed = [test_results.passed, 3];
    else
        fprintf('  ✗ 位置编码测试失败\n');
        test_results.failed = [test_results.failed, 3];
    end
    
    % 测试注意力权重计算
    d_k = 16;
    d_v = 16;
    Q = randn(batch_size, seq_len, d_k);
    K = randn(batch_size, seq_len, d_k);
    V = randn(batch_size, seq_len, d_v);
    
    [attn_weights, context] = compute_attention_weights(Q, K, V);
    
    if all(size(attn_weights) == [batch_size, seq_len, seq_len]) && ...
       all(size(context) == [batch_size, seq_len, d_v])
        fprintf('  ✓ 注意力权重计算测试通过\n');
        test_results.passed = [test_results.passed, 3];
    else
        fprintf('  ✗ 注意力权重计算测试失败\n');
        test_results.failed = [test_results.failed, 3];
    end
    
    % 测试因果掩码
    causal_mask = create_causal_mask(seq_len);
    if all(size(causal_mask) == [seq_len, seq_len])
        fprintf('  ✓ 因果掩码测试通过\n');
        test_results.passed = [test_results.passed, 3];
    else
        fprintf('  ✗ 因果掩码测试失败\n');
        test_results.failed = [test_results.failed, 3];
    end
    
    fprintf('注意力机制测试完成\n\n');
    
catch ME
    fprintf('  ✗ 注意力机制测试异常: %s\n\n', ME.message);
    test_results.failed = [test_results.failed, 3];
end

%% ================ 测试4: 网络架构 ================

fprintf('测试 4/6: 网络架构...\n');
try
    % 创建网络配置
    net_config = struct();
    net_config.num_receivers = 3;
    net_config.num_features = 3;
    net_config.sequence_length = 50;
    net_config.hidden_size = 64;  % 使用较小的网络进行测试
    net_config.num_lstm_layers = 1;
    net_config.lstm_type = 'lstm';
    net_config.num_attention_heads = 4;
    net_config.attention_key_dim = 32;
    net_config.attention_value_dim = 32;
    net_config.use_self_attention = false;  % 简化测试
    net_config.use_maneuver_attention = false;
    net_config.dropout_rate = 0.2;
    net_config.state_output_size = 9;
    net_config.num_maneuver_classes = 4;
    net_config.plot_network = false;
    
    % 创建简化网络
    network = create_simple_network(net_config);
    
    if ~isempty(network)
        fprintf('  ✓ 网络创建测试通过 (层数: %d)\n', length(network));
        test_results.passed = [test_results.passed, 4];
    else
        fprintf('  ✗ 网络创建测试失败\n');
        test_results.failed = [test_results.failed, 4];
    end
    
    % 估计参数数量
    total_params = estimate_total_parameters(net_config);
    if total_params > 0
        fprintf('  ✓ 参数估计测试通过 (约%.2fM参数)\n', total_params/1e6);
        test_results.passed = [test_results.passed, 4];
    else
        fprintf('  ✗ 参数估计测试失败\n');
        test_results.failed = [test_results.failed, 4];
    end
    
    fprintf('网络架构测试完成\n\n');
    
catch ME
    fprintf('  ✗ 网络架构测试异常: %s\n\n', ME.message);
    test_results.failed = [test_results.failed, 4];
end

%% ================ 测试5: 可视化工具 ================

fprintf('测试 5/6: 可视化工具...\n');
try
    % 生成测试数据
    t = linspace(0, 10, 100)';
    
    % 测试2D轨迹绘制
    traj_2d = [100*cos(t), 100*sin(t), zeros(size(t))];
    fig1 = plot_trajectory_2d({traj_2d}, {'测试轨迹'}, '2D轨迹测试');
    if ishandle(fig1)
        fprintf('  ✓ 2D轨迹绘制测试通过\n');
        test_results.passed = [test_results.passed, 5];
        close(fig1);
    else
        fprintf('  ✗ 2D轨迹绘制测试失败\n');
        test_results.failed = [test_results.failed, 5];
    end
    
    % 测试3D轨迹绘制
    traj_3d = [100*cos(t), 100*sin(t), 10*t];
    fig2 = plot_trajectory_3d({traj_3d}, {'测试轨迹'}, '3D轨迹测试');
    if ishandle(fig2)
        fprintf('  ✓ 3D轨迹绘制测试通过\n');
        test_results.passed = [test_results.passed, 5];
        close(fig2);
    else
        fprintf('  ✗ 3D轨迹绘制测试失败\n');
        test_results.failed = [test_results.failed, 5];
    end
    
    % 测试注意力热图
    attn = rand(20, 20);
    attn = attn ./ sum(attn, 2);
    fig3 = plot_attention_heatmap(attn, '注意力热图测试');
    if ishandle(fig3)
        fprintf('  ✓ 注意力热图绘制测试通过\n');
        test_results.passed = [test_results.passed, 5];
        close(fig3);
    else
        fprintf('  ✗ 注意力热图绘制测试失败\n');
        test_results.failed = [test_results.failed, 5];
    end
    
    % 测试混淆矩阵绘制
    conf_mat = [85, 5, 5, 5; 10, 75, 10, 5; 5, 10, 80, 5; 8, 7, 5, 80];
    fig4 = plot_confusion_matrix(conf_mat, {'CV', 'CA', 'CT', 'Singer'}, '混淆矩阵测试');
    if ishandle(fig4)
        fprintf('  ✓ 混淆矩阵绘制测试通过\n');
        test_results.passed = [test_results.passed, 5];
        close(fig4);
    else
        fprintf('  ✗ 混淆矩阵绘制测试失败\n');
        test_results.failed = [test_results.failed, 5];
    end
    
    fprintf('可视化工具测试完成\n\n');
    
catch ME
    fprintf('  ✗ 可视化工具测试异常: %s\n\n', ME.message);
    test_results.failed = [test_results.failed, 5];
end

%% ================ 测试6: 数据加载器（需要数据集）================

fprintf('测试 6/6: 数据加载器...\n');

dataset_path = '../enhanced_2d_dataset_v2';

if exist(dataset_path, 'dir')
    try
        % 创建测试配置
        test_config = struct();
        test_config.sequence_length = 30;  % 使用较短序列测试
        test_config.stride = 10;
        test_config.train_ratio = 0.7;
        test_config.val_ratio = 0.15;
        test_config.test_ratio = 0.15;
        test_config.normalize_method = 'minmax';
        test_config.random_seed = 42;
        
        % 加载数据
        data_loader = data_loader_maneuver_tracking(dataset_path, test_config);
        
        % 验证数据形状
        [N, num_rx, num_feat, seq_len] = size(data_loader.normalized_samples);
        
        if N > 0 && num_rx == 3 && num_feat == 3 && seq_len == test_config.sequence_length
            fprintf('  ✓ 数据加载测试通过 (样本数: %d)\n', N);
            test_results.passed = [test_results.passed, 6];
        else
            fprintf('  ✗ 数据加载测试失败（数据形状不正确）\n');
            test_results.failed = [test_results.failed, 6];
        end
        
        % 验证标签
        if size(data_loader.labels_state, 1) == N && ...
           size(data_loader.labels_state, 2) == 9
            fprintf('  ✓ 状态标签测试通过\n');
            test_results.passed = [test_results.passed, 6];
        else
            fprintf('  ✗ 状态标签测试失败\n');
            test_results.failed = [test_results.failed, 6];
        end
        
        % 验证归一化
        sample_data = data_loader.normalized_samples(:);
        if min(sample_data) >= 0 && max(sample_data) <= 1
            fprintf('  ✓ 数据归一化测试通过\n');
            test_results.passed = [test_results.passed, 6];
        else
            fprintf('  ⚠ 数据归一化可能有问题（范围: [%.3f, %.3f]）\n', ...
                min(sample_data), max(sample_data));
        end
        
        fprintf('数据加载器测试完成\n\n');
        
    catch ME
        fprintf('  ✗ 数据加载器测试异常: %s\n\n', ME.message);
        test_results.failed = [test_results.failed, 6];
    end
else
    fprintf('  ⚠ 跳过数据加载器测试（数据集不存在）\n');
    fprintf('  请先运行 radar_simulation_v3.m 生成数据集\n\n');
end

%% ================ 测试总结 ================

fprintf('=====================================================\n');
fprintf('  测试总结\n');
fprintf('=====================================================\n\n');

num_passed = length(test_results.passed);
num_failed = length(test_results.failed);
total_tests = num_passed + num_failed;

fprintf('总测试数: %d\n', total_tests);
fprintf('通过: %d (%.1f%%)\n', num_passed, 100*num_passed/total_tests);
fprintf('失败: %d (%.1f%%)\n', num_failed, 100*num_failed/total_tests);

if num_failed == 0
    fprintf('\n✓ 所有测试通过！系统运行正常。\n');
else
    fprintf('\n⚠ 有 %d 个测试失败，请检查相关模块。\n', num_failed);
end

fprintf('\n=====================================================\n');

% 关闭所有图表
close all;
