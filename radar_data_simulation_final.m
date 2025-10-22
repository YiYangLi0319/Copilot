%% =====================================================================
%  增强版双基地雷达数据集生成器 v4.0 (完整版)
%  =====================================================================
%  作者: YiYangLi0319
%  日期: 2025-10-21
%  
%  核心改进：
%  1. 清晰的状态转移矩阵（每个模型独立实现Φ和Q）
%  2. 修正的过程噪声注入方式：next_state = Phi * state + noise_vec
%  3. 三层时变过程噪声（目标类型/运动模型/时间阶梯）
%  4. SNR驱动的时变量测噪声 + 1-5%厚尾噪声（Student-t/高斯混合）
%  5. 物理约束的模型切换（连续性、最小持续时间）
%  
%  使用说明：
%  - 直接运行此脚本即可生成2D和3D数据集
%  - 生成的数据集保存在当前目录下的文件夹中
%  - 可通过修改配置参数自定义数据集
%  =====================================================================

clear all; close all; clc;

%% ================ 主程序入口 ================

fprintf('=====================================================\n');
fprintf('  增强版双基地雷达数据集生成器 v4.0\n');
fprintf('  作者: YiYangLi0319\n');
fprintf('  日期: %s\n', datestr(now));
fprintf('=====================================================\n\n');

% 设置全局随机种子
GLOBAL_SEED = 42;
rng(GLOBAL_SEED, 'twister');
fprintf('全局随机种子: %d\n', GLOBAL_SEED);

% 运行配置
run_demo = true;              % 是否运行可视化演示
save_data = true;             % 是否保存数据
num_scenarios = 20;           % 生成场景数量
run_validation = true;        % 是否运行数据验证
export_python = false;        % 是否导出Python格式（可选）

%% 生成2D数据集
fprintf('\n--- 生成2D数据集 ---\n');
config_2d = create_radar_config_v4(2);
dataset_2d = generate_dataset_v4(config_2d, num_scenarios, ...
    'enhanced_v4_2d_dataset', GLOBAL_SEED);

%% 生成3D数据集  
fprintf('\n--- 生成3D数据集 ---\n');
config_3d = create_radar_config_v4(3);
dataset_3d = generate_dataset_v4(config_3d, num_scenarios, ...
    'enhanced_v4_3d_dataset', GLOBAL_SEED + 100000);

%% 演示可视化
if run_demo
    fprintf('\n--- 可视化演示 ---\n');
    visualize_scenario_v4(dataset_2d.scenarios{1}, '2D场景示例 v4.0');
    visualize_scenario_v4(dataset_3d.scenarios{1}, '3D场景示例 v4.0');
end

%% 数据集统计分析
fprintf('\n=====================================================\n');
fprintf('  数据集统计分析\n');
fprintf('=====================================================\n');
analyze_dataset_v4('enhanced_v4_2d_dataset');
analyze_dataset_v4('enhanced_v4_3d_dataset');

%% 数据验证（可选）
if run_validation
    fprintf('\n--- 数据验证 ---\n');
    validate_dataset_v4('enhanced_v4_2d_dataset');
    validate_dataset_v4('enhanced_v4_3d_dataset');
end

%% 目标类型比较分析
fprintf('\n--- 目标类型比较分析 ---\n');
compare_target_types_v4(dataset_2d);

%% 导出Python格式（可选）
if export_python
    fprintf('\n--- 导出Python格式 ---\n');
    export_to_python_format_v4('enhanced_v4_2d_dataset', 'enhanced_v4_2d_python');
    export_to_python_format_v4('enhanced_v4_3d_dataset', 'enhanced_v4_3d_python');
end

fprintf('\n=====================================================\n');
fprintf('  数据集生成完成！\n');
fprintf('  2D数据集: enhanced_v4_2d_dataset/\n');
fprintf('  3D数据集: enhanced_v4_3d_dataset/\n');
fprintf('=====================================================\n\n');

%% ================ 配置函数 ================

function config = create_radar_config_v4(dimension)
    % 创建雷达系统配置
    % 输入: dimension - 2 (2D) 或 3 (3D)
    % 输出: config - 配置结构体
    
    config = struct();
    config.dimension = dimension;
    config.state_dimension = 9;  % [x,y,z, vx,vy,vz, ax,ay,az]
    config.num_receivers = 3;
    config.simulation_time = 100;  % 仿真时间(秒)
    config.dt = 0.1;  % 采样周期(秒)
    
    % ===== 接收机配置 =====
    config.receiver_area_radius = 8000;      % 接收机分布半径(米)
    config.min_receiver_distance = 1000;     % 接收机最小间距(米)
    config.min_receiver_radius = 2000;       % 接收机最小半径(米)
    
    % ===== FIM几何阈值 =====
    config.use_fim_geometry_check = true;
    config.gdop_threshold = 50.0;            % GDOP阈值(米)
    config.fim_condition_threshold = 1e8;
    config.geometry_area_threshold_2d = 1e6;
    config.geometry_volume_threshold_3d = 500;
    
    % ===== SNR参数 =====
    config.use_snr_noise_model = true;
    config.target_rcs = 2.0;                 % 目标RCS(平方米)
    config.tx_power = 10e3;                  % 发射功率(瓦)
    config.freq = 3e9;                       % 频率(赫兹)
    config.bandwidth = 1e6;                  % 带宽(赫兹)
    config.noise_figure = 1.5;               % 噪声系数(分贝)
    config.min_snr_db = 10.0;                % 最小SNR(分贝)
    config.max_snr_db = 80.0;                % 最大SNR(分贝)
    
    % ===== 厚尾噪声配置 =====
    config.heavy_tail_probability = 0.02;    % 厚尾噪声概率(1-5%)
    config.use_student_t = true;             % 使用Student-t分布
    config.student_t_dof = 3;                % Student-t自由度
    config.use_gaussian_mixture = false;     % 使用高斯混合
    config.mixture_large_var_factor = 25.0;  % 高斯混合大方差因子
    
    % ===== 目标类型定义（三层噪声第一层） =====
    config.target_types = {'slow', 'medium', 'fast'};
    
    % 慢速目标 (无人机/低速飞行器)
    config.slow_target = struct();
    config.slow_target.max_velocity = 50.0;           % m/s
    config.slow_target.max_acceleration = 3.0;        % m/s^2
    config.slow_target.max_turn_rate = 0.1;           % rad/s
    config.slow_target.noise_ranges = struct(...
        'CV', [0.1, 0.5], ...      % CV模型噪声范围 (m/s^2)
        'CA', [0.2, 0.8], ...      % CA模型噪声范围
        'CT', [0.15, 0.6], ...     % CT模型噪声范围
        'Singer', [0.1, 0.5]);     % Singer模型σ_m范围
    
    % 中速目标 (常规飞机)
    config.medium_target = struct();
    config.medium_target.max_velocity = 300.0;
    config.medium_target.max_acceleration = 6.0;
    config.medium_target.max_turn_rate = 0.07;
    config.medium_target.noise_ranges = struct(...
        'CV', [0.3, 1.0], ...
        'CA', [0.5, 1.5], ...
        'CT', [0.4, 1.2], ...
        'Singer', [0.3, 1.0]);
    
    % 快速目标 (战斗机/导弹)
    config.fast_target = struct();
    config.fast_target.max_velocity = 500.0;
    config.fast_target.max_acceleration = 40.0;
    config.fast_target.max_turn_rate = 0.3;
    config.fast_target.noise_ranges = struct(...
        'CV', [0.5, 2.0], ...
        'CA', [1.0, 3.0], ...
        'CT', [0.8, 2.5], ...
        'Singer', [0.5, 2.0]);
    
    % ===== 运动模型切换配置 =====
    config.available_models = {'CV', 'CA', 'CT', 'Singer'};
    config.min_model_duration = 20.0;        % 模型最小持续时间(秒)
    config.max_model_duration = 40.0;        % 模型最大持续时间(秒)
    config.min_noise_step_duration = 5.0;    % 噪声阶梯最小持续时间(秒)
    config.max_noise_step_duration = 15.0;   % 噪声阶梯最大持续时间(秒)
    
    % ===== 空间约束 =====
    config.initial_position_range = 4000;     % 初始位置范围(米)
    config.workspace_radius = 20000.0;       % 工作空间半径(米)
    
    if dimension == 3
        config.altitude_min = 1000.0;         % 最小高度(米)
        config.altitude_max = 8000.0;        % 最大高度(米)
    end
end

%% ================ 数据集生成 ================

function dataset = generate_dataset_v4(config, num_scenarios, output_dir, base_seed)
    % 生成完整数据集
    % 输入:
    %   config - 配置结构体
    %   num_scenarios - 场景数量
    %   output_dir - 输出目录
    %   base_seed - 基础随机种子
    % 输出:
    %   dataset - 数据集结构体
    
    if nargin < 3, output_dir = 'enhanced_v4_dataset'; end
    if nargin < 4, base_seed = 42; end
    
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    dataset = struct();
    dataset.config = config;
    dataset.scenarios = {};
    dataset.metadata = struct();
    dataset.metadata.creation_time = datestr(now);
    dataset.metadata.num_scenarios = num_scenarios;
    dataset.metadata.version = '4.0';
    dataset.metadata.base_seed = base_seed;
    dataset.metadata.author = 'YiYangLi0319';
    
    fprintf('生成 %d 个场景...\n', num_scenarios);
    
    for scenario_idx = 1:num_scenarios
        if mod(scenario_idx, 10) == 0 || scenario_idx == 1
            fprintf('  进度: %d/%d (%.1f%%)\n', scenario_idx, num_scenarios, ...
                    100*scenario_idx/num_scenarios);
        end
        
        scenario_seed = base_seed + scenario_idx * 1000;
        rng(scenario_seed, 'twister');
        
        % 随机选择目标类型
        target_type = config.target_types{randi(length(config.target_types))};
        
        scenario = generate_scenario_v4(config, target_type);
        scenario.scenario_id = scenario_idx - 1;
        scenario.seed = scenario_seed;
        scenario.target_type = target_type;
        dataset.scenarios{end+1} = scenario;
        
        % 保存单个场景为JSON
        save_scenario_json_v4(scenario, output_dir, scenario_idx - 1);
    end
    
    % 保存完整数据集
    dataset_file = fullfile(output_dir, 'dataset.mat');
    save(dataset_file, 'dataset', '-v7.3');
    fprintf('数据集已保存: %s\n', dataset_file);
    
    % 保存配置和元数据
    dataset_info = struct();
    dataset_info.config = config;
    dataset_info.metadata = dataset.metadata;
    save_json_file_v4(dataset_info, fullfile(output_dir, 'dataset_info.json'));
end

function scenario = generate_scenario_v4(config, target_type)
    % 生成单个场景（带重试机制）
    
    max_attempts = 200;
    
    for attempt = 1:max_attempts
        transmitter_pos = zeros(1, 3);
        receivers_pos = generate_receiver_positions_v4(config, transmitter_pos);
        
        scenario = struct();
        scenario.config = config;
        scenario.transmitter_position = transmitter_pos;
        scenario.receivers_positions = receivers_pos;
        scenario.target_type = target_type;
        
        time_steps = floor(config.simulation_time / config.dt);
        
        [target_info, measurements, detailed_records, is_valid] = ...
            generate_target_trajectory_v4(config, time_steps, target_type, ...
                                         transmitter_pos, receivers_pos);
        
        if is_valid
            scenario.target_info = target_info;
            scenario.measurements = measurements;
            scenario.detailed_records = detailed_records;
            return;
        end
        
        if attempt > 1 && mod(attempt, 50) == 0
            fprintf('    警告：第%d次尝试生成有效轨迹...\n', attempt);
        end
    end
    
    error('轨迹生成失败：已尝试%d次，请放宽参数约束。', max_attempts);
end

%% ================ 接收机位置生成 ================

function receivers_pos = generate_receiver_positions_v4(config, tx_pos)
    % 生成满足几何约束的接收机位置
    
    max_attempts = 1000;
    best_receivers = {};
    best_gdop = inf;
    
    for attempt = 1:max_attempts
        temp_receivers = {};
        
        min_r = config.min_receiver_radius;
        max_r = config.receiver_area_radius;
        
        for i = 1:config.num_receivers
            if config.dimension == 2
                angle = rand() * 2 * pi;
                radius = min_r + rand() * (max_r - min_r);
                pos = [radius * cos(angle), radius * sin(angle), 0];
            else
                phi = rand() * 2 * pi;
                theta = pi/6 + rand() * (2*pi/3);
                radius = min_r + rand() * (max_r - min_r);
                pos = [radius * sin(theta) * cos(phi), ...
                       radius * sin(theta) * sin(phi), ...
                       radius * cos(theta)];
            end
            temp_receivers{end+1} = pos;
        end
        
        if ~check_basic_geometry_v4(temp_receivers, config)
            continue;
        end
        
        if config.use_fim_geometry_check
            gdop = evaluate_geometry_gdop_v4(temp_receivers, tx_pos, config);
            if gdop < config.gdop_threshold && gdop < best_gdop
                best_receivers = temp_receivers;
                best_gdop = gdop;
            end
            
            if best_gdop < config.gdop_threshold * 0.5
                break;
            end
        else
            receivers_pos = temp_receivers;
            return;
        end
    end
    
    if isempty(best_receivers)
        warning('使用次优接收机配置 (GDOP=%.2f)', best_gdop);
        receivers_pos = temp_receivers;
    else
        receivers_pos = best_receivers;
    end
end

function is_valid = check_basic_geometry_v4(receivers, config)
    % 检查基本几何约束
    
    is_valid = true;
    
    % 检查接收机间距
    for i = 1:length(receivers)
        for j = i+1:length(receivers)
            dist = norm(receivers{i} - receivers{j});
            if dist < config.min_receiver_distance
                is_valid = false;
                return;
            end
        end
    end
    
    % 检查几何构型（面积或体积）
    if length(receivers) >= 3
        p1 = receivers{1};
        p2 = receivers{2};
        p3 = receivers{3};
        
        v1 = p2 - p1;
        v2 = p3 - p1;
        
        if config.dimension == 2
            cross_2d = abs(v1(1)*v2(2) - v1(2)*v2(1));
            if cross_2d < config.geometry_area_threshold_2d
                is_valid = false;
            end
        else
            cross_prod = cross(v1, v2);
            if norm(cross_prod) < config.geometry_volume_threshold_3d
                is_valid = false;
            end
        end
    end
end

function gdop = evaluate_geometry_gdop_v4(receivers, tx_pos, config)
    % 评估几何精度衰减因子
    
    num_test_points = 20;
    test_positions = generate_test_positions_v4(config, num_test_points);
    
    gdops = zeros(1, num_test_points);
    
    for i = 1:num_test_points
        target_pos = test_positions{i};
        
        FIM = zeros(config.dimension, config.dimension);
        
        for j = 1:length(receivers)
            rx_pos = receivers{j};
            
            H = compute_jacobian_v4(target_pos, tx_pos, rx_pos, config.dimension);
            
            if config.dimension == 2
                R = diag([25, (0.1*pi/180)^2]);
            else
                R = diag([25, (0.1*pi/180)^2, (0.1*pi/180)^2]);
            end
            
            FIM = FIM + H' * (R \ H);
        end
        
        try
            P_crlb = inv(FIM + 1e-12 * eye(config.dimension));
            gdops(i) = sqrt(trace(P_crlb));
        catch
            gdops(i) = 1e6;
        end
    end
    
    gdop = max(gdops);
end

function test_positions = generate_test_positions_v4(config, num_points)
    % 生成测试位置点
    
    test_positions = {};
    radius = config.receiver_area_radius * 0.6;
    
    for i = 1:num_points
        if config.dimension == 2
            angle = (i-1) * 2*pi / num_points;
            pos = [radius * cos(angle), radius * sin(angle), 0];
        else
            % Fibonacci球面采样
            phi = acos(1 - 2*(i-0.5)/num_points);
            theta = pi * (1 + sqrt(5)) * i;
            pos = [radius * sin(phi) * cos(theta), ...
                   radius * sin(phi) * sin(theta), ...
                   radius * cos(phi)];
        end
        test_positions{end+1} = pos;
    end
end

%% ================ 轨迹生成（核心） ================

function [target_info, measurements, detailed_records, is_valid] = ...
    generate_target_trajectory_v4(config, time_steps, target_type, tx, rxs)
    % 生成目标轨迹和量测
    
    % 获取目标参数
    target_params = get_target_params_v4(config, target_type);
    
    % 初始化状态
    pos_range = config.initial_position_range;
    vel_range = target_params.max_velocity * 0.3;
    
    if config.dimension == 2
        initial_pos = [(rand() - 0.5) * pos_range, (rand() - 0.5) * pos_range, 0];
        initial_vel = [(rand() - 0.5) * vel_range, (rand() - 0.5) * vel_range, 0];
    else
        initial_pos = [(rand() - 0.5) * pos_range, (rand() - 0.5) * pos_range, ...
                      rand() * pos_range/2];
        initial_vel = [(rand() - 0.5) * vel_range, (rand() - 0.5) * vel_range, ...
                      (rand() - 0.5) * vel_range/2];
    end
    initial_acc = [0, 0, 0];
    
    state = [initial_pos, initial_vel, initial_acc];
    
    % 生成运动模型序列和噪声调度
    [model_sequence, noise_schedule] = generate_model_and_noise_sequence_v4(...
        config, target_params, time_steps);
    
    % 初始化记录
    target_info = struct();
    target_info.initial_state = state;
    target_info.target_type = target_type;
    target_info.motion_model_sequence = model_sequence;
    target_info.noise_schedule = noise_schedule;
    target_info.trajectory = {};
    
    measurements = {};
    detailed_records = {};
    
    % 轨迹生成主循环
    for t = 1:time_steps
        current_time = (t-1) * config.dt;
        current_pos = state(1:3);
        current_vel = state(4:6);
        current_acc = state(7:9);
        
        % 记录轨迹点
        traj_point = struct();
        traj_point.time = current_time;
        traj_point.position = current_pos;
        traj_point.velocity = current_vel;
        traj_point.acceleration = current_acc;
        traj_point.full_state = state;
        traj_point.motion_model = model_sequence{t}.model;
        traj_point.noise_intensity = noise_schedule{t}.intensity;
        target_info.trajectory{end+1} = traj_point;
        
        % 生成量测
        [target_measurements, measurement_noise_info] = generate_measurements_v4(...
            current_pos, tx, rxs, config);
        
        % 计算CRLB
        crlb_result = compute_crlb_v4(current_pos, tx, rxs, ...
            measurement_noise_info, config);
        
        meas_point = struct();
        meas_point.time = current_time;
        meas_point.measurements = target_measurements;
        meas_point.noise_info = measurement_noise_info;
        meas_point.crlb = crlb_result;
        measurements{end+1} = meas_point;
        
        detailed_record = struct();
        detailed_record.time = current_time;
        detailed_record.true_state = state;
        detailed_record.measurements = target_measurements;
        detailed_record.crlb = crlb_result;
        detailed_records{end+1} = detailed_record;
        
        % 状态更新
        if t < time_steps
            state = update_state_v4(state, model_sequence{t}, noise_schedule{t}, ...
                                   config, target_params);
        end
    end
    
    % 验证轨迹有效性
    is_valid = validate_trajectory_v4(target_info.trajectory, config, target_params);
end

%% ================ 运动模型和噪声序列生成 ================

function [model_sequence, noise_schedule] = generate_model_and_noise_sequence_v4(...
    config, target_params, time_steps)
    % 生成运动模型序列和噪声调度
    % 三层噪声结构：目标类型层 -> 运动模型层 -> 时间阶梯层
    
    model_sequence = cell(1, time_steps);
    noise_schedule = cell(1, time_steps);
    
    current_step = 1;
    dt = config.dt;
    
    while current_step <= time_steps
        % 第二层：随机选择运动模型
        model_name = config.available_models{randi(length(config.available_models))};
        
        % 模型持续时间
        model_duration_sec = config.min_model_duration + ...
            rand() * (config.max_model_duration - config.min_model_duration);
        model_duration_steps = round(model_duration_sec / dt);
        model_end_step = min(current_step + model_duration_steps - 1, time_steps);
        
        % 生成模型参数
        model_params = generate_model_parameters_v4(model_name, config, target_params);
        
        % 第三层：为该段生成噪声阶梯
        noise_steps = generate_noise_steps_for_segment_v4(config, target_params, ...
            model_name, current_step, model_end_step, dt);
        
        % 填充模型序列和噪声调度
        noise_step_idx = 1;
        for t = current_step:model_end_step
            model_sequence{t} = struct('model', model_name, 'params', model_params);
            
            % 找到当前时刻对应的噪声阶梯
            while noise_step_idx < length(noise_steps) && ...
                  t > noise_steps{noise_step_idx}.end_step
                noise_step_idx = noise_step_idx + 1;
            end
            
            noise_schedule{t} = noise_steps{noise_step_idx};
        end
        
        current_step = model_end_step + 1;
    end
end

function noise_steps = generate_noise_steps_for_segment_v4(config, target_params, ...
    model_name, start_step, end_step, dt)
    % 为运动模型段生成噪声阶梯（第三层时变）
    
    noise_steps = {};
    
    % 获取该模型的噪声范围（第二层）
    noise_range = target_params.noise_ranges.(model_name);
    min_intensity = noise_range(1);
    max_intensity = noise_range(2);
    
    current_step = start_step;
    
    while current_step <= end_step
        % 噪声阶梯持续时间
        step_duration_sec = config.min_noise_step_duration + ...
            rand() * (config.max_noise_step_duration - config.min_noise_step_duration);
        step_duration_steps = round(step_duration_sec / dt);
        step_end = min(current_step + step_duration_steps - 1, end_step);
        
        % 在范围内随机噪声强度
        intensity = min_intensity + rand() * (max_intensity - min_intensity);
        
        noise_step = struct();
        noise_step.start_step = current_step;
        noise_step.end_step = step_end;
        noise_step.intensity = intensity;
        noise_step.model = model_name;
        
        noise_steps{end+1} = noise_step;
        
        current_step = step_end + 1;
    end
end

function model_params = generate_model_parameters_v4(model_name, config, target_params)
    % 生成运动模型参数
    
    model_params = struct();
    
    switch model_name
        case 'CV'
            model_params.type = 'CV';
            
        case 'CA'
            max_acc = target_params.max_acceleration;
            if config.dimension == 2
                model_params.acceleration = [(rand() - 0.5) * max_acc, ...
                                           (rand() - 0.5) * max_acc, 0];
            else
                model_params.acceleration = [(rand() - 0.5) * max_acc, ...
                                           (rand() - 0.5) * max_acc, ...
                                           (rand() - 0.5) * max_acc/2];
            end
            model_params.type = 'CA';
            
        case 'CT'
            max_turn = target_params.max_turn_rate;
            model_params.turn_rate = (rand() - 0.5) * max_turn;
            model_params.type = 'CT';
            
        case 'Singer'
            % Singer模型：τ = 10-30s
            tau = 10 + rand() * 20;
            model_params.beta = 1 / tau;
            model_params.type = 'Singer';
    end
end

%% ================ 状态更新（核心改进） ================

function next_state = update_state_v4(state, model_info, noise_info, config, target_params)
    % 状态更新：Φ矩阵 + 过程噪声注入
    % 核心修正：next_state = Phi * state + noise_vec
    
    model_name = model_info.model;
    model_params = model_info.params;
    dt = config.dt;
    dim = config.dimension;
    
    % 步骤1: 构建状态转移矩阵和过程噪声协方差矩阵
    [Phi, Q] = get_transition_matrices_v4(model_name, model_params, noise_info, dt, dim);
    
    % 步骤2: 确定性状态转移
    state_col = state(:);  % 转为列向量 [9x1]
    predicted_state = Phi * state_col;
    
    % 步骤3: 注入过程噪声（修正方式）
    try
        noise_vec = mvnrnd(zeros(9,1), Q)';
    catch
        % Q可能不正定，使用近似修正
        [V, D] = eig(Q);
        D = max(D, 0);
        Q_fixed = V * D * V';
        Q_fixed = (Q_fixed + Q_fixed') / 2;  % 确保对称性
        noise_vec = mvnrnd(zeros(9,1), Q_fixed)';
    end
    
    next_state = predicted_state + noise_vec;  % 关键修正：正确的噪声叠加
    next_state = next_state';  % 转回行向量 [1x9]
    
    % 步骤4: 2D场景Z分量置零
    if dim == 2
        next_state([3, 6, 9]) = 0;
    end
    
    % 步骤5: 应用物理约束
    next_state = apply_physical_constraints_v4(next_state, target_params, dim);
end

function [Phi, Q] = get_transition_matrices_v4(model_name, model_params, noise_info, dt, dim)
    % 根据运动模型构建状态转移矩阵和过程噪声协方差矩阵
    % 使用Kronecker积扩展3x3基础矩阵到9x9
    
    sigma = noise_info.intensity;  % 时变噪声强度
    
    switch model_name
        case 'CV'
            % 匀速模型：a=0, 但速度有噪声驱动
            Phi1 = [1, dt, 0; 
                    0, 1,  0; 
                    0, 0,  0];
            
            % 离散白噪声加速度 (DWNA) 驱动速度
            Q1 = sigma^2 * [dt^3/3, dt^2/2, 0; 
                            dt^2/2, dt,     0; 
                            0,      0,      0];
            
        case 'CA'
            % 恒加速度模型
            Phi1 = [1, dt, 0.5*dt^2; 
                    0, 1,  dt; 
                    0, 0,  1];
            
            % 白噪声加速度
            Q1 = sigma^2 * [dt^4/4, dt^3/2, dt^2/2; 
                            dt^3/2, dt^2,   dt; 
                            dt^2/2, dt,     1];
            
        case 'CT'
            % 协调转弯（线性化近似）
            omega = model_params.turn_rate;
            
            if abs(omega * dt) < 0.01
                % 小角度近似
                Phi1 = [1, dt, 0; 
                        0, 1,  0; 
                        0, 0,  1];
            else
                c = cos(omega * dt);
                s = sin(omega * dt);
                Phi1 = [1, s/omega, 0; 
                        0, c,       0; 
                        0, 0,       1];
            end
            
            Q1 = sigma^2 * [dt^3/3, dt^2/2, 0; 
                            dt^2/2, dt,     0; 
                            0,      0,      dt];
            
        case 'Singer'
            % Singer模型（精确离散化）
            beta = model_params.beta;
            [Phi1, Q1] = singer_discretization_v4(beta, sigma, dt);
    end
    
    % 使用Kronecker积扩展到3D (9x9)
    I3 = eye(3);
    Phi = kron(Phi1, I3);
    Q = kron(Q1, I3);
    
    % 2D场景：Z分量清零
    if dim == 2
        Phi([3,6,9], :) = 0;
        Phi(:, [3,6,9]) = 0;
        Phi([3,6,9], [3,6,9]) = eye(3);
        
        Q([3,6,9], :) = 0;
        Q(:, [3,6,9]) = 0;
    end
    
    % 确保Q正定
    Q = (Q + Q') / 2;
    Q = Q + 1e-10 * eye(9);
end

function [Phi, Q] = singer_discretization_v4(beta, sigma, dt)
    % Singer模型的精确离散化（Van Loan方法）
    
    % 连续时间状态空间
    Fc = [0, 1, 0; 
          0, 0, 1; 
          0, 0, -beta];
    L = [0; 0; 1];
    
    % 构建增广矩阵
    A = [-Fc, L*sigma^2*L'; 
         zeros(3), Fc'];
    Ad = expm(A * dt);
    
    % 提取Φ和Q
    Phi = Ad(4:6, 4:6)';
    Q = Phi * Ad(1:3, 4:6);
    
    % 确保对称性
    Q = (Q + Q') / 2;
    Q = Q + 1e-10 * eye(3);
end

function next_state = apply_physical_constraints_v4(state, target_params, dim)
    % 应用物理约束（速度和加速度限幅）
    
    % 速度限幅
    v = state(4:6);
    v_norm = norm(v);
    if v_norm > target_params.max_velocity
        state(4:6) = v * (target_params.max_velocity / v_norm);
    end
    
    % 加速度限幅
    a = state(7:9);
    a_norm = norm(a);
    if a_norm > target_params.max_acceleration
        state(7:9) = a * (target_params.max_acceleration / a_norm);
    end
    
    next_state = state;
end

function is_valid = validate_trajectory_v4(trajectory, config, target_params)
    % 验证轨迹是否满足所有物理约束
    
    is_valid = true;
    
    for k = 1:length(trajectory)
        p = trajectory{k}.position;
        v = trajectory{k}.velocity;
        a = trajectory{k}.acceleration;
        
        % 空间边界检查
        if config.dimension == 2
            if norm(p(1:2)) > config.workspace_radius
                is_valid = false;
                return;
            end
        else
            if norm(p) > config.workspace_radius
                is_valid = false;
                return;
            end
        end
        
        % 速度检查（允许10%误差）
        if norm(v) > target_params.max_velocity * 1.1
            is_valid = false;
            return;
        end
        
        % 加速度检查（允许20%误差）
        if norm(a) > target_params.max_acceleration * 1.2
            is_valid = false;
            return;
        end
    end
end

%% ================ 量测生成（改进厚尾噪声） ================

function [measurements, noise_info] = generate_measurements_v4(target_pos, tx_pos, receivers_pos, config)
    % 生成带时变噪声和厚尾噪声的测量值
    
    measurements = {};
    noise_info = {};
    
    for i = 1:length(receivers_pos)
        rx_pos = receivers_pos{i};
        
        % 计算真实测量值
        [true_range, true_azimuth, true_elevation] = compute_true_measurement_v4(...
            target_pos, tx_pos, rx_pos, config.dimension);
        
        % SNR驱动的噪声标准差
        if config.use_snr_noise_model
            snr_db = compute_snr_v4(target_pos, tx_pos, rx_pos, config);
            [range_noise_std, angle_noise_std] = snr_to_noise_std_v4(snr_db, config);
        else
            range_noise_std = 5.0;
            angle_noise_std = 0.1 * pi/180;
        end
        
        % 决定是否使用厚尾噪声（1-5%概率）
        use_heavy_tail = rand() < config.heavy_tail_probability;
        
        if use_heavy_tail
            if config.use_student_t
                % Student-t噪声
                dof = config.student_t_dof;
                noisy_range = true_range + trnd(dof) * range_noise_std;
                noisy_azimuth = wrapToPi(true_azimuth + trnd(dof) * angle_noise_std);
                
                if config.dimension == 3
                    noisy_elevation = clipElev(true_elevation + trnd(dof) * angle_noise_std);
                else
                    noisy_elevation = 0;
                end
                
                dist_type = 'student_t';
            elseif config.use_gaussian_mixture
                % 高斯混合（大方差分量）
                large_var_factor = config.mixture_large_var_factor;
                noisy_range = true_range + randn() * range_noise_std * sqrt(large_var_factor);
                noisy_azimuth = wrapToPi(true_azimuth + randn() * angle_noise_std * sqrt(large_var_factor));
                
                if config.dimension == 3
                    noisy_elevation = clipElev(true_elevation + randn() * angle_noise_std * sqrt(large_var_factor));
                else
                    noisy_elevation = 0;
                end
                
                dist_type = 'gaussian_mixture';
            else
                % 默认高斯
                noisy_range = true_range + randn() * range_noise_std;
                noisy_azimuth = wrapToPi(true_azimuth + randn() * angle_noise_std);
                if config.dimension == 3 
                    noisy_elevation =clipElev(true_elevation + randn() * angle_noise_std) ;
                else 
                    noisy_elevation =0;
                end
                dist_type = 'gaussian';
            end
        else
            % 标准高斯噪声（95-99%情况）
            noisy_range = true_range + randn() * range_noise_std;
            noisy_azimuth = wrapToPi(true_azimuth + randn() * angle_noise_std);
            if config.dimension == 3 
                noisy_elevation =clipElev(true_elevation + randn() * angle_noise_std) ;
            else 
                noisy_elevation =0;
            end          
            dist_type = 'gaussian';
        end
        
        measurements{end+1} = [noisy_range, noisy_azimuth, noisy_elevation];
        
        % 记录噪声信息
        receiver_noise_info = struct();
        receiver_noise_info.receiver_id = i;
        receiver_noise_info.true_range = true_range;
        receiver_noise_info.true_azimuth = true_azimuth;
        receiver_noise_info.true_elevation = true_elevation;
        receiver_noise_info.range_noise_std = range_noise_std;
        receiver_noise_info.angle_noise_std = angle_noise_std;
        receiver_noise_info.dist_type = dist_type;
        receiver_noise_info.is_heavy_tail = use_heavy_tail;
        
        if config.use_snr_noise_model
            receiver_noise_info.snr_db = snr_db;
        end
        
        % 等效协方差（用于CRLB计算）
        if config.dimension == 2
            receiver_noise_info.R = diag([range_noise_std^2, angle_noise_std^2]);
        else
            receiver_noise_info.R = diag([range_noise_std^2, angle_noise_std^2, angle_noise_std^2]);
        end
        
        noise_info{end+1} = receiver_noise_info;
    end
end

function snr_db = compute_snr_v4(target_pos, tx_pos, rx_pos, config)
    % 计算信噪比（雷达方程）
    
    lambda = 3e8 / config.freq;
    Pt = config.tx_power;
    Gt = 10^(27/10);  % 天线增益 30dBi
    Gr = 10^(27/10);
    sigma = config.target_rcs;
    
    R_tx = norm(target_pos - tx_pos);
    R_rx = norm(target_pos - rx_pos);
    
    % 接收功率
    Pr = (Pt * Gt * Gr * lambda^2 * sigma) / ((4*pi)^3 * R_tx^2 * R_rx^2);
    
    % 噪声功率
    k = 1.38e-23;  % 玻尔兹曼常数
    T = 290;       % 温度(K)
    B = config.bandwidth;
    Ls = 10^(13/10);  % 系统损耗
    Lt = 10^(8/10);   % 传输损耗
    NF = 10^(config.noise_figure / 10);
    Pn = k * T * B * NF * Ls * Lt;
    
    % SNR
    snr = Pr / Pn;
    snr_db = 10 * log10(max(snr, 1e-10));
    if snr_db < 0
        snr_db
    end
    
    % 限制范围
%     snr_db = max(snr_db, config.min_snr_db);
%     snr_db = min(snr_db, config.max_snr_db);
end

function [range_std, angle_std] = snr_to_noise_std_v4(snr_db, config)
    % 将SNR转换为噪声标准差
    
    snr_linear = 10^(snr_db / 10);
    
    % 距离噪声（基于带宽）
    B_rms = config.bandwidth / sqrt(3);
    sigma_tau = 1 / (2 * pi * B_rms * sqrt(2 * snr_linear));
    range_std = 3e8 * sigma_tau;
    range_std = max(5.0, min(range_std, 200.0));
    
    % 角度噪声
    angle_std = (0.1 * pi/180) / sqrt(snr_linear);
    angle_std = max(0.05*pi/180, min(angle_std, 2.0*pi/180));
end

%% ================ 辅助函数 ================

function target_params = get_target_params_v4(config, target_type)
    % 获取目标类型参数
    
    switch target_type
        case 'slow'
            target_params = config.slow_target;
        case 'medium'
            target_params = config.medium_target;
        case 'fast'
            target_params = config.fast_target;
        otherwise
            error('未知目标类型: %s', target_type);
    end
end

function [range_sum, azimuth, elevation] = compute_true_measurement_v4(...
    target_pos, tx_pos, rx_pos, dimension)
    % 计算真实测量值
    
    range_tx = norm(target_pos - tx_pos);
    range_rx = norm(target_pos - rx_pos);
    range_sum = range_tx + range_rx;
    
    direction = target_pos - rx_pos;
    
    if dimension == 2
        azimuth = atan2(direction(2), direction(1));
        elevation = 0;
    else
        azimuth = atan2(direction(2), direction(1));
        direction_norm = norm(direction);
        if direction_norm > 1e-9
            elevation = asin(direction(3) / direction_norm);
        else
            elevation = 0;
        end
    end
end

function a = wrapToPi(a)
    % 将角度包裹到[-π, π]
    a = mod(a + pi, 2*pi) - pi;
end

function e = clipElev(e)
    % 限制仰角到[-π/2, π/2]
    e = max(min(e, pi/2), -pi/2);
end

function H = compute_jacobian_v4(target_pos, tx_pos, rx_pos, dimension)
    % 计算量测雅可比矩阵
    
    eps_val = 1e-9;
    
    delta_tx = target_pos - tx_pos;
    dist_tx = max(norm(delta_tx), eps_val);
    
    delta_rx = target_pos - rx_pos;
    dist_rx = max(norm(delta_rx), eps_val);
    
    if dimension == 2
        dr_dx = delta_tx(1)/dist_tx + delta_rx(1)/dist_rx;
        dr_dy = delta_tx(2)/dist_tx + delta_rx(2)/dist_rx;
        
        d_squared = max(delta_rx(1)^2 + delta_rx(2)^2, eps_val);
        dtheta_dx = -delta_rx(2) / d_squared;
        dtheta_dy = delta_rx(1) / d_squared;
        
        H = [dr_dx, dr_dy;
             dtheta_dx, dtheta_dy];
    else
        dr_dx = delta_tx(1)/dist_tx + delta_rx(1)/dist_rx;
        dr_dy = delta_tx(2)/dist_tx + delta_rx(2)/dist_rx;
        dr_dz = delta_tx(3)/dist_tx + delta_rx(3)/dist_rx;
        
        d_horizontal_squared = max(delta_rx(1)^2 + delta_rx(2)^2, eps_val);
        dtheta_dx = -delta_rx(2) / d_horizontal_squared;
        dtheta_dy = delta_rx(1) / d_horizontal_squared;
        dtheta_dz = 0;
        
        d_horizontal = sqrt(d_horizontal_squared);
        d_total_squared = max(d_horizontal_squared + delta_rx(3)^2, eps_val);
        dphi_dx = -delta_rx(3) * delta_rx(1) / (d_horizontal * d_total_squared);
        dphi_dy = -delta_rx(3) * delta_rx(2) / (d_horizontal * d_total_squared);
        dphi_dz = d_horizontal / d_total_squared;
        
        H = [dr_dx, dr_dy, dr_dz;
             dtheta_dx, dtheta_dy, dtheta_dz;
             dphi_dx, dphi_dy, dphi_dz];
    end
end

function crlb_result = compute_crlb_v4(target_pos, tx_pos, receivers_pos, ...
    measurement_noise_info, config)
    % 计算Cramér-Rao下界
    
    dim = config.dimension;
    FIM = zeros(dim, dim);
    
    for i = 1:length(receivers_pos)
        rx_pos = receivers_pos{i};
        H = compute_jacobian_v4(target_pos, tx_pos, rx_pos, dim);
        R = measurement_noise_info{i}.R;
        
        try
            FIM = FIM + H' * (R \ H);
        catch
            FIM = FIM + H' * pinv(R) * H;
        end
    end
    
    lambda = 1e-12;
    try
        P_crlb = inv(FIM + lambda * eye(dim));
    catch
        P_crlb = pinv(FIM);
    end
    
    [V, D] = eig(P_crlb);
    D = max(D, 0);
    P_crlb = V * D * V';
    
    sigma_pos = sqrt(diag(P_crlb));
    
    try
        cond_num = cond(FIM);
    catch
        cond_num = 1e10;
    end
    
    gdop = sqrt(sum(sigma_pos.^2));
    
    crlb_result = struct();
    crlb_result.P_crlb = P_crlb;
    crlb_result.sigma_pos = sigma_pos;
    crlb_result.cond_num = cond_num;
    crlb_result.gdop = gdop;
end

%% ================ JSON存储 ================

function save_scenario_json_v4(scenario, output_dir, scenario_id)
    % 保存场景为JSON文件
    
    filename = fullfile(output_dir, sprintf('scenario_%04d.json', scenario_id));
    json_scenario = convert_to_json_format_v4(scenario);
    save_json_file_v4(json_scenario, filename);
end

function json_data = convert_to_json_format_v4(matlab_data)
    % 递归转换MATLAB数据为JSON格式
    
    if isstruct(matlab_data)
        json_data = struct();
        fields = fieldnames(matlab_data);
        for i = 1:length(fields)
            field_name = fields{i};
            field_value = matlab_data.(field_name);
            json_data.(field_name) = convert_to_json_format_v4(field_value);
        end
    elseif iscell(matlab_data)
        json_data = cell(size(matlab_data));
        for i = 1:numel(matlab_data)
            json_data{i} = convert_to_json_format_v4(matlab_data{i});
        end
    else
        json_data = matlab_data;
    end
end

function save_json_file_v4(data, filename)
    % 保存数据为JSON文件
    
    try
        json_str = jsonencode(data, 'PrettyPrint', true);
        fid = fopen(filename, 'w', 'n', 'UTF-8');
        fprintf(fid, '%s', json_str);
        fclose(fid);
    catch ME
%         warning('JSON保存失败: %s', ME.message);
        [filepath, name, ~] = fileparts(filename);
        mat_filename = fullfile(filepath, [name, '.mat']);
        save(mat_filename, 'data');
        fprintf('  已保存为MAT格式: %s\n', mat_filename);
    end
end

%% ================ 可视化 ================

function visualize_scenario_v4(scenario, title_str)
    % 可视化场景
    
    if nargin < 2, title_str = '场景 v4.0'; end
    
    config = scenario.config;
    dimension = config.dimension;
    
    figure('Position', [100, 100, 1800, 1200], 'Name', title_str);
    
    if dimension == 2
        % 2D可视化（9个子图）
        subplot(3, 3, 1);
        plot_trajectory_and_radars_v4(scenario);
        
        subplot(3, 3, 2);
        plot_motion_model_sequence_v4(scenario);
        
        subplot(3, 3, 3);
        plot_noise_intensity_evolution_v4(scenario);
        
        subplot(3, 3, 4);
        plot_velocity_acceleration_v4(scenario);
        
        subplot(3, 3, 5);
        plot_snr_evolution_v4(scenario);
        
        subplot(3, 3, 6);
        plot_range_measurements_v4(scenario);
        
        subplot(3, 3, 7);
        plot_angle_measurements_v4(scenario);
        
        subplot(3, 3, 8);
        plot_gdop_analysis_v4(scenario);
        
        subplot(3, 3, 9);
        plot_heavy_tail_statistics_v4(scenario);
        
    else
        % 3D可视化（4个子图）
        subplot(2, 2, 1);
        plot_3d_trajectory_and_radars_v4(scenario);
        
        subplot(2, 2, 2);
        plot_motion_model_sequence_v4(scenario);
        
        subplot(2, 2, 3);
        plot_noise_intensity_evolution_v4(scenario);
        
        subplot(2, 2, 4);
        plot_velocity_acceleration_v4(scenario);
    end
    
    sgtitle(sprintf('%s - 目标类型: %s', title_str, scenario.target_type), ...
            'FontSize', 16, 'FontWeight', 'bold');
end

function plot_trajectory_and_radars_v4(scenario)
    % 绘制轨迹和雷达配置
    
    hold on;
    
    tx_pos = scenario.transmitter_position;
    plot(tx_pos(1), tx_pos(2), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'red', ...
        'DisplayName', '发射机');
    
    for i = 1:length(scenario.receivers_positions)
        rx_pos = scenario.receivers_positions{i};
        if i == 1
            plot(rx_pos(1), rx_pos(2), 'b^', 'MarkerSize', 12, 'MarkerFaceColor', 'blue', ...
                 'DisplayName', '接收机');
        else
            plot(rx_pos(1), rx_pos(2), 'b^', 'MarkerSize', 12, 'MarkerFaceColor', 'blue', ...
                 'HandleVisibility', 'off');
        end
    end
    
    trajectory = scenario.target_info.trajectory;
    positions = zeros(length(trajectory), 2);
    for j = 1:length(trajectory)
        positions(j, :) = trajectory{j}.position(1:2);
    end
    
    plot(positions(:, 1), positions(:, 2), 'g-', 'LineWidth', 2.5, 'DisplayName', '轨迹');
    plot(positions(1, 1), positions(1, 2), 'go', 'MarkerSize', 10, ...
         'MarkerFaceColor', 'green', 'DisplayName', '起点');
    plot(positions(end, 1), positions(end, 2), 'mo', 'MarkerSize', 10, ...
         'MarkerFaceColor', 'magenta', 'DisplayName', '终点');
    
    legend('Location', 'best');
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('轨迹与雷达配置');
    axis equal;
end

function plot_motion_model_sequence_v4(scenario)
    % 绘制运动模型时序
    
    hold on;
    
    trajectory = scenario.target_info.trajectory;
    model_colors = containers.Map({'CV', 'CA', 'CT', 'Singer'}, ...
                                  {[1 0 0], [0 1 0], [0 0 1], [1 0 1]});
    
    times = zeros(1, length(trajectory));
    for j = 1:length(trajectory)
        times(j) = trajectory{j}.time;
        model = trajectory{j}.motion_model;
        
        color = [0 0 0];
        if isKey(model_colors, model)
            color = model_colors(model);
        end
        
        if j < length(trajectory)
            plot([times(j), times(j+1)], [1, 1], 'Color', color, 'LineWidth', 6);
        end
    end
    
    xlabel('时间 (s)');
    title('运动模型时序');
    grid on;
    ylim([0.5, 1.5]);
    yticks([]);
    
    % 图例
    legend_models = {'CV', 'CA', 'CT', 'Singer'};
    for k = 1:length(legend_models)
        plot(NaN, NaN, 'Color', model_colors(legend_models{k}), 'LineWidth', 4, ...
             'DisplayName', legend_models{k});
    end
    legend('Location', 'eastoutside');
end

function plot_noise_intensity_evolution_v4(scenario)
    % 绘制过程噪声强度演化
    
    trajectory = scenario.target_info.trajectory;
    times = zeros(1, length(trajectory));
    intensities = zeros(1, length(trajectory));
    
    for j = 1:length(trajectory)
        times(j) = trajectory{j}.time;
        intensities(j) = trajectory{j}.noise_intensity;
    end
    
    stairs(times, intensities, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('时间 (s)');
    ylabel('噪声强度 (m/s² 或 σ_m)');
    title('过程噪声强度演化（阶梯时变）');
end

function plot_velocity_acceleration_v4(scenario)
    % 绘制速度和加速度
    
    hold on;
    
    trajectory = scenario.target_info.trajectory;
    times = zeros(1, length(trajectory));
    velocities = zeros(length(trajectory), 3);
    accelerations = zeros(length(trajectory), 3);
    
    for j = 1:length(trajectory)
        times(j) = trajectory{j}.time;
        velocities(j, :) = trajectory{j}.velocity;
        accelerations(j, :) = trajectory{j}.acceleration;
    end
    
    vel_magnitude = sqrt(sum(velocities.^2, 2));
    acc_magnitude = sqrt(sum(accelerations.^2, 2));
    
    yyaxis left;
    plot(times, vel_magnitude, 'b-', 'LineWidth', 2);
    ylabel('速度 (m/s)', 'Color', 'b');
    
    yyaxis right;
    plot(times, acc_magnitude, 'r-', 'LineWidth', 2);
    ylabel('加速度 (m/s²)', 'Color', 'r');
    
    grid on;
    xlabel('时间 (s)');
    title('速度和加速度');
end

function plot_snr_evolution_v4(scenario)
    % 绘制SNR演化
    
    measurements = scenario.measurements;
    times = zeros(1, length(measurements));
    snrs = zeros(1, length(measurements));
    
    for j = 1:length(measurements)
        times(j) = measurements{j}.time;
        if isfield(measurements{j}.noise_info{1}, 'snr_db')
            snrs(j) = measurements{j}.noise_info{1}.snr_db;
        else
            snrs(j) = NaN;
        end
    end
    
    if all(isnan(snrs))
        text(0.5, 0.5, 'SNR模型未启用', 'HorizontalAlignment', 'center', ...
             'FontSize', 12);
        axis off;
    else
        plot(times, snrs, 'b-', 'LineWidth', 2);
        grid on;
        xlabel('时间 (s)');
        ylabel('SNR (dB)');
        title('信噪比演化');
    end
end

function plot_range_measurements_v4(scenario)
    % 绘制距离测量
    
    measurements = scenario.measurements;
    times = zeros(1, length(measurements));
    noisy_ranges = zeros(1, length(measurements));
    
    for j = 1:length(measurements)
        times(j) = measurements{j}.time;
        noisy_ranges(j) = measurements{j}.measurements{1}(1);
    end
    
    plot(times, noisy_ranges, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)');
    ylabel('双基地距离和 (m)');
    title('距离测量');
end

function plot_angle_measurements_v4(scenario)
    % 绘制角度测量
    
    measurements = scenario.measurements;
    times = zeros(1, length(measurements));
    noisy_azimuths = zeros(1, length(measurements));
    
    for j = 1:length(measurements)
        times(j) = measurements{j}.time;
        noisy_azimuths(j) = measurements{j}.measurements{1}(2) * 180/pi;
    end
    
    plot(times, noisy_azimuths, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)');
    ylabel('方位角 (度)');
    title('角度测量');
end

function plot_gdop_analysis_v4(scenario)
    % 绘制GDOP分析
    
    measurements = scenario.measurements;
    times = zeros(1, length(measurements));
    gdops = zeros(1, length(measurements));
    
    for j = 1:length(measurements)
        times(j) = measurements{j}.time;
        gdops(j) = measurements{j}.crlb.gdop;
    end
    
    plot(times, gdops, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('时间 (s)');
    ylabel('GDOP (m)');
    title('几何精度衰减因子');
end

function plot_heavy_tail_statistics_v4(scenario)
    % 绘制厚尾噪声统计
    
    measurements = scenario.measurements;
    total_meas = 0;
    heavy_tail_count = 0;
    
    for j = 1:length(measurements)
        for k = 1:length(measurements{j}.noise_info)
            total_meas = total_meas + 1;
            if isfield(measurements{j}.noise_info{k}, 'is_heavy_tail') && ...
               measurements{j}.noise_info{k}.is_heavy_tail
                heavy_tail_count = heavy_tail_count + 1;
            end
        end
    end
    
    heavy_tail_rate = heavy_tail_count / max(total_meas, 1) * 100;
    
    bar([1, 2], [total_meas - heavy_tail_count, heavy_tail_count]);
    set(gca, 'XTickLabel', {'高斯', '厚尾'});
    ylabel('数量');
    title(sprintf('量测噪声类型分布 (厚尾: %.2f%%)', heavy_tail_rate));
    grid on;
end

function plot_3d_trajectory_and_radars_v4(scenario)
    % 绘制3D轨迹和雷达配置
    
    hold on;
    
    tx_pos = scenario.transmitter_position;
    scatter3(tx_pos(1), tx_pos(2), tx_pos(3), 150, 'red', 's', 'filled', ...
             'DisplayName', '发射机');
    
    for i = 1:length(scenario.receivers_positions)
        rx_pos = scenario.receivers_positions{i};
        if i == 1
            scatter3(rx_pos(1), rx_pos(2), rx_pos(3), 100, 'blue', '^', 'filled', ...
                     'DisplayName', '接收机');
        else
            scatter3(rx_pos(1), rx_pos(2), rx_pos(3), 100, 'blue', '^', 'filled', ...
                     'HandleVisibility', 'off');
        end
    end
    
    trajectory = scenario.target_info.trajectory;
    positions = zeros(length(trajectory), 3);
    for j = 1:length(trajectory)
        positions(j, :) = trajectory{j}.position;
    end
    
    plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'g-', ...
          'LineWidth', 2.5, 'DisplayName', '轨迹');
    scatter3(positions(1, 1), positions(1, 2), positions(1, 3), 80, ...
             'green', 'o', 'filled', 'DisplayName', '起点');
    scatter3(positions(end, 1), positions(end, 2), positions(end, 3), 80, ...
             'magenta', 'o', 'filled', 'DisplayName', '终点');
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D轨迹与雷达配置');
    legend('Location', 'best');
    grid on;
    view(3);
end

%% ================ 数据分析 ================

function analyze_dataset_v4(dataset_dir)
    % 分析数据集统计信息
    
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    
    if ~exist(dataset_file, 'file')
        fprintf('数据集不存在: %s\n', dataset_file);
        return;
    end
    
    load(dataset_file, 'dataset');
    
    fprintf('\n=== %s 分析 ===\n', dataset_dir);
    fprintf('版本: %s\n', dataset.metadata.version);
    fprintf('维度: %dD\n', dataset.config.dimension);
    fprintf('场景数: %d\n', length(dataset.scenarios));
    fprintf('dt: %.2f s\n', dataset.config.dt);
    fprintf('仿真时间: %.1f s\n', dataset.config.simulation_time);
    
    if ~isempty(dataset.scenarios)
        % 统计所有场景
        all_model_stats = containers.Map();
        all_target_types = containers.Map();
        all_gdops = [];
        all_velocities = [];
        all_accelerations = [];
        all_heavy_tail_rates = [];
        all_noise_intensities = [];
        
        for scenario_idx = 1:length(dataset.scenarios)
            scenario = dataset.scenarios{scenario_idx};
            
            % 目标类型统计
            target_type = scenario.target_type;
            if isKey(all_target_types, target_type)
                all_target_types(target_type) = all_target_types(target_type) + 1;
            else
                all_target_types(target_type) = 1;
            end
            
            trajectory = scenario.target_info.trajectory;
            
            % 运动模型统计
            for j = 1:length(trajectory)
                model = trajectory{j}.motion_model;
                if isKey(all_model_stats, model)
                    all_model_stats(model) = all_model_stats(model) + 1;
                else
                    all_model_stats(model) = 1;
                end
                
                % 噪声强度统计
                all_noise_intensities = [all_noise_intensities, trajectory{j}.noise_intensity];
            end
            
            % GDOP统计
            for i = 1:length(scenario.measurements)
                all_gdops = [all_gdops, scenario.measurements{i}.crlb.gdop];
            end
            
            % 速度/加速度统计
            for j = 1:length(trajectory)
                v = trajectory{j}.velocity;
                a = trajectory{j}.acceleration;
                all_velocities = [all_velocities, norm(v)];
                all_accelerations = [all_accelerations, norm(a)];
            end
            
            % 厚尾噪声统计
            heavy_tail_count = 0;
            total_count = 0;
            for i = 1:length(scenario.measurements)
                for j = 1:length(scenario.measurements{i}.noise_info)
                    total_count = total_count + 1;
                    if isfield(scenario.measurements{i}.noise_info{j}, 'is_heavy_tail') && ...
                       scenario.measurements{i}.noise_info{j}.is_heavy_tail
                        heavy_tail_count = heavy_tail_count + 1;
                    end
                end
            end
            all_heavy_tail_rates = [all_heavy_tail_rates, 100 * heavy_tail_count / total_count];
        end
        
        % 打印目标类型分布
        fprintf('\n--- 目标类型分布 ---\n');
        target_types = keys(all_target_types);
        for i = 1:length(target_types)
            fprintf('  %s: %d 场景 (%.1f%%)\n', target_types{i}, ...
                    all_target_types(target_types{i}), ...
                    100 * all_target_types(target_types{i}) / length(dataset.scenarios));
        end
        
        % 打印运动模型统计
        fprintf('\n--- 运动模型使用统计 ---\n');
        models = keys(all_model_stats);
        total_steps = 0;
        for i = 1:length(models)
            total_steps = total_steps + all_model_stats(models{i});
        end
        for i = 1:length(models)
            fprintf('  %s: %d 次 (%.1f%%)\n', models{i}, ...
                    all_model_stats(models{i}), ...
                    100 * all_model_stats(models{i}) / total_steps);
        end
        
        % GDOP统计
        fprintf('\n--- GDOP统计 ---\n');
        fprintf('  均值: %.2f m\n', mean(all_gdops));
        fprintf('  标准差: %.2f m\n', std(all_gdops));
        fprintf('  范围: [%.2f, %.2f] m\n', min(all_gdops), max(all_gdops));
        fprintf('  中位数: %.2f m\n', median(all_gdops));
        
        % 速度统计
        fprintf('\n--- 速度统计 ---\n');
        fprintf('  均值: %.2f m/s\n', mean(all_velocities));
        fprintf('  标准差: %.2f m/s\n', std(all_velocities));
        fprintf('  范围: [%.2f, %.2f] m/s\n', min(all_velocities), max(all_velocities));
        fprintf('  中位数: %.2f m/s\n', median(all_velocities));
        
        % 加速度统计
        fprintf('\n--- 加速度统计 ---\n');
        fprintf('  均值: %.2f m/s²\n', mean(all_accelerations));
        fprintf('  标准差: %.2f m/s²\n', std(all_accelerations));
        fprintf('  范围: [%.2f, %.2f] m/s²\n', min(all_accelerations), max(all_accelerations));
        fprintf('  中位数: %.2f m/s²\n', median(all_accelerations));
        
        % 过程噪声强度统计
        fprintf('\n--- 过程噪声强度统计 ---\n');
        fprintf('  均值: %.3f\n', mean(all_noise_intensities));
        fprintf('  标准差: %.3f\n', std(all_noise_intensities));
        fprintf('  范围: [%.3f, %.3f]\n', min(all_noise_intensities), max(all_noise_intensities));
        fprintf('  中位数: %.3f\n', median(all_noise_intensities));
        
        % 厚尾噪声统计
        fprintf('\n--- 厚尾噪声统计 ---\n');
        fprintf('  平均比例: %.2f%%\n', mean(all_heavy_tail_rates));
        fprintf('  标准差: %.2f%%\n', std(all_heavy_tail_rates));
        fprintf('  范围: [%.2f%%, %.2f%%]\n', min(all_heavy_tail_rates), max(all_heavy_tail_rates));
        
        % 绘制统计图
        plot_dataset_statistics_v4(dataset, all_model_stats, all_target_types, ...
                                  all_gdops, all_velocities, all_accelerations, ...
                                  all_noise_intensities, all_heavy_tail_rates);
    end
end

function plot_dataset_statistics_v4(dataset, model_stats, target_types, ...
                                   gdops, velocities, accelerations, ...
                                   noise_intensities, heavy_tail_rates)
    % 绘制数据集统计图
    
    figure('Position', [100, 100, 1600, 1000], 'Name', '数据集统计概览');
    
    % 子图1: 目标类型分布
    subplot(3, 3, 1);
    types = keys(target_types);
    counts = zeros(1, length(types));
    for i = 1:length(types)
        counts(i) = target_types(types{i});
    end
    bar(counts);
    set(gca, 'XTickLabel', types);
    ylabel('场景数');
    title('目标类型分布');
    grid on;
    
    % 子图2: 运动模型分布
    subplot(3, 3, 2);
    models = keys(model_stats);
    model_counts = zeros(1, length(models));
    for i = 1:length(models)
        model_counts(i) = model_stats(models{i});
    end
    bar(model_counts);
    set(gca, 'XTickLabel', models);
    ylabel('使用次数');
    title('运动模型分布');
    grid on;
    
    % 子图3: GDOP分布
    subplot(3, 3, 3);
    histogram(gdops, 50);
    xlabel('GDOP (m)');
    ylabel('频次');
    title(sprintf('GDOP分布 (μ=%.2f m)', mean(gdops)));
    grid on;
    
    % 子图4: 速度分布
    subplot(3, 3, 4);
    histogram(velocities, 50);
    xlabel('速度 (m/s)');
    ylabel('频次');
    title(sprintf('速度分布 (μ=%.2f m/s)', mean(velocities)));
    grid on;
    
    % 子图5: 加速度分布
    subplot(3, 3, 5);
    histogram(accelerations, 50);
    xlabel('加速度 (m/s²)');
    ylabel('频次');
    title(sprintf('加速度分布 (μ=%.2f m/s²)', mean(accelerations)));
    grid on;
    
    % 子图6: 过程噪声强度分布
    subplot(3, 3, 6);
    histogram(noise_intensities, 50);
    xlabel('噪声强度');
    ylabel('频次');
    title(sprintf('过程噪声强度分布 (μ=%.3f)', mean(noise_intensities)));
    grid on;
    
    % 子图7: 厚尾噪声比例分布
    subplot(3, 3, 7);
    histogram(heavy_tail_rates, 30);
    xlabel('厚尾噪声比例 (%)');
    ylabel('场景数');
    title(sprintf('厚尾噪声比例分布 (μ=%.2f%%)', mean(heavy_tail_rates)));
    grid on;
    
    % 子图8: 速度 vs 加速度散点图
    subplot(3, 3, 8);
    scatter(velocities, accelerations, 5, 'filled', 'MarkerFaceAlpha', 0.3);
    xlabel('速度 (m/s)');
    ylabel('加速度 (m/s²)');
    title('速度-加速度关系');
    grid on;
    
    % 子图9: GDOP vs 噪声强度散点图
    subplot(3, 3, 9);
    sample_size = min(length(gdops), length(noise_intensities));
    gdop_sample = gdops(randperm(length(gdops), sample_size));
    noise_sample = noise_intensities(randperm(length(noise_intensities), sample_size));
    scatter(noise_sample, gdop_sample, 5, 'filled', 'MarkerFaceAlpha', 0.3);
    xlabel('噪声强度');
    ylabel('GDOP (m)');
    title('噪声强度-GDOP关系');
    grid on;
    
    sgtitle(sprintf('数据集统计概览 - v%s (%dD)', ...
                    dataset.metadata.version, dataset.config.dimension), ...
            'FontSize', 16, 'FontWeight', 'bold');
end

%% ================ 目标类型比较分析 ================

function compare_target_types_v4(dataset)
    % 比较不同目标类型的特征
    
    if isempty(dataset.scenarios)
        fprintf('数据集为空\n');
        return;
    end
    
    % 按目标类型分组
    type_groups = containers.Map();
    
    for scenario_idx = 1:length(dataset.scenarios)
        scenario = dataset.scenarios{scenario_idx};
        target_type = scenario.target_type;
        
        if ~isKey(type_groups, target_type)
            type_groups(target_type) = {};
        end
        
        type_groups(target_type) = [type_groups(target_type), {scenario}];
    end
    
    % 为每个类型计算统计量
    types = keys(type_groups);
    
    fprintf('\n=== 目标类型详细比较 ===\n');
    
    for i = 1:length(types)
        target_type = types{i};
        scenarios = type_groups(target_type);
        
        fprintf('\n--- %s 目标 (共 %d 个场景) ---\n', target_type, length(scenarios));
        
        velocities = [];
        accelerations = [];
        noise_intensities = [];
        
        for j = 1:length(scenarios)
            scenario = scenarios{j};
            trajectory = scenario.target_info.trajectory;
            
            for k = 1:length(trajectory)
                v = trajectory{k}.velocity;
                a = trajectory{k}.acceleration;
                velocities = [velocities, norm(v)];
                accelerations = [accelerations, norm(a)];
                noise_intensities = [noise_intensities, trajectory{k}.noise_intensity];
            end
        end
        
        fprintf('速度: %.2f ± %.2f m/s (范围: [%.2f, %.2f] m/s)\n', ...
                mean(velocities), std(velocities), min(velocities), max(velocities));
        fprintf('加速度: %.2f ± %.2f m/s² (范围: [%.2f, %.2f] m/s²)\n', ...
                mean(accelerations), std(accelerations), min(accelerations), max(accelerations));
        fprintf('噪声强度: %.3f ± %.3f (范围: [%.3f, %.3f])\n', ...
                mean(noise_intensities), std(noise_intensities), ...
                min(noise_intensities), max(noise_intensities));
    end
    
    % 可视化比较
    plot_target_type_comparison_v4(type_groups);
end

function plot_target_type_comparison_v4(type_groups)
    % 绘制目标类型比较图
    
    types = keys(type_groups);
    num_types = length(types);
    
    figure('Position', [100, 100, 1600, 800], 'Name', '目标类型比较');
    
    colors = {[1 0 0], [0 1 0], [0 0 1], [1 0 1], [0 1 1]};
    
    % 速度分布比较
    subplot(1, 3, 1);
    hold on;
    for i = 1:num_types
        target_type = types{i};
        scenarios = type_groups(target_type);
        
        velocities = [];
        for j = 1:length(scenarios)
            trajectory = scenarios{j}.target_info.trajectory;
            for k = 1:length(trajectory)
                v = trajectory{k}.velocity;
                velocities = [velocities, norm(v)];
            end
        end
        
        histogram(velocities, 30, 'FaceColor', colors{mod(i-1, length(colors))+1}, ...
                 'FaceAlpha', 0.5, 'DisplayName', target_type);
    end
    xlabel('速度 (m/s)');
    ylabel('频次');
    title('速度分布比较');
    legend('Location', 'best');
    grid on;
    
    % 加速度分布比较
    subplot(1, 3, 2);
    hold on;
    for i = 1:num_types
        target_type = types{i};
        scenarios = type_groups(target_type);
        
        accelerations = [];
        for j = 1:length(scenarios)
            trajectory = scenarios{j}.target_info.trajectory;
            for k = 1:length(trajectory)
                a = trajectory{k}.acceleration;
                accelerations = [accelerations, norm(a)];
            end
        end
        
        histogram(accelerations, 30, 'FaceColor', colors{mod(i-1, length(colors))+1}, ...
                 'FaceAlpha', 0.5, 'DisplayName', target_type);
    end
    xlabel('加速度 (m/s²)');
    ylabel('频次');
    title('加速度分布比较');
    legend('Location', 'best');
    grid on;
    
    % 噪声强度分布比较
    subplot(1, 3, 3);
    hold on;
    for i = 1:num_types
        target_type = types{i};
        scenarios = type_groups(target_type);
        
        noise_intensities = [];
        for j = 1:length(scenarios)
            trajectory = scenarios{j}.target_info.trajectory;
            for k = 1:length(trajectory)
                noise_intensities = [noise_intensities, trajectory{k}.noise_intensity];
            end
        end
        
        histogram(noise_intensities, 30, 'FaceColor', colors{mod(i-1, length(colors))+1}, ...
                 'FaceAlpha', 0.5, 'DisplayName', target_type);
    end
    xlabel('噪声强度');
    ylabel('频次');
    title('过程噪声强度分布比较');
    legend('Location', 'best');
    grid on;
    
    sgtitle('不同目标类型特征比较', 'FontSize', 16, 'FontWeight', 'bold');
end

%% ================ 数据验证工具 ================

function validate_dataset_v4(dataset_dir)
    % 验证数据集的完整性和正确性
    
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    
    if ~exist(dataset_file, 'file')
        fprintf('数据集不存在: %s\n', dataset_file);
        return;
    end
    
    load(dataset_file, 'dataset');
    
    fprintf('\n=== 数据集验证 ===\n');
    fprintf('数据集目录: %s\n', dataset_dir);
    fprintf('版本: %s\n', dataset.metadata.version);
    
    issues = {};
    warnings_list = {};
    
    % 检查场景数
    if length(dataset.scenarios) ~= dataset.metadata.num_scenarios
        issues{end+1} = sprintf('场景数不匹配：期望 %d，实际 %d', ...
                               dataset.metadata.num_scenarios, length(dataset.scenarios));
    end
    
    % 逐场景检查
    for scenario_idx = 1:length(dataset.scenarios)
        scenario = dataset.scenarios{scenario_idx};
        
        % 检查ID一致性
        if scenario.scenario_id ~= scenario_idx - 1
            issues{end+1} = sprintf('场景 %d ID不匹配', scenario_idx);
        end
        
        % 检查轨迹点数
        trajectory = scenario.target_info.trajectory;
        measurements = scenario.measurements;
        
        if length(trajectory) ~= length(measurements)
            issues{end+1} = sprintf('场景 %d: 轨迹点数(%d) != 测量点数(%d)', ...
                                   scenario_idx, length(trajectory), length(measurements));
        end
        
        % 检查物理约束
        target_params = get_target_params_v4(dataset.config, scenario.target_type);
        
        for j = 1:length(trajectory)
            v = trajectory{j}.velocity;
            a = trajectory{j}.acceleration;
            
            v_norm = norm(v);
            a_norm = norm(a);
            
            % 速度超限警告
            if v_norm > target_params.max_velocity * 1.05
                warnings_list{end+1} = sprintf('场景 %d, t=%.2fs: 速度超限 (%.2f > %.2f m/s)', ...
                                         scenario_idx, trajectory{j}.time, ...
                                         v_norm, target_params.max_velocity);
            end
            
            % 加速度超限警告
            if a_norm > target_params.max_acceleration * 1.1
                warnings_list{end+1} = sprintf('场景 %d, t=%.2fs: 加速度超限 (%.2f > %.2f m/s²)', ...
                                         scenario_idx, trajectory{j}.time, ...
                                         a_norm, target_params.max_acceleration);
            end
        end
        
        % 检查GDOP
        for j = 1:length(measurements)
            gdop = measurements{j}.crlb.gdop;
            if gdop > dataset.config.gdop_threshold * 2
                warnings_list{end+1} = sprintf('场景 %d, t=%.2fs: GDOP过大 (%.2f m)', ...
                                         scenario_idx, measurements{j}.time, gdop);
            end
        end
        
        % 检查JSON文件存在性
        json_file = fullfile(dataset_dir, sprintf('scenario_%04d.json', scenario_idx - 1));
        if ~exist(json_file, 'file')
            mat_file = fullfile(dataset_dir, sprintf('scenario_%04d.mat', scenario_idx - 1));
            if ~exist(mat_file, 'file')
                issues{end+1} = sprintf('场景 %d: JSON/MAT文件缺失', scenario_idx);
            end
        end
    end
    
    % 报告结果
    fprintf('\n--- 验证结果 ---\n');
    
    if isempty(issues)
        fprintf('✓ 无严重问题\n');
    else
        fprintf('✗ 发现 %d 个严重问题:\n', length(issues));
        for i = 1:min(10, length(issues))
            fprintf('  %d. %s\n', i, issues{i});
        end
        if length(issues) > 10
            fprintf('  ... 还有 %d 个问题\n', length(issues) - 10);
        end
    end
    
    if ~isempty(warnings_list)
        fprintf('\n⚠ 发现 %d 个警告:\n', length(warnings_list));
        for i = 1:min(10, length(warnings_list))
            fprintf('  %d. %s\n', i, warnings_list{i});
        end
        if length(warnings_list) > 10
            fprintf('  ... 还有 %d 个警告\n', length(warnings_list) - 10);
        end
    else
        fprintf('✓ 无警告\n');
    end
    
    fprintf('\n数据集验证完成。\n');
end

%% ================ 导出Python格式 ================

function export_to_python_format_v4(dataset_dir, output_dir)
    % 将数据集导出为Python友好的格式
    
    if nargin < 2
        output_dir = fullfile(dataset_dir, 'python_export');
    end
    
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    
    if ~exist(dataset_file, 'file')
        fprintf('数据集不存在: %s\n', dataset_file);
        return;
    end
    
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    load(dataset_file, 'dataset');
    
    fprintf('导出数据集到Python格式...\n');
    fprintf('输出目录: %s\n', output_dir);
    
    % 导出配置
    config_file = fullfile(output_dir, 'config.json');
    save_json_file_v4(dataset.config, config_file);
    fprintf('配置已导出: %s\n', config_file);
    
    % 导出元数据
    metadata_file = fullfile(output_dir, 'metadata.json');
    save_json_file_v4(dataset.metadata, metadata_file);
    fprintf('元数据已导出: %s\n', metadata_file);
    
    % 逐场景导出
    for scenario_idx = 1:length(dataset.scenarios)
        scenario = dataset.scenarios{scenario_idx};
        
        % 提取轨迹数据
        trajectory = scenario.target_info.trajectory;
        num_steps = length(trajectory);
        
        times = zeros(num_steps, 1);
        states = zeros(num_steps, 9);
        models = cell(num_steps, 1);
        noise_intensities = zeros(num_steps, 1);
        
        for j = 1:num_steps
            times(j) = trajectory{j}.time;
            states(j, :) = trajectory{j}.full_state;
            models{j} = trajectory{j}.motion_model;
            noise_intensities(j) = trajectory{j}.noise_intensity;
        end
        
        % 提取测量数据
        measurements_data = cell(num_steps, 1);
        measurement_stds = cell(num_steps, 1);
        snrs = zeros(num_steps, 1);
        gdops = zeros(num_steps, 1);
        
        for j = 1:num_steps
            meas_point = scenario.measurements{j};
            
            num_rx = length(meas_point.measurements);
            meas_matrix = zeros(num_rx, 3);
            std_matrix = zeros(num_rx, 2);
            
            for k = 1:num_rx
                meas_matrix(k, :) = meas_point.measurements{k};
                std_matrix(k, 1) = meas_point.noise_info{k}.range_noise_std;
                std_matrix(k, 2) = meas_point.noise_info{k}.angle_noise_std;
            end
            
            measurements_data{j} = meas_matrix;
            measurement_stds{j} = std_matrix;
            
            if isfield(meas_point.noise_info{1}, 'snr_db')
                snrs(j) = meas_point.noise_info{1}.snr_db;
            else
                snrs(j) = NaN;
            end
            
            gdops(j) = meas_point.crlb.gdop;
        end
        
        % 保存为.mat文件
        scenario_data = struct();
        scenario_data.scenario_id = scenario.scenario_id;
        scenario_data.target_type = scenario.target_type;
        scenario_data.transmitter_position = scenario.transmitter_position;
        scenario_data.receivers_positions = cell2mat(scenario.receivers_positions');
        scenario_data.times = times;
        scenario_data.states = states;
        scenario_data.motion_models = models;
        scenario_data.noise_intensities = noise_intensities;
        scenario_data.measurements = measurements_data;
        scenario_data.measurement_stds = measurement_stds;
        scenario_data.snrs = snrs;
        scenario_data.gdops = gdops;
        
        output_file = fullfile(output_dir, sprintf('scenario_%04d.mat', scenario.scenario_id));
        save(output_file, '-struct', 'scenario_data', '-v7');
        
        if mod(scenario_idx, 10) == 0 || scenario_idx == 1
            fprintf('  已导出 %d/%d 个场景\n', scenario_idx, length(dataset.scenarios));
        end
    end
    
    % 创建Python加载脚本
    create_python_loader_script_v4(output_dir);
    
    fprintf('\n导出完成！\n');
    fprintf('在Python中使用:\n');
    fprintf('  from load_dataset import load_scenario\n');
    fprintf('  scenario = load_scenario(''%s'', 0)\n', output_dir);
end

function create_python_loader_script_v4(output_dir)
    % 创建Python数据加载脚本
    
    python_script = ['import numpy as np\n', ...
                    'import scipy.io as sio\n', ...
                    'import json\n', ...
                    'import os\n\n', ...
                    'def load_scenario(dataset_dir, scenario_id):\n', ...
                    '    """\n', ...
                    '    加载单个场景数据\n', ...
                    '    \n', ...
                    '    参数:\n', ...
                    '        dataset_dir: 数据集目录路径\n', ...
                    '        scenario_id: 场景ID (0开始)\n', ...
                    '    \n', ...
                    '    返回:\n', ...
                    '        包含场景数据的字典\n', ...
                    '    """\n', ...
                    '    filename = os.path.join(dataset_dir, f''scenario_{scenario_id:04d}.mat'')\n', ...
                    '    data = sio.loadmat(filename, squeeze_me=True)\n', ...
                    '    return data\n\n', ...
                    'def load_config(dataset_dir):\n', ...
                    '    """加载配置"""\n', ...
                    '    with open(os.path.join(dataset_dir, ''config.json''), ''r'') as f:\n', ...
                    '        config = json.load(f)\n', ...
                    '    return config\n\n', ...
                    'def load_metadata(dataset_dir):\n', ...
                    '    """加载元数据"""\n', ...
                    '    with open(os.path.join(dataset_dir, ''metadata.json''), ''r'') as f:\n', ...
                    '        metadata = json.load(f)\n', ...
                    '    return metadata\n\n', ...
                    '# 使用示例:\n', ...
                    '# scenario = load_scenario(''./python_export'', 0)\n', ...
                    '# print(scenario[''states''].shape)  # (1000, 9)\n', ...
                    '# print(scenario[''target_type''])   # ''slow'', ''medium'', or ''fast''\n'];
    
    script_file = fullfile(output_dir, 'load_dataset.py');
    fid = fopen(script_file, 'w');
    fprintf(fid, '%s', python_script);
    fclose(fid);
    
    fprintf('Python加载脚本已创建: %s\n', script_file);
end

%% =====================================================================
%  脚本结束
%  =====================================================================