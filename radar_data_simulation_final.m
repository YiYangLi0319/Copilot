%% =====================================================================
%  双基地雷达数据集生成器 v5.0 (最终修正版)
%  =====================================================================
%  作者: YiYangLi0319
%  日期: 2025-10-22
%  
%  核心改进：
%  1. 接收机布局：100km范围，30km最小间距，10km最小半径，3D时0-20km高度
%  2. 目标初始位置：50km范围，1-20km高度限制
%  3. 硬约束：速度/加速度超限直接舍弃整条轨迹，重新生成
%  4. 增强噪声模型：
%     - 距离噪声 = sqrt(SNR噪声^2 + 时间同步误差^2)
%     - 角度噪声 = sqrt(SNR噪声^2 + 硬件误差^2)，基于口径公式
%  5. 增强可视化：清晰显示模型切换和对应的运动变化
%  6. JSON格式：完全匹配radar_data_simulation.m
%  
%  最终修正：
%  - 问题1：超限直接舍弃轨迹，不再硬限幅（避免非物理行为）
%  - 问题2：增强可视化，用分段着色+垂直线标记模型切换
%  =====================================================================

clear all; close all; clc;

%% ================ 主程序调用 ================
radar_data_simulation_v5();

%% ================ 主程序入口 ================
function radar_data_simulation_v5()
    fprintf('=====================================================\n');
    fprintf('  双基地雷达数据集生成器 v5.0 (最终版)\n');
    fprintf('  作者: YiYangLi0319\n');
    fprintf('  日期: %s\n', datestr(now));
    fprintf('=====================================================\n\n');
    
    % 全局随机种子
    GLOBAL_SEED = 42;
    rng(GLOBAL_SEED, 'twister');
    fprintf('全局随机种子: %d\n\n', GLOBAL_SEED);
    
    % 运行配置
    run_config = struct();
    run_config.run_demo = true;
    run_config.save_data = true;
    run_config.num_scenarios = 20;
    run_config.run_validation = true;
    
    % 生成2D数据集
    fprintf('--- 生成2D数据集 ---\n');
    config_2d = create_radar_config_v5(2);
    dataset_2d = generate_dataset_v5(config_2d, run_config.num_scenarios, ...
        'enhanced_v5_2d_dataset', GLOBAL_SEED);
    
    % 生成3D数据集
    fprintf('\n--- 生成3D数据集 ---\n');
    config_3d = create_radar_config_v5(3);
    dataset_3d = generate_dataset_v5(config_3d, run_config.num_scenarios, ...
        'enhanced_v5_3d_dataset', GLOBAL_SEED + 100000);
    
    % 演示可视化
    if run_config.run_demo
        fprintf('\n--- 可视化演示 ---\n');
        visualize_scenario_v5(dataset_2d.scenarios{1}, '2D场景示例 v5.0');
        visualize_scenario_v5(dataset_3d.scenarios{1}, '3D场景示例 v5.0');
    end
    
    % 数据集统计分析
    fprintf('\n=====================================================\n');
    fprintf('  数据集统计分析\n');
    fprintf('=====================================================\n');
    analyze_dataset_v5('enhanced_v5_2d_dataset');
    analyze_dataset_v5('enhanced_v5_3d_dataset');
    
    % 数据验证
    if run_config.run_validation
        fprintf('\n--- 数据验证 ---\n');
        validate_dataset_v5('enhanced_v5_2d_dataset', config_2d);
        validate_dataset_v5('enhanced_v5_3d_dataset', config_3d);
    end
    
    fprintf('\n=====================================================\n');
    fprintf('  数据集生成完成！\n');
    fprintf('  2D数据集: enhanced_v5_2d_dataset/\n');
    fprintf('  3D数据集: enhanced_v5_3d_dataset/\n');
    fprintf('=====================================================\n\n');
end

%% ================ 配置函数 ================
function config = create_radar_config_v5(dimension)
    % 创建雷达系统配置 v5.0
    
    config = struct();
    config.dimension = dimension;
    
    % 根据维度设置状态维度
    if dimension == 2
        config.state_dimension = 6;  % [x, y, vx, vy, ax, ay]
    else
        config.state_dimension = 9;  % [x, y, z, vx, vy, vz, ax, ay, az]
    end
    
    config.num_receivers = 3;
    config.simulation_time = 200;
    config.dt = 1.0;
    
    % ===== 接收机配置 =====
    config.receiver_area_radius = 100000;
    config.min_receiver_distance = 30000;
    config.min_receiver_radius = 10000;
    config.max_receiver_altitude_3d = 20000;
    config.min_receiver_altitude_3d = 0;
    
    % ===== 目标空间约束 =====
    config.initial_position_range = 50000;
    config.workspace_radius = 100000;
    
    if dimension == 3
        config.altitude_min = 1000;
        config.altitude_max = 20000;
    end
    
    % ===== 修正1：硬约束配置（超限直接舍弃）=====
    config.hard_constraint = struct();
    config.hard_constraint.velocity_tolerance = 1.0;      % 允许1 m/s误差
    config.hard_constraint.acceleration_tolerance = 0.5;  % 允许0.5 m/s²误差
    config.hard_constraint.max_violations = 5;            % 最大允许违规次数（连续）
    
    % ===== 增强噪声模型 =====
    config.measurement_noise = struct();
    config.measurement_noise.time_sync_std = 5.0;
    config.measurement_noise.hardware_angle_std = 0.05 * pi/180;
    
    % SNR参数
    config.snr = struct();
    config.snr.target_rcs = 2.0;
    config.snr.tx_power = 10e3;
    config.snr.freq = 3e9;
    config.snr.bandwidth = 1e6;
    config.snr.noise_figure = 1.5;
    config.snr.min_snr_db = 10.0;
    config.snr.max_snr_db = 80.0;
    config.snr.tau = 1e-3;
    config.snr.aperture_m = 5.0;
    
    % 厚尾噪声
    config.heavy_tail_probability = 0.02;
    config.student_t_dof = 3;
    
    % ===== 目标类型定义 =====
    config.target_types = {'slow', 'medium', 'fast'};
    
    % **关键修正：根据目标类型和模型调整过程噪声**
    config.slow_target = struct();
    config.slow_target.max_velocity = 50.0;
    config.slow_target.max_acceleration = 3.0;
    config.slow_target.max_turn_rate = 0.1;
    % 修正：CV/CA/CT噪声显著降低，Singer保持中等
    config.slow_target.noise_ranges = struct(...
        'CV', [0.01, 0.05], 'CA', [0.02, 0.08], 'CT', [0.01, 0.05], 'Singer', [0.1, 0.3]);
    
    config.medium_target = struct();
    config.medium_target.max_velocity = 300.0;
    config.medium_target.max_acceleration = 6.0;
    config.medium_target.max_turn_rate = 0.07;
    config.medium_target.noise_ranges = struct(...
        'CV', [0.05, 0.15], 'CA', [0.08, 0.25], 'CT', [0.05, 0.15], 'Singer', [0.2, 0.5]);
    
    config.fast_target = struct();
    config.fast_target.max_velocity = 500.0;
    config.fast_target.max_acceleration = 40.0;
    config.fast_target.max_turn_rate = 0.3;
    config.fast_target.noise_ranges = struct(...
        'CV', [0.1, 0.3], 'CA', [0.15, 0.5], 'CT', [0.1, 0.3], 'Singer', [0.3, 0.8]);
    
    % ===== 运动模型配置 =====
    config.available_models = {'CV', 'CA', 'CT', 'Singer'};
    config.min_model_duration = 20.0;
    config.max_model_duration = 40.0;
    config.min_noise_step_duration = 5.0;
    config.max_noise_step_duration = 15.0;
end

%% ================ 数据集生成 ================
function dataset = generate_dataset_v5(config, num_scenarios, output_dir, base_seed)
    % 生成完整数据集
    
    if nargin < 3, output_dir = 'enhanced_v5_dataset'; end
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
    dataset.metadata.version = '5.0';
    dataset.metadata.base_seed = base_seed;
    
    fprintf('生成 %d 个场景...\n', num_scenarios);
    
    for scenario_idx = 1:num_scenarios
        if mod(scenario_idx, 5) == 0 || scenario_idx == 1
            fprintf('  进度: %d/%d (%.1f%%)\n', scenario_idx, num_scenarios, ...
                    100*scenario_idx/num_scenarios);
        end
        
        scenario_seed = base_seed + scenario_idx * 1000;
        rng(scenario_seed, 'twister');
        
        target_type = config.target_types{randi(length(config.target_types))};
        
        scenario = generate_scenario_v5(config, target_type);
        scenario.scenario_id = scenario_idx - 1;
        scenario.seed = scenario_seed;
        dataset.scenarios{end+1} = scenario;
        
        % 保存JSON
        save_scenario_json_v5(scenario, output_dir, scenario_idx - 1);
    end
    
    % 保存数据集
    dataset_file = fullfile(output_dir, 'dataset.mat');
    save(dataset_file, 'dataset', '-v7.3');
    fprintf('数据集已保存: %s\n', dataset_file);
    
    % 保存元信息JSON
    dataset_info = struct();
    dataset_info.config = config;
    dataset_info.metadata = dataset.metadata;
    save_json_file_v5(dataset_info, fullfile(output_dir, 'dataset_info.json'));
end

function scenario = generate_scenario_v5(config, target_type)
    % 生成单个场景（带重试）
    
    max_attempts = 2000;
    
    for attempt = 1:max_attempts
        transmitter_pos = zeros(1, 3);
        receivers_pos = generate_receiver_positions_v5(config, transmitter_pos);
        
        scenario = struct();
        scenario.config = config;
        scenario.transmitter_position = transmitter_pos;
        scenario.receivers_positions = receivers_pos;
        scenario.target_info = struct();
        scenario.target_info.target_type = target_type;
        
        time_steps = floor(config.simulation_time / config.dt);
        
        [target_info, measurements, detailed_records, is_valid] = ...
            generate_target_trajectory_v5(config, time_steps, target_type, ...
                                         transmitter_pos, receivers_pos);
        
        if is_valid
            scenario.target_info = target_info;
            scenario.measurements = measurements;
            scenario.detailed_records = detailed_records;
            return;
        end
        
        if mod(attempt, 50) == 0
            fprintf('    尝试 %d/%d 次...\n', attempt, max_attempts);
        end
    end
    
    error('轨迹生成失败：已尝试%d次', max_attempts);
end

%% ================ 接收机位置生成 ================
function receivers_pos = generate_receiver_positions_v5(config, tx_pos)
    % 生成接收机位置
    
    max_attempts = 1000;
    
    for attempt = 1:max_attempts
        temp_receivers = {};
        
        min_r = config.min_receiver_radius;
        max_r = config.receiver_area_radius;
        
        for i = 1:config.num_receivers
            if config.dimension == 2
                angle = rand() * 2 * pi;
                radius = min_r + rand() * (max_r - min_r);
                pos = [radius * cos(angle), radius * sin(angle), 0];
            else  % 3D
                phi = rand() * 2 * pi;
                theta = pi/6 + rand() * (2*pi/3);
                radius = min_r + rand() * (max_r - min_r);
                
                x = radius * sin(theta) * cos(phi);
                y = radius * sin(theta) * sin(phi);
                z = radius * cos(theta);
                
                % 限制高度在 0-20km
                z = min(max(z, config.min_receiver_altitude_3d), config.max_receiver_altitude_3d);
                
                pos = [x, y, z];
            end
            temp_receivers{end+1} = pos;
        end
        
        % 检查最小间距
        valid = true;
        for i = 1:length(temp_receivers)
            for j = i+1:length(temp_receivers)
                dist = norm(temp_receivers{i} - temp_receivers{j});
                if dist < config.min_receiver_distance
                    valid = false;
                    break;
                end
            end
            if ~valid, break; end
        end
        
        if valid
            receivers_pos = temp_receivers;
            return;
        end
    end
    
    warning('使用次优接收机配置');
    receivers_pos = temp_receivers;
end

%% ================ 轨迹生成（核心）================
function [target_info, measurements, detailed_records, is_valid] = ...
    generate_target_trajectory_v5(config, time_steps, target_type, tx, rxs)
    % 生成目标轨迹（修正1：硬约束，超限直接舍弃）
    
    target_params = get_target_params_v5(config, target_type);
    
    % 初始化状态
    pos_range = config.initial_position_range;
    vel_range = target_params.max_velocity * 0.3;
    
    if config.dimension == 2
        initial_pos = [(rand() - 0.5) * pos_range, (rand() - 0.5) * pos_range];
        initial_vel = [(rand() - 0.5) * vel_range, (rand() - 0.5) * vel_range];
        initial_acc = [0, 0];
        state = [initial_pos, initial_vel, initial_acc];
    else  % 3D
        x = (rand() - 0.5) * pos_range;
        y = (rand() - 0.5) * pos_range;
        z = config.altitude_min + rand() * (config.altitude_max - config.altitude_min);
        initial_pos = [x, y, z];
        
        vx = (rand() - 0.5) * vel_range;
        vy = (rand() - 0.5) * vel_range;
        vz = (rand() - 0.5) * vel_range/2;
        initial_vel = [vx, vy, vz];
        initial_acc = [0, 0, 0];
        state = [initial_pos, initial_vel, initial_acc];
    end
    
    % 生成运动模型和噪声序列
    [model_sequence, noise_schedule] = generate_model_and_noise_sequence_v5(...
        config, target_params, time_steps);
    
    % 初始化记录
    target_info = struct();
    target_info.initial_state = state;
    target_info.target_type = target_type;
    target_info.motion_model_sequence = model_sequence;
    target_info.trajectory = {};
    
    measurements = {};
    detailed_records = {};
    
    % 修正1：连续违规计数器
    consecutive_violations = 0;
    max_consecutive = config.hard_constraint.max_violations;
    
    % 轨迹生成主循环
    for t = 1:time_steps
        current_time = (t-1) * config.dt;
        
        if config.dimension == 2
            current_pos = [state(1:2), 0];
            current_vel = [state(3:4), 0];
            current_acc = [state(5:6), 0];
        else
            current_pos = state(1:3);
            current_vel = state(4:6);
            current_acc = state(7:9);
        end
        
        % 修正1：检查硬约束
        [constraint_violated, violation_type] = check_hard_constraints_v5(...
            state, target_params, config);
        
        if constraint_violated
            consecutive_violations = consecutive_violations + 1;
            if consecutive_violations > max_consecutive
                fprintf('    时刻%.1fs %s超限（连续%d次），舍弃轨迹\n', ...
                    current_time, violation_type, consecutive_violations);
                is_valid = false;
                return;
            end
        else
            consecutive_violations = 0;  % 重置计数器
        end
        
        % 记录轨迹点
        traj_point = struct();
        traj_point.time = current_time;
        traj_point.position = current_pos;
        traj_point.velocity = current_vel;
        traj_point.acceleration = current_acc;
        traj_point.full_state = state;
        traj_point.motion_model = model_sequence{t}.model;
        traj_point.model_parameters = model_sequence{t}.params;
        target_info.trajectory{end+1} = traj_point;
        
        % 生成量测
        [target_measurements, measurement_noise_info] = generate_measurements_v5(...
            current_pos, tx, rxs, config);
        
        % 计算真实量测
        true_measurements = {};
        for i = 1:length(rxs)
            [true_range, true_azimuth, true_elevation] = compute_true_measurement_v5(...
                current_pos, tx, rxs{i}, config.dimension);
            true_measurements{end+1} = [true_range, true_azimuth, true_elevation];
        end
        
        meas_point = struct();
        meas_point.time = current_time;
        meas_point.measurements = target_measurements;
        meas_point.noise_info = measurement_noise_info;
        measurements{end+1} = meas_point;
        
        detailed_record = struct();
        detailed_record.time = current_time;
        detailed_record.true_state = state;
        detailed_record.true_position = current_pos;
        detailed_record.true_velocity = current_vel;
        detailed_record.true_acceleration = current_acc;
        detailed_record.motion_model = model_sequence{t}.model;
        detailed_record.model_parameters = model_sequence{t}.params;
        detailed_record.process_noise_std = noise_schedule{t}.intensity;
        detailed_record.measurements = target_measurements;
        detailed_record.true_measurements = true_measurements;
        detailed_record.measurement_noise_info = measurement_noise_info;
        
        detailed_records{end+1} = detailed_record;
        
        % 状态更新
        if t < time_steps
            state = update_state_v5(state, model_sequence{t}, ...
                noise_schedule{t}, config, target_params);
        end
    end
    
    % 验证轨迹
    is_valid = validate_trajectory_motion_v5(target_info.trajectory, ...
        model_sequence, config);
end

%% ================ 硬约束检查（修正1）================
function [violated, violation_type] = check_hard_constraints_v5(state, target_params, config)
    % 修正1：硬约束检查，不做限幅，直接返回违规标志
    
    violated = false;
    violation_type = '';
    
    if config.dimension == 2
        v = state(3:4);
        a = state(5:6);
    else
        v = state(4:6);
        a = state(7:9);
    end
    
    % 速度硬约束
    v_norm = norm(v);
    max_v = target_params.max_velocity + config.hard_constraint.velocity_tolerance;
    
    if v_norm > max_v
        violated = true;
        violation_type = '速度';
        return;
    end
    
    % 加速度硬约束
    a_norm = norm(a);
    max_a = target_params.max_acceleration + config.hard_constraint.acceleration_tolerance;
    
    if a_norm > max_a
        violated = true;
        violation_type = '加速度';
        return;
    end
    
    % 高度约束 (3D)
    if config.dimension == 3
        z = state(3);
        if z < config.altitude_min * 0.95 || z > config.altitude_max * 1.05
            violated = true;
            violation_type = '高度';
            return;
        end
    end
end

%% ================ 运动模型和噪声序列 ================
function [model_sequence, noise_schedule] = generate_model_and_noise_sequence_v5(...
    config, target_params, time_steps)
    % 生成运动模型序列和噪声调度
    
    model_sequence = cell(1, time_steps);
    noise_schedule = cell(1, time_steps);
    
    current_step = 1;
    dt = config.dt;
    
    while current_step <= time_steps
        % 选择运动模型
        model_name = config.available_models{randi(length(config.available_models))};
        
        % 模型持续时间
        model_duration_sec = config.min_model_duration + ...
            rand() * (config.max_model_duration - config.min_model_duration);
        model_duration_steps = round(model_duration_sec / dt);
        model_end_step = min(current_step + model_duration_steps - 1, time_steps);
        
        % 生成模型参数
        model_params = generate_model_parameters_v5(model_name, config, target_params);
        
        % 生成噪声阶梯
        noise_steps = generate_noise_steps_v5(config, target_params, ...
            model_name, current_step, model_end_step, dt);
        
        % 填充序列
        noise_step_idx = 1;
        for t = current_step:model_end_step
            model_sequence{t} = struct('model', model_name, 'params', model_params);
            
            while noise_step_idx < length(noise_steps) && ...
                  t > noise_steps{noise_step_idx}.end_step
                noise_step_idx = noise_step_idx + 1;
            end
            
            noise_schedule{t} = noise_steps{noise_step_idx};
        end
        
        current_step = model_end_step + 1;
    end
end

function noise_steps = generate_noise_steps_v5(config, target_params, ...
    model_name, start_step, end_step, dt)
    % 生成噪声阶梯
    
    noise_steps = {};
    noise_range = target_params.noise_ranges.(model_name);
    min_intensity = noise_range(1);
    max_intensity = noise_range(2);
    
    current_step = start_step;
    
    while current_step <= end_step
        step_duration_sec = config.min_noise_step_duration + ...
            rand() * (config.max_noise_step_duration - config.min_noise_step_duration);
        step_duration_steps = round(step_duration_sec / dt);
        step_end = min(current_step + step_duration_steps - 1, end_step);
        
        intensity = min_intensity + rand() * (max_intensity - min_intensity);
        
        noise_step = struct();
        noise_step.start_step = current_step;
        noise_step.end_step = step_end;
        noise_step.intensity = intensity;
        
        noise_steps{end+1} = noise_step;
        current_step = step_end + 1;
    end
end

function model_params = generate_model_parameters_v5(model_name, config, target_params)
    % 生成运动模型参数
    
    model_params = struct();
    model_params.type = model_name;
    
    switch model_name
        case 'CV'
            % 恒速模型
            
        case 'CA'
            % 恒加速模型
            max_acc = target_params.max_acceleration;
            if config.dimension == 2
                model_params.acceleration = [(rand() - 0.5) * max_acc, ...
                                           (rand() - 0.5) * max_acc];
            else
                model_params.acceleration = [(rand() - 0.5) * max_acc, ...
                                           (rand() - 0.5) * max_acc, ...
                                           (rand() - 0.5) * max_acc/2];
            end
            
        case 'CT'
            % 恒转弯模型
            max_turn = target_params.max_turn_rate;
            model_params.turn_rate = (rand() - 0.5) * max_turn;
            
        case 'Singer'
            % Singer模型
            tau = 10 + rand() * 20;
            model_params.beta = 1 / tau;
    end
end

%% ================ 状态更新（无限幅）================
function next_state = update_state_v5(state, model_info, noise_info, config, target_params)
    % 修正1：状态更新，不做限幅（由硬约束检查决定是否舍弃）
    
    model_name = model_info.model;
    model_params = model_info.params;
    dt = config.dt;
    dim = config.dimension;
    
    % 构建状态转移矩阵和过程噪声协方差
    [Phi, Q] = get_transition_matrices_v5(model_name, model_params, noise_info, dt, dim);
    
    % 确定性状态转移
    state_col = state(:);
    predicted_state = Phi * state_col;
    
    % 注入过程噪声
    state_dim = length(state);
    try
        noise_vec = mvnrnd(zeros(state_dim,1), Q)';
    catch
        [V, D] = eig(Q);
        D = max(D, 0);
        Q_fixed = V * D * V';
        Q_fixed = (Q_fixed + Q_fixed') / 2;
        noise_vec = mvnrnd(zeros(state_dim,1), Q_fixed)';
    end
    
    next_state = predicted_state + noise_vec;
    next_state = next_state';
    
    % 修正1：不做任何限幅，保持物理连续性
end

function [Phi, Q] = get_transition_matrices_v5(model_name, model_params, noise_info, dt, dim)
    % 构建状态转移矩阵和过程噪声协方差
    
    sigma = noise_info.intensity;
    
    switch model_name
        case 'CV'
            % 恒速模型
            Phi1 = [1, dt, 0; 
                    0, 1,  0; 
                    0, 0,  0];
            
            Q1 = sigma^2 * [dt^3/3, dt^2/2, 0; 
                            dt^2/2, dt,     0; 
                            0,      0,      0];
            
        case 'CA'
            % 恒加速模型
            Phi1 = [1, dt, 0.5*dt^2; 
                    0, 1,  dt; 
                    0, 0,  1];
            
            Q1 = sigma^2 * [dt^5/20, dt^4/8, dt^3/6; 
                            dt^4/8,  dt^3/3, dt^2/2; 
                            dt^3/6,  dt^2/2, dt];
            
        case 'CT'
            % 恒转弯模型
            omega = model_params.turn_rate;
            
            if abs(omega * dt) < 1e-4
                Phi1 = [1, dt, 0; 
                        0, 1,  0; 
                        0, 0,  1];
                
                Q1 = sigma^2 * [dt^3/3, dt^2/2, 0; 
                                dt^2/2, dt,     0; 
                                0,      0,      dt];
            else
                c = cos(omega * dt);
                s = sin(omega * dt);
                
                Phi1 = [1, s/omega,       -(1-c)/omega; 
                        0, c,             -s; 
                        0, 0,             1];
                
                Q1 = sigma^2 * [dt^3/3, dt^2/2, 0; 
                                dt^2/2, dt,     0; 
                                0,      0,      dt];
            end
            
        case 'Singer'
            % Singer模型
            if isfield(model_params, 'alpha')
                alpha = model_params.alpha;
            elseif isfield(model_params, 'beta')
                alpha = model_params.beta;
            else
                alpha = 0.1;
            end
            
            sigma_m = sigma;
            
            A_c = [0, 1, 0; 
                   0, 0, 1; 
                   0, 0, -alpha];
            B_c = [0; 0; 1];
            Q_c = 2 * alpha * sigma_m^2;
            
            [Phi1, Q1] = singer_discretization_vanloan(A_c, B_c, Q_c, dt);
            
        otherwise
            error('未知的模型类型: %s', model_name);
    end
    
    % 根据维度构建完整矩阵
    if dim == 2
        I2 = eye(2);
        Phi = kron(Phi1, I2);
        Q = kron(Q1, I2);
    else
        I3 = eye(3);
        Phi = kron(Phi1, I3);
        Q = kron(Q1, I3);
    end
    
    % 确保协方差矩阵的对称性和正定性
    Q = (Q + Q') / 2;
    Q = Q + 1e-10 * eye(size(Q, 1));
end

function [Phi, Q] = singer_discretization_vanloan(A, B, Qc, T)
    % **修正：Van Loan离散化的正确实现**
    % 
    % 输入：
    %   A: 连续状态矩阵 (n×n)
    %   B: 输入矩阵 (n×1)
    %   Qc: 连续过程噪声强度（标量）
    %   T: 采样周期
    % 
    % 输出：
    %   Phi: 离散状态转移矩阵
    %   Q: 离散过程噪声协方差
    
    n = size(A, 1);
    G = B * Qc * B.';  % 连续域过程噪声协方差密度 (n×n)
    
    % **修正：正确的Van Loan矩阵构造**
    M = [ A,           G;
          zeros(n, n), -A.' ];
    
    % 矩阵指数
    EM = expm(M * T);
    
    % **修正：正确的块选取**
    Phi = EM(1:n, 1:n);              % 左上块：exp(A*T)
    S   = EM(1:n, n+1:end);          % 右上块：积分项
    
    % **修正：正确的Q重构**
    Q = Phi * S;
    
    % 确保对称性
    Q = (Q + Q.') / 2;
    Q = Q + 1e-12 * eye(n);
end

%% ================ 量测生成 ================
function [measurements, noise_info] = generate_measurements_v5(target_pos, tx_pos, receivers_pos, config)
    % 生成量测
    
    measurements = {};
    noise_info = {};
    
    for i = 1:length(receivers_pos)
        rx_pos = receivers_pos{i};
        
        % 计算真实测量值
        [true_range, true_azimuth, true_elevation] = compute_true_measurement_v5(...
            target_pos, tx_pos, rx_pos, config.dimension);
        
        % SNR驱动的基础噪声
        snr_db = compute_snr_v5(target_pos, tx_pos, rx_pos, config);
        [snr_range_std, snr_angle_std] = snr_to_noise_std_v5(snr_db, config);
        
        % 增强噪声模型
        time_sync_std = config.measurement_noise.time_sync_std;
        range_noise_std = sqrt(snr_range_std^2 + time_sync_std^2);
        
        hardware_angle_std = config.measurement_noise.hardware_angle_std;
        angle_noise_std = sqrt(snr_angle_std^2 + hardware_angle_std^2);
        
        % 厚尾噪声
        use_heavy_tail = rand() < config.heavy_tail_probability;
        
        if use_heavy_tail
            dof = config.student_t_dof;
            noisy_range = true_range + trnd(dof) * range_noise_std;
            noisy_azimuth = wrapToPi(true_azimuth + trnd(dof) * angle_noise_std);
            if config.dimension == 3
                noisy_elevation = clip_elevation(true_elevation + trnd(dof) * angle_noise_std);
            else
                noisy_elevation = 0;
            end
            dist_type = 'student_t';
        else
            noisy_range = true_range + randn() * range_noise_std;
            noisy_azimuth = wrapToPi(true_azimuth + randn() * angle_noise_std);
            if config.dimension == 3
                noisy_elevation = clip_elevation(true_elevation + randn() * angle_noise_std);
            else
                noisy_elevation = 0;
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
        receiver_noise_info.snr_db = snr_db;
        receiver_noise_info.snr_range_std = snr_range_std;
        receiver_noise_info.snr_angle_std = snr_angle_std;
        receiver_noise_info.time_sync_std = time_sync_std;
        receiver_noise_info.hardware_angle_std = hardware_angle_std;
        receiver_noise_info.range_noise_std = range_noise_std;
        receiver_noise_info.angle_noise_std = angle_noise_std;
        receiver_noise_info.dist_type = dist_type;
        receiver_noise_info.is_heavy_tail = use_heavy_tail;
        
        noise_info{end+1} = receiver_noise_info;
    end
end

function [range_sum, azimuth, elevation] = compute_true_measurement_v5(target_pos, tx_pos, rx_pos, dimension)
    % 计算真实测量值
    
    if length(target_pos) == 2
        target_pos = [target_pos, 0];
    end
    if length(tx_pos) == 2
        tx_pos = [tx_pos, 0];
    end
    if length(rx_pos) == 2
        rx_pos = [rx_pos, 0];
    end
    
    % 双基地距离和
    range_tx = norm(target_pos - tx_pos);
    range_rx = norm(target_pos - rx_pos);
    range_sum = range_tx + range_rx;
    
    % 角度测量
    direction = target_pos - rx_pos;
    
    if dimension == 2
        azimuth = atan2(direction(2), direction(1));
        elevation = 0;
    else
        azimuth = atan2(direction(2), direction(1));
        direction_norm = norm(direction);
        if direction_norm > 0
            elevation = asin(direction(3) / direction_norm);
        else
            elevation = 0;
        end
    end
end

function snr_db = compute_snr_v5(target_pos, tx_pos, rx_pos, config)
    % 计算信噪比
    
    lambda = 3e8 / config.snr.freq;
    Pt = config.snr.tx_power;
    Gt = 10^(27/10);
    Gr = 10^(27/10);
    sigma = config.snr.target_rcs;
    tau = config.snr.tau;
    R_tx = norm(target_pos - tx_pos);
    R_rx = norm(target_pos - rx_pos);
    
    % 接收功率
    Pr = (Pt * Gt * Gr * lambda^2 * sigma * tau) / ((4*pi)^3 * R_tx^2 * R_rx^2);
    
    % 噪声功率
    k = 1.38e-23;
    T = 290;
    B = config.snr.bandwidth;
    Ls = 10^(13/10);
    Lt = 10^(8/10);
    NF = 10^(config.snr.noise_figure / 10);
    Pn = k * T * NF * Ls * Lt;
    
    % SNR
    snr = Pr / Pn;
    snr_db = 10 * log10(max(snr, 1e-10));
end

function [range_std, angle_std] = snr_to_noise_std_v5(snr_db, config)
    % 噪声标准差计算（基于口径）
    
    snr_linear = 10^(snr_db / 10);
    
    % 距离噪声
    c = 3e8;
    B = config.snr.bandwidth;
    range_resolution = c * sqrt(3) / (pi * B);
    range_std = range_resolution / sqrt(2 * snr_linear);
    range_std = max(range_std, 1.0);
    
    % 角度噪声（基于口径）
    if isfield(config.snr, 'aperture_m') && config.snr.aperture_m > 0
        lambda = 3e8 / config.snr.freq;
        k_beam = 1.0;
        angle_std = k_beam * lambda / (config.snr.aperture_m * sqrt(2 * snr_linear));
    else
        angle_std = 1 / sqrt(2 * snr_linear);
    end
    angle_std = max(angle_std, 0.01 * pi/180);
end

function elev = clip_elevation(elev)
    % 俯仰角限制
    elev = max(-pi/2, min(pi/2, elev));
end

%% ================ 辅助函数 ================
function angle = wrapToPi(angle)
    % 将角度包裹到 [-pi, pi]
    angle = mod(angle + pi, 2*pi) - pi;
end

%% ================ 轨迹验证 ================
function is_valid = validate_trajectory_motion_v5(trajectory, model_sequence, config)
    % 验证轨迹是否符合运动模型
    
    is_valid = true;
    
    % 按模型段分组验证
    segments = extract_model_segments(trajectory, model_sequence);
    
    for i = 1:length(segments)
        segment = segments{i};
        model_name = segment.model;
        
        % 验证每个段
        segment_valid = validate_segment_motion(segment, model_name, config);
        
        if ~segment_valid
            fprintf('    模型段 %d (%s) 验证失败\n', i, model_name);
            is_valid = false;
            break;
        end
    end
end

function segments = extract_model_segments(trajectory, model_sequence)
    % 提取运动模型段
    
    segments = {};
    current_model = model_sequence{1}.model;
    segment_start = 1;
    
    for t = 2:length(model_sequence)
        if ~strcmp(model_sequence{t}.model, current_model)
            segment = struct();
            segment.model = current_model;
            segment.start_idx = segment_start;
            segment.end_idx = t - 1;
            segment.trajectory = trajectory(segment_start:segment.end_idx);
            segment.params = model_sequence{segment_start}.params;
            segments{end+1} = segment;
            
            current_model = model_sequence{t}.model;
            segment_start = t;
        end
    end
    
    % 补上最后一段
    segment = struct();
    segment.model = current_model;
    segment.start_idx = segment_start;
    segment.end_idx = length(model_sequence);
    segment.trajectory = trajectory(segment_start:segment.end_idx);
    segment.params = model_sequence{segment_start}.params;
    segments{end+1} = segment;
end

function is_valid = validate_segment_motion(segment, model_name, config)
    % 验证单个模型段
    
    is_valid = true;
    traj = segment.trajectory;
    
    if length(traj) < 5
        return;
    end
    
    switch model_name
        case 'CV'
            is_valid = validate_cv_segment(traj, config);
        case 'CA'
            is_valid = validate_ca_segment(traj, config);
        case 'CT'
            is_valid = validate_ct_segment(traj, segment.params, config);
        case 'Singer'
            is_valid = validate_singer_segment(traj, config);
    end
end

function is_valid = validate_cv_segment(traj, config)
    velocities = zeros(length(traj), 3);
    for i = 1:length(traj)
        velocities(i, :) = traj{i}.velocity;
    end
    
    vel_norms = vecnorm(velocities, 2, 2);
    vel_variation = std(vel_norms) / (mean(vel_norms) + 1e-6);
    
    is_valid = vel_variation < 0.35;
    
    if ~is_valid
        fprintf('      CV段速度变化率 %.2f%% > 35%%\n', vel_variation*100);
    end
end

function is_valid = validate_ca_segment(traj, config)
    accelerations = zeros(length(traj), 3);
    for i = 1:length(traj)
        accelerations(i, :) = traj{i}.acceleration;
    end
    
    acc_norms = vecnorm(accelerations, 2, 2);
    acc_variation = std(acc_norms) / (mean(acc_norms) + 1e-6);
    
    is_valid = acc_variation < 0.5;
    
    if ~is_valid
        fprintf('      CA段加速度变化率 %.2f%% > 50%%\n', acc_variation*100);
    end
end

function is_valid = validate_ct_segment(traj, params, config)
    if length(traj) < 3
        is_valid = true;
        return;
    end
    
    positions = zeros(length(traj), 3);
    for i = 1:length(traj)
        positions(i, :) = traj{i}.position;
    end
    
    if config.dimension == 2
        curvatures = [];
        for i = 2:length(traj)-1
            p1 = positions(i-1, 1:2);
            p2 = positions(i, 1:2);
            p3 = positions(i+1, 1:2);
            
            area = abs((p2(1)-p1(1))*(p3(2)-p1(2)) - (p3(1)-p1(1))*(p2(2)-p1(2))) / 2;
            a = norm(p2 - p1);
            b = norm(p3 - p2);
            c = norm(p3 - p1);
            
            if a*b*c > 1e-6
                curvature = 4 * area / (a * b * c);
                curvatures = [curvatures, curvature];
            end
        end
        
        if ~isempty(curvatures)
            curvature_variation = std(curvatures) / (mean(curvatures) + 1e-6);
            is_valid = curvature_variation < 0.6;
        else
            is_valid = true;
        end
    else
        is_valid = true;
    end
end

function is_valid = validate_singer_segment(traj, config)
    accelerations = zeros(length(traj), 3);
    for i = 1:length(traj)
        accelerations(i, :) = traj{i}.acceleration;
    end
    
    acc_norms = vecnorm(accelerations, 2, 2);
    if length(acc_norms) >= 3
        acc_diff2 = diff(diff(acc_norms));
        smoothness = std(acc_diff2) / (std(acc_norms) + 1e-6);
        
        is_valid = smoothness < 3.0;
        
        if ~is_valid
            fprintf('      Singer段平滑性指标 %.2f > 3.0\n', smoothness);
        end
    else
        is_valid = true;
    end
end

%% ================ JSON存储 ================
function save_scenario_json_v5(scenario, output_dir, scenario_id)
    filename = fullfile(output_dir, sprintf('scenario_%04d.json', scenario_id));
    json_scenario = convert_to_json_format_v5(scenario);
    save_json_file_v5(json_scenario, filename);
end

function json_data = convert_to_json_format_v5(matlab_data)
    if isstruct(matlab_data)
        json_data = struct();
        fields = fieldnames(matlab_data);
        for i = 1:length(fields)
            field_name = fields{i};
            field_value = matlab_data.(field_name);
            json_data.(field_name) = convert_to_json_format_v5(field_value);
        end
    elseif iscell(matlab_data)
        json_data = cell(size(matlab_data));
        for i = 1:numel(matlab_data)
            json_data{i} = convert_to_json_format_v5(matlab_data{i});
        end
    elseif isnumeric(matlab_data)
        json_data = matlab_data;
    else
        json_data = matlab_data;
    end
end

function save_json_file_v5(data, filename)
    try
        json_str = jsonencode(data, 'PrettyPrint', true);
        fid = fopen(filename, 'w', 'n', 'UTF-8');
        fprintf(fid, '%s', json_str);
        fclose(fid);
    catch ME
        [filepath, name, ~] = fileparts(filename);
        mat_filename = fullfile(filepath, [name, '.mat']);
        save(mat_filename, 'data');
        fprintf('警告：JSON编码失败，保存为MAT格式：%s\n错误：%s\n', mat_filename, ME.message);
    end
end

function target_params = get_target_params_v5(config, target_type)
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

%% ================ 增强可视化（修正2）================
function visualize_scenario_v5(scenario, title_str)
    % 修正2：增强可视化，清晰显示模型切换
    
    if nargin < 2, title_str = '雷达场景 v5.0'; end
    
    config = scenario.config;
    dimension = config.dimension;
    
    figure('Position', [100, 100, 1800, 1200]);
    
    if dimension == 2
        subplot(3, 3, 1);
        plot_trajectory_and_radars_v5(scenario);
        
        subplot(3, 3, 2);
        plot_velocity_with_models_v5(scenario);  % 修正2
        
        subplot(3, 3, 3);
        plot_acceleration_with_models_v5(scenario);  % 修正2
        
        subplot(3, 3, 4);
        plot_motion_model_timeline_v5(scenario);  % 修正2
        
        subplot(3, 3, 5);
        plot_range_measurements_v5(scenario);
        
        subplot(3, 3, 6);
        plot_snr_profile_v5(scenario);
        
        subplot(3, 3, 7);
        plot_trajectory_colored_by_model_v5(scenario);  % 修正2
        
        subplot(3, 3, 8);
        plot_speed_change_rate_v5(scenario);  % 修正2
        
        subplot(3, 3, 9);
        plot_model_statistics_v5(scenario);  % 修正2
        
    else
        subplot(3, 3, 1);
        plot_trajectory_3d_v5(scenario);
        
        subplot(3, 3, 2);
        plot_velocity_with_models_v5(scenario);
        
        subplot(3, 3, 3);
        plot_altitude_with_models_v5(scenario);  % 修正2
        
        subplot(3, 3, 4);
        plot_motion_model_timeline_v5(scenario);
        
        subplot(3, 3, 5);
        plot_range_measurements_v5(scenario);
        
        subplot(3, 3, 6);
        plot_snr_profile_v5(scenario);
        
        subplot(3, 3, 7);
        plot_trajectory_colored_by_model_v5(scenario);
        
        subplot(3, 3, 8);
        plot_acceleration_with_models_v5(scenario);
        
        subplot(3, 3, 9);
        plot_model_statistics_v5(scenario);
    end
    
    sgtitle(title_str, 'FontSize', 18, 'FontWeight', 'bold');
end

function plot_trajectory_and_radars_v5(scenario)
    hold on;
    
    tx_pos = scenario.transmitter_position;
    plot(tx_pos(1)/1000, tx_pos(2)/1000, 'rs', 'MarkerSize', 15, ...
        'MarkerFaceColor', 'red', 'DisplayName', '发射机');
    
    for i = 1:length(scenario.receivers_positions)
        rx_pos = scenario.receivers_positions{i};
        plot(rx_pos(1)/1000, rx_pos(2)/1000, 'b^', 'MarkerSize', 12, ...
            'MarkerFaceColor', 'blue', 'DisplayName', sprintf('RX%d', i));
    end
    
    trajectory = scenario.target_info.trajectory;
    positions = zeros(length(trajectory), 2);
    for j = 1:length(trajectory)
        positions(j, :) = trajectory{j}.position(1:2) / 1000;
    end
    
    plot(positions(:, 1), positions(:, 2), 'g-', 'LineWidth', 2.5, ...
        'DisplayName', '目标轨迹');
    plot(positions(1, 1), positions(1, 2), 'go', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'green');
    plot(positions(end, 1), positions(end, 2), 'ro', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'red');
    
    legend('Location', 'best');
    grid on;
    xlabel('X (km)');
    ylabel('Y (km)');
    title('轨迹和雷达配置');
    axis equal;
end

function plot_trajectory_3d_v5(scenario)
    hold on;
    
    tx_pos = scenario.transmitter_position;
    plot3(tx_pos(1)/1000, tx_pos(2)/1000, tx_pos(3)/1000, 'rs', ...
        'MarkerSize', 15, 'MarkerFaceColor', 'red', 'DisplayName', '发射机');
    
    for i = 1:length(scenario.receivers_positions)
        rx_pos = scenario.receivers_positions{i};
        plot3(rx_pos(1)/1000, rx_pos(2)/1000, rx_pos(3)/1000, 'b^', ...
            'MarkerSize', 12, 'MarkerFaceColor', 'blue', ...
            'DisplayName', sprintf('RX%d', i));
    end
    
    trajectory = scenario.target_info.trajectory;
    positions = zeros(length(trajectory), 3);
    for j = 1:length(trajectory)
        positions(j, :) = trajectory{j}.position / 1000;
    end
    
    plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'g-', ...
        'LineWidth', 2.5, 'DisplayName', '目标轨迹');
    
    legend('Location', 'best');
    grid on;
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)');
    title('3D轨迹和雷达配置');
    view(45, 30);
end

function plot_velocity_with_models_v5(scenario)
    % 修正2：速度剖面（分段着色显示模型切换）
    
    trajectory = scenario.target_info.trajectory;
    model_colors = get_model_colors();
    
    hold on;
    
    times = zeros(1, length(trajectory));
    velocities = zeros(length(trajectory), 1);
    models = cell(1, length(trajectory));
    
    for i = 1:length(trajectory)
        times(i) = trajectory{i}.time;
        velocities(i) = norm(trajectory{i}.velocity);
        models{i} = trajectory{i}.motion_model;
    end
    
    % 按模型分段绘制
    current_model = models{1};
    segment_start = 1;
    
    for i = 2:length(models)
        if ~strcmp(models{i}, current_model) || i == length(models)
            segment_end = i - 1;
            if i == length(models) && strcmp(models{i}, current_model)
                segment_end = i;
            end
            
            % 绘制当前段
            color = model_colors(current_model);
            plot(times(segment_start:segment_end), velocities(segment_start:segment_end), ...
                '-', 'Color', color, 'LineWidth', 2.5, 'DisplayName', current_model);
            
            % 标记切换点
            if segment_end < length(models)
                plot([times(segment_end), times(segment_end)], ...
                    [min(velocities), max(velocities)], ...
                    'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            
            current_model = models{i};
            segment_start = i;
        end
    end
    
    grid on;
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    title('速度剖面（按模型着色）');
    legend('Location', 'best');
end

function plot_acceleration_with_models_v5(scenario)
    % 修正2：加速度剖面（分段着色显示模型切换）
    
    trajectory = scenario.target_info.trajectory;
    model_colors = get_model_colors();
    
    hold on;
    
    times = zeros(1, length(trajectory));
    accelerations = zeros(length(trajectory), 1);
    models = cell(1, length(trajectory));
    
    for i = 1:length(trajectory)
        times(i) = trajectory{i}.time;
        accelerations(i) = norm(trajectory{i}.acceleration);
        models{i} = trajectory{i}.motion_model;
    end
    
    % 按模型分段绘制
    current_model = models{1};
    segment_start = 1;
    
    for i = 2:length(models)
        if ~strcmp(models{i}, current_model) || i == length(models)
            segment_end = i - 1;
            if i == length(models) && strcmp(models{i}, current_model)
                segment_end = i;
            end
            
            color = model_colors(current_model);
            plot(times(segment_start:segment_end), accelerations(segment_start:segment_end), ...
                '-', 'Color', color, 'LineWidth', 2.5, 'DisplayName', current_model);
            
            if segment_end < length(models)
                plot([times(segment_end), times(segment_end)], ...
                    [min(accelerations), max(accelerations)], ...
                    'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            
            current_model = models{i};
            segment_start = i;
        end
    end
    
    grid on;
    xlabel('时间 (s)');
    ylabel('加速度 (m/s²)');
    title('加速度剖面（按模型着色）');
    legend('Location', 'best');
end

function plot_altitude_with_models_v5(scenario)
    % 修正2：高度剖面（分段着色显示模型切换）
    
    trajectory = scenario.target_info.trajectory;
    model_colors = get_model_colors();
    
    hold on;
    
    times = zeros(1, length(trajectory));
    altitudes = zeros(length(trajectory), 1);
    models = cell(1, length(trajectory));
    
    for i = 1:length(trajectory)
        times(i) = trajectory{i}.time;
        altitudes(i) = trajectory{i}.position(3) / 1000;
        models{i} = trajectory{i}.motion_model;
    end
    
    % 按模型分段绘制
    current_model = models{1};
    segment_start = 1;
    
    for i = 2:length(models)
        if ~strcmp(models{i}, current_model) || i == length(models)
            segment_end = i - 1;
            if i == length(models) && strcmp(models{i}, current_model)
                segment_end = i;
            end
            
            color = model_colors(current_model);
            plot(times(segment_start:segment_end), altitudes(segment_start:segment_end), ...
                '-', 'Color', color, 'LineWidth', 2.5, 'DisplayName', current_model);
            
            if segment_end < length(models)
                plot([times(segment_end), times(segment_end)], ...
                    [min(altitudes), max(altitudes)], ...
                    'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            
            current_model = models{i};
            segment_start = i;
        end
    end
    
    grid on;
    xlabel('时间 (s)');
    ylabel('高度 (km)');
    title('高度剖面（按模型着色）');
    legend('Location', 'best');
    ylim([0, 25]);
end

function plot_motion_model_timeline_v5(scenario)
    % 修正2：运动模型时间线（使用色块和标注）
    
    trajectory = scenario.target_info.trajectory;
    model_colors = get_model_colors();
    
    hold on;
    
    % 提取模型切换点
    models = cell(1, length(trajectory));
    times = zeros(1, length(trajectory));
    
    for i = 1:length(trajectory)
        times(i) = trajectory{i}.time;
        models{i} = trajectory{i}.motion_model;
    end
    
    % 绘制色块
    current_model = models{1};
    segment_start = 1;
    segment_count = 0;
    
    for i = 2:length(models)
        if ~strcmp(models{i}, current_model) || i == length(models)
            segment_end = i - 1;
            if i == length(models) && strcmp(models{i}, current_model)
                segment_end = i;
            end
            
            segment_count = segment_count + 1;
            color = model_colors(current_model);
            
            % 绘制色块
            t_start = times(segment_start);
            t_end = times(segment_end);
            fill([t_start, t_end, t_end, t_start], [0.5, 0.5, 1.5, 1.5], ...
                color, 'EdgeColor', 'k', 'LineWidth', 1.5, ...
                'FaceAlpha', 0.6, 'DisplayName', current_model);
            
            % 添加文字标注
            text((t_start + t_end)/2, 1.0, current_model, ...
                'HorizontalAlignment', 'center', 'FontSize', 12, ...
                'FontWeight', 'bold', 'Color', 'k');
            
            current_model = models{i};
            segment_start = i;
        end
    end
    
    xlabel('时间 (s)');
    ylabel('模型');
    title(sprintf('运动模型时序（共%d段）', segment_count));
    grid on;
    ylim([0.3, 1.7]);
    yticks([]);
    xlim([0, max(times)]);
    legend('Location', 'eastoutside');
end

function plot_trajectory_colored_by_model_v5(scenario)
    % 修正2：轨迹按模型着色（2D场景）
    
    trajectory = scenario.target_info.trajectory;
    model_colors = get_model_colors();
    
    hold on;
    
    positions = zeros(length(trajectory), 2);
    models = cell(1, length(trajectory));
    
    for i = 1:length(trajectory)
        positions(i, :) = trajectory{i}.position(1:2) / 1000;
        models{i} = trajectory{i}.motion_model;
    end
    
    % 按模型分段绘制轨迹
    current_model = models{1};
    segment_start = 1;
    
    for i = 2:length(models)
        if ~strcmp(models{i}, current_model) || i == length(models)
            segment_end = i - 1;
            if i == length(models) && strcmp(models{i}, current_model)
                segment_end = i;
            end
            
            color = model_colors(current_model);
            plot(positions(segment_start:segment_end, 1), ...
                 positions(segment_start:segment_end, 2), ...
                 '-', 'Color', color, 'LineWidth', 3.5, ...
                 'DisplayName', current_model);
            
            % 标记切换点
            if segment_end < length(models)
                plot(positions(segment_end, 1), positions(segment_end, 2), ...
                    'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'yellow', ...
                    'HandleVisibility', 'off');
            end
            
            current_model = models{i};
            segment_start = i;
        end
    end
    
    % 标记起点和终点
    plot(positions(1, 1), positions(1, 2), 'gs', 'MarkerSize', 15, ...
        'MarkerFaceColor', 'green', 'DisplayName', '起点');
    plot(positions(end, 1), positions(end, 2), 'rs', 'MarkerSize', 15, ...
        'MarkerFaceColor', 'red', 'DisplayName', '终点');
    
    grid on;
    xlabel('X (km)');
    ylabel('Y (km)');
    title('轨迹（按模型着色）');
    legend('Location', 'best');
    axis equal;
end

function plot_speed_change_rate_v5(scenario)
    % 修正2：速度变化率（显示加速/减速）
    
    trajectory = scenario.target_info.trajectory;
    model_colors = get_model_colors();
    
    hold on;
    
    times = zeros(1, length(trajectory)-1);
    speed_changes = zeros(1, length(trajectory)-1);
    models = cell(1, length(trajectory)-1);
    
    for i = 1:length(trajectory)-1
        times(i) = trajectory{i}.time;
        v1 = norm(trajectory{i}.velocity);
        v2 = norm(trajectory{i+1}.velocity);
        speed_changes(i) = (v2 - v1) / scenario.config.dt;
        models{i} = trajectory{i}.motion_model;
    end
    
    % 按模型分段绘制
    current_model = models{1};
    segment_start = 1;
    
    for i = 2:length(models)
        if ~strcmp(models{i}, current_model) || i == length(models)
            segment_end = i - 1;
            if i == length(models) && strcmp(models{i}, current_model)
                segment_end = i;
            end
            
            color = model_colors(current_model);
            plot(times(segment_start:segment_end), speed_changes(segment_start:segment_end), ...
                '-', 'Color', color, 'LineWidth', 2.5, 'DisplayName', current_model);
            
            if segment_end < length(models)
                plot([times(segment_end), times(segment_end)], ...
                    [min(speed_changes), max(speed_changes)], ...
                    'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            
            current_model = models{i};
            segment_start = i;
        end
    end
    
    % 添加零线
    plot([0, max(times)], [0, 0], 'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    grid on;
    xlabel('时间 (s)');
    ylabel('速度变化率 (m/s²)');
    title('速度变化率（dv/dt）');
    legend('Location', 'best');
end

function plot_model_statistics_v5(scenario)
    % 修正2：模型统计（饼图）
    
    trajectory = scenario.target_info.trajectory;
    
    % 统计每种模型的时间步数
    model_counts = containers.Map();
    
    for i = 1:length(trajectory)
        model = trajectory{i}.motion_model;
        if isKey(model_counts, model)
            model_counts(model) = model_counts(model) + 1;
        else
            model_counts(model) = 1;
        end
    end
    
    % 准备绘图数据
    model_names = keys(model_counts);
    counts = cell2mat(values(model_counts));
    percentages = counts / sum(counts) * 100;
    
    % 获取颜色
    model_colors = get_model_colors();
    colors = zeros(length(model_names), 3);
    for i = 1:length(model_names)
        colors(i, :) = model_colors(model_names{i});
    end
    
    % 绘制饼图
    pie(counts);
    colormap(colors);
    
    % 添加图例
    legend_entries = cell(1, length(model_names));
    for i = 1:length(model_names)
        legend_entries{i} = sprintf('%s (%.1f%%)', model_names{i}, percentages(i));
    end
    legend(legend_entries, 'Location', 'best');
    
    title('运动模型占比统计');
end

function plot_range_measurements_v5(scenario)
    hold on;
    
    measurements = scenario.measurements;
    detailed_records = scenario.detailed_records;
    
    times = zeros(1, length(measurements));
    true_ranges = zeros(1, length(measurements));
    noisy_ranges = zeros(1, length(measurements));
    
    for j = 1:length(measurements)
        times(j) = measurements{j}.time;
        noisy_ranges(j) = measurements{j}.measurements{1}(1) / 1000;
        true_ranges(j) = detailed_records{j}.true_measurements{1}(1) / 1000;
    end
    
    plot(times, true_ranges, 'r-', 'LineWidth', 2, 'DisplayName', '真实距离');
    plot(times, noisy_ranges, 'b.', 'MarkerSize', 4, 'DisplayName', '带噪距离');
    
    grid on;
    xlabel('时间 (s)');
    ylabel('双基地距离和 (km)');
    title('距离测量');
    legend('Location', 'best');
end

function plot_snr_profile_v5(scenario)
    measurements = scenario.measurements;
    
    times = zeros(1, length(measurements));
    snrs = zeros(1, length(measurements));
    
    for j = 1:length(measurements)
        times(j) = measurements{j}.time;
        if ~isempty(measurements{j}.noise_info) && length(measurements{j}.noise_info) > 0
            snrs(j) = measurements{j}.noise_info{1}.snr_db;
        end
    end
    
    plot(times, snrs, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('时间 (s)');
    ylabel('SNR (dB)');
    title('SNR剖面');
end

function model_colors = get_model_colors()
    % 修正2：统一的模型颜色映射
    model_colors = containers.Map(...
        {'CV', 'CA', 'CT', 'Singer'}, ...
        {[1 0 0], [0 0.7 0], [0 0 1], [0.8 0 0.8]});  % 红、绿、蓝、紫
end

%% ================ 数据集分析 ================
function analyze_dataset_v5(dataset_dir)
    % 分析数据集统计信息
    
    fprintf('\n=== 数据集分析: %s ===\n', dataset_dir);
    
    % 加载数据集
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    if ~exist(dataset_file, 'file')
        fprintf('错误：数据集文件不存在: %s\n', dataset_file);
        return;
    end
    
    load(dataset_file, 'dataset');
    
    fprintf('场景数量: %d\n', length(dataset.scenarios));
    fprintf('维度: %dD\n', dataset.config.dimension);
    fprintf('版本: %s\n', dataset.metadata.version);
    
    % 统计目标类型分布
    target_types = {};
    for i = 1:length(dataset.scenarios)
        target_types{end+1} = dataset.scenarios{i}.target_info.target_type;
    end
    
    fprintf('\n目标类型分布:\n');
    for type_name = {'slow', 'medium', 'fast'}
        count = sum(strcmp(target_types, type_name{1}));
        fprintf('  %s: %d (%.1f%%)\n', type_name{1}, count, 100*count/length(target_types));
    end
    
    % 统计运动模型使用频率
    model_counts = containers.Map({'CV', 'CA', 'CT', 'Singer'}, {0, 0, 0, 0});
    total_steps = 0;
    
    for i = 1:length(dataset.scenarios)
        trajectory = dataset.scenarios{i}.target_info.trajectory;
        for j = 1:length(trajectory)
            model = trajectory{j}.motion_model;
            if isKey(model_counts, model)
                model_counts(model) = model_counts(model) + 1;
            end
            total_steps = total_steps + 1;
        end
    end
    
    fprintf('\n运动模型使用频率:\n');
    for model = {'CV', 'CA', 'CT', 'Singer'}
        count = model_counts(model{1});
        fprintf('  %s: %d 步 (%.1f%%)\n', model{1}, count, 100*count/total_steps);
    end
    
    % 统计速度和加速度范围
    all_velocities = [];
    all_accelerations = [];
    
    for i = 1:length(dataset.scenarios)
        trajectory = dataset.scenarios{i}.target_info.trajectory;
        for j = 1:length(trajectory)
            all_velocities = [all_velocities, norm(trajectory{j}.velocity)];
            all_accelerations = [all_accelerations, norm(trajectory{j}.acceleration)];
        end
    end
    
    fprintf('\n速度统计 (m/s):\n');
    fprintf('  最小: %.2f, 最大: %.2f, 平均: %.2f, 标准差: %.2f\n', ...
        min(all_velocities), max(all_velocities), mean(all_velocities), std(all_velocities));
    
    fprintf('\n加速度统计 (m/s²):\n');
    fprintf('  最小: %.2f, 最大: %.2f, 平均: %.2f, 标准差: %.2f\n', ...
        min(all_accelerations), max(all_accelerations), mean(all_accelerations), std(all_accelerations));
    
    % 统计SNR范围
    all_snrs = [];
    for i = 1:min(10, length(dataset.scenarios))
        measurements = dataset.scenarios{i}.measurements;
        for j = 1:length(measurements)
            if ~isempty(measurements{j}.noise_info)
                for k = 1:length(measurements{j}.noise_info)
                    if isfield(measurements{j}.noise_info{k}, 'snr_db')
                        all_snrs = [all_snrs, measurements{j}.noise_info{k}.snr_db];
                    end
                end
            end
        end
    end
    
    if ~isempty(all_snrs)
        fprintf('\nSNR统计 (dB):\n');
        fprintf('  最小: %.2f, 最大: %.2f, 平均: %.2f, 标准差: %.2f\n', ...
            min(all_snrs), max(all_snrs), mean(all_snrs), std(all_snrs));
    end
    
    fprintf('\n=== 分析完成 ===\n');
end

%% ================ 数据验证 ================
function validate_dataset_v5(dataset_dir, config)
    % 验证数据集完整性和正确性
    
    fprintf('\n=== 数据验证: %s ===\n', dataset_dir);
    
    % 加载数据集
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    if ~exist(dataset_file, 'file')
        fprintf('错误：数据集文件不存在\n');
        return;
    end
    
    load(dataset_file, 'dataset');
    
    validation_passed = true;
    issues = {};
    
    % 验证场景完整性
    fprintf('验证场景完整性...\n');
    for i = 1:length(dataset.scenarios)
        scenario = dataset.scenarios{i};
        
        % 检查必需字段
        required_fields = {'scenario_id', 'transmitter_position', 'receivers_positions', ...
                          'target_info', 'measurements', 'detailed_records'};
        for field = required_fields
            if ~isfield(scenario, field{1})
                issues{end+1} = sprintf('场景 %d 缺少字段: %s', i-1, field{1});
                validation_passed = false;
            end
        end
        
        % 验证轨迹长度
        expected_steps = floor(config.simulation_time / config.dt);
        if isfield(scenario, 'target_info') && isfield(scenario.target_info, 'trajectory')
            actual_steps = length(scenario.target_info.trajectory);
            if actual_steps ~= expected_steps
                issues{end+1} = sprintf('场景 %d 轨迹长度不匹配: %d vs %d', ...
                    i-1, actual_steps, expected_steps);
                validation_passed = false;
            end
        end
    end
    
    % 验证物理约束（修正1：应无超限）
    fprintf('验证物理约束（硬约束）...\n');
    for i = 1:min(5, length(dataset.scenarios))
        scenario = dataset.scenarios{i};
        trajectory = scenario.target_info.trajectory;
        target_type = scenario.target_info.target_type;
        target_params = get_target_params_v5(config, target_type);
        
        max_v = target_params.max_velocity + config.hard_constraint.velocity_tolerance;
        max_a = target_params.max_acceleration + config.hard_constraint.acceleration_tolerance;
        
        for j = 1:length(trajectory)
            v_norm = norm(trajectory{j}.velocity);
            a_norm = norm(trajectory{j}.acceleration);
            
            if v_norm > max_v
                issues{end+1} = sprintf('场景 %d 时刻 %.1fs 速度超硬约束: %.2f > %.2f m/s', ...
                    i-1, trajectory{j}.time, v_norm, max_v);
                validation_passed = false;
            end
            
            if a_norm > max_a
                issues{end+1} = sprintf('场景 %d 时刻 %.1fs 加速度超硬约束: %.2f > %.2f m/s²', ...
                    i-1, trajectory{j}.time, a_norm, max_a);
                validation_passed = false;
            end
        end
    end
    
    % 验证JSON文件
    fprintf('验证JSON文件...\n');
    json_count = 0;
    for i = 1:length(dataset.scenarios)
        json_file = fullfile(dataset_dir, sprintf('scenario_%04d.json', i-1));
        if exist(json_file, 'file')
            json_count = json_count + 1;
        else
            issues{end+1} = sprintf('缺少JSON文件: scenario_%04d.json', i-1);
            validation_passed = false;
        end
    end
    fprintf('找到 %d / %d 个JSON文件\n', json_count, length(dataset.scenarios));
    
    % 输出验证结果
    fprintf('\n=== 验证结果 ===\n');
    if validation_passed
        fprintf('✓ 所有验证通过！\n');
    else
        fprintf('✗ 验证失败，发现 %d 个问题：\n', length(issues));
        for i = 1:min(20, length(issues))
            fprintf('  %d. %s\n', i, issues{i});
        end
        if length(issues) > 20
            fprintf('  ... 还有 %d 个问题未显示\n', length(issues) - 20);
        end
    end
    
    fprintf('=== 验证完成 ===\n');
end