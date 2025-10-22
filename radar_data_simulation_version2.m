%% =====================================================================
%  双基地雷达数据集生成器 v5.1 (运动学修正版)
%  =====================================================================
%  作者: YiYangLi0319
%  日期: 2025-10-22
%  
%  核心修正：
%  1. CA模型：模型切换时直接将加速度参数写入状态
%  2. CT模型：正确处理x-y耦合转弯，z方向独立CV
%     - 加速度计算：ax = -ω*vy, ay = ω*vx
%     - 状态转移：x-y耦合，z独立
%  3. Singer/CV模型：x-y-z独立，保持kron结构
%  4. 修复JSON保存时的结构体字段不匹配问题
%  =====================================================================

clear all; close all; clc;

%% ================ 主程序调用 ================
 radar_data_simulation_v5();

% 取消下面的注释来运行单元测试
%  run_all_tests_v5();
%% ================ 主程序入口 ================
function radar_data_simulation_v5()
    fprintf('=====================================================\n');
    fprintf('  双基地雷达数据集生成器 v5.1 (运动学修正版)\n');
    fprintf('  作者: YiYangLi\n');
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
    run_config.num_scenarios = 200;
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
        visualize_scenario_v5(dataset_2d.scenarios{1}, '2D场景示例 v5.1');
        visualize_scenario_v5(dataset_3d.scenarios{1}, '3D场景示例 v5.1');
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
    % 创建雷达系统配置 v5.1
    
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
    config.dt = 0.1;
    
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
    
    % ===== 硬约束配置 =====
    config.hard_constraint = struct();
    config.hard_constraint.velocity_tolerance = 1.0;
    config.hard_constraint.acceleration_tolerance = 0.5;
    config.hard_constraint.max_violations = 5;
    
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
    
    config.slow_target = struct();
    config.slow_target.max_velocity = 50.0;
    config.slow_target.max_acceleration = 3.0;
    config.slow_target.max_turn_rate = 0.1;
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
    dataset.metadata.version = '5.1';
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
    
    dataset_file = fullfile(output_dir, 'dataset.mat');
    save(dataset_file, 'dataset', '-v7.3');
    fprintf('数据集已保存: %s\n', dataset_file);
    
    dataset_info = struct();
    dataset_info.config = config;
    dataset_info.metadata = dataset.metadata;
    save_json_file_v5(dataset_info, fullfile(output_dir, 'dataset_info.json'));
end

function scenario = generate_scenario_v5(config, target_type)
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
            else
                phi = rand() * 2 * pi;
                theta = pi/6 + rand() * (2*pi/3);
                radius = min_r + rand() * (max_r - min_r);
                
                x = radius * sin(theta) * cos(phi);
                y = radius * sin(theta) * sin(phi);
                z = radius * cos(theta);
                
                z = min(max(z, config.min_receiver_altitude_3d), config.max_receiver_altitude_3d);
                
                pos = [x, y, z];
            end
            temp_receivers{end+1} = pos;
        end
        
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
    
    target_params = get_target_params_v5(config, target_type);
    
    % 初始化状态
    pos_range = config.initial_position_range;
    vel_range = target_params.max_velocity * 0.3;
    
    if config.dimension == 2
        initial_pos = [(rand() - 0.5) * pos_range, (rand() - 0.5) * pos_range];
        initial_vel = [(rand() - 0.5) * vel_range, (rand() - 0.5) * vel_range];
        initial_acc = [0, 0];
        state = [initial_pos, initial_vel, initial_acc];
    else
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
    
    consecutive_violations = 0;
    max_consecutive = config.hard_constraint.max_violations;
    
    previous_model = '';  % 用于检测模型切换
    
    % 轨迹生成主循环
    for t = 1:time_steps
        current_time = (t-1) * config.dt;
        current_model = model_sequence{t}.model;
        
        % **修正1: 检测模型切换，CA模型时直接写入加速度**
        if t > 1 && ~strcmp(current_model, previous_model)
            if strcmp(current_model, 'CA')
                % CA模型切换：直接将加速度参数写入状态
                if config.dimension == 2
                    state(5:6) = model_sequence{t}.params.acceleration;
                else
                    state(7:9) = model_sequence{t}.params.acceleration;
                end
                fprintf('    时刻%.1fs: 切换到CA模型，加速度设置为 [%.2f, %.2f]\n', ...
                    current_time, state(5), state(6));
            end
        end
        previous_model = current_model;
        
        if config.dimension == 2
            current_pos = [state(1:2), 0];
            current_vel = [state(3:4), 0];
            current_acc = [state(5:6), 0];
        else
            current_pos = state(1:3);
            current_vel = state(4:6);
            current_acc = state(7:9);
        end
        
        % 硬约束检查
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
            consecutive_violations = 0;
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
    
    is_valid = validate_trajectory_motion_v5(target_info.trajectory, ...
        model_sequence, config);
end

%% ================ 硬约束检查 ================
function [violated, violation_type] = check_hard_constraints_v5(state, target_params, config)
    violated = false;
    violation_type = '';
    
    if config.dimension == 2
        v = state(3:4);
        a = state(5:6);
    else
        v = state(4:6);
        a = state(7:9);
    end
    
    v_norm = norm(v);
    max_v = target_params.max_velocity + config.hard_constraint.velocity_tolerance;
    
    if v_norm > max_v
        violated = true;
        violation_type = '速度';
        return;
    end
    
    a_norm = norm(a);
    max_a = target_params.max_acceleration + config.hard_constraint.acceleration_tolerance;
    
    if a_norm > max_a
        violated = true;
        violation_type = '加速度';
        return;
    end
    
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
    
    model_sequence = cell(1, time_steps);
    noise_schedule = cell(1, time_steps);
    
    current_step = 1;
    dt = config.dt;
    
    while current_step <= time_steps
        model_name = config.available_models{randi(length(config.available_models))};
        
        model_duration_sec = config.min_model_duration + ...
            rand() * (config.max_model_duration - config.min_model_duration);
        model_duration_steps = round(model_duration_sec / dt);
        model_end_step = min(current_step + model_duration_steps - 1, time_steps);
        
        model_params = generate_model_parameters_v5(model_name, config, target_params);
        
        noise_steps = generate_noise_steps_v5(config, target_params, ...
            model_name, current_step, model_end_step, dt);
        
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

%% ================ 状态更新（修正版）================
function next_state = update_state_v5(state, model_info, noise_info, config, target_params)
    % **修正：CA模型不需要特殊处理（加速度已在切换时写入）**
    % **修正：CT模型正确处理x-y耦合**
    
    model_name = model_info.model;
    model_params = model_info.params;
    dt = config.dt;
    dim = config.dimension;
    
    % 构建状态转移矩阵和过程噪声协方差
    [Phi, Q] = get_transition_matrices_v5(model_name, model_params, noise_info, dt, dim, state);
    
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
end

function [Phi, Q] = get_transition_matrices_v5(model_name, model_params, noise_info, dt, dim, state)
    % **修正：CT模型单独处理x-y耦合，其他模型用kron**
    
    sigma = noise_info.intensity;
    
    if strcmp(model_name, 'CT')
        % **修正2：CT模型 - x-y耦合，z独立CV**
        omega = model_params.turn_rate;
        
        if dim == 2
            % 2D: 只有x-y
            [Phi, Q] = get_ct_matrices_2d(omega, dt, sigma);
        else
            % 3D: x-y耦合 + z独立CV
            [Phi_xy, Q_xy] = get_ct_matrices_2d(omega, dt, sigma);
            
            % z方向CV模型
            Phi_z = [1, dt, 0;
                     0, 1,  0;
                     0, 0,  0];
            Q_z = sigma^2 * [dt^3/3, dt^2/2, 0;
                             dt^2/2, dt,     0;
                             0,      0,      0];
            
            % 组合为9x9矩阵 [x,y,z,vx,vy,vz,ax,ay,az]
            Phi = zeros(9, 9);
            Phi(1:2, 1:2) = Phi_xy(1:2, 1:2);  % x,y 位置
            Phi(1:2, 4:5) = Phi_xy(1:2, 3:4);  % x,y 速度贡献
            Phi(1:2, 7:8) = Phi_xy(1:2, 5:6);  % x,y 加速度贡献
            Phi(4:5, 4:5) = Phi_xy(3:4, 3:4);  % vx,vy 速度
            Phi(4:5, 7:8) = Phi_xy(3:4, 5:6);  % vx,vy 加速度贡献
            Phi(7:8, 7:8) = Phi_xy(5:6, 5:6);  % ax,ay 加速度
            
            % z方向
            Phi(3, 3) = Phi_z(1, 1);  % z位置
            Phi(3, 6) = Phi_z(1, 2);  % z速度贡献
            Phi(6, 6) = Phi_z(2, 2);  % vz
            
            % 过程噪声协方差
            Q = zeros(9, 9);
            Q(1:2, 1:2) = Q_xy(1:2, 1:2);
            Q(1:2, 4:5) = Q_xy(1:2, 3:4);
            Q(1:2, 7:8) = Q_xy(1:2, 5:6);
            Q(4:5, 1:2) = Q_xy(3:4, 1:2);
            Q(4:5, 4:5) = Q_xy(3:4, 3:4);
            Q(4:5, 7:8) = Q_xy(3:4, 5:6);
            Q(7:8, 1:2) = Q_xy(5:6, 1:2);
            Q(7:8, 4:5) = Q_xy(5:6, 3:4);
            Q(7:8, 7:8) = Q_xy(5:6, 5:6);
            
            Q(3, 3) = Q_z(1, 1);
            Q(3, 6) = Q_z(1, 2);
            Q(6, 3) = Q_z(2, 1);
            Q(6, 6) = Q_z(2, 2);
        end
        
    else
        % **其他模型（CV/CA/Singer）：x-y-z独立，使用kron**
        switch model_name
            case 'CV'
                Phi1 = [1, dt, 0;
                        0, 1,  0;
                        0, 0,  0];
                Q1 = sigma^2 * [dt^3/3, dt^2/2, 0;
                                dt^2/2, dt,     0;
                                0,      0,      0];
                
            case 'CA'
                % **修正1：CA模型正常递推（加速度已在切换时写入状态）**
                Phi1 = [1, dt, 0.5*dt^2;
                        0, 1,  dt;
                        0, 0,  1];
                Q1 = sigma^2 * [dt^5/20, dt^4/8, dt^3/6;
                                dt^4/8,  dt^3/3, dt^2/2;
                                dt^3/6,  dt^2/2, dt];
                
            case 'Singer'
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
        end
        
        % Kron扩展
        if dim == 2
            Phi = kron(Phi1, eye(2));
            Q = kron(Q1, eye(2));
        else
            Phi = kron(Phi1, eye(3));
            Q = kron(Q1, eye(3));
        end
    end
    
    % 确保协方差矩阵对称正定
    Q = (Q + Q') / 2;
    Q = Q + 1e-10 * eye(size(Q, 1));
end

function [Phi, Q] = get_ct_matrices_2d(omega, dt, sigma)
    % **CT模型2D：x-y耦合转弯**
    % 状态：[x, y, vx, vy, ax, ay]
    % 动力学：ax = -ω*vy, ay = ω*vx
    
    if abs(omega * dt) < 1e-6
        % 转弯率接近0，退化为CV模型
        Phi1 = [1, dt, 0;
                0, 1,  0;
                0, 0,  0];
        Phi = kron(Phi1, eye(2));
        
        Q1 = sigma^2 * [dt^3/3, dt^2/2, 0;
                        dt^2/2, dt,     0;
                        0,      0,      0];
        Q = kron(Q1, eye(2));
    else
        c = cos(omega * dt);
        s = sin(omega * dt);
        
        % 状态转移矩阵 (6x6)
        Phi = zeros(6, 6);
        
        % 位置更新：考虑速度和加速度
        Phi(1, 1) = 1;  % x
        Phi(1, 3) = s / omega;  % vx的贡献
        Phi(1, 4) = -(1 - c) / omega;  % vy的贡献
        
        Phi(2, 2) = 1;  % y
        Phi(2, 3) = (1 - c) / omega;  % vx的贡献
        Phi(2, 4) = s / omega;  % vy的贡献
        
        % 速度更新：受转弯率影响
        Phi(3, 3) = c;  % vx
        Phi(3, 4) = -s;  % vy对vx的影响
        
        Phi(4, 3) = s;  % vx对vy的影响
        Phi(4, 4) = c;  % vy
        
        % 加速度更新：ax = -ω*vy, ay = ω*vx
        Phi(5, 4) = -omega;  % ax由vy决定
        Phi(6, 3) = omega;   % ay由vx决定
        
        % 过程噪声（简化模型，主要在速度和加速度）
        Q = sigma^2 * eye(6);
        Q(1:2, 1:2) = sigma^2 * dt^3/3 * eye(2);  % 位置噪声
        Q(3:4, 3:4) = sigma^2 * dt * eye(2);      % 速度噪声
        Q(5:6, 5:6) = sigma^2 * dt * eye(2);      % 加速度噪声
        
        % 交叉项
        Q(1:2, 3:4) = sigma^2 * dt^2/2 * eye(2);
        Q(3:4, 1:2) = Q(1:2, 3:4)';
    end
end

function [Phi, Q] = singer_discretization_vanloan(A, B, Qc, T)
    n = size(A, 1);
    G = B * Qc * B.';
    
    M = [ A,           G;
          zeros(n, n), -A.' ];
    
    EM = expm(M * T);
    
    Phi = EM(1:n, 1:n);
    S   = EM(1:n, n+1:end);
    
    Q = Phi * S;
    Q = (Q + Q.') / 2;
    Q = Q + 1e-12 * eye(n);
end

%% ================ 量测生成 ================
function [measurements, noise_info] = generate_measurements_v5(target_pos, tx_pos, receivers_pos, config)
    measurements = {};
    noise_info = {};
    
    for i = 1:length(receivers_pos)
        rx_pos = receivers_pos{i};
        
        [true_range, true_azimuth, true_elevation] = compute_true_measurement_v5(...
            target_pos, tx_pos, rx_pos, config.dimension);
        
        snr_db = compute_snr_v5(target_pos, tx_pos, rx_pos, config);
        [snr_range_std, snr_angle_std] = snr_to_noise_std_v5(snr_db, config);
        
        time_sync_std = config.measurement_noise.time_sync_std;
        range_noise_std = sqrt(snr_range_std^2 + time_sync_std^2);
        
        hardware_angle_std = config.measurement_noise.hardware_angle_std;
        angle_noise_std = sqrt(snr_angle_std^2 + hardware_angle_std^2);
        
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
    if length(target_pos) == 2
        target_pos = [target_pos, 0];
    end
    if length(tx_pos) == 2
        tx_pos = [tx_pos, 0];
    end
    if length(rx_pos) == 2
        rx_pos = [rx_pos, 0];
    end
    
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
        if direction_norm > 0
            elevation = asin(direction(3) / direction_norm);
        else
            elevation = 0;
        end
    end
end

function snr_db = compute_snr_v5(target_pos, tx_pos, rx_pos, config)
    lambda = 3e8 / config.snr.freq;
    Pt = config.snr.tx_power;
    Gt = 10^(27/10);
    Gr = 10^(27/10);
    sigma = config.snr.target_rcs;
    tau = config.snr.tau;
    R_tx = norm(target_pos - tx_pos);
    R_rx = norm(target_pos - rx_pos);
    
    Pr = (Pt * Gt * Gr * lambda^2 * sigma * tau) / ((4*pi)^3 * R_tx^2 * R_rx^2);
    
    k = 1.38e-23;
    T = 290;
    B = config.snr.bandwidth;
    Ls = 10^(13/10);
    Lt = 10^(8/10);
    NF = 10^(config.snr.noise_figure / 10);
    Pn = k * T * NF * Ls * Lt;
    
    snr = Pr / Pn;
    snr_db = 10 * log10(max(snr, 1e-10));
end

function [range_std, angle_std] = snr_to_noise_std_v5(snr_db, config)
    snr_linear = 10^(snr_db / 10);
    
    c = 3e8;
    B = config.snr.bandwidth;
    range_resolution = c * sqrt(3) / (pi * B);
    range_std = range_resolution / sqrt(2 * snr_linear);
    range_std = max(range_std, 1.0);
    
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
    elev = max(-pi/2, min(pi/2, elev));
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

%% ================ 辅助函数 ================
function params = get_target_params_v5(config, target_type)
    switch target_type
        case 'slow'
            params = config.slow_target;
        case 'medium'
            params = config.medium_target;
        case 'fast'
            params = config.fast_target;
        otherwise
            error('Unknown target type: %s', target_type);
    end
end

%% ================ 轨迹验证 ================
function is_valid = validate_trajectory_motion_v5(trajectory, model_sequence, config)
    is_valid = true;
    segments = extract_model_segments(trajectory, model_sequence);
    
    for i = 1:length(segments)
        segment = segments{i};
        model_name = segment.model;
        
        segment_valid = validate_segment_motion(segment, model_name, config);
        
        if ~segment_valid
            fprintf('    模型段 %d (%s) 验证失败\n', i, model_name);
            is_valid = false;
            break;
        end
    end
end

function segments = extract_model_segments(trajectory, model_sequence)
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
    
    segment = struct();
    segment.model = current_model;
    segment.start_idx = segment_start;
    segment.end_idx = length(model_sequence);
    segment.trajectory = trajectory(segment_start:segment.end_idx);
    segment.params = model_sequence{segment_start}.params;
    segments{end+1} = segment;
end

function is_valid = validate_segment_motion(segment, model_name, config)
    is_valid = true;
    
    % 简化验证：检查是否有明显的非物理行为
    for i = 1:length(segment.trajectory)
        point = segment.trajectory{i};
        
        vel_norm = norm(point.velocity);
        acc_norm = norm(point.acceleration);
        
        if vel_norm > 1000 || acc_norm > 100
            is_valid = false;
            return;
        end
    end
end

%% ================ 可视化函数 ================
function visualize_scenario_v5(scenario, title_str)
    if nargin < 2
        title_str = 'Scenario Visualization';
    end
    
    config = scenario.config;
    trajectory = scenario.target_info.trajectory;
    
    % 提取轨迹数据
    times = [];
    positions = [];
    velocities = [];
    accelerations = [];
    models = {};
    
    for i = 1:length(trajectory)
        point = trajectory{i};
        times(end+1) = point.time;
        positions(end+1, :) = point.position;
        velocities(end+1, :) = point.velocity;
        accelerations(end+1, :) = point.acceleration;
        models{end+1} = point.motion_model;
    end
    
    % 识别模型切换点
    model_changes = [1];
    for i = 2:length(models)
        if ~strcmp(models{i}, models{i-1})
            model_changes(end+1) = i;
        end
    end
    model_changes(end+1) = length(models) + 1;
    
    % 创建图形
    figure('Position', [100, 100, 1400, 800]);
    
    if config.dimension == 2
        % 2D轨迹
        subplot(2, 3, [1, 4]);
        hold on; grid on;
        
        % 分段绘制轨迹
        for i = 1:length(model_changes)-1
            seg_start = model_changes(i);
            seg_end = model_changes(i+1) - 1;
            
            model_name = models{seg_start};
            color = get_model_color(model_name);
            
            plot(positions(seg_start:seg_end, 1)/1000, ...
                 positions(seg_start:seg_end, 2)/1000, ...
                 'LineWidth', 2, 'Color', color, ...
                 'DisplayName', sprintf('%s (%.0f-%.0fs)', ...
                 model_name, times(seg_start), times(seg_end)));
            
            % 模型切换标记
            if i < length(model_changes)-1
                plot(positions(seg_end, 1)/1000, positions(seg_end, 2)/1000, ...
                     'ko', 'MarkerSize', 8, 'MarkerFaceColor', color);
            end
        end
        
        % 绘制发射机和接收机
        plot(0, 0, 'r^', 'MarkerSize', 15, 'MarkerFaceColor', 'r', ...
             'DisplayName', 'Transmitter');
        
        for i = 1:length(scenario.receivers_positions)
            rx = scenario.receivers_positions{i};
            plot(rx(1)/1000, rx(2)/1000, 'bs', 'MarkerSize', 12, ...
                 'MarkerFaceColor', 'b', 'DisplayName', sprintf('Rx%d', i));
        end
        
        xlabel('X (km)');
        ylabel('Y (km)');
        title(sprintf('%s - 2D Trajectory', title_str));
        legend('Location', 'best');
        axis equal;
        
    else
        % 3D轨迹
        subplot(2, 3, [1, 4]);
        hold on; grid on;
        
        for i = 1:length(model_changes)-1
            seg_start = model_changes(i);
            seg_end = model_changes(i+1) - 1;
            
            model_name = models{seg_start};
            color = get_model_color(model_name);
            
            plot3(positions(seg_start:seg_end, 1)/1000, ...
                  positions(seg_start:seg_end, 2)/1000, ...
                  positions(seg_start:seg_end, 3)/1000, ...
                  'LineWidth', 2, 'Color', color, ...
                  'DisplayName', sprintf('%s (%.0f-%.0fs)', ...
                  model_name, times(seg_start), times(seg_end)));
            
            if i < length(model_changes)-1
                plot3(positions(seg_end, 1)/1000, positions(seg_end, 2)/1000, ...
                      positions(seg_end, 3)/1000, 'ko', 'MarkerSize', 8, ...
                      'MarkerFaceColor', color);
            end
        end
        
        plot3(0, 0, 0, 'r^', 'MarkerSize', 15, 'MarkerFaceColor', 'r', ...
              'DisplayName', 'Transmitter');
        
        for i = 1:length(scenario.receivers_positions)
            rx = scenario.receivers_positions{i};
            plot3(rx(1)/1000, rx(2)/1000, rx(3)/1000, 'bs', ...
                  'MarkerSize', 12, 'MarkerFaceColor', 'b', ...
                  'DisplayName', sprintf('Rx%d', i));
        end
        
        xlabel('X (km)');
        ylabel('Y (km)');
        zlabel('Z (km)');
        title(sprintf('%s - 3D Trajectory', title_str));
        legend('Location', 'best');
        grid on;
        view(3);
    end
    
    % 速度曲线
    subplot(2, 3, 2);
    hold on; grid on;
    
    vel_norms = sqrt(sum(velocities.^2, 2));
    
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        plot(times(seg_start:seg_end), vel_norms(seg_start:seg_end), ...
             'LineWidth', 2, 'Color', color);
        
        % 模型切换竖线
        if i < length(model_changes)-1
            plot([times(seg_end), times(seg_end)], ylim, '--k', 'LineWidth', 1);
        end
    end
    
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Magnitude');
    
    % 加速度曲线
    subplot(2, 3, 3);
    hold on; grid on;
    
    acc_norms = sqrt(sum(accelerations.^2, 2));
    
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        plot(times(seg_start:seg_end), acc_norms(seg_start:seg_end), ...
             'LineWidth', 2, 'Color', color);
        
        if i < length(model_changes)-1
            plot([times(seg_end), times(seg_end)], ylim, '--k', 'LineWidth', 1);
        end
    end
    
    xlabel('Time (s)');
    ylabel('Acceleration (m/s²)');
    title('Acceleration Magnitude');
    
    % 高度曲线（仅3D）
    if config.dimension == 3
        subplot(2, 3, 5);
        hold on; grid on;
        
        for i = 1:length(model_changes)-1
            seg_start = model_changes(i);
            seg_end = model_changes(i+1) - 1;
            
            model_name = models{seg_start};
            color = get_model_color(model_name);
            
            plot(times(seg_start:seg_end), positions(seg_start:seg_end, 3)/1000, ...
                 'LineWidth', 2, 'Color', color);
            
            if i < length(model_changes)-1
                plot([times(seg_end), times(seg_end)], ylim, '--k', 'LineWidth', 1);
            end
        end
        
        xlabel('Time (s)');
        ylabel('Altitude (km)');
        title('Altitude Profile');
    end
    
    % 模型时间线
    subplot(2, 3, 6);
    hold on;
    
    y_pos = 1;
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        rectangle('Position', [times(seg_start), y_pos-0.3, ...
                  times(seg_end)-times(seg_start), 0.6], ...
                  'FaceColor', color, 'EdgeColor', 'k');
        
        text(mean([times(seg_start), times(seg_end)]), y_pos, ...
             model_name, 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'middle', 'FontWeight', 'bold');
    end
    
    xlim([0, max(times)]);
    ylim([0.5, 1.5]);
    xlabel('Time (s)');
    title('Motion Model Timeline');
    set(gca, 'YTick', []);
    grid on;
end

function color = get_model_color(model_name)
    switch model_name
        case 'CV'
            color = [0, 0.4470, 0.7410];  % 蓝色
        case 'CA'
            color = [0.8500, 0.3250, 0.0980];  % 橙色
        case 'CT'
            color = [0.9290, 0.6940, 0.1250];  % 黄色
        case 'Singer'
            color = [0.4940, 0.1840, 0.5560];  % 紫色
        otherwise
            color = [0.5, 0.5, 0.5];  % 灰色
    end
end

%% ================ 数据集分析函数 ================
function analyze_dataset_v5(dataset_dir)
    if ~exist(dataset_dir, 'dir')
        fprintf('数据集目录不存在: %s\n', dataset_dir);
        return;
    end
    
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    if ~exist(dataset_file, 'file')
        fprintf('数据集文件不存在: %s\n', dataset_file);
        return;
    end
    
    load(dataset_file, 'dataset');
    
    fprintf('\n--- 数据集分析: %s ---\n', dataset_dir);
    fprintf('场景数量: %d\n', length(dataset.scenarios));
    fprintf('维度: %dD\n', dataset.config.dimension);
    fprintf('版本: %s\n', dataset.metadata.version);
    
    % 统计目标类型
    target_types = {};
    for i = 1:length(dataset.scenarios)
        target_types{end+1} = dataset.scenarios{i}.target_info.target_type;
    end
    
    fprintf('\n目标类型分布:\n');
    unique_types = unique(target_types);
    for i = 1:length(unique_types)
        count = sum(strcmp(target_types, unique_types{i}));
        fprintf('  %s: %d (%.1f%%)\n', unique_types{i}, count, ...
                100*count/length(target_types));
    end
    
    % 统计运动模型
    all_models = {};
    for i = 1:length(dataset.scenarios)
        trajectory = dataset.scenarios{i}.target_info.trajectory;
        for j = 1:length(trajectory)
            all_models{end+1} = trajectory{j}.motion_model;
        end
    end
    
    fprintf('\n运动模型使用统计:\n');
    unique_models = unique(all_models);
    for i = 1:length(unique_models)
        count = sum(strcmp(all_models, unique_models{i}));
        fprintf('  %s: %d 时间步 (%.1f%%)\n', unique_models{i}, count, ...
                100*count/length(all_models));
    end
end

function validate_dataset_v5(dataset_dir, config)
    if ~exist(dataset_dir, 'dir')
        fprintf('数据集目录不存在: %s\n', dataset_dir);
        return;
    end
    
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    if ~exist(dataset_file, 'file')
        fprintf('数据集文件不存在: %s\n', dataset_file);
        return;
    end
    
    load(dataset_file, 'dataset');
    
    fprintf('\n--- 数据验证: %s ---\n', dataset_dir);
    
    num_valid = 0;
    num_invalid = 0;
    
    for i = 1:length(dataset.scenarios)
        scenario = dataset.scenarios{i};
        
        % 验证轨迹物理约束
        is_valid = true;
        trajectory = scenario.target_info.trajectory;
        target_type = scenario.target_info.target_type;
        target_params = get_target_params_v5(config, target_type);
        
        for j = 1:length(trajectory)
            point = trajectory{j};
            
            vel_norm = norm(point.velocity);
            acc_norm = norm(point.acceleration);
            
            if vel_norm > target_params.max_velocity * 1.1
                is_valid = false;
                fprintf('  场景 %d: 速度超限 (%.2f > %.2f)\n', ...
                    i, vel_norm, target_params.max_velocity);
                break;
            end
            
            if acc_norm > target_params.max_acceleration * 1.1
                is_valid = false;
                fprintf('  场景 %d: 加速度超限 (%.2f > %.2f)\n', ...
                    i, acc_norm, target_params.max_acceleration);
                break;
            end
            
            if config.dimension == 3
                z = point.position(3);
                if z < config.altitude_min * 0.9 || z > config.altitude_max * 1.1
                    is_valid = false;
                    fprintf('  场景 %d: 高度超限 (%.2f)\n', i, z);
                    break;
                end
            end
        end
        
        if is_valid
            num_valid = num_valid + 1;
        else
            num_invalid = num_invalid + 1;
        end
    end
    
    fprintf('\n验证结果:\n');
    fprintf('  有效场景: %d\n', num_valid);
    fprintf('  无效场景: %d\n', num_invalid);
    fprintf('  有效率: %.1f%%\n', 100 * num_valid / length(dataset.scenarios));
end

%% ================ JSON保存函数（修复版）================
function save_scenario_json_v5(scenario, output_dir, scenario_id)
    % **修复：统一所有traj_entry的字段结构，避免串联错误**
    
    json_data = struct();
    json_data.scenario_id = scenario_id;
    json_data.seed = scenario.seed;
    
    % 配置信息
    json_data.config = struct();
    json_data.config.dimension = scenario.config.dimension;
    json_data.config.num_receivers = scenario.config.num_receivers;
    json_data.config.simulation_time = scenario.config.simulation_time;
    json_data.config.dt = scenario.config.dt;
    
    % 发射机和接收机位置
    json_data.transmitter_position = scenario.transmitter_position;
    json_data.receivers_positions = cell2mat(scenario.receivers_positions')';
    
    % 目标信息
    json_data.target = struct();
    json_data.target.target_type = scenario.target_info.target_type;
    json_data.target.initial_state = scenario.target_info.initial_state;
    
    % **修复：先预分配结构体数组，确保所有字段统一**
    num_points = length(scenario.target_info.trajectory);
    
    % 创建统一的模板结构体（包含所有可能的字段）
    template_entry = struct(...
        'time', [], ...
        'position', [], ...
        'velocity', [], ...
        'acceleration', [], ...
        'motion_model', '', ...
        'model_acceleration', [], ...  % CA模型专用
        'model_turn_rate', [], ...      % CT模型专用
        'model_beta', [] ...            % Singer模型专用
    );
    
    % 预分配数组
    trajectory_array = repmat(template_entry, num_points, 1);
    
    % 填充数据
    for i = 1:num_points
        point = scenario.target_info.trajectory{i};
        
        trajectory_array(i).time = point.time;
        trajectory_array(i).position = point.position;
        trajectory_array(i).velocity = point.velocity;
        trajectory_array(i).acceleration = point.acceleration;
        trajectory_array(i).motion_model = point.motion_model;
        
        % 根据模型类型填充对应参数（其他字段保留为空）
        if strcmp(point.motion_model, 'CA')
            trajectory_array(i).model_acceleration = point.model_parameters.acceleration;
            trajectory_array(i).model_turn_rate = [];  % 显式设为空
            trajectory_array(i).model_beta = [];
        elseif strcmp(point.motion_model, 'CT')
            trajectory_array(i).model_acceleration = [];
            trajectory_array(i).model_turn_rate = point.model_parameters.turn_rate;
            trajectory_array(i).model_beta = [];
        elseif strcmp(point.motion_model, 'Singer')
            trajectory_array(i).model_acceleration = [];
            trajectory_array(i).model_turn_rate = [];
            trajectory_array(i).model_beta = point.model_parameters.beta;
        else  % CV模型
            trajectory_array(i).model_acceleration = [];
            trajectory_array(i).model_turn_rate = [];
            trajectory_array(i).model_beta = [];
        end
    end
    
    json_data.target.trajectory = trajectory_array;
    
    % **修复：量测数据也采用相同方法**
    num_meas = length(scenario.measurements);
    
    % 量测模板结构体
    meas_template = struct(...
        'time', [], ...
        'receiver_measurements', [] ...
    );
    
    measurements_array = repmat(meas_template, num_meas, 1);
    
    for i = 1:num_meas
        meas_point = scenario.measurements{i};
        
        measurements_array(i).time = meas_point.time;
        
        % 每个接收机的量测
        num_rx = length(meas_point.measurements);
        rx_meas_template = struct(...
            'receiver_id', [], ...
            'range', [], ...
            'azimuth', [], ...
            'elevation', [], ...
            'snr_db', [], ...
            'range_noise_std', [], ...
            'angle_noise_std', [], ...
            'is_heavy_tail', [] ...
        );
        
        rx_meas_array = repmat(rx_meas_template, num_rx, 1);
        
        for j = 1:num_rx
            rx_meas_array(j).receiver_id = j;
            rx_meas_array(j).range = meas_point.measurements{j}(1);
            rx_meas_array(j).azimuth = meas_point.measurements{j}(2);
            rx_meas_array(j).elevation = meas_point.measurements{j}(3);
            
            % 噪声信息
            noise_info = meas_point.noise_info{j};
            rx_meas_array(j).snr_db = noise_info.snr_db;
            rx_meas_array(j).range_noise_std = noise_info.range_noise_std;
            rx_meas_array(j).angle_noise_std = noise_info.angle_noise_std;
            rx_meas_array(j).is_heavy_tail = noise_info.is_heavy_tail;
        end
        
        measurements_array(i).receiver_measurements = rx_meas_array;
    end
    
    json_data.measurements = measurements_array;
    
    % 保存JSON文件
    json_filename = fullfile(output_dir, sprintf('scenario_%04d.json', scenario_id));
    save_json_file_v5(json_data, json_filename);
end

function save_json_file_v5(data, filename)
    % 保存JSON文件（兼容不同MATLAB版本）
    
    try
        % MATLAB R2016b及以上版本
        json_str = jsonencode(data, 'PrettyPrint', true);
        fid = fopen(filename, 'w');
        fprintf(fid, '%s', json_str);
        fclose(fid);
    catch
        % 旧版本MATLAB，使用savejson（需要JSONlab工具箱）
        try
            savejson('', data, filename);
        catch
            warning('无法保存JSON文件，请安装JSONlab工具箱');
        end
    end
end

%% ================ 可视化增强函数 ================
function plot_model_comparison_v5(dataset_dir)
    % 对比不同运动模型的特征
    
    if ~exist(dataset_dir, 'dir')
        fprintf('数据集目录不存在: %s\n', dataset_dir);
        return;
    end
    
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    load(dataset_file, 'dataset');
    
    % 提取每个模型的速度和加速度统计
    model_stats = struct();
    
    for i = 1:length(dataset.scenarios)
        trajectory = dataset.scenarios{i}.target_info.trajectory;
        
        for j = 1:length(trajectory)
            point = trajectory{j};
            model_name = point.motion_model;
            
            if ~isfield(model_stats, model_name)
                model_stats.(model_name) = struct();
                model_stats.(model_name).velocities = [];
                model_stats.(model_name).accelerations = [];
            end
            
            vel_norm = norm(point.velocity);
            acc_norm = norm(point.acceleration);
            
            model_stats.(model_name).velocities(end+1) = vel_norm;
            model_stats.(model_name).accelerations(end+1) = acc_norm;
        end
    end
    
    % 绘制对比图
    figure('Position', [100, 100, 1200, 500]);
    
    models = fieldnames(model_stats);
    colors = [
        0, 0.4470, 0.7410;     % CV - 蓝色
        0.8500, 0.3250, 0.0980; % CA - 橙色
        0.9290, 0.6940, 0.1250; % CT - 黄色
        0.4940, 0.1840, 0.5560  % Singer - 紫色
    ];
    
    % 速度分布
    subplot(1, 2, 1);
    hold on; grid on;
    
    for i = 1:length(models)
        model_name = models{i};
        velocities = model_stats.(model_name).velocities;
        
        histogram(velocities, 50, 'FaceColor', colors(i, :), ...
                 'FaceAlpha', 0.5, 'DisplayName', model_name);
    end
    
    xlabel('Velocity (m/s)');
    ylabel('Frequency');
    title('Velocity Distribution by Model');
    legend('Location', 'best');
    
    % 加速度分布
    subplot(1, 2, 2);
    hold on; grid on;
    
    for i = 1:length(models)
        model_name = models{i};
        accelerations = model_stats.(model_name).accelerations;
        
        histogram(accelerations, 50, 'FaceColor', colors(i, :), ...
                 'FaceAlpha', 0.5, 'DisplayName', model_name);
    end
    
    xlabel('Acceleration (m/s²)');
    ylabel('Frequency');
    title('Acceleration Distribution by Model');
    legend('Location', 'best');
end

%% ================ 运动学验证函数 ================
function verify_kinematics_v5(scenario)
    % 验证运动学一致性（用于调试）
    
    fprintf('\n--- 运动学验证 ---\n');
    
    trajectory = scenario.target_info.trajectory;
    config = scenario.config;
    
    for i = 2:min(10, length(trajectory))  % 只检查前10个点
        prev_point = trajectory{i-1};
        curr_point = trajectory{i};
        
        dt = curr_point.time - prev_point.time;
        
        % 数值微分验证
        pos_diff = curr_point.position - prev_point.position;
        vel_numerical = pos_diff / dt;
        vel_stored = curr_point.velocity;
        
        vel_error = norm(vel_numerical - vel_stored);
        
        fprintf('时刻 %.1fs:\n', curr_point.time);
        fprintf('  模型: %s\n', curr_point.motion_model);
        fprintf('  速度误差: %.4f m/s\n', vel_error);
        
        % CA模型特殊检查
        if strcmp(curr_point.motion_model, 'CA')
            model_acc = curr_point.model_parameters.acceleration;
            state_acc = curr_point.acceleration;
            
            if config.dimension == 2
                acc_match = norm(model_acc - state_acc(1:2));
            else
                acc_match = norm(model_acc - state_acc);
            end
            
            fprintf('  CA加速度匹配度: %.6f m/s² (应接近0)\n', acc_match);
        end
        
        % CT模型特殊检查
        if strcmp(curr_point.motion_model, 'CT')
            omega = curr_point.model_parameters.turn_rate;
            vx = curr_point.velocity(1);
            vy = curr_point.velocity(2);
            ax_expected = -omega * vy;
            ay_expected = omega * vx;
            
            ax_actual = curr_point.acceleration(1);
            ay_actual = curr_point.acceleration(2);
            
            ax_error = abs(ax_expected - ax_actual);
            ay_error = abs(ay_expected - ay_actual);
            
            fprintf('  CT加速度验证:\n');
            fprintf('    ax期望: %.4f, 实际: %.4f, 误差: %.4f\n', ...
                    ax_expected, ax_actual, ax_error);
            fprintf('    ay期望: %.4f, 实际: %.4f, 误差: %.4f\n', ...
                    ay_expected, ay_actual, ay_error);
        end
        
        fprintf('\n');
    end
end

%% ================ 测试函数 ================
function test_ct_model_v5()
    % 单独测试CT模型的正确性
    
    fprintf('\n=====================================================\n');
    fprintf('  CT模型单元测试\n');
    fprintf('=====================================================\n\n');
    
    % 创建简单的2D配置
    config = create_radar_config_v5(2);
    
    % 初始状态：[x, y, vx, vy, ax, ay]
    state = [0, 0, 100, 0, 0, 0];  % 100 m/s 沿x方向
    
    % CT模型参数：转弯率
    omega = 0.1;  % rad/s
    model_params = struct('turn_rate', omega, 'type', 'CT');
    
    noise_info = struct('intensity', 0.01);  % 很小的噪声
    
    dt = 1.0;
    
    fprintf('初始状态:\n');
    fprintf('  位置: [%.2f, %.2f] m\n', state(1), state(2));
    fprintf('  速度: [%.2f, %.2f] m/s\n', state(3), state(4));
    fprintf('  加速度: [%.2f, %.2f] m/s²\n', state(5), state(6));
    fprintf('  转弯率: %.4f rad/s\n\n', omega);
    
    % 模拟10步
    states = zeros(10, 6);
    states(1, :) = state;
    
    for t = 2:10
        [Phi, Q] = get_transition_matrices_v5('CT', model_params, noise_info, dt, 2, state);
        
        % 不加噪声，纯确定性传播
        state = (Phi * state(:))';
        states(t, :) = state;
        
        % 验证CT动力学
        vx = state(3);
        vy = state(4);
        ax_expected = -omega * vy;
        ay_expected = omega * vx;
        
        fprintf('时刻 %d:\n', t);
        fprintf('  位置: [%.2f, %.2f] m\n', state(1), state(2));
        fprintf('  速度: [%.2f, %.2f] m/s (模 %.2f)\n', ...
                state(3), state(4), norm(state(3:4)));
        fprintf('  加速度期望: [%.2f, %.2f] m/s²\n', ax_expected, ay_expected);
        fprintf('  加速度实际: [%.2f, %.2f] m/s²\n', state(5), state(6));
        fprintf('  误差: [%.4f, %.4f] m/s²\n\n', ...
                abs(ax_expected - state(5)), abs(ay_expected - state(6)));
    end
    
    % 绘制轨迹
    figure;
    plot(states(:, 1), states(:, 2), 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
    hold on;
    quiver(states(:, 1), states(:, 2), states(:, 3), states(:, 4), 0.5, 'r');
    grid on;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    title(sprintf('CT模型测试 (ω=%.2f rad/s)', omega));
    legend('轨迹', '速度方向');
    
    fprintf('CT模型测试完成！\n');
    fprintf('预期：目标应沿圆弧运动，速度大小恒定，加速度指向圆心\n');
    fprintf('=====================================================\n\n');
end

function test_ca_model_v5()
    % 单独测试CA模型的正确性
    
    fprintf('\n=====================================================\n');
    fprintf('  CA模型单元测试\n');
    fprintf('=====================================================\n\n');
    
    config = create_radar_config_v5(2);
    
    % 初始状态
    state = [0, 0, 10, 0, 0, 0];  % 初始速度10 m/s沿x方向
    
    % CA模型参数
    target_acc = [2.0, 1.0];  % 目标加速度
    model_params = struct('acceleration', target_acc, 'type', 'CA');
    
    noise_info = struct('intensity', 0.01);
    dt = 1.0;
    
    fprintf('初始状态:\n');
    fprintf('  位置: [%.2f, %.2f] m\n', state(1), state(2));
    fprintf('  速度: [%.2f, %.2f] m/s\n', state(3), state(4));
    fprintf('  加速度: [%.2f, %.2f] m/s²\n', state(5), state(6));
    fprintf('  目标加速度: [%.2f, %.2f] m/s²\n\n', target_acc(1), target_acc(2));
    
    % 模拟模型切换（应直接写入加速度）
    fprintf('模型切换到CA，写入目标加速度...\n');
    state(5:6) = target_acc;
    fprintf('  切换后加速度: [%.2f, %.2f] m/s²\n\n', state(5), state(6));
    
    % 模拟10步
    states = zeros(10, 6);
    states(1, :) = state;
    
    for t = 2:10
        [Phi, Q] = get_transition_matrices_v5('CA', model_params, noise_info, dt, 2, state);
        
        state = (Phi * state(:))';
        states(t, :) = state;
        
        fprintf('时刻 %d:\n', t);
        fprintf('  位置: [%.2f, %.2f] m\n', state(1), state(2));
        fprintf('  速度: [%.2f, %.2f] m/s\n', state(3), state(4));
        fprintf('  加速度: [%.2f, %.2f] m/s² (应保持 [%.2f, %.2f])\n', ...
                state(5), state(6), target_acc(1), target_acc(2));
        fprintf('  加速度误差: [%.4f, %.4f]\n\n', ...
                abs(state(5) - target_acc(1)), abs(state(6) - target_acc(2)));
    end
    
    % 绘制轨迹
    figure;
    subplot(1, 2, 1);
    plot(states(:, 1), states(:, 2), 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
    hold on;
    quiver(states(:, 1), states(:, 2), states(:, 5), states(:, 6), 10, 'r');
    grid on;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('CA模型轨迹');
    legend('轨迹', '加速度方向');
    
    subplot(1, 2, 2);
    plot(1:10, states(:, 5), 'r-o', 'LineWidth', 2, 'DisplayName', 'ax');
    hold on;
    plot(1:10, states(:, 6), 'b-o', 'LineWidth', 2, 'DisplayName', 'ay');
    plot([1, 10], [target_acc(1), target_acc(1)], 'r--', 'DisplayName', 'ax目标');
    plot([1, 10], [target_acc(2), target_acc(2)], 'b--', 'DisplayName', 'ay目标');
    grid on;
    xlabel('时间步');
    ylabel('加速度 (m/s²)');
    title('CA模型加速度验证');
    legend('Location', 'best');
    
    fprintf('CA模型测试完成！\n');
    fprintf('预期：加速度应保持恒定为目标值\n');
    fprintf('=====================================================\n\n');
end

%% ================ 主测试入口 ================
function run_all_tests_v5()
    % 运行所有单元测试
    
    fprintf('\n');
    fprintf('*****************************************************\n');
    fprintf('*  运动学模型单元测试套件 v5.1                      *\n');
    fprintf('*****************************************************\n');
    
    % 测试CA模型
    test_ca_model_v5();
    
    % 测试CT模型
    test_ct_model_v5();
    
    fprintf('\n*****************************************************\n');
    fprintf('*  所有测试完成！                                   *\n');
    fprintf('*****************************************************\n\n');
end
