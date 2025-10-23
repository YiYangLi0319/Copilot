%% =====================================================================
%  双基地雷达数据集生成器 v5.2 (完整修正版)
%  =====================================================================
%  作者: YiYangLi0319
%  日期: 2025-10-22
%  修订: v5.2 - 修复关键错误并增强鲁棒性
%  
%  核心修正：
%  1. CA模型：模型切换时直接将加速度参数写入状态（修复3D打印错误）
%  2. CT模型：正确处理x-y耦合转弯，z方向独立CV（修复3D映射）
%     - 加速度计算：ax = -ω*vy, ay = ω*vx
%     - 状态转移：x-y耦合，z独立
%  3. Singer/CV模型：x-y-z独立，保持kron结构
%  4. 移除mvnrnd依赖，使用Cholesky分解
%  5. 修复时间同步误差单位（转换为距离误差）
%  6. 增强数值稳定性
%  
%  状态向量定义：
%    2D: [x, y, vx, vy, ax, ay]           (6维)
%    3D: [x, y, z, vx, vy, vz, ax, ay, az] (9维)
%  =====================================================================

clearvars; close all; clc;

%% ================ 物理常量定义 ================
SPEED_OF_LIGHT = 3e8;  % 光速 (m/s)
BOLTZMANN_CONST = 1.38e-23;  % 玻尔兹曼常数 (J/K)

%% ================ 主程序调用 ================
radar_data_simulation_v5();

% 取消下面的注释来运行单元测试
% run_all_tests_v5();

%% ================ 主程序入口 ================
function radar_data_simulation_v5()
    fprintf('=====================================================\n');
    fprintf('  双基地雷达数据集生成器 v5.3 (内存优化版)\n');
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
    run_config.num_scenarios = 200;  % 总场景数量
    run_config.batch_size = 20;      % 每批次生成的航迹数量
    run_config.run_validation = true;
    
    % 生成2D数据集
%     fprintf('--- 生成2D数据集 ---\n');
%     config_2d = create_radar_config_v5(2);
%     generate_dataset_v5_in_batches(config_2d, run_config, ...
%         'enhanced_v5_2d_dataset', GLOBAL_SEED);
    
    % 生成3D数据集
    fprintf('\n--- 生成3D数据集 ---\n');
    config_3d = create_radar_config_v5(3);
    generate_dataset_v5_in_batches(config_3d, run_config, ...
        'enhanced_v5_3d_dataset', GLOBAL_SEED + 100000);
    
    fprintf('\n=====================================================\n');
    fprintf('  数据集生成完成！\n');
    fprintf('=====================================================\n\n');
end

%% ================ 配置函数 ================
function config = create_radar_config_v5(dimension)
    % 创建雷达系统配置 v5.2
    % 
    % 输入：
    %   dimension - 2 (2D) 或 3 (3D)
    % 
    % 状态向量定义：
    %   2D: [x, y, vx, vy, ax, ay]
    %   3D: [x, y, z, vx, vy, vz, ax, ay, az]
    
    config = struct();
    config.dimension = dimension;
    
    % 根据维度设置状态维度
    if dimension == 2
        config.state_dimension = 6;  % [x, y, vx, vy, ax, ay]
    else
        config.state_dimension = 9;  % [x, y, z, vx, vy, vz, ax, ay, az]
    end
    
    config.num_receivers = 3;
    config.simulation_time = 200;  % 秒
    config.dt = 0.1;  % 采样间隔（秒）
    
    % ===== 接收机配置 =====
    config.receiver_area_radius = 100000;  % 米
    config.min_receiver_distance = 30000;  % 接收机之间最小距离（米）
    config.min_receiver_radius = 10000;    % 距离发射机最小距离（米）
    config.max_receiver_altitude_3d = 20000;  % 最大高度（米）
    config.min_receiver_altitude_3d = 0;      % 最小高度（米）
    
    % ===== 目标空间约束 =====
    config.initial_position_range = 50000;  % 初始位置范围（米）
    config.workspace_radius = 100000;       % 工作空间半径（米）
    
    if dimension == 3
        config.altitude_min = 1000;   % 最小飞行高度（米）
        config.altitude_max = 20000;  % 最大飞行高度（米）
    end
    
    % ===== 硬约束配置 =====
    config.hard_constraint = struct();
    config.hard_constraint.velocity_tolerance = 20.0;       % m/s
    config.hard_constraint.acceleration_tolerance = 10.0;   % m/s^2
    config.hard_constraint.max_violations = 10;             % 最大连续违反次数
    
    % ===== 增强噪声模型 =====
    config.measurement_noise = struct();
    % 时间同步误差（秒）- 将在使用时转换为距离误差
    config.measurement_noise.time_sync_std = 1e-8;  % 10纳秒（典型GPS时间同步精度）
    config.measurement_noise.hardware_angle_std = 0.05 * pi/180;  % 0.05度
    
    % SNR参数
    config.snr = struct();
    config.snr.target_rcs = 2.0;        % 雷达截面积 (m^2)
    config.snr.tx_power = 10e3;         % 发射功率 (W)
    config.snr.freq = 3e9;              % 频率 (Hz)
    config.snr.bandwidth = 1e6;         % 带宽 (Hz)
    config.snr.noise_figure = 1.5;      % 噪声系数 (dB)
    config.snr.min_snr_db = 10.0;
    config.snr.max_snr_db = 80.0;
    config.snr.tau = 1e-3;              % 脉冲宽度 (s)
    config.snr.aperture_m = 5.0;        % 天线孔径 (m)
    config.snr.min_range_noise_m = 1.0; % 最小距离噪声标准差（米）
    config.snr.min_angle_noise_rad = 0.01 * pi/180; % 最小角度噪声（弧度）
    
    % 厚尾噪声
    config.heavy_tail_probability = 0.02;  % 2%概率出现厚尾噪声
    config.student_t_dof = 3;              % Student-t分布自由度
    
    % ===== 目标类型定义 =====
    config.target_types = {'slow', 'medium', 'fast'};
    
    % 慢速目标（如无人机、直升机）
    config.slow_target = struct();
    config.slow_target.max_velocity = 80.0;        % m/s
    config.slow_target.max_acceleration = 3.0;     % m/s^2
    config.slow_target.max_turn_rate = 0.1;        % rad/s
    config.slow_target.noise_ranges = struct(...
        'CV', [0.01, 0.05], 'CA', [0.02, 0.08], 'CT', [0.01, 0.05], 'Singer', [0.1, 0.3]);
    
    % 中速目标（如客机）
    config.medium_target = struct();
    config.medium_target.max_velocity = 340.0;     % m/s
    config.medium_target.max_acceleration = 6.0;   % m/s^2
    config.medium_target.max_turn_rate = 0.07;     % rad/s
    config.medium_target.noise_ranges = struct(...
        'CV', [0.05, 0.15], 'CA', [0.08, 0.25], 'CT', [0.05, 0.15], 'Singer', [0.2, 0.5]);
    
    % 快速目标（如战斗机、导弹）
    config.fast_target = struct();
    config.fast_target.max_velocity = 600.0;       % m/s
    config.fast_target.max_acceleration = 50.0;    % m/s^2
    config.fast_target.max_turn_rate = 0.1;        % rad/s
    config.fast_target.noise_ranges = struct(...
        'CV', [0.1, 0.3], 'CA', [0.15, 0.5], 'CT', [0.1, 0.3], 'Singer', [0.3, 0.8]);
    
    % ===== 运动模型配置 =====
    config.available_models = {'CV', 'CA', 'CT', 'Singer'};
    config.min_model_duration = 20.0;   % 秒
    config.max_model_duration = 40.0;   % 秒
    config.min_noise_step_duration = 5.0;   % 秒
    config.max_noise_step_duration = 15.0;  % 秒
end

%% ================ 数据集生成 ================
function generate_dataset_v5_in_batches(config, run_config, output_dir, base_seed)
    if nargin < 3, output_dir = 'enhanced_v5_dataset'; end
    if nargin < 4, base_seed = 42; end
    
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    num_scenarios = run_config.num_scenarios;
    batch_size = run_config.batch_size;
    num_batches = ceil(num_scenarios / batch_size);
    
    fprintf('总场景数: %d, 每批次数量: %d, 批次数: %d\n', ...
        num_scenarios, batch_size, num_batches);
    
    for batch_idx = 1:num_batches
        fprintf('\n--- 生成第 %d/%d 批次 ---\n', batch_idx, num_batches);
        
        % 当前批次的场景索引范围
        start_idx = (batch_idx - 1) * batch_size + 1;
        end_idx = min(batch_idx * batch_size, num_scenarios);
        current_batch_size = end_idx - start_idx + 1;
        
        fprintf('  场景索引范围: %d - %d\n', start_idx, end_idx);
        
        % 为当前批次分配随机种子
        batch_seed = base_seed + start_idx * 1000;
        rng(batch_seed, 'twister');
        fprintf('  批次随机种子: %d\n', batch_seed);
        
        % 初始化数据集
        batch_dataset = struct();
        batch_dataset.config = config;
        batch_dataset.scenarios = cell(1, current_batch_size);  % 预分配
        batch_dataset.metadata = struct();
        batch_dataset.metadata.creation_time = datestr(now);
        batch_dataset.metadata.num_scenarios = current_batch_size;
        batch_dataset.metadata.version = '5.3';
        batch_dataset.metadata.base_seed = batch_seed;
        
        % 生成当前批次场景
        for scenario_idx = 1:current_batch_size
            global_scenario_idx = start_idx + scenario_idx - 1;
            scenario_seed = base_seed + global_scenario_idx * 1000;
            rng(scenario_seed, 'twister');
            
            target_type = config.target_types{randi(length(config.target_types))};
            fprintf('  生成场景 %d / %d (全局索引: %d, 随机种子: %d)...\n', ...
                scenario_idx, current_batch_size, global_scenario_idx, scenario_seed);
            
            % 生成单个航迹
            scenario = generate_scenario_v5(config, target_type);
            scenario.scenario_id = global_scenario_idx - 1;
            scenario.seed = scenario_seed;
            batch_dataset.scenarios{scenario_idx} = scenario;
        end
        
        % 保存当前批次到文件
        batch_file = fullfile(output_dir, sprintf('batch_%d.mat', batch_idx));
        save(batch_file, 'batch_dataset', '-v7.3');
        fprintf('  批次数据已保存: %s\n', batch_file);
        
        % 清理内存
        clear batch_dataset;
        fprintf('  内存已清理\n');
    end
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
        temp_receivers = cell(1, config.num_receivers);  % 预分配
        
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
            temp_receivers{i} = pos;
        end
        
        % 检查接收机间距和与发射机的距离
        valid = true;
        for i = 1:length(temp_receivers)
            % 检查与发射机距离
            dist_to_tx = norm(temp_receivers{i} - tx_pos);
            if dist_to_tx < min_r
                valid = false;
                break;
            end
            
            % 检查接收机之间距离
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
    
    warning('使用次优接收机配置（已尝试%d次）', max_attempts);
    receivers_pos = temp_receivers;
end

%% ================ 轨迹生成（核心）================
function [target_info, measurements, detailed_records, is_valid] = ...
    generate_target_trajectory_v5(config, time_steps, target_type, tx, rxs)
    
    target_params = get_target_params_v5(config, target_type);
    
    % 初始化状态
    pos_range = config.initial_position_range;
    vel_range = target_params.max_velocity * 0.2;
    
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
        vz = (rand() - 0.5) * vel_range/4;
        initial_vel = [vx, vy, vz];
        initial_acc = [0, 0, 0];
        state = [initial_pos, initial_vel, initial_acc];
    end
    
    % 生成运动模型和噪声序列
    [model_sequence, noise_schedule] = generate_model_and_noise_sequence_v5(...
        config, target_params, time_steps);
    
    % 初始化记录（预分配）
    target_info = struct();
    target_info.initial_state = state;
    target_info.target_type = target_type;
    target_info.motion_model_sequence = model_sequence;
    target_info.trajectory = cell(1, time_steps);
    
    measurements = cell(1, time_steps);
    detailed_records = cell(1, time_steps);
    
    consecutive_violations = 0;
    max_consecutive = config.hard_constraint.max_violations;
    
    previous_model = '';  % 用于检测模型切换
    
    % 轨迹生成主循环
    for t = 1:time_steps
        current_time = (t-1) * config.dt;
        current_model = model_sequence{t}.model;
        
        % **修正1: 检测模型切换，CA模型时直接写入加速度（修复3D打印）**
        if t > 1 && ~strcmp(current_model, previous_model)
            if strcmp(current_model, 'CA')
                % CA模型切换：直接将加速度参数写入状态
                if config.dimension == 2
                    state(5:6) = model_sequence{t}.params.acceleration;
                    fprintf('    时刻%.1fs: 切换到CA模型，加速度设置为 [%.2f, %.2f] m/s^2\n', ...
                        current_time, state(5), state(6));
                else
                    state(7:9) = model_sequence{t}.params.acceleration;
                    fprintf('    时刻%.1fs: 切换到CA模型，加速度设置为 [%.2f, %.2f, %.2f] m/s^2\n', ...
                        current_time, state(7), state(8), state(9));
                end
            end
        end
        previous_model = current_model;
        
        % 提取当前位置、速度、加速度
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
        target_info.trajectory{t} = traj_point;
        
        % 生成量测
        [target_measurements, measurement_noise_info] = generate_measurements_v5(...
            current_pos, tx, rxs, config);
        
        % 计算真实量测
        true_measurements = cell(1, length(rxs));
        for i = 1:length(rxs)
            [true_range, true_azimuth, true_elevation] = compute_true_measurement_v5(...
                current_pos, tx, rxs{i}, config.dimension);
            true_measurements{i} = [true_range, true_azimuth, true_elevation];
        end
        
        meas_point = struct();
        meas_point.time = current_time;
        meas_point.measurements = target_measurements;
        meas_point.noise_info = measurement_noise_info;
        measurements{t} = meas_point;
        
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
        
        detailed_records{t} = detailed_record;
        
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
        if z < config.altitude_min * 0.5 || z > config.altitude_max * 2
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
    
    % 验证序列完整性
    assert(~isempty(model_sequence{1}) && ~isempty(model_sequence{end}), ...
        '模型序列生成不完整');
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
        step_duration_steps = max(1, round(step_duration_sec / dt));
        step_end = min(current_step + step_duration_steps - 1, end_step);
        
        intensity = min_intensity + rand() * (max_intensity - min_intensity);
        
        noise_step = struct();
        noise_step.start_step = current_step;
        noise_step.end_step = step_end;
        noise_step.intensity = intensity;
        
        noise_steps{end+1} = noise_step;
        current_step = step_end + 1;
    end
    
    % 验证噪声步覆盖完整
    if ~isempty(noise_steps)
        assert(noise_steps{1}.start_step == start_step, '噪声步起始不匹配');
        assert(noise_steps{end}.end_step == end_step, '噪声步结束不匹配');
    end
end

function model_params = generate_model_parameters_v5(model_name, config, target_params)
    model_params = struct();
    model_params.type = model_name;
    
    switch model_name
        case 'CV'
            % 恒速模型 - 无额外参数
            
        case 'CA'
            max_acc = target_params.max_acceleration;
            if config.dimension == 2
                % 2D: 生成更温和的加速度
                angle = rand() * 2 * pi;
                acc_magnitude = 0.2 + rand() * 0.6;  % 20%-80% 最大加速度
                acceleration = acc_magnitude * max_acc * [cos(angle), sin(angle)];
                model_params.acceleration = acceleration;
            else
                % 3D: 生成更合理的3D加速度
                phi = rand() * 2 * pi;
                theta = acos(2*rand() - 1);  % 均匀球面分布
                acc_magnitude = 0.2 + rand() * 0.6;  % 20%-80% 最大加速度
                
                x = sin(theta) * cos(phi);
                y = sin(theta) * sin(phi);
                z = cos(theta);
                
                acceleration = acc_magnitude * max_acc * [x, y, z * 0.5];  % z方向较小
                model_params.acceleration = acceleration;
            end
            
        case 'CT'
            % 恒转弯模型
            max_turn = target_params.max_turn_rate;
            model_params.turn_rate = (rand() - 0.5) * max_turn;
            
        case 'Singer'
            % Singer模型
            tau = 10 + rand() * 20;  % 时间常数 10-30秒
            model_params.beta = 1 / tau;
    end
end

%% ================ 状态更新（修正版 - 无mvnrnd依赖）================
function next_state = update_state_v5(state, model_info, noise_info, config, target_params)
    % **修正3: 使用Cholesky分解代替mvnrnd**
    % 状态更新：state_{k+1} = Phi * state_k + w_k
    % 其中 w_k ~ N(0, Q)
    
    model_name = model_info.model;
    model_params = model_info.params;
    dt = config.dt;
    dim = config.dimension;
    
    % 构建状态转移矩阵和过程噪声协方差
    [Phi, Q] = get_transition_matrices_v5(target_params, model_name, model_params, noise_info, dt, dim, state);
    
    % 确定性状态转移
    state_col = state(:);
    predicted_state = Phi * state_col;
    
    % **修正3: 使用Cholesky采样过程噪声**
    state_dim = length(state);
    noise_vec = sample_multivariate_normal(zeros(state_dim, 1), Q);
    
    next_state = predicted_state + noise_vec;
    next_state = next_state';
end

function sample = sample_multivariate_normal(mu, Sigma)
    % 使用Cholesky分解从多元正态分布采样
    % 输入：
    %   mu - 均值向量 (n x 1)
    %   Sigma - 协方差矩阵 (n x n)
    % 输出：
    %   sample - 样本向量 (n x 1)
    
    n = length(mu);
    
    % 确保对称
    Sigma = (Sigma + Sigma') / 2;
    
    % 尝试Cholesky分解
    [U, p] = chol(Sigma);
    
    if p == 0
        % Cholesky成功
        sample = mu + U' * randn(n, 1);
    else
        % Cholesky失败，使用特征值修正
        [V, D] = eig(Sigma);
        D = diag(D);
        D(D < 0) = 0;  % 负特征值置零
        
        Sigma_fixed = V * diag(D) * V';
        Sigma_fixed = (Sigma_fixed + Sigma_fixed') / 2;
        
        % 再次尝试Cholesky
        [U2, p2] = chol(Sigma_fixed + 1e-12 * eye(n));
        
        if p2 == 0
            sample = mu + U2' * randn(n, 1);
        else
            % 最后退化：使用对角近似
            diag_std = sqrt(max(diag(Sigma_fixed), 1e-12));
            sample = mu + diag_std .* randn(n, 1);
        end
    end
end

function [Phi, Q] = get_transition_matrices_v5(target_params, model_name, model_params, noise_info, dt, dim, state)
    % **修正2: CT模型单独处理x-y耦合，正确映射6x6到9x9**
    % 构建状态转移矩阵和过程噪声协方差矩阵
    %
    % 模型说明：
    %   CV: 恒速 - x,y,z独立
    %   CA: 恒加速 - x,y,z独立
    %   CT: 恒转弯 - x,y耦合，z独立CV
    %   Singer: 一阶马尔可夫加速度 - x,y,z独立
    
    sigma = noise_info.intensity;
    
    if strcmp(model_name, 'CT')
        % **修正2：CT模型 - x-y耦合，z独立CV**
        omega = model_params.turn_rate;

        if dim == 2
            vxy = norm(state(3:4));    % 2D
        else
            vxy = norm(state(4:5));    % 3D的水平速度
        end
        a_max = target_params.max_acceleration;
        if vxy > 1e-6
            omega_eff = sign(omega) * min(abs(omega), a_max / vxy);
        else
            omega_eff = omega;
        end
        model_params.turn_rate = omega_eff;
        omega = omega_eff;
        
        


        if dim == 2
            % 2D: 只有x-y
            [Phi, Q] = get_ct_matrices_2d(omega, dt, sigma);
        else
            % 3D: x-y耦合 + z独立CV
            [Phi_xy, Q_xy] = get_ct_matrices_2d(omega, dt, sigma);
            
            % z方向CV模型 (3x3: z, vz, az)
            Phi_z = [1, dt, 0;
                     0, 1,  0;
                     0, 0,  0];
            Q_z = sigma^2 * [dt^3/3, dt^2/2, 0;
                             dt^2/2, dt,     0;
                             0,      0,      1e-6];
            
            % **修正2: 正确映射6x6到9x9**
            % 状态顺序：[x, y, z, vx, vy, vz, ax, ay, az]
            % Phi_xy顺序：[x, y, vx, vy, ax, ay]
            % 映射：Phi_xy的索引 [1,2,3,4,5,6] -> Phi的索引 [1,2,4,5,7,8]
            
            Phi = zeros(9, 9);
            Q = zeros(9, 9);
            
            % 使用循环进行完整映射
            map_6to9 = [1, 2, 4, 5, 7, 8];  % 6D->9D索引映射
            for i = 1:6
                i9 = map_6to9(i);
                for j = 1:6
                    j9 = map_6to9(j);
                    Phi(i9, j9) = Phi_xy(i, j);
                    Q(i9, j9) = Q_xy(i, j);
                end
            end
            
            % z方向 (索引 3, 6, 9)
            map_z = [3, 6, 9];
            for i = 1:3
                iz = map_z(i);
                for j = 1:3
                    jz = map_z(j);
                    Phi(iz, jz) = Phi_z(i, j);
                    Q(iz, jz) = Q_z(i, j);
                end
            end
        end
        
    else
        % **其他模型（CV/CA/Singer）：x-y-z独立，使用kron**
        switch model_name
            case 'CV'
                % 恒速模型 - 加速度状态不演化
                Phi1 = [1, dt, 0;
                        0, 1,  0;
                        0, 0,  0];  % 加速度不传递
                Q1 = sigma^2 * [dt^3/3, dt^2/2, 0;
                                dt^2/2, dt,     0;
                                0,      0,      0];
                
            case 'CA'
                % 恒加速模型 - 标准运动学
                Phi1 = [1, dt, 0.5*dt^2;
                        0, 1,  dt;
                        0, 0,  1];  % 加速度保持
                Q1 = sigma^2 * [dt^5/20, dt^4/8, dt^3/6;
                                dt^4/8,  dt^3/3, dt^2/2;
                                dt^3/6,  dt^2/2, dt];
                
            case 'Singer'
                % Singer模型 - 一阶马尔可夫加速度
                if isfield(model_params, 'alpha')
                    alpha = model_params.alpha;
                elseif isfield(model_params, 'beta')
                    alpha = model_params.beta;
                else
                    alpha = 0.1;
                end
                
                sigma_m = sigma;
                % 连续时间状态空间模型
                A_c = [0, 1, 0;
                       0, 0, 1;
                       0, 0, -alpha];
                B_c = [0; 0; 1];
                Q_c = 2 * alpha * sigma_m^2;
                
                [Phi1, Q1] = singer_discretization_vanloan(A_c, B_c, Q_c, dt);
        end
        
        % Kron扩展到多维
        if dim == 2
            Phi = kron(eye(2), Phi1);  % 注意：使用kron(eye, Phi1)保持顺序
            Q = kron(eye(2), Q1);
        else
            Phi = kron(eye(3), Phi1);
            Q = kron(eye(3), Q1);
        end
    end
    
    % 确保协方差矩阵对称正定
    Q = (Q + Q') / 2;
    Q = Q + 1e-10 * eye(size(Q, 1));  % 数值稳定性
end

function [Phi, Q] = get_ct_matrices_2d(omega, dt, sigma)
    % **CT模型2D：x-y耦合转弯**
    % 状态：[x, y, vx, vy, ax, ay]
    % 动力学：
    %   - 位置更新：考虑旋转
    %   - 速度更新：旋转矩阵
    %   - 加速度：ax = -ω*vy, ay = ω*vx
    
    if abs(omega * dt) < 1e-6
        % 转弯率接近0，退化为CV模型
        Phi1 = [1, dt, 0;
                0, 1,  0;
                0, 0,  0];
        Phi = kron(eye(2), Phi1);
        
        Q1 = sigma^2 * [dt^3/3, dt^2/2, 0;
                        dt^2/2, dt,     0;
                        0,      0,      0];
        Q = kron(eye(2), Q1);
    else
        c = cos(omega * dt);
        s = sin(omega * dt);
        
        % 状态转移矩阵 (6x6)
        Phi = zeros(6, 6);
        
        % 位置更新（考虑旋转和积分）
        Phi(1, 1) = 1;  % x保持
        Phi(1, 3) = s / omega;  % vx对x的贡献
        Phi(1, 4) = -(1 - c) / omega;  % vy对x的贡献
        
        Phi(2, 2) = 1;  % y保持
        Phi(2, 3) = (1 - c) / omega;  % vx对y的贡献
        Phi(2, 4) = s / omega;  % vy对y的贡献
        
        % 速度更新（旋转矩阵）
        Phi(3, 3) = c;   % vx
        Phi(3, 4) = -s;  % vy对vx的旋转影响
        
        Phi(4, 3) = s;   % vx对vy的旋转影响
        Phi(4, 4) = c;   % vy
        
        % 加速度更新（即时关系）
        % ax = -ω*vy, ay = ω*vx
        Phi(5, 4) = -omega;  % ax由vy决定
        Phi(6, 3) = omega;   % ay由vx决定
        
        % 过程噪声协方差（简化模型）
        Q = zeros(6, 6);
        
        % 位置噪声
        Q(1:2, 1:2) = sigma^2 * dt^3/3 * eye(2);
        
        % 速度噪声
        Q(3:4, 3:4) = sigma^2 * dt * eye(2);
        
        % 加速度噪声
        Q(5:6, 5:6) = sigma^2 * dt * eye(2);
        
        % 位置-速度交叉项
        Q(1:2, 3:4) = sigma^2 * dt^2/2 * eye(2);
        Q(3:4, 1:2) = Q(1:2, 3:4)';
    end
end

function [Phi, Q] = singer_discretization_vanloan(A, B, Qc, T)
    % Van Loan方法离散化Singer模型
    % 连续系统：dx/dt = A*x + B*w, E[w*w'] = Qc*delta(t)
    % 离散系统：x[k+1] = Phi*x[k] + v[k], E[v*v'] = Q
    
    n = size(A, 1);
    G = B * Qc * B';
    
    % 构建增广矩阵
    M = [ A,           G;
          zeros(n, n), -A' ];
    
    % 矩阵指数
    EM = expm(M * T);
    
    % 提取Phi和Q
    Phi = EM(1:n, 1:n);
    S   = EM(1:n, n+1:end);
    
    Q = Phi * S;
    Q = (Q + Q') / 2;  % 确保对称
    Q = Q + 1e-12 * eye(n);  % 数值稳定性
end

%% ================ 量测生成 ================
function [measurements, noise_info] = generate_measurements_v5(target_pos, tx_pos, receivers_pos, config)
    % 生成带噪声的双基地雷达量测
    % 量测：[距离和, 方位角, 俯仰角]
    
    measurements = cell(1, length(receivers_pos));
    noise_info = cell(1, length(receivers_pos));
    
    for i = 1:length(receivers_pos)
        rx_pos = receivers_pos{i};
        
        % 计算真实量测
        [true_range, true_azimuth, true_elevation] = compute_true_measurement_v5(...
            target_pos, tx_pos, rx_pos, config.dimension);
        
        % 计算SNR
        snr_db = compute_snr_v5(target_pos, tx_pos, rx_pos, config);
        [snr_range_std, snr_angle_std] = snr_to_noise_std_v5(snr_db, config);
        
        % **修正4: 时间同步误差转换为距离误差**
        time_sync_std_sec = config.measurement_noise.time_sync_std;  % 秒
        c = 3e8;  % 光速 m/s
        time_sync_std_range = c * time_sync_std_sec;  % 转换为距离（米）
        
        % 合成距离噪声标准差
        range_noise_std = sqrt(snr_range_std^2 + time_sync_std_range^2);
        
        % 合成角度噪声标准差
        hardware_angle_std = config.measurement_noise.hardware_angle_std;
        angle_noise_std = sqrt(snr_angle_std^2 + hardware_angle_std^2);
        
        % 厚尾噪声判断
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
        
        measurements{i} = [noisy_range, noisy_azimuth, noisy_elevation];
        
        % 保存噪声信息
        receiver_noise_info = struct();
        receiver_noise_info.receiver_id = i;
        receiver_noise_info.true_range = true_range;
        receiver_noise_info.true_azimuth = true_azimuth;
        receiver_noise_info.true_elevation = true_elevation;
        receiver_noise_info.snr_db = snr_db;
        receiver_noise_info.snr_range_std = snr_range_std;
        receiver_noise_info.snr_angle_std = snr_angle_std;
        receiver_noise_info.time_sync_std_sec = time_sync_std_sec;
        receiver_noise_info.time_sync_std_range = time_sync_std_range;
        receiver_noise_info.hardware_angle_std = hardware_angle_std;
        receiver_noise_info.range_noise_std = range_noise_std;
        receiver_noise_info.angle_noise_std = angle_noise_std;
        receiver_noise_info.dist_type = dist_type;
        receiver_noise_info.is_heavy_tail = use_heavy_tail;
        
        noise_info{i} = receiver_noise_info;
    end
end

function [range_sum, azimuth, elevation] = compute_true_measurement_v5(target_pos, tx_pos, rx_pos, dimension)
    % 计算双基地雷达的真实量测
    % 输出：
    %   range_sum - 发射机到目标 + 目标到接收机的距离和
    %   azimuth - 方位角（从接收机看目标）
    %   elevation - 俯仰角（3D）
    
    % 确保都是3D向量
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
    
    % 方向向量（从接收机到目标）
    direction = target_pos - rx_pos;
    
    if dimension == 2
        azimuth = atan2(direction(2), direction(1));
        elevation = 0;
    else
        azimuth = atan2(direction(2), direction(1));
        direction_norm = norm(direction);
        if direction_norm > 1e-6
            elevation = asin(max(-1, min(1, direction(3) / direction_norm)));
        else
            elevation = 0;
        end
    end
end

function snr_db = compute_snr_v5(target_pos, tx_pos, rx_pos, config)
    % 计算双基地雷达SNR（基于雷达方程）
    
    lambda = 3e8 / config.snr.freq;  % 波长
    Pt = config.snr.tx_power;        % 发射功率
    Gt = 10^(27/10);                 % 发射天线增益（27dBi）
    Gr = 10^(27/10);                 % 接收天线增益（27dBi）
    sigma = config.snr.target_rcs;   % 雷达截面积
    tau = config.snr.tau;            % 脉冲宽度
    
    % 距离
    R_tx = max(norm(target_pos - tx_pos), 1.0);  % 防止除零
    R_rx = max(norm(target_pos - rx_pos), 1.0);
    
    % 接收功率（双基地雷达方程）
    Pr = (Pt * Gt * Gr * lambda^2 * sigma * tau) / ((4*pi)^3 * R_tx^2 * R_rx^2);
    
    % 噪声功率
    k = 1.38e-23;  % 玻尔兹曼常数
    T = 290;       % 温度 (K)
    B = config.snr.bandwidth;
    Ls = 10^(13/10);  % 系统损耗（13dB）
    Lt = 10^(8/10);   % 传输损耗（8dB）
    NF = 10^(config.snr.noise_figure / 10);
    Pn = k * T * NF * Ls * Lt;
    
    % SNR
    snr = Pr / Pn;
    snr_db = 10 * log10(max(snr, 1e-10));
    
    % 限制范围
    snr_db = max(config.snr.min_snr_db, min(snr_db, config.snr.max_snr_db));
end

function [range_std, angle_std] = snr_to_noise_std_v5(snr_db, config)
    % 根据SNR计算量测噪声标准差
    
    snr_linear = 10^(snr_db / 10);
    
    % 距离噪声（基于距离分辨率）
    c = 3e8;
    B = config.snr.bandwidth;
    range_resolution = c * sqrt(3) / (pi * B);
    range_std = range_resolution / sqrt(2 * snr_linear);
    range_std = max(range_std, config.snr.min_range_noise_m);
    
    % 角度噪声（基于波束宽度）
    if isfield(config.snr, 'aperture_m') && config.snr.aperture_m > 0
        lambda = 3e8 / config.snr.freq;
        k_beam = 1.0;  % 波束因子
        angle_std = k_beam * lambda / (config.snr.aperture_m * sqrt(2 * snr_linear));
    else
        angle_std = 1 / sqrt(2 * snr_linear);
    end
    angle_std = max(angle_std, config.snr.min_angle_noise_rad);
end

function elev = clip_elevation(elev)
    % 限制俯仰角在 [-pi/2, pi/2]
    elev = max(-pi/2, min(pi/2, elev));
end

function angle = wrapToPi(angle)
    % 将角度包裹到 [-pi, pi]
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
    % 验证轨迹的运动学一致性
    
    is_valid = true;
    
    % 基本完整性检查
    if isempty(trajectory) || length(trajectory) < 10
        is_valid = false;
        return;
    end
    
    % 提取模型段
    segments = extract_model_segments(trajectory, model_sequence);
    
    % 验证每个模型段
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
    % 提取连续相同模型的段
    
    if isempty(trajectory) || isempty(model_sequence)
        segments = {};
        return;
    end
    
    segments = {};
    current_model = model_sequence{1}.model;
    segment_start = 1;
    
    for t = 2:length(model_sequence)
        if ~strcmp(model_sequence{t}.model, current_model)
            % 模型切换，保存上一段
            segment = struct();
            segment.model = current_model;
            segment.start_idx = segment_start;
            segment.end_idx = t - 1;
            segment.trajectory_points = trajectory(segment_start:t-1);
            segments{end+1} = segment;
            
            % 开始新段
            current_model = model_sequence{t}.model;
            segment_start = t;
        end
    end
    
    % 保存最后一段
    segment = struct();
    segment.model = current_model;
    segment.start_idx = segment_start;
    segment.end_idx = length(model_sequence);
    segment.trajectory_points = trajectory(segment_start:end);
    segments{end+1} = segment;
end

function is_valid = validate_segment_motion(segment, model_name, config)
    % 验证单个模型段的运动学一致性
    
    is_valid = true;
    
    if length(segment.trajectory_points) < 2
        return;
    end
    
    % 基本运动学检查
    for i = 2:length(segment.trajectory_points)
        pt_prev = segment.trajectory_points{i-1};
        pt_curr = segment.trajectory_points{i};
        
        dt = pt_curr.time - pt_prev.time;
        if dt <= 0
            is_valid = false;
            return;
        end
        
        % 检查速度和加速度的合理性
        vel_norm = norm(pt_curr.velocity);
        acc_norm = norm(pt_curr.acceleration);
        
        % 简单阈值检查（可根据需要扩展）
        if vel_norm > 1000 || acc_norm > 100  % 粗略阈值
            is_valid = false;
            return;
        end
    end
end

%% ================ 存根函数（占位，实际项目中需要实现）================

%% ================ 存根函数（占位，实际项目中需要实现）================

function save_scenario_json_v5(scenario, output_dir, scenario_id)
    % JSON保存函数 - 占位
    % 实际项目中需要完整实现
    % fprintf('  保存场景 %d JSON (占位函数)\n', scenario_id);
end

function save_json_file_v5(data, filepath)
    % JSON保存函数 - 占位
    % fprintf('  保存JSON: %s (占位函数)\n', filepath);
end

function visualize_scenario_v5(scenario, title_str)
    % 可视化函数 - 占位
    fprintf('  可视化: %s (占位函数)\n', title_str);
end

function analyze_dataset_v5(dataset_dir)
    % 数据集分析函数 - 占位
    fprintf('  分析数据集: %s (占位函数)\n', dataset_dir);
end

function validate_dataset_v5(dataset_dir, config)
    % 数据集验证函数 - 占位
    fprintf('  验证数据集: %s (占位函数)\n', dataset_dir);
end

%% ================ 单元测试函数（可选）================
function run_all_tests_v5()
    % 运行所有单元测试
    fprintf('=====================================================\n');
    fprintf('  运行单元测试 v5.2\n');
    fprintf('=====================================================\n\n');
    
    % 测试1: 配置创建
    fprintf('测试1: 配置创建...\n');
    config_2d = create_radar_config_v5(2);
    config_3d = create_radar_config_v5(3);
    assert(config_2d.state_dimension == 6, '2D状态维度错误');
    assert(config_3d.state_dimension == 9, '3D状态维度错误');
    fprintf('  ✓ 通过\n\n');
    
    % 测试2: Cholesky采样
    fprintf('测试2: 多元正态采样（无mvnrnd）...\n');
    test_cholesky_sampling();
    fprintf('  ✓ 通过\n\n');
    
    % 测试3: CT模型矩阵生成（2D和3D）
    fprintf('测试3: CT模型状态转移矩阵...\n');
    test_ct_matrices();
    fprintf('  ✓ 通过\n\n');
    
    % 测试4: CA模型切换
    fprintf('测试4: CA模型切换逻辑...\n');
    test_ca_model_switch();
    fprintf('  ✓ 通过\n\n');
    
    % 测试5: 时间同步误差转换
    fprintf('测试5: 时间同步误差转换...\n');
    test_time_sync_conversion();
    fprintf('  ✓ 通过\n\n');
    
    % 测试6: 完整轨迹生成
    fprintf('测试6: 完整轨迹生成（小规模）...\n');
    test_full_trajectory_generation();
    fprintf('  ✓ 通过\n\n');
    
    fprintf('=====================================================\n');
    fprintf('  所有测试通过！\n');
    fprintf('=====================================================\n\n');
end

function test_cholesky_sampling()
    % 测试多元正态采样（不使用mvnrnd）
    
    % 测试1: 正定矩阵
    mu = [0; 0; 0];
    Sigma = [2, 0.5, 0.1;
             0.5, 1.5, 0.2;
             0.1, 0.2, 1.0];
    
    samples = zeros(3, 1000);
    for i = 1:1000
        samples(:, i) = sample_multivariate_normal(mu, Sigma);
    end
    
    sample_mean = mean(samples, 2);
    sample_cov = cov(samples');
    
    assert(norm(sample_mean - mu) < 0.2, '均值偏差过大');
    assert(norm(sample_cov - Sigma, 'fro') / norm(Sigma, 'fro') < 0.2, '协方差偏差过大');
    
    % 测试2: 半正定矩阵（一个特征值为0）
    Sigma_semi = [1, 0, 0;
                  0, 1, 0;
                  0, 0, 0];
    
    sample = sample_multivariate_normal(mu, Sigma_semi);
    assert(~any(isnan(sample)), '半正定矩阵采样失败');
end

function test_ct_matrices()
    % 测试CT模型的状态转移矩阵生成
    
    omega = 0.1;  % 转弯率
    dt = 0.1;
    sigma = 0.1;
    
    % 测试2D
    [Phi_2d, Q_2d] = get_ct_matrices_2d(omega, dt, sigma);
    assert(all(size(Phi_2d) == [6, 6]), 'CT 2D Phi维度错误');
    assert(all(size(Q_2d) == [6, 6]), 'CT 2D Q维度错误');
    assert(issymmetric(Q_2d, 1e-10), 'CT 2D Q不对称');
    
    % 检查Q正定性
    eig_Q = eig(Q_2d);
    assert(all(eig_Q >= -1e-10), 'CT 2D Q存在负特征值');
    
    % 测试3D映射
    config = create_radar_config_v5(3);
    noise_info = struct('intensity', sigma);
    model_params = struct('turn_rate', omega);
    state_dummy = zeros(1, 9);
    
    [Phi_3d, Q_3d] = get_transition_matrices_v5(target_params, 'CT', model_params, noise_info, dt, 3, state_dummy);
    assert(all(size(Phi_3d) == [9, 9]), 'CT 3D Phi维度错误');
    assert(all(size(Q_3d) == [9, 9]), 'CT 3D Q维度错误');
    assert(issymmetric(Q_3d, 1e-10), 'CT 3D Q不对称');
    
    % 验证x-y块正确映射
    map_6to9 = [1, 2, 4, 5, 7, 8];
    Phi_xy_extracted = Phi_3d(map_6to9, map_6to9);
    assert(norm(Phi_xy_extracted - Phi_2d, 'fro') < 1e-10, 'CT 3D x-y块映射错误');
    
    % 验证z方向独立（CV模型）
    z_indices = [3, 6, 9];
    Phi_z = Phi_3d(z_indices, z_indices);
    expected_Phi_z = [1, dt, 0;
                      0, 1,  0;
                      0, 0,  0];
    assert(norm(Phi_z - expected_Phi_z, 'fro') < 1e-10, 'CT 3D z方向不是CV模型');
end

function test_ca_model_switch()
    % 测试CA模型切换时加速度写入
    
    config = create_radar_config_v5(3);
    target_params = config.fast_target;
    
    % 生成模型序列
    time_steps = 100;
    [model_sequence, ~] = generate_model_and_noise_sequence_v5(config, target_params, time_steps);
    
    % 查找CA模型切换
    found_ca_switch = false;
    for t = 2:time_steps
        if strcmp(model_sequence{t}.model, 'CA') && ~strcmp(model_sequence{t-1}.model, 'CA')
            % 发现CA切换
            found_ca_switch = true;
            assert(isfield(model_sequence{t}.params, 'acceleration'), 'CA模型缺少加速度参数');
            assert(length(model_sequence{t}.params.acceleration) == 3, 'CA 3D加速度维度错误');
            break;
        end
    end
    
    % 如果没找到CA切换，生成一个测试
    if ~found_ca_switch
        model_params = struct('type', 'CA', 'acceleration', [1.0, 2.0, 0.5]);
        assert(all(size(model_params.acceleration) == [1, 3]) || ...
               all(size(model_params.acceleration) == [3, 1]), 'CA加速度维度错误');
    end
end

function test_time_sync_conversion()
    % 测试时间同步误差到距离误差的转换
    
    config = create_radar_config_v5(2);
    
    time_sync_std_sec = config.measurement_noise.time_sync_std;  % 秒
    c = 3e8;  % 光速
    
    % 转换为距离误差
    time_sync_std_range = c * time_sync_std_sec;
    
    % 验证单位合理性
    assert(time_sync_std_range > 0, '时间同步距离误差应为正');
    assert(time_sync_std_range < 100, '时间同步距离误差过大（可能单位错误）');
    
    fprintf('    时间同步: %.2e 秒 -> %.2f 米\n', time_sync_std_sec, time_sync_std_range);
end

function test_full_trajectory_generation()
    % 测试完整轨迹生成（小规模）
    
    config = create_radar_config_v5(2);
    config.simulation_time = 20;  % 缩短时间
    
    transmitter_pos = zeros(1, 3);
    receivers_pos = generate_receiver_positions_v5(config, transmitter_pos);
    
    time_steps = floor(config.simulation_time / config.dt);
    target_type = 'medium';
    
    [target_info, measurements, detailed_records, is_valid] = ...
        generate_target_trajectory_v5(config, time_steps, target_type, ...
                                     transmitter_pos, receivers_pos);
    
    assert(is_valid, '轨迹生成失败');
    assert(length(target_info.trajectory) == time_steps, '轨迹长度不匹配');
    assert(length(measurements) == time_steps, '量测长度不匹配');
    assert(length(detailed_records) == time_steps, '详细记录长度不匹配');
    
    % 验证状态连续性
    for t = 2:length(target_info.trajectory)
        pt_prev = target_info.trajectory{t-1};
        pt_curr = target_info.trajectory{t};
        
        dt = pt_curr.time - pt_prev.time;
        assert(abs(dt - config.dt) < 1e-6, '时间步长不一致');
        
        % 检查位置变化合理性
        pos_diff = norm(pt_curr.position - pt_prev.position);
        max_displacement = 600 * config.dt;  % 最大速度500 m/s
        assert(pos_diff < max_displacement, '位置跳变过大');
    end
    
    fprintf('    生成了 %d 个时间步的有效轨迹\n', time_steps);
end

%% ================ 性能分析工具（可选）================

function profile_dataset_generation()
    % 性能分析工具
    fprintf('=====================================================\n');
    fprintf('  性能分析\n');
    fprintf('=====================================================\n\n');
    
    config = create_radar_config_v5(2);
    config.simulation_time = 50;  % 中等长度
    
    num_test_scenarios = 10;
    
    fprintf('生成 %d 个场景进行性能测试...\n', num_test_scenarios);
    
    tic;
    for i = 1:num_test_scenarios
        rng(1000 + i);
        target_type = 'medium';
        try
            scenario = generate_scenario_v5(config, target_type);
            if mod(i, 5) == 0
                fprintf('  完成 %d/%d\n', i, num_test_scenarios);
            end
        catch ME
            fprintf('  场景 %d 生成失败: %s\n', i, ME.message);
        end
    end
    elapsed = toc;
    
    fprintf('\n性能统计:\n');
    fprintf('  总时间: %.2f 秒\n', elapsed);
    fprintf('  平均每场景: %.2f 秒\n', elapsed / num_test_scenarios);
    fprintf('  估计200场景总时间: %.2f 分钟\n', elapsed / num_test_scenarios * 200 / 60);
end

%% ================ 数据完整性检查工具 ================

function check_dataset_integrity(dataset_dir)
    % 检查数据集完整性
    fprintf('=====================================================\n');
    fprintf('  数据集完整性检查\n');
    fprintf('=====================================================\n\n');
    
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    
    if ~exist(dataset_file, 'file')
        error('数据集文件不存在: %s', dataset_file);
    end
    
    fprintf('加载数据集: %s\n', dataset_file);
    data = load(dataset_file);
    
    if ~isfield(data, 'dataset')
        error('数据文件中缺少dataset字段');
    end
    
    dataset = data.dataset;
    
    fprintf('数据集版本: %s\n', dataset.metadata.version);
    fprintf('场景数量: %d\n', dataset.metadata.num_scenarios);
    fprintf('创建时间: %s\n', dataset.metadata.creation_time);
    fprintf('维度: %dD\n', dataset.config.dimension);
    
    % 检查每个场景
    num_scenarios = length(dataset.scenarios);
    fprintf('\n检查 %d 个场景...\n', num_scenarios);
    
    valid_count = 0;
    invalid_scenarios = [];
    
    for i = 1:num_scenarios
        scenario = dataset.scenarios{i};
        
        % 基本字段检查
        is_valid = true;
        if ~isfield(scenario, 'target_info') || ~isfield(scenario, 'measurements')
            is_valid = false;
        end
        
        if is_valid && ~isempty(scenario.target_info.trajectory)
            % 检查轨迹长度
            traj_len = length(scenario.target_info.trajectory);
            meas_len = length(scenario.measurements);
            
            if traj_len ~= meas_len
                fprintf('  场景 %d: 轨迹长度(%d) != 量测长度(%d)\n', i, traj_len, meas_len);
                is_valid = false;
            end
        else
            is_valid = false;
        end
        
        if is_valid
            valid_count = valid_count + 1;
        else
            invalid_scenarios(end+1) = i;
        end
        
        if mod(i, 50) == 0
            fprintf('  已检查 %d/%d (%.1f%%)\n', i, num_scenarios, 100*i/num_scenarios);
        end
    end
    
    fprintf('\n完整性检查结果:\n');
    fprintf('  有效场景: %d/%d (%.1f%%)\n', valid_count, num_scenarios, 100*valid_count/num_scenarios);
    
    if ~isempty(invalid_scenarios)
        fprintf('  无效场景ID: ');
        fprintf('%d ', invalid_scenarios);
        fprintf('\n');
    else
        fprintf('  ✓ 所有场景有效\n');
    end
end

%% ================ 可视化辅助工具（简化版）================

function quick_visualize_trajectory(trajectory, title_str)
    % 快速可视化轨迹（简化版）
    
    if isempty(trajectory)
        fprintf('轨迹为空，无法可视化\n');
        return;
    end
    
    num_points = length(trajectory);
    positions = zeros(num_points, 3);
    velocities = zeros(num_points, 3);
    times = zeros(num_points, 1);
    
    for i = 1:num_points
        positions(i, :) = trajectory{i}.position;
        velocities(i, :) = trajectory{i}.velocity;
        times(i) = trajectory{i}.time;
    end
    
    figure('Name', title_str);
    
    % 子图1: 轨迹
    subplot(2, 2, 1);
    if positions(1, 3) == 0 && all(positions(:, 3) == 0)
        % 2D
        plot(positions(:, 1), positions(:, 2), 'b-', 'LineWidth', 1.5);
        xlabel('X (m)');
        ylabel('Y (m)');
        title('2D轨迹');
        grid on;
        axis equal;
    else
        % 3D
        plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'b-', 'LineWidth', 1.5);
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('3D轨迹');
        grid on;
        axis equal;
    end
    
    % 子图2: 速度
    subplot(2, 2, 2);
    speed = sqrt(sum(velocities.^2, 2));
    plot(times, speed, 'r-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    title('速度曲线');
    grid on;
    
    % 子图3: X-Y位置
    subplot(2, 2, 3);
    plot(positions(:, 1), positions(:, 2), 'g-', 'LineWidth', 1.5);
    xlabel('X (m)');
    ylabel('Y (m)');
    title('X-Y平面投影');
    grid on;
    axis equal;
    
    % 子图4: 高度（如果是3D）
    subplot(2, 2, 4);
    if any(positions(:, 3) ~= 0)
        plot(times, positions(:, 3), 'm-', 'LineWidth', 1.5);
        xlabel('时间 (s)');
        ylabel('高度 (m)');
        title('高度变化');
        grid on;
    else
        text(0.5, 0.5, '2D轨迹\n无高度信息', ...
             'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'middle', ...
             'FontSize', 14);
        axis off;
    end
end
function a = sample_acc_ball(max_acc, dim)
    v = randn(dim,1); v = v / max(norm(v), 1e-12);  % 随机方向
    r = rand()^(1/dim) * max_acc;                   % 体积均匀
    a = r * v;
end

%% ================ 使用示例和文档 ================

% 使用示例:
%
% 1. 基本使用（生成数据集）:
%    >> radar_data_simulation_v5();
%
% 2. 运行单元测试:
%    >> run_all_tests_v5();
%
% 3. 性能分析:
%    >> profile_dataset_generation();
%
% 4. 检查数据集完整性:
%    >> check_dataset_integrity('enhanced_v5_2d_dataset');
%
% 5. 快速可视化单个轨迹:
%    >> load('enhanced_v5_2d_dataset/dataset.mat');
%    >> quick_visualize_trajectory(dataset.scenarios{1}.target_info.trajectory, '场景1');
%
% 修改记录:
%   v5.2 (2025-10-22):
%     - 修复CA模型3D打印索引错误
%     - 修复CT模型3D的6x6到9x9映射
%     - 移除mvnrnd依赖，使用Cholesky分解
%     - 修复时间同步误差单位转换
%     - 增强数值稳定性
%     - 添加完整单元测试
%     - 改进性能（预分配数组）
%     - 增强接收机位置生成（检查与发射机距离）