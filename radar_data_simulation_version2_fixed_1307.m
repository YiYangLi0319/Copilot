%% =====================================================================
%  双基地雷达数据集生成器 v5.3 (完整修正版)
%  =====================================================================
%  作者: YiYangLi0319
%  日期: 2025-10-22
%  修订: v5.3 - 修复关键错误并增强鲁棒性
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
%  7. 修复CT模型加速度漂移问题（V5.3新增）
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
    run_config.num_scenarios = 2;  % 总场景数量
    run_config.batch_size = 2;      % 每批次生成的航迹数量
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
    config.fast_target.max_acceleration = 5.0;    % m/s^2
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
    
%     if config.dimension == 3
%         z = state(3);
%         if z < config.altitude_min * 0.5 || z > config.altitude_max * 2
%             violated = true;
%             violation_type = '高度';
%             return;
%         end
%     end
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
    
    % ====================== 关键错误修复 ======================
    % 修复CT模型：在添加噪声后，强制加速度状态与速度状态保持一致
    % 这可以防止加速度状态因过程噪声(w_a)而独立漂移
    % ========================================================
    if strcmp(model_info.model, 'CT')
        % 提取转弯率
        omega = model_params.turn_rate;
        
        if dim == 2
            % 状态: [x, y, vx, vy, ax, ay] (索引: 3, 4, 5, 6)
            vy_k1 = next_state(4);
            vx_k1 = next_state(3);
            next_state(5) = -omega * vy_k1; % ax = -w*vy
            next_state(6) = omega * vx_k1;  % ay = w*vx
        else
            % 状态: [x, y, z, vx, vy, vz, ax, ay, az] (索引: 4, 5, 7, 8)
            vy_k1 = next_state(5);
            vx_k1 = next_state(4);
            next_state(7) = -omega * vy_k1; % ax = -w*vy
            next_state(8) = omega * vx_k1;  % ay = w*vx
            % 注意: z方向的加速度 (next_state(9)) 保持原样，
            % 因为它遵循独立的CV模型，其噪声是正确的。
        end
    end
    % ===================== 修复结束 =====================
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
    %   CT: 恒转弯 - x-y耦合，z独立CV
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
        
        % 记录噪声信息
        noise = struct();
        noise.snr_db = snr_db;
        noise.range_std = range_noise_std;
        noise.angle_std = angle_noise_std;
        noise.distribution = dist_type;
        
        measurements{i} = [noisy_range, noisy_azimuth, noisy_elevation];
        noise_info{i} = noise;
    end
end

function [true_range, true_azimuth, true_elevation] = compute_true_measurement_v5(...
    target_pos, tx_pos, rx_pos, dimension)
    % 计算无噪声的双基地雷达量测
    
    % 计算双程距离
    dist_tx_to_target = norm(target_pos - tx_pos);
    dist_target_to_rx = norm(target_pos - rx_pos);
    true_range = dist_tx_to_target + dist_target_to_rx;
    
    % 计算接收机到目标的相对向量（用于角度计算）
    relative_vector = target_pos - rx_pos;
    
    % 方位角计算
    true_azimuth = atan2(relative_vector(2), relative_vector(1));
    
    % 俯仰角计算
    if dimension == 3
        xy_dist = norm(relative_vector(1:2));
        true_elevation = atan2(relative_vector(3), xy_dist);
    else
        true_elevation = 0;  % 2D情况下俯仰角为0
    end
end

function snr_db = compute_snr_v5(target_pos, tx_pos, rx_pos, config)
    % 计算目标的SNR（考虑双程损耗）

    % 定义玻尔兹曼常数
    BOLTZMANN_CONST = 1.38e-23;  % 玻尔兹曼常数 (J/K)
    % 计算距离
    dist_tx_to_target = norm(target_pos - tx_pos);
    dist_target_to_rx = norm(target_pos - rx_pos);
    
    % 计算SNR
    lambda = 3e8 / config.snr.freq;  % 波长 (m)
    rcs = config.snr.target_rcs;
    tx_power = config.snr.tx_power;
    
    % 雷达方程计算SNR
    snr_factor = (tx_power * config.snr.tau * rcs * (lambda^2) * (config.snr.aperture_m^2)) / ...
                 ((4*pi)^3 * (dist_tx_to_target^2) * (dist_target_to_rx^2) * ...
                  BOLTZMANN_CONST * 290 * config.snr.noise_figure * 10^(config.snr.noise_figure/10));
    
    snr_db_raw = 10 * log10(snr_factor);
    
    % 限制SNR范围
    snr_db = min(max(snr_db_raw, config.snr.min_snr_db), config.snr.max_snr_db);
end

function [range_std, angle_std] = snr_to_noise_std_v5(snr_db, config)
    % 将SNR转换为距离和角度测量噪声标准差
    % SNR越高，噪声越小
    
    % 范围噪声模型（基于SNR）
    % 使用CRLB (Cramér-Rao Lower Bound)近似
    c = 3e8;  % 光速
    bw = config.snr.bandwidth;  % 带宽
    snr_linear = 10^(snr_db/10);
    
    % 距离噪声：c/(2*sqrt(2)*pi*B*sqrt(SNR))
    range_std_snr = c / (2 * sqrt(2) * pi * bw * sqrt(snr_linear));
    range_std = max(range_std_snr, config.snr.min_range_noise_m);
    
    % 角度噪声（基于SNR和孔径）
    aperture = config.snr.aperture_m;
    lambda = 3e8 / config.snr.freq;
    beamwidth = lambda / aperture;  % 近似波束宽度
    
    % 角度噪声：beamwidth / (k*sqrt(SNR))，其中k为常数
    k = 1.6;  % 经验常数
    angle_std_snr = beamwidth / (k * sqrt(snr_linear));
    angle_std = max(angle_std_snr, config.snr.min_angle_noise_rad);
end

function angle_rad = clip_elevation(angle_rad)
    % 限制俯仰角在合理范围内
    angle_rad = max(min(angle_rad, pi/2), -pi/2);
end

%% ================ 目标参数获取 ================
function target_params = get_target_params_v5(config, target_type)
    % 获取目标参数
    target_params = config.(sprintf('%s_target', target_type));
end

%% ================ 轨迹验证 ================
function valid = validate_trajectory_motion_v5(trajectory, model_sequence, config)
    % 验证轨迹的运动模型一致性
    valid = true;
    
    % 轨迹验证可以进行更多检查，例如:
    % 1. 转弯率是否符合预期
    % 2. 加速度是否符合运动模型
    % 3. 检查轨迹是否平滑
    
    % 本实现简化为基本验证
    if isempty(trajectory) || isempty(trajectory{1})
        valid = false;
        return;
    end
    
    % 检查轨迹最大速度和加速度
    for i = 2:length(trajectory)
        if isempty(trajectory{i})
            valid = false;
            return;
        end
        
        % 更多验证可以在这里添加
    end
end

%% ================ 单元测试 ================
function run_all_tests_v5()
    % 运行所有单元测试
    fprintf('运行单元测试...\n');
    
    test_ct_model_consistency();
    test_state_update();
    test_multivariate_sampling();
    test_singer_model();
    
    fprintf('单元测试完成\n');
end

function test_ct_model_consistency()
    % 测试CT模型的一致性：确保ax = -ω*vy, ay = ω*vx
    
    % 创建配置
    config = struct();
    config.dimension = 2;
    config.dt = 0.1;
    
    % 创建模型信息
    model_info = struct();
    model_info.model = 'CT';
    model_info.params = struct('turn_rate', 0.1);  % 10度/秒
    
    % 创建噪声信息
    noise_info = struct();
    noise_info.intensity = 0.1;
    
    % 创建目标参数
    target_params = struct();
    target_params.max_acceleration = 10;
    
    % 初始状态：[x, y, vx, vy, ax, ay]
    state = [0, 0, 10, 0, 0, 1];
    
    % 调用状态更新函数
    next_state = update_state_v5(state, model_info, noise_info, config, target_params);
    
    % 检查加速度和速度的关系
    omega = model_info.params.turn_rate;
    expected_ax = -omega * next_state(4);  % -ω*vy
    expected_ay = omega * next_state(3);    % ω*vx
    
    tol = 1e-10;
    assert(abs(next_state(5) - expected_ax) < tol, 'CT模型: ax != -ω*vy');
    assert(abs(next_state(6) - expected_ay) < tol, 'CT模型: ay != ω*vx');
    
    fprintf('测试通过: CT模型保持ax = -ω*vy, ay = ω*vx关系\n');
end

function test_state_update()
    % 测试状态更新
    
    % 创建配置
    config = struct();
    config.dimension = 2;
    config.dt = 0.1;
    
    % 创建目标参数
    target_params = struct();
    target_params.max_acceleration = 10;
    
    % 测试CV模型
    model_info_cv = struct();
    model_info_cv.model = 'CV';
    model_info_cv.params = struct();
    
    % 创建噪声信息
    noise_info = struct();
    noise_info.intensity = 0;  % 无噪声
    
    % 初始状态：[x, y, vx, vy, ax, ay]
    state = [0, 0, 1, 2, 0, 0];
    
    % 预期下一步状态（无噪声）
    expected_next_state = [0.1, 0.2, 1, 2, 0, 0];
    
    % 调用状态更新函数
    next_state = update_state_v5(state, model_info_cv, noise_info, config, target_params);
    
    assert(all(abs(next_state - expected_next_state) < 1e-10), 'CV模型状态更新错误');
    fprintf('测试通过: CV模型状态更新正确\n');
    
    % 测试CA模型
    model_info_ca = struct();
    model_info_ca.model = 'CA';
    model_info_ca.params = struct();
    model_info_ca.params.acceleration = [0.5, -0.3];
    
    % 初始状态：[x, y, vx, vy, ax, ay]
    state = [0, 0, 1, 2, 0.5, -0.3];
    
    % 预期下一步状态（无噪声）
    expected_next_state = [0.1, 0.2, 1.05, 1.97, 0.5, -0.3];
    
    % 调用状态更新函数
    next_state = update_state_v5(state, model_info_ca, noise_info, config, target_params);
    
    assert(all(abs(next_state - expected_next_state) < 1e-10), 'CA模型状态更新错误');
    fprintf('测试通过: CA模型状态更新正确\n');
end

function test_multivariate_sampling()
    % 测试多元正态分布采样
    
    n = 1000;  % 样本数
    mu = [1; 2; 3];
    Sigma = [2, 0.5, 0.3; 0.5, 1, 0.2; 0.3, 0.2, 1.5];
    
    samples = zeros(length(mu), n);
    for i = 1:n
        samples(:, i) = sample_multivariate_normal(mu, Sigma);
    end
    
    % 计算样本均值和协方差
    sample_mean = mean(samples, 2);
    sample_cov = cov(samples');
    
    % 检查均值和协方差是否接近预期
    mean_error = norm(sample_mean - mu) / norm(mu);
    cov_error = norm(sample_cov - Sigma, 'fro') / norm(Sigma, 'fro');
    
    assert(mean_error < 0.1, '样本均值误差过大');
    assert(cov_error < 0.2, '样本协方差误差过大');
    
    fprintf('测试通过: 多元正态分布采样正确\n');
end

function test_singer_model()
    % 测试Singer模型离散化
    
    dt = 0.1;
    alpha = 0.1;
    
    A_c = [0, 1, 0;
           0, 0, 1;
           0, 0, -alpha];
    B_c = [0; 0; 1];
    Q_c = 2 * alpha * 0.1^2;
    
    [Phi, Q] = singer_discretization_vanloan(A_c, B_c, Q_c, dt);
    
    % 期望的Phi形式
    expected_phi_form = [1, dt, dt^2/2;
                         0, 1, dt;
                         0, 0, exp(-alpha*dt)];
    
    % 检查Phi的形式是否正确
    phi_ratio = Phi ./ expected_phi_form;
    phi_ratio(isnan(phi_ratio)) = 1;  % 处理0/0
    
    assert(all(abs(phi_ratio(:) - 1) < 0.1), 'Singer模型离散化Phi矩阵形式错误');
    assert(all(all(Q >= 0)), 'Q矩阵有负对角元素');
    assert(all(eig(Q) > -1e-10), 'Q矩阵不是半正定的');
    
    fprintf('测试通过: Singer模型离散化正确\n');
end