%% =====================================================================
%  双基地雷达数据集生成器 v5.4 (增强CRLB计算)
%  =====================================================================
%  原作者: YiYangLi0319
%  修改者: Copilot AI Assistant
%  日期: 2025-10-27
%  
%  核心修改：
%  1. CA模型：加速度方向有50%概率与速度方向一致，50%概率随机选择
%  2. 批处理存储：每20条轨迹保存一次，减少内存占用
%  3. 模型切换平滑过渡：添加过渡期，使模型参数平滑变化
%  4. 增强可视化：显示轨迹、速度、加速度和模型切换
%  5. 完善测试方法：各个模型单独测试 + 组合场景验证
%  6. **新增CRLB计算**：计算每个目标点和整条轨迹的理论定位精度
%  =====================================================================

clear all; close all; clc;

%% ================ 主程序调用 ================
radar_data_simulation_v54();

% 取消下面的注释来运行单元测试
% run_all_tests_v53();

%% ================ 主程序入口 ================
function radar_data_simulation_v54()
    fprintf('=====================================================\n');
    fprintf('  双基地雷达数据集生成器 v5.4 (增强CRLB计算)\n');
    fprintf('  原作者: YiYangLi\n');
    fprintf('  修改者: Copilot AI Assistant\n');
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
    run_config.num_scenarios = 20;  % 总场景数量
    run_config.batch_size = 20;      % 批处理大小
    run_config.run_validation = true;
    
    % 生成2D数据集
    fprintf('--- 生成2D数据集 ---\n');
    config_2d = create_radar_config_v5(2);
    dataset_2d = generate_dataset_v54(config_2d, run_config.num_scenarios, ...
        'enhanced_2d_dataset_v2', GLOBAL_SEED, run_config.batch_size);
    
    % 生成3D数据集
    fprintf('\n--- 生成3D数据集 ---\n');
    config_3d = create_radar_config_v5(3);
    dataset_3d = generate_dataset_v54(config_3d, run_config.num_scenarios, ...
        'enhanced_3d_dataset_v2', GLOBAL_SEED + 100000, run_config.batch_size);
    
    % 演示可视化
    if run_config.run_demo
        fprintf('\n--- 可视化演示 ---\n');
        % 从保存的文件中加载示例场景进行可视化
        if exist('enhanced_2d_dataset_v2/scenario_0000.mat', 'file')
            demo_2d = load(fullfile('enhanced_2d_dataset_v2', 'scenario_0000.mat'));
            visualize_scenario_v54(demo_2d.scenario, '2D场景示例 v5.4');
        else
            fprintf('2D场景文件不存在，跳过可视化\n');
        end
        
        if exist('enhanced_3d_dataset_v2/scenario_0000.mat', 'file')
            demo_3d = load(fullfile('enhanced_3d_dataset_v2', 'scenario_0000.mat'));
            visualize_scenario_v54(demo_3d.scenario, '3D场景示例 v5.4');
        else
            fprintf('3D场景文件不存在，跳过可视化\n');
        end
    end
    
    % 数据集统计分析
    fprintf('\n=====================================================\n');
    fprintf('  数据集统计分析\n');
    fprintf('=====================================================\n');
    analyze_dataset_v5('enhanced_2d_dataset_v2');
    analyze_dataset_v5('enhanced_3d_dataset_v2');
    
    % 数据验证
    if run_config.run_validation
        fprintf('\n--- 数据验证 ---\n');
        validate_dataset_v5('enhanced_2d_dataset_v2', config_2d);
        validate_dataset_v5('enhanced_3d_dataset_v2', config_3d);
    end
    
    fprintf('\n=====================================================\n');
    fprintf('  数据集生成完成！\n');
    fprintf('  2D数据集: enhanced_2d_dataset_v2/\n');
    fprintf('  3D数据集: enhanced_3d_dataset_v2/\n');
    fprintf('=====================================================\n\n');
end

%% ================ 配置函数 ================
function config = create_radar_config_v5(dimension)
    % 创建雷达系统配置 v5.4
    
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
    
    % ===== 平滑过渡配置 =====
    config.model_transition = struct();
    config.model_transition.enable = true;
    config.model_transition.duration = 2.0;  % 过渡时间（秒）
    
    % ===== CA模型配置 =====
    config.ca_model = struct();
    config.ca_model.aligned_probability = 0.5;  % 加速度与速度方向一致的概率
end

%% ================ 数据集生成（批处理版，增强CRLB）================
function dataset_info = generate_dataset_v54(config, num_scenarios, output_dir, base_seed, batch_size)
    if nargin < 3, output_dir = 'enhanced_v54_dataset'; end
    if nargin < 4, base_seed = 42; end
    if nargin < 5, batch_size = 20; end
    
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    % 创建空白数据集信息结构体
    dataset_info = struct();
    dataset_info.config = config;
    dataset_info.metadata = struct();
    dataset_info.metadata.creation_time = datestr(now);
    dataset_info.metadata.num_scenarios = num_scenarios;
    dataset_info.metadata.version = '5.4';
    dataset_info.metadata.base_seed = base_seed;
    dataset_info.metadata.batch_size = batch_size;
    
    fprintf('批处理生成 %d 个场景（每批 %d 个）...\n', num_scenarios, batch_size);
    
    % 计算需要的批次数
    num_batches = ceil(num_scenarios / batch_size);
    scenarios_processed = 0;
    
    % 批次处理循环
    for batch = 1:num_batches
        fprintf('\n=== 处理批次 %d/%d ===\n', batch, num_batches);
        
        % 确定当前批次的场景数量
        batch_scenarios = min(batch_size, num_scenarios - scenarios_processed);
        
        % 当前批次的场景
        batch_data = struct();
        batch_data.scenarios = cell(1, batch_scenarios);
        
        for i = 1:batch_scenarios
            scenario_idx = scenarios_processed + i;
            if mod(i, 5) == 0 || i == 1
                fprintf('  批次内进度: %d/%d (总体: %d/%d，%.1f%%)\n', ...
                    i, batch_scenarios, scenario_idx, num_scenarios, ...
                    100*scenario_idx/num_scenarios);
            end
            
            scenario_seed = base_seed + scenario_idx * 1000;
            rng(scenario_seed, 'twister');
            
            target_type = config.target_types{randi(length(config.target_types))};
            
            scenario = generate_scenario_v54(config, target_type);
            scenario.scenario_id = scenario_idx - 1;
            scenario.seed = scenario_seed;
            
            % 保存单个场景文件（.mat和.json）
            scenario_file = fullfile(output_dir, sprintf('scenario_%04d.mat', scenario_idx - 1));
            save(scenario_file, 'scenario');
            
            % 保存到批次数据中（用于分析）
            batch_data.scenarios{i} = scenario;
        end
        
        % 批次完成后保存批次数据文件
        batch_file = fullfile(output_dir, sprintf('batch_%02d.mat', batch));
        save(batch_file, 'batch_data');
        
        fprintf('  批次 %d 完成，已保存 %d 个场景。\n', batch, batch_scenarios);
        scenarios_processed = scenarios_processed + batch_scenarios;
        
        % 显式清理内存
        clear batch_data;
        clear scenario;
        fprintf('  已清理内存，准备下一批次...\n');
    end
    
    % 保存数据集信息
    dataset_info_file = fullfile(output_dir, 'dataset_info.json');
    save_json_file_v5(dataset_info, dataset_info_file);
    fprintf('数据集信息已保存: %s\n', dataset_info_file);
    
    % 在主目录创建索引文件
    create_dataset_index(output_dir, num_scenarios);
end

function create_dataset_index(output_dir, num_scenarios)
    % 创建数据集索引文件，便于快速访问
    index = struct();
    index.num_scenarios = num_scenarios;
    index.scenario_files = cell(1, num_scenarios);
    
    for i = 0:num_scenarios-1
        mat_file = sprintf('scenario_%04d.mat', i);
        json_file = sprintf('scenario_%04d.json', i);
        
        index.scenario_files{i+1} = struct(...
            'id', i, ...
            'mat_file', mat_file, ...
            'json_file', json_file ...
        );
    end
    
    % 保存索引文件
    index_file = fullfile(output_dir, 'index.json');
    save_json_file_v5(index, index_file);
    fprintf('已创建数据集索引文件: %s\n', index_file);
end

function scenario = generate_scenario_v54(config, target_type)
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
            generate_target_trajectory_v54(config, time_steps, target_type, ...
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

%% ================ 轨迹生成（增强CRLB）================
function [target_info, measurements, detailed_records, is_valid] = ...
    generate_target_trajectory_v54(config, time_steps, target_type, tx, rxs)
    
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
    [model_sequence, noise_schedule] = generate_model_and_noise_sequence_v53(...
        config, target_params, time_steps, state);
    
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
    
    previous_model = '';
    transition_active = false;
    transition_data = struct();
    
    % **新增：CRLB累积变量**
    crlb_sum_position = zeros(1, config.dimension);  % 位置CRLB平方和
    crlb_count = 0;  % 有效CRLB计数
    
    % 轨迹生成主循环
    for t = 1:time_steps
        current_time = (t-1) * config.dt;
        current_model = model_sequence{t}.model;
        
        % 检测模型切换
        if t > 1 && ~strcmp(current_model, previous_model)
            if strcmp(current_model, 'CA') && config.model_transition.enable
                if config.dimension == 2
                    fprintf('    时刻%.1fs: 平滑切换到CA模型，目标加速度为 [%.2f, %.2f]\n', ...
                        current_time, model_sequence{t}.params.acceleration(1), ...
                        model_sequence{t}.params.acceleration(2));
                    
                    transition_active = true;
                    transition_data = struct();
                    transition_data.start_time = current_time;
                    transition_data.duration = config.model_transition.duration;
                    transition_data.start_acc = state(5:6);
                    transition_data.target_acc = model_sequence{t}.params.acceleration;
                    
                    if isfield(model_sequence{t}.params, 'aligned_with_velocity')
                        transition_data.aligned_with_velocity = model_sequence{t}.params.aligned_with_velocity;
                    else
                        transition_data.aligned_with_velocity = false;
                    end
                    
                    model_sequence{t}.transition = transition_data;
                else
                    target_acc = model_sequence{t}.params.acceleration;
                    
                    if length(target_acc) ~= 3
                        fprintf('    警告: CA模型目标加速度维度错误 (%d)，修复为3D向量\n', ...
                            length(target_acc));
                        if length(target_acc) < 3
                            fixed_acc = zeros(1, 3);
                            fixed_acc(1:length(target_acc)) = target_acc;
                            target_acc = fixed_acc;
                        else
                            target_acc = target_acc(1:3);
                        end
                        model_sequence{t}.params.acceleration = target_acc;
                    end
                    
                    fprintf('    时刻%.1fs: 平滑切换到CA模型，目标加速度为 [%.2f, %.2f, %.2f]\n', ...
                        current_time, target_acc(1), target_acc(2), target_acc(3));
                    
                    current_acc = state(7:9);
                    if length(current_acc) ~= 3
                        fprintf('    警告: 当前状态加速度维度错误 (%d)，修复为3D向量\n', ...
                            length(current_acc));
                        current_acc = zeros(1, 3);
                        for i = 1:min(3, length(state) - 6)
                            current_acc(i) = state(6 + i);
                        end
                    end
                    
                    transition_active = true;
                    transition_data = struct();
                    transition_data.start_time = current_time;
                    transition_data.duration = config.model_transition.duration;
                    transition_data.start_acc = current_acc;
                    transition_data.target_acc = target_acc;
                    
                    if isfield(model_sequence{t}.params, 'aligned_with_velocity')
                        transition_data.aligned_with_velocity = model_sequence{t}.params.aligned_with_velocity;
                    else
                        transition_data.aligned_with_velocity = false;
                    end
                    
                    model_sequence{t}.transition = transition_data;
                end
            end
        end
        previous_model = current_model;
        
        % 应用过渡
        if transition_active
            elapsed = current_time - transition_data.start_time;
            
            if elapsed <= transition_data.duration
                progress = elapsed / transition_data.duration;
                
                if length(transition_data.start_acc) ~= length(transition_data.target_acc)
                    fprintf('    警告: 过渡加速度维度不匹配! 调整为相同维度\n');
                    dim = max(length(transition_data.start_acc), length(transition_data.target_acc));
                    new_start = zeros(1, dim);
                    new_target = zeros(1, dim);
                    new_start(1:length(transition_data.start_acc)) = transition_data.start_acc;
                    new_target(1:length(transition_data.target_acc)) = transition_data.target_acc;
                    transition_data.start_acc = new_start;
                    transition_data.target_acc = new_target;
                end
                
                interpolated_acc = transition_data.start_acc + ...
                    progress * (transition_data.target_acc - transition_data.start_acc);
                
                if config.dimension == 2
                    state(5:6) = interpolated_acc;
                else
                    if length(interpolated_acc) ~= 3
                        fprintf('    警告: 插值加速度维度错误，调整为3D向量\n');
                        if length(interpolated_acc) < 3
                            fixed_acc = zeros(1, 3);
                            fixed_acc(1:length(interpolated_acc)) = interpolated_acc;
                            interpolated_acc = fixed_acc;
                        else
                            interpolated_acc = interpolated_acc(1:3);
                        end
                    end
                    
                    state(7:9) = interpolated_acc;
                end
                
                model_sequence{t}.transition = transition_data;
                model_sequence{t}.transition.progress = progress;
            else
                transition_active = false;
            end
        end
        
        % 构造当前状态的位置、速度和加速度
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
        
        if isfield(model_sequence{t}, 'transition')
            traj_point.transition_info = model_sequence{t}.transition;
        end
        
        % 生成量测
        [target_measurements, measurement_noise_info] = generate_measurements_v5(...
            current_pos, tx, rxs, config);
        
        true_measurements = {};
        for i = 1:length(rxs)
            [true_range, true_azimuth, true_elevation] = compute_true_measurement_v5(...
                current_pos, tx, rxs{i}, config.dimension);
            true_measurements{end+1} = [true_range, true_azimuth, true_elevation];
        end
        
        % **新增：计算当前点的CRLB**
        [crlb_pos, crlb_details] = compute_crlb_at_point(current_pos, tx, rxs, ...
            measurement_noise_info, config);
        
        % 累积CRLB平方和
        if ~any(isnan(crlb_pos)) && ~any(isinf(crlb_pos))
            crlb_sum_position = crlb_sum_position + crlb_pos.^2;
            crlb_count = crlb_count + 1;
        end
        
        % 将CRLB信息添加到轨迹点
        traj_point.crlb_position = crlb_pos;
        traj_point.crlb_details = crlb_details;
        
        target_info.trajectory{end+1} = traj_point;
        
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
        detailed_record.crlb_position = crlb_pos;
        detailed_record.crlb_details = crlb_details;
        
        if isfield(model_sequence{t}, 'transition')
            detailed_record.transition_info = model_sequence{t}.transition;
        end
        
        detailed_records{end+1} = detailed_record;
        
        % 状态更新
        if t < time_steps
            state = update_state_v5(state, model_sequence{t}, ...
                noise_schedule{t}, config, target_params);
        end
    end
    
    % **新增：计算轨迹的平均CRLB（理论精度）**
    if crlb_count > 0
        % 计算均方根CRLB
        mean_crlb_position = sqrt(crlb_sum_position / crlb_count);
        target_info.theoretical_accuracy = struct();
        target_info.theoretical_accuracy.mean_crlb_position = mean_crlb_position;
        target_info.theoretical_accuracy.crlb_count = crlb_count;
        
        if config.dimension == 2
            target_info.theoretical_accuracy.horizontal_accuracy = norm(mean_crlb_position(1:2));
            fprintf('    轨迹CRLB: 水平精度=%.2fm (x=%.2fm, y=%.2fm)\n', ...
                target_info.theoretical_accuracy.horizontal_accuracy, ...
                mean_crlb_position(1), mean_crlb_position(2));
        else
            target_info.theoretical_accuracy.horizontal_accuracy = norm(mean_crlb_position(1:2));
            target_info.theoretical_accuracy.vertical_accuracy = mean_crlb_position(3);
            target_info.theoretical_accuracy.total_accuracy = norm(mean_crlb_position);
            fprintf('    轨迹CRLB: 总精度=%.2fm (水平=%.2fm, 垂直=%.2fm)\n', ...
                target_info.theoretical_accuracy.total_accuracy, ...
                target_info.theoretical_accuracy.horizontal_accuracy, ...
                target_info.theoretical_accuracy.vertical_accuracy);
        end
    else
        fprintf('    警告：无有效CRLB计算\n');
        target_info.theoretical_accuracy = struct();
        target_info.theoretical_accuracy.mean_crlb_position = nan(1, config.dimension);
        target_info.theoretical_accuracy.crlb_count = 0;
    end
    
    is_valid = validate_trajectory_motion_v5(target_info.trajectory, ...
        model_sequence, config);
end

%% ================ CRLB计算函数（新增）================
function [crlb_pos, crlb_details] = compute_crlb_at_point(target_pos, tx_pos, rxs_pos, noise_info, config)
    % 计算目标点的位置CRLB
    % 输入:
    %   target_pos: 目标位置 [x, y, z] 或 [x, y, 0]
    %   tx_pos: 发射机位置
    %   rxs_pos: 接收机位置 cell数组
    %   noise_info: 量测噪声信息 cell数组
    %   config: 配置参数
    % 输出:
    %   crlb_pos: 位置CRLB [sigma_x, sigma_y] 或 [sigma_x, sigma_y, sigma_z]
    %   crlb_details: 详细的CRLB计算信息
    
    num_rx = length(rxs_pos);
    dim = config.dimension;
    
    % 构建Fisher信息矩阵
    if dim == 2
        % 2D情况：状态为[x, y]
        FIM = zeros(2, 2);
    else
        % 3D情况：状态为[x, y, z]
        FIM = zeros(3, 3);
    end
    
    % 累积每个接收机的Fisher信息
    for i = 1:num_rx
        rx_pos = rxs_pos{i};
        
        % 确保位置是3D向量
        if length(target_pos) == 2
            target_pos = [target_pos, 0];
        end
        if length(tx_pos) == 2
            tx_pos = [tx_pos, 0];
        end
        if length(rx_pos) == 2
            rx_pos = [rx_pos, 0];
        end
        
        % 计算几何关系
        vec_tx = target_pos - tx_pos;
        vec_rx = target_pos - rx_pos;
        
        r_tx = norm(vec_tx);
        r_rx = norm(vec_rx);
        
        % 单位方向向量
        u_tx = vec_tx / r_tx;
        u_rx = vec_rx / r_rx;
        
        % 双基地和距离的梯度
        grad_range = u_tx + u_rx;  % 3D向量
        
        % 方位角梯度（仅x-y平面）
        dx = target_pos(1) - rx_pos(1);
        dy = target_pos(2) - rx_pos(2);
        rho_xy = sqrt(dx^2 + dy^2);
        
        if rho_xy > 1e-6
            grad_azimuth = [-dy / rho_xy^2; dx / rho_xy^2; 0];  % 3D向量
        else
            grad_azimuth = [0; 0; 0];
        end
        
        % 俯仰角梯度（3D）
        if dim == 3
            dz = target_pos(3) - rx_pos(3);
            rho = r_rx;
            
            if rho > 1e-6 && rho_xy > 1e-6
                grad_elevation = [
                    -dx * dz / (rho^2 * rho_xy);
                    -dy * dz / (rho^2 * rho_xy);
                    rho_xy / rho^2
                ];
            else
                grad_elevation = [0; 0; 0];
            end
        end
        
        % 噪声标准差（从noise_info获取）
        if i <= length(noise_info)
            sigma_range = noise_info{i}.range_noise_std;
            sigma_angle = noise_info{i}.angle_noise_std;
        else
            % 默认值
            sigma_range = 5.0;
            sigma_angle = 0.01;
        end
        
        % Fisher信息矩阵贡献（只考虑x,y或x,y,z）
        if dim == 2
            % 2D：只提取x,y分量
            grad_r_2d = grad_range(1:2);
            grad_az_2d = grad_azimuth(1:2);
            
            % 距离贡献
            FIM = FIM + (1 / sigma_range^2) * (grad_r_2d(:) * grad_r_2d(:)');
            
            % 方位角贡献
            FIM = FIM + (1 / sigma_angle^2) * (grad_az_2d(:) * grad_az_2d(:)');
        else
            % 3D：使用x,y,z分量
            % 距离贡献
            FIM = FIM + (1 / sigma_range^2) * (grad_range(:) * grad_range(:)');
            
            % 方位角贡献
            FIM = FIM + (1 / sigma_angle^2) * (grad_azimuth(:) * grad_azimuth(:)');
            
            % 俯仰角贡献
            FIM = FIM + (1 / sigma_angle^2) * (grad_elevation(:) * grad_elevation(:)');
        end
    end
    
    % 计算CRLB（Fisher信息矩阵的逆的对角元素开方）
    try
        % 确保FIM对称正定
        FIM = (FIM + FIM') / 2;
        
        % 添加正则化以避免奇异
        epsilon = 1e-10;
        FIM = FIM + epsilon * eye(size(FIM));
        
        % 求逆得到协方差矩阵
        CRB_matrix = inv(FIM);
        
        % 提取对角元素并开方得到标准差
        crlb_pos = sqrt(diag(CRB_matrix))';
        
        % 保存详细信息
        crlb_details = struct();
        crlb_details.FIM = FIM;
        crlb_details.CRB_matrix = CRB_matrix;
        crlb_details.num_receivers = num_rx;
        
    catch ME
        % 如果计算失败，返回NaN
%         warning('CRLB计算失败: %s', ME.message);
        if dim == 2
            crlb_pos = [NaN, NaN];
        else
            crlb_pos = [NaN, NaN, NaN];
        end
        
        crlb_details = struct();
        crlb_details.error = ME.message;
    end
end

%% ================ 运动模型和噪声序列（改进版）================
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

function [model_sequence, noise_schedule] = generate_model_and_noise_sequence_v53(...
    config, target_params, time_steps, initial_state)
    
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
        
        current_state = initial_state;
        
        model_params = generate_model_parameters_v53(model_name, config, target_params, current_state);
        
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

%% ================ 改进的模型参数生成（修正CA模型加速度）================
function model_params = generate_model_parameters_v53(model_name, config, target_params, current_state)
    model_params = struct();
    model_params.type = model_name;
    
    switch model_name
        case 'CV'
            % 恒速模型 - 无需特殊参数
            
        case 'CA'
            % 恒加速模型 - 新增50%概率与速度方向一致
            max_acc = target_params.max_acceleration;
            
            % 确定是否与速度方向一致（50%概率）
            aligned_with_velocity = rand() < config.ca_model.aligned_probability;
            model_params.aligned_with_velocity = aligned_with_velocity;
            
            if config.dimension == 2
                if aligned_with_velocity
                    % 与速度方向一致
                    current_vel = current_state(3:4);
                    
                    % 检查速度是否为零向量
                    vel_norm = norm(current_vel);
                    if vel_norm < 1e-6
                        % 速度接近0，退化为随机方向
                        theta = rand() * 2 * pi;
                        direction = [cos(theta), sin(theta)];
                    else
                        % 与速度方向一致的单位向量
                        direction = current_vel / vel_norm;
                    end
                else
                    % 随机方向
                    theta = rand() * 2 * pi;
                    direction = [cos(theta), sin(theta)];
                end
                
                % 随机加速度大小，范围在最大加速度的10%-100%之间
                acc_magnitude = (0.1 + 0.9 * rand()) * max_acc;
                
                model_params.acceleration = direction * acc_magnitude;
                model_params.acc_magnitude = acc_magnitude;
                model_params.direction = direction;
                
            else
                % 3D情况
                if aligned_with_velocity
                    % 与速度方向一致
                    current_vel = current_state(4:6);
                    
                    % 检查速度是否为零向量
                    vel_norm = norm(current_vel);
                    if vel_norm < 1e-6
                        % 速度接近0，退化为随机方向
                        phi = rand() * 2 * pi;
                        theta = acos(2 * rand() - 1);
                        direction = [
                            sin(theta) * cos(phi),
                            sin(theta) * sin(phi),
                            cos(theta)
                        ];
                    else
                        % 与速度方向一致的单位向量
                        direction = current_vel / vel_norm;
                    end
                else
                    % 在单位球面上随机选择方向
                    phi = rand() * 2 * pi;
                    theta = acos(2 * rand() - 1);  % 均匀分布在球面上
                    
                    direction = [
                        sin(theta) * cos(phi),
                        sin(theta) * sin(phi),
                        cos(theta)
                    ];
                end
                
                % 随机加速度大小
                acc_magnitude = (0.1 + 0.9 * rand()) * max_acc;
                
                % 明确创建3D向量 - 确保精确的维度
                model_params.acceleration = zeros(1, 3);
                model_params.acceleration(1) = direction(1) * acc_magnitude;
                model_params.acceleration(2) = direction(2) * acc_magnitude;
                model_params.acceleration(3) = direction(3) * acc_magnitude;
                model_params.acc_magnitude = acc_magnitude;
                model_params.direction = direction;
                
                % 调试信息
                if length(model_params.acceleration) ~= 3
                    error('3D CA模型加速度维度错误: %d', length(model_params.acceleration));
                end
            end
            
        case 'CT'
            % 恒转弯模型
            max_turn = target_params.max_turn_rate;
            % 加入正负号随机性
            turn_sign = sign(rand() - 0.5);
            if turn_sign == 0, turn_sign = 1; end
            
            % 转弯率范围在最大转弯率的20%-100%之间
            turn_magnitude = (0.2 + 0.8 * rand()) * max_turn;
            model_params.turn_rate = turn_sign * turn_magnitude;
            
        case 'Singer'
            % Singer模型
            tau = 10 + rand() * 20;  % 时间常数范围：10-30秒
            model_params.beta = 1 / tau;
    end
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

%% ================ 状态更新（修正版）================
function next_state = update_state_v5(state, model_info, noise_info, config, target_params)
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
    sigma = noise_info.intensity;
    
    if strcmp(model_name, 'CT')
        omega = model_params.turn_rate;
        
        if dim == 2
            [Phi, Q] = get_ct_matrices_2d(omega, dt, sigma);
        else
            [Phi_xy, Q_xy] = get_ct_matrices_2d(omega, dt, sigma);
            
            Phi_z = [1, dt, 0;
                     0, 1,  0;
                     0, 0,  0];
            Q_z = sigma^2 * [dt^3/3, dt^2/2, 0;
                             dt^2/2, dt,     0;
                             0,      0,      0];
            
            Phi = zeros(9, 9);
            Phi(1:2, 1:2) = Phi_xy(1:2, 1:2);
            Phi(1:2, 4:5) = Phi_xy(1:2, 3:4);
            Phi(1:2, 7:8) = Phi_xy(1:2, 5:6);
            Phi(4:5, 4:5) = Phi_xy(3:4, 3:4);
            Phi(4:5, 7:8) = Phi_xy(3:4, 5:6);
            Phi(7:8, 4:5) = Phi_xy(5:6, 3:4);
            
            Phi(3, 3) = Phi_z(1, 1);
            Phi(3, 6) = Phi_z(1, 2);
            Phi(6, 6) = Phi_z(2, 2);
            
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
        switch model_name
            case 'CV'
                Phi1 = [1, dt, 0;
                        0, 1,  0;
                        0, 0,  0];
                Q1 = sigma^2 * [dt^3/3, dt^2/2, 0;
                                dt^2/2, dt,     0;
                                0,      0,      0];
                
            case 'CA'
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
        
        if dim == 2
            Phi = kron(Phi1, eye(2));
            Q = kron(Q1, eye(2));
        else
            Phi = kron(Phi1, eye(3));
            Q = kron(Q1, eye(3));
        end
    end
    
    Q = (Q + Q') / 2;
    Q = Q + 1e-10 * eye(size(Q, 1));
end

function [Phi, Q] = get_ct_matrices_2d(omega, dt, sigma)
    if abs(omega * dt) < 1e-6
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
        
        Phi = zeros(6, 6);
        
        Phi = [1, 0, s/omega, -(1-c)/omega, 0, 0;
           0, 1, (1-c)/omega, s/omega, 0, 0;
           0, 0, c, -s, 0, 0;
           0, 0, s, c, 0, 0;
           0, 0, -omega*s, -omega*c, 0, 0;
           0, 0, omega*c, -omega*s, 0, 0];
        
        Q1 = sigma^2 * [dt^3/3, dt^2/2, 0;
                        dt^2/2, dt,     0;
                        0,      0,      0];
        Q = kron(Q1, eye(2));
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
    
    for i = 1:length(segment.trajectory)
        point = segment.trajectory{i};
        
        vel_norm = norm(point.velocity);
        acc_norm = norm(point.acceleration);
        
        if vel_norm > 1000 || acc_norm > 100
            is_valid = false;
            return;
        end
    end
    
    switch model_name
        case 'CA'
            if isfield(segment.params, 'aligned_with_velocity') && segment.params.aligned_with_velocity
                for i = 1:length(segment.trajectory)
                    point = segment.trajectory{i};
                    vel = point.velocity(1:min(2, length(point.velocity)));
                    acc = point.acceleration(1:min(2, length(point.acceleration)));
                    
                    vel_norm = norm(vel);
                    acc_norm = norm(acc);
                    
                    if vel_norm > 1e-3 && acc_norm > 1e-3
                        vel_unit = vel / vel_norm;
                        acc_unit = acc / acc_norm;
                        
                        cos_angle = dot(vel_unit, acc_unit);
                        
                        if cos_angle < 0.9
                            fprintf('    CA模型与速度方向一致性验证失败：角度余弦=%.3f\n', cos_angle);
                            is_valid = false;
                            break;
                        end
                    end
                end
            end
    end
end

%% ================ 增强可视化函数（增加CRLB显示）================
function visualize_scenario_v54(scenario, title_str)
    if nargin < 2
        title_str = 'Scenario Visualization v5.4';
    end
    
    config = scenario.config;
    trajectory = scenario.target_info.trajectory;
    
    % 提取轨迹数据
    times = [];
    positions = [];
    velocities = [];
    accelerations = [];
    models = {};
    transitions = [];
    ca_aligned = [];
    crlb_pos = [];  % **新增：CRLB数据**
    
    for i = 1:length(trajectory)
        point = trajectory{i};
        times(end+1) = point.time;
        positions(end+1, :) = point.position;
        velocities(end+1, :) = point.velocity;
        accelerations(end+1, :) = point.acceleration;
        models{end+1} = point.motion_model;
        
        if isfield(point, 'transition_info')
            transitions(end+1) = 1;
        else
            transitions(end+1) = 0;
        end
        
        if strcmp(point.motion_model, 'CA') && isfield(point.model_parameters, 'aligned_with_velocity')
            ca_aligned(end+1) = point.model_parameters.aligned_with_velocity;
        else
            ca_aligned(end+1) = NaN;
        end
        
        % **新增：提取CRLB数据**
        if isfield(point, 'crlb_position')
            if config.dimension == 2
                crlb_pos(end+1) = norm(point.crlb_position(1:2));
            else
                crlb_pos(end+1) = norm(point.crlb_position);
            end
        else
            crlb_pos(end+1) = NaN;
        end
    end
    
    % 识别模型切换点
    model_changes = [1];
    for i = 2:length(models)
        if ~strcmp(models{i}, models{i-1})
            model_changes(end+1) = i;
        end
    end
    model_changes(end+1) = length(models) + 1;
    
    % 计算速度和加速度模
    vel_norms = sqrt(sum(velocities.^2, 2));
    acc_norms = sqrt(sum(accelerations.^2, 2));
    
    % 创建图形 - 使用更丰富的布局
    figure('Position', [100, 100, 1600, 1000], 'Name', title_str);
    
    % 轨迹可视化 (左上角)
    if config.dimension == 2
        subplot(4, 3, [1, 4]);
        hold on; grid on;
        
        % 绘制轨迹
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
            
            if strcmp(model_name, 'CA') && ~isnan(ca_aligned(seg_start))
                if ca_aligned(seg_start)
                    text(positions(seg_start, 1)/1000, positions(seg_start, 2)/1000, ...
                         'CA-A', 'FontSize', 8, 'Color', color);
                else
                    text(positions(seg_start, 1)/1000, positions(seg_start, 2)/1000, ...
                         'CA-R', 'FontSize', 8, 'Color', color);
                end
            end
            
            for j = seg_start:seg_end
                if j <= length(transitions) && transitions(j) == 1
                    plot(positions(j, 1)/1000, positions(j, 2)/1000, ...
                         'o', 'MarkerSize', 6, 'Color', [1 0.5 0], ...
                         'MarkerFaceColor', [1 0.5 0], 'HandleVisibility', 'off');
                end
            end
            
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
        
        quiver_step = 20;
        if length(times) > quiver_step
            quiver_indices = 1:quiver_step:length(times);
            quiver(positions(quiver_indices, 1)/1000, positions(quiver_indices, 2)/1000, ...
                   velocities(quiver_indices, 1)*0.01, velocities(quiver_indices, 2)*0.01, ...
                   0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'DisplayName', 'Velocity');
        end
        
        xlabel('X (km)');
        ylabel('Y (km)');
        title(sprintf('%s - 2D Trajectory', title_str));
        legend('Location', 'best');
        axis equal;
        
    else
        % 3D轨迹
        subplot(4, 3, [1, 4]);
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
            
            if strcmp(model_name, 'CA') && ~isnan(ca_aligned(seg_start))
                if ca_aligned(seg_start)
                    text(positions(seg_start, 1)/1000, positions(seg_start, 2)/1000, ...
                         positions(seg_start, 3)/1000, 'CA-A', 'FontSize', 8, 'Color', color);
                else
                    text(positions(seg_start, 1)/1000, positions(seg_start, 2)/1000, ...
                         positions(seg_start, 3)/1000, 'CA-R', 'FontSize', 8, 'Color', color);
                end
            end
            
            for j = seg_start:seg_end
                if j <= length(transitions) && transitions(j) == 1
                    plot3(positions(j, 1)/1000, positions(j, 2)/1000, positions(j, 3)/1000, ...
                         'o', 'MarkerSize', 6, 'Color', [1 0.5 0], ...
                         'MarkerFaceColor', [1 0.5 0], 'HandleVisibility', 'off');
                end
            end
            
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
        
        quiver_step = 20;
        if length(times) > quiver_step
            quiver_indices = 1:quiver_step:length(times);
            quiver3(positions(quiver_indices, 1)/1000, positions(quiver_indices, 2)/1000, ...
                   positions(quiver_indices, 3)/1000, velocities(quiver_indices, 1)*0.01, ...
                   velocities(quiver_indices, 2)*0.01, velocities(quiver_indices, 3)*0.01, ...
                   0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'DisplayName', 'Velocity');
        end
        
        xlabel('X (km)');
        ylabel('Y (km)');
        zlabel('Z (km)');
        title(sprintf('%s - 3D Trajectory', title_str));
        legend('Location', 'best');
        grid on;
        view(3);
    end
    
    % 速度曲线 (右上)
    subplot(4, 3, 2);
    hold on; grid on;
    
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        plot(times(seg_start:seg_end), vel_norms(seg_start:seg_end), ...
             'LineWidth', 2, 'Color', color);
        
        for j = seg_start:seg_end
            if j <= length(transitions) && transitions(j) == 1
                plot(times(j), vel_norms(j), ...
                     'o', 'MarkerSize', 5, 'Color', [1 0.5 0], ...
                     'MarkerFaceColor', [1 0.5 0], 'HandleVisibility', 'off');
            end
        end
        
        if i < length(model_changes)-1
            plot([times(seg_end), times(seg_end)], ylim, '--k', 'LineWidth', 1);
        end
    end
    
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Magnitude');
    
    % 加速度曲线 (右中)
    subplot(4, 3, 5);
    hold on; grid on;
    
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        plot(times(seg_start:seg_end), acc_norms(seg_start:seg_end), ...
             'LineWidth', 2, 'Color', color);
        
        for j = seg_start:seg_end
            if j <= length(transitions) && transitions(j) == 1
                plot(times(j), acc_norms(j), ...
                     'o', 'MarkerSize', 5, 'Color', [1 0.5 0], ...
                     'MarkerFaceColor', [1 0.5 0], 'HandleVisibility', 'off');
            end
        end
        
        if i < length(model_changes)-1
            plot([times(seg_end), times(seg_end)], ylim, '--k', 'LineWidth', 1);
        end
    end
    
    xlabel('Time (s)');
    ylabel('Acceleration (m/s²)');
    title('Acceleration Magnitude');
    
    % **新增：CRLB曲线 (右上角第3个)**
    subplot(4, 3, 3);
    hold on; grid on;
    
    % 绘制CRLB随时间变化
    valid_crlb = ~isnan(crlb_pos);
    if any(valid_crlb)
        plot(times(valid_crlb), crlb_pos(valid_crlb), 'b-', 'LineWidth', 2);
        
        % 标记模型切换点
        for i = 2:length(model_changes)-1
            change_time = times(model_changes(i));
            plot([change_time, change_time], ylim, 'k--', 'LineWidth', 1);
        end
        
        % 显示平均CRLB
        if isfield(scenario.target_info, 'theoretical_accuracy')
            if config.dimension == 2
                avg_crlb = scenario.target_info.theoretical_accuracy.horizontal_accuracy;
            else
                avg_crlb = scenario.target_info.theoretical_accuracy.total_accuracy;
            end
            plot([min(times), max(times)], [avg_crlb, avg_crlb], 'r--', 'LineWidth', 2, ...
                 'DisplayName', sprintf('Mean CRLB=%.2fm', avg_crlb));
            legend('Location', 'best');
        end
    end
    
    xlabel('Time (s)');
    ylabel('Position CRLB (m)');
    title('Theoretical Positioning Accuracy (CRLB)');
    grid on;
    
    % 速度-加速度夹角 (中间第6个)
    subplot(4, 3, 6);
    hold on; grid on;
    
    vel_acc_angles = zeros(size(times));
    for i = 1:length(times)
        vel = velocities(i, 1:min(3, size(velocities, 2)));
        acc = accelerations(i, 1:min(3, size(accelerations, 2)));
        
        vel_norm = norm(vel);
        acc_norm = norm(acc);
        
        if vel_norm > 1e-3 && acc_norm > 1e-3
            cos_angle = dot(vel, acc) / (vel_norm * acc_norm);
            vel_acc_angles(i) = acos(min(max(cos_angle, -1), 1)) * 180/pi;
        else
            vel_acc_angles(i) = NaN;
        end
    end
    
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        plot(times(seg_start:seg_end), vel_acc_angles(seg_start:seg_end), ...
             'LineWidth', 2, 'Color', color);
        
        if strcmp(model_name, 'CA') && ~isnan(ca_aligned(seg_start))
            mid_idx = floor((seg_start + seg_end)/2);
            if ca_aligned(seg_start)
                plot(times(mid_idx), vel_acc_angles(mid_idx), 'ko', ...
                    'MarkerSize', 8, 'MarkerFaceColor', 'y');
                text(times(mid_idx), vel_acc_angles(mid_idx)+10, 'CA-A', ...
                    'FontSize', 8, 'HorizontalAlignment', 'center');
            else
                plot(times(mid_idx), vel_acc_angles(mid_idx), 'ko', ...
                    'MarkerSize', 8, 'MarkerFaceColor', 'c');
                text(times(mid_idx), vel_acc_angles(mid_idx)+10, 'CA-R', ...
                    'FontSize', 8, 'HorizontalAlignment', 'center');
            end
        end
        
        if i < length(model_changes)-1
            plot([times(seg_end), times(seg_end)], ylim, '--k', 'LineWidth', 1);
        end
    end
    
    xlabel('Time (s)');
    ylabel('Angle (degrees)');
    title('Velocity-Acceleration Angle');
    ylim([0, 180]);
    yticks([0, 45, 90, 135, 180]);
    grid on;
    
    % 速度分量 (左下)
    subplot(4, 3, 7);
    hold on; grid on;
    
    if config.dimension == 2
        plot(times, velocities(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vx');
        plot(times, velocities(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Vy');
    else
        plot(times, velocities(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vx');
        plot(times, velocities(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Vy');
        plot(times, velocities(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Vz');
    end
    
    for i = 2:length(model_changes)-1
        change_time = times(model_changes(i));
        plot([change_time, change_time], ylim, 'k--');
    end
    
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Components');
    legend('Location', 'best');
    
    % 加速度分量 (中下)
    subplot(4, 3, 8);
    hold on; grid on;
    
    if config.dimension == 2
        plot(times, accelerations(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ax');
        plot(times, accelerations(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ay');
    else
        plot(times, accelerations(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ax');
        plot(times, accelerations(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Ay');
        plot(times, accelerations(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Az');
    end
    
    for i = 2:length(model_changes)-1
        change_time = times(model_changes(i));
        plot([change_time, change_time], ylim, 'k--');
    end
    
    xlabel('Time (s)');
    ylabel('Acceleration (m/s²)');
    title('Acceleration Components');
    legend('Location', 'best');
    
    % 模型时间线 (右下)
    subplot(4, 3, 9);
    hold on;
    
    y_pos = 1;
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        if strcmp(model_name, 'CA') && ~isnan(ca_aligned(seg_start))
            ca_type = '';
            if ca_aligned(seg_start)
                ca_type = 'CA-Aligned';
                rectangle('Position', [times(seg_start), y_pos-0.3, ...
                          times(seg_end)-times(seg_start), 0.6], ...
                          'FaceColor', [0.95, 0.6, 0.3], 'EdgeColor', 'k');
            else
                ca_type = 'CA-Random';
                rectangle('Position', [times(seg_start), y_pos-0.3, ...
                          times(seg_end)-times(seg_start), 0.6], ...
                          'FaceColor', [0.8, 0.4, 0.2], 'EdgeColor', 'k');
            end
            text(mean([times(seg_start), times(seg_end)]), y_pos, ...
                 ca_type, 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'middle', 'FontWeight', 'bold', ...
                 'Color', 'white');
        else
            rectangle('Position', [times(seg_start), y_pos-0.3, ...
                      times(seg_end)-times(seg_start), 0.6], ...
                      'FaceColor', color, 'EdgeColor', 'k');
            
            text(mean([times(seg_start), times(seg_end)]), y_pos, ...
                 model_name, 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'middle', 'FontWeight', 'bold');
        end
        
        for j = seg_start:seg_end
            if j <= length(transitions) && transitions(j) == 1
                plot([times(j), times(j)], [y_pos-0.3, y_pos+0.3], ...
                     'Color', [1 0.5 0], 'LineWidth', 2);
            end
        end
    end
    
    xlim([0, max(times)]);
    ylim([0.5, 1.5]);
    xlabel('Time (s)');
    title('Motion Model Timeline');
    set(gca, 'YTick', []);
    grid on;
    
    % **新增：CRLB统计信息显示 (底部)**
    subplot(4, 3, [10, 11, 12]);
    axis off;
    
    info_text = {};
    info_text{end+1} = sprintf('\\bf轨迹统计信息 (v5.4 with CRLB)');
    info_text{end+1} = sprintf('----------------------------------------');
    info_text{end+1} = sprintf('目标类型: %s', scenario.target_info.target_type);
    info_text{end+1} = sprintf('轨迹点数: %d', length(trajectory));
    info_text{end+1} = sprintf('仿真时间: %.1f 秒', max(times));
    
    if isfield(scenario.target_info, 'theoretical_accuracy')
        info_text{end+1} = sprintf('----------------------------------------');
        info_text{end+1} = sprintf('\\bf理论定位精度 (CRLB):');
        
        acc = scenario.target_info.theoretical_accuracy;
        if config.dimension == 2
            info_text{end+1} = sprintf('水平精度: %.2f m', acc.horizontal_accuracy);
            info_text{end+1} = sprintf('X方向CRLB: %.2f m', acc.mean_crlb_position(1));
            info_text{end+1} = sprintf('Y方向CRLB: %.2f m', acc.mean_crlb_position(2));
        else
            info_text{end+1} = sprintf('总体精度: %.2f m', acc.total_accuracy);
            info_text{end+1} = sprintf('水平精度: %.2f m', acc.horizontal_accuracy);
            info_text{end+1} = sprintf('垂直精度: %.2f m', acc.vertical_accuracy);
            info_text{end+1} = sprintf('X方向CRLB: %.2f m', acc.mean_crlb_position(1));
            info_text{end+1} = sprintf('Y方向CRLB: %.2f m', acc.mean_crlb_position(2));
            info_text{end+1} = sprintf('Z方向CRLB: %.2f m', acc.mean_crlb_position(3));
        end
        info_text{end+1} = sprintf('有效CRLB点数: %d / %d (%.1f%%)', ...
            acc.crlb_count, length(trajectory), 100*acc.crlb_count/length(trajectory));
    end
    
    text(0.1, 0.5, info_text, 'FontSize', 10, 'VerticalAlignment', 'middle', ...
         'Interpreter', 'tex');
    
    % 添加总体图例说明
    try
        annotation('textbox', [0.02, 0.01, 0.96, 0.03], ...
                  'String', '橙色标记=过渡期; CA-A=与速度方向一致的CA模型; CA-R=随机方向CA模型; CRLB=理论定位精度下界', ...
                  'EdgeColor', 'none', 'FaceColor', [1 0.8 0.8], ...
                  'BackgroundColor', [1 1 0.9], 'FontSize', 9);
    catch
        annotation('textbox', [0.02, 0.01, 0.96, 0.03], ...
                  'String', '橙色标记=过渡期; CA-A=与速度方向一致的CA模型; CA-R=随机方向CA模型; CRLB=理论定位精度下界', ...
                  'EdgeColor', 'none', ...
                  'BackgroundColor', [1 1 0.9], 'FontSize', 9);
    end
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
    
    index_file = fullfile(dataset_dir, 'index.json');
    if exist(index_file, 'file')
        try
            index_data = jsondecode(fileread(index_file));
            num_scenarios = index_data.num_scenarios;
            fprintf('\n--- 数据集分析: %s (从索引读取) ---\n', dataset_dir);
            fprintf('场景数量: %d\n', num_scenarios);
            
            first_scenario_file = fullfile(dataset_dir, 'scenario_0000.mat');
            if exist(first_scenario_file, 'file')
                scenario_data = load(first_scenario_file);
                config = scenario_data.scenario.config;
                fprintf('维度: %dD\n', config.dimension);
                
                % **新增：显示CRLB统计**
                if isfield(scenario_data.scenario.target_info, 'theoretical_accuracy')
                    acc = scenario_data.scenario.target_info.theoretical_accuracy;
                    fprintf('\n示例场景CRLB:\n');
                    if config.dimension == 2
                        fprintf('  水平精度: %.2f m\n', acc.horizontal_accuracy);
                    else
                        fprintf('  总体精度: %.2f m\n', acc.total_accuracy);
                        fprintf('  水平精度: %.2f m\n', acc.horizontal_accuracy);
                        fprintf('  垂直精度: %.2f m\n', acc.vertical_accuracy);
                    end
                end
            end
        catch
            fprintf('索引文件解析失败，尝试传统方式读取\n');
        end
    end
    
    if ~exist('num_scenarios', 'var')
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
    end
    
    if exist('dataset', 'var') && isfield(dataset, 'scenarios')
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
        
        all_models = {};
        all_ca_aligned = [];
        for i = 1:length(dataset.scenarios)
            trajectory = dataset.scenarios{i}.target_info.trajectory;
            for j = 1:length(trajectory)
                all_models{end+1} = trajectory{j}.motion_model;
                
                if strcmp(trajectory{j}.motion_model, 'CA') && ...
                   isfield(trajectory{j}.model_parameters, 'aligned_with_velocity')
                    all_ca_aligned(end+1) = trajectory{j}.model_parameters.aligned_with_velocity;
                end
            end
        end
        
        fprintf('\n运动模型使用统计:\n');
        unique_models = unique(all_models);
        for i = 1:length(unique_models)
            count = sum(strcmp(all_models, unique_models{i}));
            fprintf('  %s: %d 时间步 (%.1f%%)\n', unique_models{i}, count, ...
                    100*count/length(all_models));
        end
        
        if ~isempty(all_ca_aligned)
            aligned_count = sum(all_ca_aligned);
            random_count = length(all_ca_aligned) - aligned_count;
            
            fprintf('\nCA模型类型统计:\n');
            fprintf('  与速度方向一致: %d (%.1f%%)\n', aligned_count, ...
                    100*aligned_count/length(all_ca_aligned));
            fprintf('  随机方向: %d (%.1f%%)\n', random_count, ...
                    100*random_count/length(all_ca_aligned));
        end
    else
        fprintf('\n注意：使用批处理模式，无法直接统计目标类型和模型分布。\n');
    end
end

function validate_dataset_v5(dataset_dir, config)
    if ~exist(dataset_dir, 'dir')
        fprintf('数据集目录不存在: %s\n', dataset_dir);
        return;
    end
    
    index_file = fullfile(dataset_dir, 'index.json');
    if exist(index_file, 'file')
        try
            index_data = jsondecode(fileread(index_file));
            num_scenarios = index_data.num_scenarios;
            fprintf('\n--- 数据验证: %s (批处理模式) ---\n', dataset_dir);
            fprintf('总场景数: %d，将验证随机抽样...\n', num_scenarios);
            
            sample_size = min(10, ceil(num_scenarios * 0.1));
            sample_indices = sort(randperm(num_scenarios, sample_size));
            
            num_valid = 0;
            num_invalid = 0;
            
            for i = 1:length(sample_indices)
                scenario_idx = sample_indices(i) - 1;
                scenario_file = fullfile(dataset_dir, sprintf('scenario_%04d.mat', scenario_idx));
                
                if exist(scenario_file, 'file')
                    scenario_data = load(scenario_file);
                    scenario = scenario_data.scenario;
                    
                    is_valid = validate_scenario_v5(scenario, config);
                    
                    if is_valid
                        num_valid = num_valid + 1;
                    else
                        num_invalid = num_invalid + 1;
                        fprintf('  场景 %d 验证失败\n', scenario_idx);
                    end
                end
            end
            
            fprintf('\n验证结果 (抽样):\n');
            fprintf('  有效场景: %d (%.1f%%)\n', num_valid, 100 * num_valid / length(sample_indices));
            fprintf('  无效场景: %d (%.1f%%)\n', num_invalid, 100 * num_invalid / length(sample_indices));
        catch
            fprintf('索引文件解析失败，尝试传统方式验证\n');
        end
    end
    
    if ~exist('num_valid', 'var')
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
            
            is_valid = validate_scenario_v5(scenario, config);
            
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
end

function is_valid = validate_scenario_v5(scenario, config)
    trajectory = scenario.target_info.trajectory;
    target_type = scenario.target_info.target_type;
    
    if nargin < 2 || isempty(config)
        config = scenario.config;
    end
    
    try
        target_params = get_target_params_v5(config, target_type);
    catch
        fprintf('警告：无效的目标类型 %s，使用默认参数\n', target_type);
        target_params = struct('max_velocity', 500, 'max_acceleration', 40);
    end
    
    is_valid = true;
    
    for j = 1:length(trajectory)
        point = trajectory{j};
        
        vel_norm = norm(point.velocity);
        acc_norm = norm(point.acceleration);
        
        if vel_norm > target_params.max_velocity * 1.1
            is_valid = false;
            break;
        end
        
        if acc_norm > target_params.max_acceleration * 1.1
            is_valid = false;
            break;
        end
        
        if config.dimension == 3
            z = point.position(3);
            if z < config.altitude_min * 0.9 || z > config.altitude_max * 1.05
                is_valid = false;
                break;
            end
        end
    end
    
    return;
end

%% ================ JSON保存函数（修复版）================
function save_json_file_v5(data, filename)
    try
        json_str = jsonencode(data, 'PrettyPrint', true);
        fid = fopen(filename, 'w');
        fprintf(fid, '%s', json_str);
        fclose(fid);
    catch
        try
            savejson('', data, filename);
        catch
            warning('无法保存JSON文件，请安装JSONlab工具箱或使用R2016b以上版本MATLAB');
        end
    end
end