%% =====================================================================
%  双基地雷达数据集生成器 v5.3 (运动学修正+批处理版)
%  =====================================================================
%  原作者: YiYangLi0319
%  修改者: Copilot AI Assistant
%  日期: 2025-10-23
%  
%  核心修改：
%  1. CA模型：加速度方向有50%概率与速度方向一致，50%概率随机选择
%  2. 批处理存储：每20条轨迹保存一次，减少内存占用
%  3. 模型切换平滑过渡：添加过渡期，使模型参数平滑变化
%  4. 增强可视化：显示轨迹、速度、加速度和模型切换
%  5. 完善测试方法：各个模型单独测试 + 组合场景验证
%  =====================================================================

clear all; close all; clc;

%% ================ 主程序调用 ================
radar_data_simulation_v53();

% 取消下面的注释来运行单元测试
%  run_all_tests_v53();

%% ================ 主程序入口 ================
function radar_data_simulation_v53()
    fprintf('=====================================================\n');
    fprintf('  双基地雷达数据集生成器 v5.3 (运动学修正+批处理版)\n');
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
    dataset_2d = generate_dataset_v53(config_2d, run_config.num_scenarios, ...
        'enhanced_2d_dataset_v2', GLOBAL_SEED, run_config.batch_size);
    
    % 生成3D数据集
    fprintf('\n--- 生成3D数据集 ---\n');
    config_3d = create_radar_config_v5(3);
    dataset_3d = generate_dataset_v53(config_3d, run_config.num_scenarios, ...
        'enhanced_3d_dataset_v2', GLOBAL_SEED + 100000, run_config.batch_size);
    
    % 演示可视化
    if run_config.run_demo
        fprintf('\n--- 可视化演示 ---\n');
        % 从保存的文件中加载示例场景进行可视化
        if exist('enhanced_2d_dataset_v2/scenario_0000.mat', 'file')
            demo_2d = load(fullfile('enhanced_2d_dataset_v2', 'scenario_0000.mat'));
            visualize_scenario_v53(demo_2d.scenario, '2D场景示例 v5.3');
        else
            fprintf('2D场景文件不存在，跳过可视化\n');
        end
        
        if exist('enhanced_3d_dataset_v2/scenario_0000.mat', 'file')
            demo_3d = load(fullfile('enhanced_3d_dataset_v2', 'scenario_0000.mat'));
            visualize_scenario_v53(demo_3d.scenario, '3D场景示例 v5.3');
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
    % 创建雷达系统配置 v5.3
    
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

%% ================ 数据集生成（批处理版）================
function dataset_info = generate_dataset_v53(config, num_scenarios, output_dir, base_seed, batch_size)
    if nargin < 3, output_dir = 'enhanced_v53_dataset'; end
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
    dataset_info.metadata.version = '5.3';
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
            
            scenario = generate_scenario_v53(config, target_type);
            scenario.scenario_id = scenario_idx - 1;
            scenario.seed = scenario_seed;
            
            % 保存单个场景文件（.mat和.json）
            scenario_file = fullfile(output_dir, sprintf('scenario_%04d.mat', scenario_idx - 1));
            save(scenario_file, 'scenario');
%             save_scenario_json_v5(scenario, output_dir, scenario_idx - 1);
            
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

function scenario = generate_scenario_v53(config, target_type)
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
            generate_target_trajectory_v53(config, time_steps, target_type, ...
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

function [target_info, measurements, detailed_records, is_valid] = ...
    generate_target_trajectory_v53(config, time_steps, target_type, tx, rxs)
    
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
    
    previous_model = '';  % 用于检测模型切换
    transition_active = false; % 标记是否有活动的过渡
    transition_data = struct(); % 存储过渡信息
    
    % 轨迹生成主循环
    for t = 1:time_steps
        current_time = (t-1) * config.dt;
        current_model = model_sequence{t}.model;
        
        % 检测模型切换
        if t > 1 && ~strcmp(current_model, previous_model)
            if strcmp(current_model, 'CA') && config.model_transition.enable
                % 检测到CA模型切换，记录过渡参数
                if config.dimension == 2
                    fprintf('    时刻%.1fs: 平滑切换到CA模型，目标加速度为 [%.2f, %.2f]\n', ...
                        current_time, model_sequence{t}.params.acceleration(1), ...
                        model_sequence{t}.params.acceleration(2));
                    
                    % 初始化新的过渡
                    transition_active = true;
                    transition_data = struct();
                    transition_data.start_time = current_time;
                    transition_data.duration = config.model_transition.duration;
                    transition_data.start_acc = state(5:6);
                    transition_data.target_acc = model_sequence{t}.params.acceleration;
                    
                    % 添加CA模型类型信息
                    if isfield(model_sequence{t}.params, 'aligned_with_velocity')
                        transition_data.aligned_with_velocity = model_sequence{t}.params.aligned_with_velocity;
                    else
                        transition_data.aligned_with_velocity = false;
                    end
                    
                    % 存储过渡信息到轨迹点
                    model_sequence{t}.transition = transition_data;
                else
                    % 3D情况
                    target_acc = model_sequence{t}.params.acceleration;
                    
                    % 确保目标加速度是3D向量
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
                    
                    % 初始化新的过渡
                    transition_active = true;
                    transition_data = struct();
                    transition_data.start_time = current_time;
                    transition_data.duration = config.model_transition.duration;
                    transition_data.start_acc = current_acc;
                    transition_data.target_acc = target_acc;
                    
                    % 添加CA模型类型信息
                    if isfield(model_sequence{t}.params, 'aligned_with_velocity')
                        transition_data.aligned_with_velocity = model_sequence{t}.params.aligned_with_velocity;
                    else
                        transition_data.aligned_with_velocity = false;
                    end
                    
                    % 存储过渡信息
                    model_sequence{t}.transition = transition_data;
                    
                    fprintf('    过渡开始: 起始加速度=[%.2f, %.2f, %.2f], 目标加速度=[%.2f, %.2f, %.2f]\n', ...
                        current_acc(1), current_acc(2), current_acc(3), ...
                        target_acc(1), target_acc(2), target_acc(3));
                    
                    if isfield(model_sequence{t}.params, 'aligned_with_velocity') && ...
                       model_sequence{t}.params.aligned_with_velocity
                        fprintf('    CA模型类型: 与速度方向一致的加速度\n');
                    else
                        fprintf('    CA模型类型: 随机方向加速度\n');
                    end
                end
            end
        end
        previous_model = current_model;
        
        % 应用过渡 - 关键修复点
        if transition_active
            % 计算过渡进度，考虑到时间增量
            elapsed = current_time - transition_data.start_time;
            
            % 判断过渡是否仍然活跃
            if elapsed <= transition_data.duration
                % 计算过渡进度百分比
                progress = elapsed / transition_data.duration;
                
                % 确保维度匹配
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
                
                % 线性插值加速度
                interpolated_acc = transition_data.start_acc + ...
                    progress * (transition_data.target_acc - transition_data.start_acc);
                
                % 更新状态中的加速度
                if config.dimension == 2
                    state(5:6) = interpolated_acc;
                    fprintf('    时刻%.1fs: 加速度过渡 %.1f%% [%.2f, %.2f]\n', ...
                        current_time, progress*100, interpolated_acc(1), interpolated_acc(2));
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
                    fprintf('    时刻%.1fs: 加速度过渡 %.1f%% [%.2f, %.2f, %.2f]\n', ...
                        current_time, progress*100, interpolated_acc(1), interpolated_acc(2), interpolated_acc(3));
                end
                
                % 将过渡信息保存到当前时间步的模型参数
                model_sequence{t}.transition = transition_data;
                model_sequence{t}.transition.progress = progress;
            else
                % 过渡完成
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
        
        % 添加过渡信息（如果有）
        if isfield(model_sequence{t}, 'transition')
            traj_point.transition_info = model_sequence{t}.transition;
        end
        
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
    
    is_valid = validate_trajectory_motion_v5(target_info.trajectory, ...
        model_sequence, config);
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
                        fprintf('    CA模型：速度接近0，退化为随机方向加速度\n');
                    else
                        % 与速度方向一致的单位向量
                        direction = current_vel / vel_norm;
                        fprintf('    CA模型：加速度与速度方向一致\n');
                    end
                else
                    % 随机方向
                    theta = rand() * 2 * pi;
                    direction = [cos(theta), sin(theta)];
                    fprintf('    CA模型：随机方向加速度\n');
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
                        fprintf('    CA模型(3D)：速度接近0，退化为随机方向加速度\n');
                    else
                        % 与速度方向一致的单位向量
                        direction = current_vel / vel_norm;
                        fprintf('    CA模型(3D)：加速度与速度方向一致\n');
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
                    fprintf('    CA模型(3D)：随机方向加速度\n');
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
            Phi(7:8, 4:5) = Phi_xy(5:6, 3:4);  % ax,ay 加速度
            
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
        
        Phi = [1, 0, s/omega, -(1-c)/omega, 0, 0;
           0, 1, (1-c)/omega, s/omega, 0, 0;
           0, 0, c, -s, 0, 0;
           0, 0, s, c, 0, 0;
           0, 0, -omega*s, -omega*c, 0, 0;
           0, 0, omega*c, -omega*s, 0, 0];
        
        % 过程噪声（简化模型，主要在速度和加速度）
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
    
    % 针对不同模型的特定验证
    switch model_name
        case 'CA'
            % CA模型：验证加速度方向正确性
            if isfield(segment.params, 'aligned_with_velocity') && segment.params.aligned_with_velocity
                % 检验加速度是否与速度方向一致（允许一定误差）
                for i = 1:length(segment.trajectory)
                    point = segment.trajectory{i};
                    vel = point.velocity(1:min(2, length(point.velocity)));
                    acc = point.acceleration(1:min(2, length(point.acceleration)));
                    
                    vel_norm = norm(vel);
                    acc_norm = norm(acc);
                    
                    if vel_norm > 1e-3 && acc_norm > 1e-3
                        vel_unit = vel / vel_norm;
                        acc_unit = acc / acc_norm;
                        
                        % 计算两向量夹角的余弦值
                        cos_angle = dot(vel_unit, acc_unit);
                        
                        % 与速度方向一致的加速度，cos_angle应接近1
                        if cos_angle < 0.9  % 允许约26度误差
                            fprintf('    CA模型与速度方向一致性验证失败：角度余弦=%.3f\n', cos_angle);
                            is_valid = false;
                            break;
                        end
                    end
                end
            end
            
        case 'CT'
            % CT模型可以添加转弯率验证
            % 这里只是示例，可以根据需要实现更复杂的验证
            
        case 'Singer'
            % Singer模型验证
    end
end

%% ================ 增强可视化函数 ================
function visualize_scenario_v53(scenario, title_str)
    if nargin < 2
        title_str = 'Scenario Visualization v5.3';
    end
    
    config = scenario.config;
    trajectory = scenario.target_info.trajectory;
    
    % 提取轨迹数据
    times = [];
    positions = [];
    velocities = [];
    accelerations = [];
    models = {};
    transitions = [];  % 记录过渡期
    ca_aligned = [];   % 记录CA模型是否与速度方向一致
    
    for i = 1:length(trajectory)
        point = trajectory{i};
        times(end+1) = point.time;
        positions(end+1, :) = point.position;
        velocities(end+1, :) = point.velocity;
        accelerations(end+1, :) = point.acceleration;
        models{end+1} = point.motion_model;
        
        % 检查是否在过渡期
        if isfield(point, 'transition_info')
            transitions(end+1) = 1;
        else
            transitions(end+1) = 0;
        end
        
        % 记录CA模型类型
        if strcmp(point.motion_model, 'CA') && isfield(point.model_parameters, 'aligned_with_velocity')
            ca_aligned(end+1) = point.model_parameters.aligned_with_velocity;
        else
            ca_aligned(end+1) = NaN;  % 非CA模型或无信息
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
    figure('Position', [100, 100, 1400, 900], 'Name', title_str);
    
    % 轨迹可视化 (左上角)
    if config.dimension == 2
        subplot(3, 3, [1, 4]);
        hold on; grid on;
        
        % 绘制轨迹，突出显示过渡期
        for i = 1:length(model_changes)-1
            seg_start = model_changes(i);
            seg_end = model_changes(i+1) - 1;
            
            model_name = models{seg_start};
            color = get_model_color(model_name);
            
            % 绘制主轨迹
            plot(positions(seg_start:seg_end, 1)/1000, ...
                 positions(seg_start:seg_end, 2)/1000, ...
                 'LineWidth', 2, 'Color', color, ...
                 'DisplayName', sprintf('%s (%.0f-%.0fs)', ...
                 model_name, times(seg_start), times(seg_end)));
            
            % 特殊标记CA模型类型
            if strcmp(model_name, 'CA') && ~isnan(ca_aligned(seg_start))
                if ca_aligned(seg_start)
                    text(positions(seg_start, 1)/1000, positions(seg_start, 2)/1000, ...
                         'CA-A', 'FontSize', 8, 'Color', color);
                else
                    text(positions(seg_start, 1)/1000, positions(seg_start, 2)/1000, ...
                         'CA-R', 'FontSize', 8, 'Color', color);
                end
            end
            
            % 高亮过渡期
            for j = seg_start:seg_end
                if j <= length(transitions) && transitions(j) == 1
                    plot(positions(j, 1)/1000, positions(j, 2)/1000, ...
                         'o', 'MarkerSize', 6, 'Color', [1 0.5 0], ...
                         'MarkerFaceColor', [1 0.5 0], 'HandleVisibility', 'off');
                end
            end
            
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
        
        % 添加方向矢量（每20个点绘制一个）
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
        subplot(3, 3, [1, 4]);
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
            
            % 特殊标记CA模型类型
            if strcmp(model_name, 'CA') && ~isnan(ca_aligned(seg_start))
                if ca_aligned(seg_start)
                    text(positions(seg_start, 1)/1000, positions(seg_start, 2)/1000, ...
                         positions(seg_start, 3)/1000, 'CA-A', 'FontSize', 8, 'Color', color);
                else
                    text(positions(seg_start, 1)/1000, positions(seg_start, 2)/1000, ...
                         positions(seg_start, 3)/1000, 'CA-R', 'FontSize', 8, 'Color', color);
                end
            end
            
            % 高亮过渡期
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
        
        % 添加方向矢量（每20个点绘制一个）
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
    subplot(3, 3, 2);
    hold on; grid on;
    
    % 绘制不同模型下的速度
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        plot(times(seg_start:seg_end), vel_norms(seg_start:seg_end), ...
             'LineWidth', 2, 'Color', color);
        
        % 高亮过渡期
        for j = seg_start:seg_end
            if j <= length(transitions) && transitions(j) == 1
                plot(times(j), vel_norms(j), ...
                     'o', 'MarkerSize', 5, 'Color', [1 0.5 0], ...
                     'MarkerFaceColor', [1 0.5 0], 'HandleVisibility', 'off');
            end
        end
        
        % 模型切换竖线
        if i < length(model_changes)-1
            plot([times(seg_end), times(seg_end)], ylim, '--k', 'LineWidth', 1);
        end
    end
    
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Magnitude');
    
    % 加速度曲线 (右中)
    subplot(3, 3, 5);
    hold on; grid on;
    
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        plot(times(seg_start:seg_end), acc_norms(seg_start:seg_end), ...
             'LineWidth', 2, 'Color', color);
        
        % 高亮过渡期
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
    
    % 速度-加速度夹角 (右上角)
    subplot(3, 3, 3);
    hold on; grid on;
    
    % 计算速度和加速度的夹角 (只在速度和加速度都足够大时有意义)
    vel_acc_angles = zeros(size(times));
    for i = 1:length(times)
        vel = velocities(i, 1:min(3, size(velocities, 2)));
        acc = accelerations(i, 1:min(3, size(accelerations, 2)));
        
        vel_norm = norm(vel);
        acc_norm = norm(acc);
        
        if vel_norm > 1e-3 && acc_norm > 1e-3
            cos_angle = dot(vel, acc) / (vel_norm * acc_norm);
            % 转换为角度
            vel_acc_angles(i) = acos(min(max(cos_angle, -1), 1)) * 180/pi;
        else
            vel_acc_angles(i) = NaN;  % 速度或加速度太小，角度无意义
        end
    end
    
    % 按模型分段绘制
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        plot(times(seg_start:seg_end), vel_acc_angles(seg_start:seg_end), ...
             'LineWidth', 2, 'Color', color);
        
        % 添加CA模型类型标记
        if strcmp(model_name, 'CA') && ~isnan(ca_aligned(seg_start))
            if ca_aligned(seg_start)
                % 与速度方向一致的CA模型，添加特殊标记
                mid_idx = floor((seg_start + seg_end)/2);
                plot(times(mid_idx), vel_acc_angles(mid_idx), 'ko', ...
                    'MarkerSize', 8, 'MarkerFaceColor', 'y');
                text(times(mid_idx), vel_acc_angles(mid_idx)+10, 'CA-A', ...
                    'FontSize', 8, 'HorizontalAlignment', 'center');
            else
                mid_idx = floor((seg_start + seg_end)/2);
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
    subplot(3, 3, 7);
    hold on; grid on;
    
    if config.dimension == 2
        plot(times, velocities(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vx');
        plot(times, velocities(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Vy');
    else
        plot(times, velocities(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vx');
        plot(times, velocities(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Vy');
        plot(times, velocities(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Vz');
    end
    
    % 标记模型切换点
    for i = 2:length(model_changes)-1
        change_time = times(model_changes(i));
        plot([change_time, change_time], ylim, 'k--');
    end
    
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Components');
    legend('Location', 'best');
    
    % 加速度分量 (中下)
    subplot(3, 3, 8);
    hold on; grid on;
    
    if config.dimension == 2
        plot(times, accelerations(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ax');
        plot(times, accelerations(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ay');
    else
        plot(times, accelerations(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ax');
        plot(times, accelerations(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Ay');
        plot(times, accelerations(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Az');
    end
    
    % 标记模型切换点
    for i = 2:length(model_changes)-1
        change_time = times(model_changes(i));
        plot([change_time, change_time], ylim, 'k--');
    end
    
    xlabel('Time (s)');
    ylabel('Acceleration (m/s²)');
    title('Acceleration Components');
    legend('Location', 'best');
    
    % 模型时间线 (右下)
    subplot(3, 3, 9);
    hold on;
    
    y_pos = 1;
    for i = 1:length(model_changes)-1
        seg_start = model_changes(i);
        seg_end = model_changes(i+1) - 1;
        
        model_name = models{seg_start};
        color = get_model_color(model_name);
        
        % 特别标记CA模型类型
        if strcmp(model_name, 'CA') && ~isnan(ca_aligned(seg_start))
            % 修改CA模型显示
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
            % 主模型矩形
            rectangle('Position', [times(seg_start), y_pos-0.3, ...
                      times(seg_end)-times(seg_start), 0.6], ...
                      'FaceColor', color, 'EdgeColor', 'k');
            
            text(mean([times(seg_start), times(seg_end)]), y_pos, ...
                 model_name, 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'middle', 'FontWeight', 'bold');
        end
        
        % 过渡期标记
        for j = seg_start:seg_end
            if j <= length(transitions) && transitions(j) == 1
                % 过渡期用橙色竖线标记
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
    
    % 添加图例说明
    try
        % 新版MATLAB支持的完整样式
        annotation('textbox', [0.02, 0.01, 0.8, 0.03], ...
                  'String', '橙色标记表示模型过渡期; CA-A: 与速度方向一致的CA模型; CA-R: 随机方向CA模型', ...
                  'EdgeColor', 'none', 'FaceColor', [1 0.8 0.8], ...
                  'BackgroundColor', [1 1 0.9]);
    catch
        % 兼容旧版MATLAB
        annotation('textbox', [0.02, 0.01, 0.8, 0.03], ...
                  'String', '橙色标记表示模型过渡期; CA-A: 与速度方向一致的CA模型; CA-R: 随机方向CA模型', ...
                  'EdgeColor', 'none', ...
                  'BackgroundColor', [1 1 0.9]);
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
    
    % 检查是否有索引文件
    index_file = fullfile(dataset_dir, 'index.json');
    if exist(index_file, 'file')
        try
            % 使用索引文件读取场景数量
            index_data = jsondecode(fileread(index_file));
            num_scenarios = index_data.num_scenarios;
            fprintf('\n--- 数据集分析: %s (从索引读取) ---\n', dataset_dir);
            fprintf('场景数量: %d\n', num_scenarios);
            
            % 尝试加载第一个场景获取配置信息
            first_scenario_file = fullfile(dataset_dir, 'scenario_0000.mat');
            if exist(first_scenario_file, 'file')
                scenario_data = load(first_scenario_file);
                config = scenario_data.scenario.config;
                fprintf('维度: %dD\n', config.dimension);
            end
        catch
            fprintf('索引文件解析失败，尝试传统方式读取\n');
        end
    end
    
    if ~exist('num_scenarios', 'var')
        % 尝试使用传统方法加载dataset.mat
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
    
    % 统计目标类型和模型 (仅在不使用批处理时有效)
    if exist('dataset', 'var') && isfield(dataset, 'scenarios')
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
        all_ca_aligned = [];
        for i = 1:length(dataset.scenarios)
            trajectory = dataset.scenarios{i}.target_info.trajectory;
            for j = 1:length(trajectory)
                all_models{end+1} = trajectory{j}.motion_model;
                
                % 统计CA模型类型
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
        
        % CA模型统计
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
    
    % 检查是否有索引文件
    index_file = fullfile(dataset_dir, 'index.json');
    if exist(index_file, 'file')
        try
            % 使用索引文件读取场景列表
            index_data = jsondecode(fileread(index_file));
            num_scenarios = index_data.num_scenarios;
            fprintf('\n--- 数据验证: %s (批处理模式) ---\n', dataset_dir);
            fprintf('总场景数: %d，将验证随机抽样...\n', num_scenarios);
            
            % 随机抽样10%的场景进行验证
            sample_size = min(10, ceil(num_scenarios * 0.1));
            sample_indices = sort(randperm(num_scenarios, sample_size));
            
            num_valid = 0;
            num_invalid = 0;
            
            for i = 1:length(sample_indices)
                scenario_idx = sample_indices(i) - 1;  % 0-based索引
                scenario_file = fullfile(dataset_dir, sprintf('scenario_%04d.mat', scenario_idx));
                
                if exist(scenario_file, 'file')
                    scenario_data = load(scenario_file);
                    scenario = scenario_data.scenario;
                    
                    % 验证轨迹
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
        % 尝试使用传统方法加载dataset.mat
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
    
    % 确保config参数存在，如果没有提供，从scenario获取
    if nargin < 2 || isempty(config)
        config = scenario.config;
    end
    
    % 获取目标参数，检查目标类型是否合法
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
        'model_turn_rate', [], ...     % CT模型专用
        'model_beta', [], ...          % Singer模型专用
        'aligned_with_velocity', [], ... % 新增：CA模型是否与速度方向一致
        'in_transition', false   ...      % 新增：是否处于过渡期
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
        
        % 检查是否处于过渡期
        if isfield(point, 'transition_info')
            trajectory_array(i).in_transition = true;
        else
            trajectory_array(i).in_transition = false;
        end
        
        % 根据模型类型填充对应参数（其他字段保留为空）
        if strcmp(point.motion_model, 'CA')
            if isfield(point.model_parameters, 'acceleration')
                trajectory_array(i).model_acceleration = point.model_parameters.acceleration;
            else
                trajectory_array(i).model_acceleration = [];
            end
            
            if isfield(point.model_parameters, 'aligned_with_velocity')
                trajectory_array(i).aligned_with_velocity = point.model_parameters.aligned_with_velocity;
            else
                trajectory_array(i).aligned_with_velocity = [];
            end
            
            trajectory_array(i).model_turn_rate = [];  % 显式设为空
            trajectory_array(i).model_beta = [];
        elseif strcmp(point.motion_model, 'CT')
            trajectory_array(i).model_acceleration = [];
            trajectory_array(i).aligned_with_velocity = [];
            if isfield(point.model_parameters, 'turn_rate')
                trajectory_array(i).model_turn_rate = point.model_parameters.turn_rate;
            else
                trajectory_array(i).model_turn_rate = [];
            end
            trajectory_array(i).model_beta = [];
        elseif strcmp(point.motion_model, 'Singer')
            trajectory_array(i).model_acceleration = [];
            trajectory_array(i).aligned_with_velocity = [];
            trajectory_array(i).model_turn_rate = [];
            if isfield(point.model_parameters, 'beta')
                trajectory_array(i).model_beta = point.model_parameters.beta;
            else
                trajectory_array(i).model_beta = [];
            end
        else  % CV模型
            trajectory_array(i).model_acceleration = [];
            trajectory_array(i).aligned_with_velocity = [];
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
            if isfield(meas_point, 'noise_info') && length(meas_point.noise_info) >= j
                noise_info = meas_point.noise_info{j};
                rx_meas_array(j).snr_db = noise_info.snr_db;
                rx_meas_array(j).range_noise_std = noise_info.range_noise_std;
                rx_meas_array(j).angle_noise_std = noise_info.angle_noise_std;
                rx_meas_array(j).is_heavy_tail = noise_info.is_heavy_tail;
            else
                % 默认值
                rx_meas_array(j).snr_db = 20;
                rx_meas_array(j).range_noise_std = 5;
                rx_meas_array(j).angle_noise_std = 0.01;
                rx_meas_array(j).is_heavy_tail = false;
            end
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
            warning('无法保存JSON文件，请安装JSONlab工具箱或使用R2016b以上版本MATLAB');
        end
    end
end

%% ================ 测试函数（增强版） ================
function run_all_tests_v53()
    % 运行所有单元测试（增强版）
    
    fprintf('\n');
    fprintf('*****************************************************\n');
    fprintf('*  运动学模型单元测试套件 v5.3                      *\n');
    fprintf('*****************************************************\n');
    
    % 测试CA模型（加速度范数限制和方向选择）
    test_ca_model_v53();
    
    % 测试模型平滑过渡
    test_smooth_transition_v53();
    
    % 测试不同模型组合
    test_model_transitions_v53();
    
    % 测试CA模型加速度与速度方向夹角分布
    test_ca_velocity_alignment_v53();
    
    fprintf('\n*****************************************************\n');
    fprintf('*  所有测试完成！                                   *\n');
    fprintf('*****************************************************\n\n');
end

function test_ca_model_v53()
    % 测试CA模型的两种加速度方向生成方式
    
    fprintf('\n=====================================================\n');
    fprintf('  CA模型改进版单元测试 v5.3\n');
    fprintf('=====================================================\n\n');
    
    config = create_radar_config_v5(2);
    config.ca_model.aligned_probability = 0.5;  % 50%概率与速度方向一致
    
    % 设置测试参数
    num_tests = 1000;
    max_acceleration = 10.0;
    
    % 初始化统计数据
    aligned_count = 0;
    random_count = 0;
    aligned_angles = [];
    random_angles = [];
    
    fprintf('生成%d个CA模型加速度向量测试样本...\n', num_tests);
    
    % 测试不同速度方向下的加速度生成
    for test = 1:num_tests
        % 随机生成速度向量
        vx = (rand() - 0.5) * 100;
        vy = (rand() - 0.5) * 100;
        current_state = [0, 0, vx, vy, 0, 0];
        
        % 生成CA模型参数
        model_params = generate_model_parameters_v53('CA', config, ...
            struct('max_acceleration', max_acceleration), current_state);
        
        % 检查加速度类型和范数
        acc_vector = model_params.acceleration;
        acc_norm = norm(acc_vector);
        
        % 确认加速度范数约束
        if acc_norm > max_acceleration * 1.001  % 允许1‰误差
            fprintf('警告：加速度范数超过约束: %.3f > %.3f\n', acc_norm, max_acceleration);
        end
        
        % 计算与速度向量的夹角（只在速度非零时有意义）
        vel_vector = [vx, vy];
        vel_norm = norm(vel_vector);
        
        if vel_norm > 1e-6
            vel_unit = vel_vector / vel_norm;
            acc_unit = acc_vector / acc_norm;
            cos_angle = dot(vel_unit, acc_unit);
            angle = acos(min(max(cos_angle, -1), 1)) * 180/pi;
            
            if model_params.aligned_with_velocity
                aligned_count = aligned_count + 1;
                aligned_angles(end+1) = angle;
            else
                random_count = random_count + 1;
                random_angles(end+1) = angle;
            end
        end
    end
    
    % 统计与显示结果
    fprintf('\n测试结果统计：\n');
    fprintf('与速度方向一致的CA模型: %d (%.1f%%)\n', aligned_count, 100*aligned_count/num_tests);
    fprintf('随机方向的CA模型: %d (%.1f%%)\n', random_count, 100*random_count/num_tests);
    
    fprintf('\n与速度方向一致的CA模型夹角统计：\n');
    if ~isempty(aligned_angles)
        fprintf('最小角度: %.2f°\n', min(aligned_angles));
        fprintf('平均角度: %.2f°\n', mean(aligned_angles));
        fprintf('最大角度: %.2f°\n', max(aligned_angles));
    end
    
    fprintf('\n随机方向的CA模型夹角统计：\n');
    if ~isempty(random_angles)
        fprintf('最小角度: %.2f°\n', min(random_angles));
        fprintf('平均角度: %.2f°\n', mean(random_angles));
        fprintf('最大角度: %.2f°\n', max(random_angles));
    end
    
    % 绘图展示
    figure('Position', [100, 100, 1200, 500]);
    
    % 绘制加速度方向分布
    subplot(1, 2, 1);
    histogram(aligned_angles, 10, 'FaceColor', 'b', 'FaceAlpha', 0.7, 'DisplayName', '与速度一致');
    hold on;
    histogram(random_angles, 36, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', '随机方向');
    xlabel('与速度方向的夹角 (度)');
    ylabel('频数');
    title('CA模型加速度方向分布');
    legend('Location', 'best');
    grid on;
    
    % 绘制加速度-速度方向相关性散点图
    subplot(1, 2, 2);
    if ~isempty(aligned_angles)
        plot(ones(size(aligned_angles)), aligned_angles, 'b.', 'MarkerSize', 8);
    end
    hold on;
    if ~isempty(random_angles)
        plot(2*ones(size(random_angles)), random_angles, 'r.', 'MarkerSize', 8);
    end
    xlim([0.5, 2.5]);
    ylim([0, 180]);
    xticks([1, 2]);
    xticklabels({'与速度一致', '随机方向'});
    ylabel('与速度方向的夹角 (度)');
    title('CA模型两种模式的方向分布');
    grid on;
    
    fprintf('\n=====================================================\n');
    fprintf('  CA模型改进版测试完成\n');
    fprintf('=====================================================\n\n');
end

function test_smooth_transition_v53()
    % 测试模型平滑过渡功能（两种CA模型）
    
    fprintf('\n=====================================================\n');
    fprintf('  模型平滑过渡单元测试 v5.3\n');
    fprintf('=====================================================\n\n');
    
    config = create_radar_config_v5(2);
    config.model_transition.enable = true;
    config.model_transition.duration = 1.0;  % 1秒过渡期
    config.ca_model.aligned_probability = 0.5;
    
    % 初始状态 [x, y, vx, vy, ax, ay]
    initial_state = [0, 0, 20, 0, 0, 0];  % 初始速度20 m/s沿x方向
    
    % 定义测试场景：CV -> CA(随机) -> CA(一致) -> CV
    dt = 0.1;
    total_time = 12.0;  % 12秒
    time_steps = total_time / dt;
    
    % 模型序列
    model_sequence = cell(1, time_steps);
    for t = 1:time_steps
        current_time = (t-1) * dt;
        
        if current_time < 2.0
            % CV模型
            model_sequence{t}.model = 'CV';
            model_sequence{t}.params = struct();
        elseif current_time < 5.0
            % CA模型(随机方向)
            model_sequence{t}.model = 'CA';
            model_sequence{t}.params = struct('acceleration', [2.0, 1.0], 'aligned_with_velocity', false);
            
            % 检测模型切换点
            if t > 1 && ~strcmp(model_sequence{t-1}.model, 'CA')
                fprintf('时刻 %.1f：模型切换 %s -> CA(随机方向)\n', current_time, model_sequence{t-1}.model);
                model_sequence{t}.transition = struct();
                model_sequence{t}.transition.start_time = current_time;
                model_sequence{t}.transition.duration = config.model_transition.duration;
                model_sequence{t}.transition.start_acc = [0, 0];  % 从零加速度开始
                model_sequence{t}.transition.target_acc = [2.0, 1.0];  % 目标加速度
                model_sequence{t}.transition.aligned_with_velocity = false;
            end
        elseif current_time < 9.0
            % CA模型(与速度一致)
            model_sequence{t}.model = 'CA';
            model_sequence{t}.params = struct('acceleration', [3.0, 0], 'aligned_with_velocity', true);
            
            % 检测模型切换点
            if t > 1 && (strcmp(model_sequence{t-1}.model, 'CV') || ...
                         (strcmp(model_sequence{t-1}.model, 'CA') && ...
                          ~model_sequence{t-1}.params.aligned_with_velocity))
                fprintf('时刻 %.1f：模型切换 %s -> CA(与速度一致)\n', current_time, model_sequence{t-1}.model);
                model_sequence{t}.transition = struct();
                model_sequence{t}.transition.start_time = current_time;
                model_sequence{t}.transition.duration = config.model_transition.duration;
                
                if strcmp(model_sequence{t-1}.model, 'CA')
                    model_sequence{t}.transition.start_acc = model_sequence{t-1}.params.acceleration;
                else
                    model_sequence{t}.transition.start_acc = [0, 0];
                end
                model_sequence{t}.transition.target_acc = [3.0, 0];
                model_sequence{t}.transition.aligned_with_velocity = true;
            end
        else
            % CV模型
            model_sequence{t}.model = 'CV';
            model_sequence{t}.params = struct();
            
            % 检测模型切换点
            if t > 1 && ~strcmp(model_sequence{t-1}.model, 'CV')
                fprintf('时刻 %.1f：模型切换 %s -> CV\n', current_time, model_sequence{t-1}.model);
            end
        end
    end
    
    % 噪声序列（简单起见，使用恒定值）
    noise_schedule = cell(1, time_steps);
    for t = 1:time_steps
        noise_schedule{t}.intensity = 0.01;
    end
    
    % 模拟轨迹
    state = initial_state;
    states = zeros(time_steps, 6);
    states(1, :) = state;
    
    accelerations = zeros(time_steps, 2);
    accelerations(1, :) = state(5:6);
    
    ca_aligned = zeros(time_steps, 1);
    
    for t = 2:time_steps
        current_time = (t-1) * dt;
        
        % 检查CA模型类型
        if strcmp(model_sequence{t}.model, 'CA')
            ca_aligned(t) = model_sequence{t}.params.aligned_with_velocity;
        end
        
        % 应用平滑过渡
        if isfield(model_sequence{t}, 'transition')
            transition = model_sequence{t}.transition;
            elapsed = current_time - transition.start_time;
            
            if elapsed <= transition.duration
                % 正在过渡期内，线性插值加速度
                progress = elapsed / transition.duration;
                interpolated_acc = transition.start_acc + ...
                    progress * (transition.target_acc - transition.start_acc);
                
                % 更新状态向量中的加速度
                state(5:6) = interpolated_acc;
                
                % 输出过渡信息
                fprintf('  时刻%.1fs: 加速度过渡 %.1f%% [%.2f, %.2f]', ...
                   current_time, progress*100, interpolated_acc(1), interpolated_acc(2));
                
                if isfield(transition, 'aligned_with_velocity')
                    if transition.aligned_with_velocity
                        fprintf(' (与速度方向一致的CA模型)\n');
                    else
                        fprintf(' (随机方向CA模型)\n');
                    end
                else
                    fprintf('\n');
                end
            end
        end
        
        % 记录当前加速度
        accelerations(t, :) = state(5:6);
        
        % 更新状态
        target_params = struct('max_velocity', 100, 'max_acceleration', 10);
        state = update_state_v5(state, model_sequence{t}, ...
                noise_schedule{t}, config, target_params);
        states(t, :) = state;
    end
    
    % 计算加速度与速度方向的夹角
    vel_acc_angles = zeros(time_steps, 1);
    for t = 1:time_steps
        vel = states(t, 3:4);
        acc = accelerations(t, :);
        
        vel_norm = norm(vel);
        acc_norm = norm(acc);
        
        if vel_norm > 1e-6 && acc_norm > 1e-6
            cos_angle = dot(vel, acc) / (vel_norm * acc_norm);
            vel_acc_angles(t) = acos(min(max(cos_angle, -1), 1)) * 180/pi;
        else
            vel_acc_angles(t) = NaN;
        end
    end
    
    % 可视化结果
    figure('Position', [100, 100, 1200, 900]);
    
    % 轨迹
    subplot(3, 2, 1);
    plot(states(:, 1), states(:, 2), 'b-', 'LineWidth', 2);
    hold on;
    
    % 标记模型切换点
    model_changes = [];
    for t = 2:time_steps
        if ~strcmp(model_sequence{t}.model, model_sequence{t-1}.model) || ...
           (strcmp(model_sequence{t}.model, 'CA') && strcmp(model_sequence{t-1}.model, 'CA') && ...
            model_sequence{t}.params.aligned_with_velocity ~= model_sequence{t-1}.params.aligned_with_velocity)
            model_changes = [model_changes, t];
            
            % 不同颜色标记不同类型的模型切换
            if strcmp(model_sequence{t}.model, 'CA')
                if model_sequence{t}.params.aligned_with_velocity
                    plot(states(t, 1), states(t, 2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
                    text(states(t, 1), states(t, 2) + 1, 'CA-A', 'FontSize', 8);
                else
                    plot(states(t, 1), states(t, 2), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
                    text(states(t, 1), states(t, 2) + 1, 'CA-R', 'FontSize', 8);
                end
            else
                plot(states(t, 1), states(t, 2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            end
        end
    end
    
    grid on;
    axis equal;
    xlabel('x (m)');
    ylabel('y (m)');
    title('轨迹');
    
    % 速度
    subplot(3, 2, 2);
    times = (0:time_steps-1) * dt;
    
    plot(times, states(:, 3), 'r-', 'LineWidth', 1.5, 'DisplayName', 'vx');
    hold on;
    plot(times, states(:, 4), 'b-', 'LineWidth', 1.5, 'DisplayName', 'vy');
    plot(times, sqrt(states(:, 3).^2 + states(:, 4).^2), 'k--', 'LineWidth', 1, 'DisplayName', '|v|');
    
    % 标记模型切换点
    for t = model_changes
        time_point = (t-1) * dt;
        plot([time_point, time_point], ylim, 'k--');
    end
    
    grid on;
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    title('速度分量');
    legend('Location', 'best');
    
    % 加速度
    subplot(3, 2, 3);
    
    plot(times, accelerations(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'ax');
    hold on;
    plot(times, accelerations(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'ay');
    plot(times, sqrt(accelerations(:, 1).^2 + accelerations(:, 2).^2), 'k--', 'LineWidth', 1, 'DisplayName', '|a|');
    
    % 标记模型切换点
    for t = model_changes
        time_point = (t-1) * dt;
        plot([time_point, time_point], ylim, 'k--');
    end
    
    grid on;
    xlabel('时间 (s)');
    ylabel('加速度 (m/s²)');
    title('加速度分量');
    legend('Location', 'best');
    
    % 速度-加速度夹角
    subplot(3, 2, 4);
    plot(times, vel_acc_angles, 'k-', 'LineWidth', 1.5);
    hold on;
    
    % 标记模型切换点
    for t = model_changes
        time_point = (t-1) * dt;
        plot([time_point, time_point], ylim, 'k--');
    end
    
    % 添加CA模型类型指示
    for t = 1:time_steps
        if strcmp(model_sequence{t}.model, 'CA')
            if model_sequence{t}.params.aligned_with_velocity
                % 与速度方向一致
                plot(times(t), 0, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
            else
                % 随机方向
                plot(times(t), 90, 'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
            end
        end
    end
    
    grid on;
    xlabel('时间 (s)');
    ylabel('夹角 (度)');
    title('速度-加速度夹角');
    ylim([0, 180]);
    yticks(0:30:180);
    
    % 模型时间线
    subplot(3, 2, [5, 6]);
    hold on;
    
    current_model = model_sequence{1}.model;
    current_ca_type = -1;  % 初始为非CA模型
    if strcmp(current_model, 'CA')
        current_ca_type = model_sequence{1}.params.aligned_with_velocity;
    end
    
    start_time = 0;
    y_pos = 1;
    
    % 定义颜色映射
    colors = containers.Map();
    colors('CV') = [0, 0.4470, 0.7410];  % 蓝色
    colors('CA_ALIGNED') = [0.4660, 0.6740, 0.1880];  % 绿色
    colors('CA_RANDOM') = [0.9290, 0.6940, 0.1250];   % 黄色
    colors('CT') = [0.8500, 0.3250, 0.0980];  % 橙色
    colors('Singer') = [0.4940, 0.1840, 0.5560];  % 紫色
    
    for t = 2:time_steps
        model_changed = false;
        
        if ~strcmp(model_sequence{t}.model, current_model)
            model_changed = true;
        elseif strcmp(model_sequence{t}.model, 'CA') && strcmp(current_model, 'CA')
            % 检查CA类型是否改变
            ca_type = model_sequence{t}.params.aligned_with_velocity;
            if ca_type ~= current_ca_type
                model_changed = true;
            end
        end
        
        if model_changed || t == time_steps
            end_time = (t-1) * dt;
            
            % 确定颜色和标签
            if strcmp(current_model, 'CA')
                if current_ca_type
                    color = colors('CA_ALIGNED');
                    label = 'CA-Aligned';
                else
                    color = colors('CA_RANDOM');
                    label = 'CA-Random';
                end
            else
                color = colors(current_model);
                label = current_model;
            end
            
            % 绘制模型段
            rectangle('Position', [start_time, 0.7, end_time-start_time, 0.6], ...
                     'FaceColor', color, 'EdgeColor', 'k');
            
            % 添加标签
            text((start_time + end_time)/2, 1, label, ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                'FontWeight', 'bold');
            
            % 准备下一段
            current_model = model_sequence{t}.model;
            if strcmp(current_model, 'CA')
                current_ca_type = model_sequence{t}.params.aligned_with_velocity;
            else
                current_ca_type = -1;
            end
            start_time = end_time;
        end
        
        % 标记过渡期
        if isfield(model_sequence{t}, 'transition')
            transition = model_sequence{t}.transition;
            start_trans = transition.start_time;
            end_trans = start_trans + transition.duration;
            
            % 用橙色竖线标记过渡起点
            plot([start_trans, start_trans], [0.7, 1.3], 'Color', [1 0.5 0], 'LineWidth', 2);
            
            % 用橙色竖线标记过渡终点
            plot([end_trans, end_trans], [0.7, 1.3], 'Color', [1 0.5 0], 'LineWidth', 2);
            
            % 过渡期橙色底色
            rectangle('Position', [start_trans, 0.7, end_trans-start_trans, 0.6], ...
                     'FaceColor', [1 0.8 0.6], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        end
    end
    
    xlim([0, total_time]);
    ylim([0.5, 1.5]);
    xlabel('时间 (s)');
    title('运动模型时间线');
    set(gca, 'YTick', []);
    
    % 添加图例
    legend_handles = [
        patch([0 0 0 0], [0 0 0 0], colors('CV'), 'EdgeColor', 'k'),
        patch([0 0 0 0], [0 0 0 0], colors('CA_ALIGNED'), 'EdgeColor', 'k'),
        patch([0 0 0 0], [0 0 0 0], colors('CA_RANDOM'), 'EdgeColor', 'k'),
        rectangle('Position', [0, 0, 0, 0], 'FaceColor', [1 0.8 0.6], 'EdgeColor', 'none')
    ];
    legend(legend_handles, {'CV', 'CA-Aligned', 'CA-Random', '过渡期'}, ...
           'Location', 'southoutside', 'Orientation', 'horizontal');
    
    fprintf('\n=====================================================\n');
    fprintf('  模型平滑过渡测试完成\n');
    fprintf('=====================================================\n\n');
end

function test_model_transitions_v53()
    % 测试所有可能的模型转换组合场景
    
    fprintf('\n=====================================================\n');
    fprintf('  模型切换组合场景测试 v5.3\n');
    fprintf('=====================================================\n\n');
    
    % 创建配置
    config = create_radar_config_v5(2);
    config.model_transition.enable = true;
    config.model_transition.duration = 1.0;
    config.ca_model.aligned_probability = 0.5;
    
    % 1. 测试CV->CA(随机)->CT->CA(一致)->CV的完整场景
    fprintf('【测试1】: CV->CA(随机)->CT->CA(一致)->CV 模型序列场景\n');
    
    % 定义时间步长
    dt = 0.1;
    total_time = 20.0;
    time_steps = total_time / dt;
    
    % 初始状态 [x, y, vx, vy, ax, ay]
    initial_state = [0, 0, 10, 5, 0, 0];
    
    % 生成复合模型序列
    model_sequence = create_test_model_sequence_v53(dt, time_steps);
    
    % 噪声序列（简单起见，使用恒定值）
    noise_schedule = cell(1, time_steps);
    for t = 1:time_steps
        noise_schedule{t}.intensity = 0.05;
    end
    
    % 模拟轨迹
    fprintf('生成测试轨迹...\n');
    state = initial_state;
    states = zeros(time_steps, 6);
    states(1, :) = state;
    
    accelerations = zeros(time_steps, 2);
    accelerations(1, :) = state(5:6);
    
    transition_active = false;
    transition_data = struct();
    
    for t = 2:time_steps
        current_time = (t-1) * dt;
        current_model = model_sequence{t}.model;
        
        % 检测模型切换
        if t > 1 && ~strcmp(current_model, model_sequence{t-1}.model)
            fprintf('  时刻%.1fs: 模型切换 %s -> %s\n', ...
                current_time, model_sequence{t-1}.model, current_model);
            
            if strcmp(current_model, 'CA')
                % CA模型切换，设置过渡
                ca_type = '';
                if model_sequence{t}.params.aligned_with_velocity
                    ca_type = '与速度方向一致的';
                else
                    ca_type = '随机方向的';
                end
                fprintf('    切换到%sCA模型，目标加速度=[%.2f, %.2f]\n', ...
                    ca_type, model_sequence{t}.params.acceleration(1), ...
                    model_sequence{t}.params.acceleration(2));
                
                % 初始化过渡
                transition_active = true;
                transition_data = struct();
                transition_data.start_time = current_time;
                transition_data.duration = config.model_transition.duration;
                transition_data.start_acc = state(5:6);
                transition_data.target_acc = model_sequence{t}.params.acceleration;
            end
        end
        
        % 应用过渡
        if transition_active
            elapsed = current_time - transition_data.start_time;
            if elapsed <= transition_data.duration
                progress = elapsed / transition_data.duration;
                interpolated_acc = transition_data.start_acc + ...
                    progress * (transition_data.target_acc - transition_data.start_acc);
                state(5:6) = interpolated_acc;
            else
                transition_active = false;
            end
        end
        
        % 记录当前加速度
        accelerations(t, :) = state(5:6);
        
        % 更新状态
        target_params = struct('max_velocity', 100, 'max_acceleration', 10, 'max_turn_rate', 0.2);
        state = update_state_v5(state, model_sequence{t}, ...
                noise_schedule{t}, config, target_params);
        states(t, :) = state;
    end
    
    % 绘制结果
    figure('Position', [100, 100, 1000, 800], 'Name', '模型切换组合测试');
    
    % 轨迹图
    subplot(2, 2, 1);
    plot(states(:, 1), states(:, 2), 'b-', 'LineWidth', 2);
    hold on;
    
    % 标记模型切换点和添加箭头指示方向
    quiver_step = 20;
    for t = 1:quiver_step:time_steps
        quiver(states(t, 1), states(t, 2), states(t, 3), states(t, 4), 0.5, ...
            'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    end
    
    % 标记模型切换点
    model_changes = [];
    for t = 2:time_steps
        if ~strcmp(model_sequence{t}.model, model_sequence{t-1}.model)
            model_changes = [model_changes, t];
            plot(states(t, 1), states(t, 2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            text(states(t, 1)+5, states(t, 2)+5, model_sequence{t}.model, 'FontSize', 9);
        end
    end
    
    grid on;
    axis equal;
    xlabel('x (m)');
    ylabel('y (m)');
    title('轨迹和模型切换点');
    
    % 速度图
    subplot(2, 2, 2);
    times = (0:time_steps-1) * dt;
    
    plot(times, sqrt(states(:, 3).^2 + states(:, 4).^2), 'b-', 'LineWidth', 2);
    hold on;
    
    % 标记模型切换点
    for t = model_changes
        time_point = (t-1) * dt;
        plot([time_point, time_point], ylim, 'r--', 'LineWidth', 1.5);
        text(time_point, max(ylim)*0.9, model_sequence{t}.model, ...
            'FontSize', 8, 'HorizontalAlignment', 'center');
    end
    
    grid on;
    xlabel('时间 (s)');
    ylabel('速度大小 (m/s)');
    title('速度大小随时间变化');
    
    % 加速度图
    subplot(2, 2, 3);
    acc_norms = sqrt(accelerations(:, 1).^2 + accelerations(:, 2).^2);
    plot(times, acc_norms, 'b-', 'LineWidth', 2);
    hold on;
    
    % 标记模型切换点
    for t = model_changes
        time_point = (t-1) * dt;
        plot([time_point, time_point], ylim, 'r--', 'LineWidth', 1.5);
        text(time_point, max(ylim)*0.9, model_sequence{t}.model, ...
            'FontSize', 8, 'HorizontalAlignment', 'center');
    end
    
    grid on;
    xlabel('时间 (s)');
    ylabel('加速度大小 (m/s²)');
    title('加速度大小随时间变化');
    
    % 模型时间线
    subplot(2, 2, 4);
    hold on;
    
    current_model = model_sequence{1}.model;
    start_time = 0;
    
    % 定义颜色映射
    colors = containers.Map();
    colors('CV') = [0, 0.4470, 0.7410];  % 蓝色
    colors('CA') = [0.8500, 0.3250, 0.0980];  % 橙色
    colors('CT') = [0.9290, 0.6940, 0.1250];  % 黄色
    colors('Singer') = [0.4940, 0.1840, 0.5560];  % 紫色
    
    for t = 2:time_steps
        if ~strcmp(model_sequence{t}.model, current_model) || t == time_steps
            end_time = (t-1) * dt;
            
            color = colors(current_model);
            
            % 绘制模型段
            rectangle('Position', [start_time, 0.7, end_time-start_time, 0.6], ...
                     'FaceColor', color, 'EdgeColor', 'k');
            
            % CA模型标注额外信息
            label = current_model;
            if strcmp(current_model, 'CA') && t > 1
                if isfield(model_sequence{t-1}.params, 'aligned_with_velocity') && ...
                   model_sequence{t-1}.params.aligned_with_velocity
                    label = 'CA-Aligned';
                else
                    label = 'CA-Random';
                end
            end
            
            % 添加标签
            text((start_time + end_time)/2, 1, label, ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                'FontWeight', 'bold');
            
            % 准备下一段
            current_model = model_sequence{t}.model;
            start_time = end_time;
        end
    end
    
    xlim([0, total_time]);
    ylim([0.5, 1.5]);
    xlabel('时间 (s)');
    title('模型切换时间线');
    set(gca, 'YTick', []);
    grid on;
    
    fprintf('\n模型切换测试完成。\n');
    fprintf('=====================================================\n\n');
end

function model_sequence = create_test_model_sequence_v53(dt, time_steps)
    % 创建用于测试的模型序列
    % CV->CA(随机)->CT->CA(一致)->CV
    
    model_sequence = cell(1, time_steps);
    total_time = time_steps * dt;
    
    for t = 1:time_steps
        current_time = (t-1) * dt;
        
        if current_time < 4.0
            % CV模型
            model_sequence{t}.model = 'CV';
            model_sequence{t}.params = struct();
        elseif current_time < 8.0
            % CA模型(随机方向)
            model_sequence{t}.model = 'CA';
            model_sequence{t}.params = struct('acceleration', [3.0, 2.0], 'aligned_with_velocity', false);
        elseif current_time < 12.0
            % CT模型
            model_sequence{t}.model = 'CT';
            model_sequence{t}.params = struct('turn_rate', 0.15);
        elseif current_time < 16.0
            % CA模型(与速度方向一致)
            model_sequence{t}.model = 'CA';
            model_sequence{t}.params = struct('acceleration', [4.0, 0], 'aligned_with_velocity', true);
        else
            % 回到CV模型
            model_sequence{t}.model = 'CV';
            model_sequence{t}.params = struct();
        end
    end
end

function test_ca_velocity_alignment_v53()
    % 测试CA模型加速度与速度方向夹角分布
    
    fprintf('\n=====================================================\n');
    fprintf('  CA模型加速度与速度方向夹角分布测试 v5.3\n');
    fprintf('=====================================================\n\n');
    
    % 创建配置
    config = create_radar_config_v5(2);
    config.ca_model.aligned_probability = 0.5;
    
    % 测试参数
    num_samples = 2000;
    max_acceleration = 10.0;
    
    % 预分配数组
    aligned_indices = [];
    random_indices = [];
    all_angles = zeros(num_samples, 1);
    all_is_aligned = zeros(num_samples, 1);
    
    % 生成不同速度下的加速度
    fprintf('生成%d个加速度样本...\n', num_samples);
    
    for i = 1:num_samples
        % 随机速度向量
        v_mag = 10 + 90 * rand();  % 速度大小在10-100m/s之间
        v_angle = 2 * pi * rand();  % 随机方向
        vx = v_mag * cos(v_angle);
        vy = v_mag * sin(v_angle);
        
        state = [0, 0, vx, vy, 0, 0];
        target_params = struct('max_acceleration', max_acceleration);
        
        % 生成CA模型参数
        model_params = generate_model_parameters_v53('CA', config, target_params, state);
        
        % 加速度向量
        ax = model_params.acceleration(1);
        ay = model_params.acceleration(2);
        
        % 计算夹角
        vel_vec = [vx, vy];
        acc_vec = [ax, ay];
        
        vel_mag = norm(vel_vec);
        acc_mag = norm(acc_vec);
        
        if vel_mag > 0 && acc_mag > 0
            cos_angle = dot(vel_vec, acc_vec) / (vel_mag * acc_mag);
            angle = acos(min(max(cos_angle, -1), 1)) * 180/pi;
        else
            angle = 0;
        end
        
        all_angles(i) = angle;
        all_is_aligned(i) = model_params.aligned_with_velocity;
        
        % 记录索引以便分组
        if model_params.aligned_with_velocity
            aligned_indices = [aligned_indices, i];
        else
            random_indices = [random_indices, i];
        end
    end
    
    % 分析结果
    fprintf('\n分析结果:\n');
    fprintf('总样本数: %d\n', num_samples);
    fprintf('与速度方向一致的样本数: %d (%.1f%%)\n', length(aligned_indices), 100*length(aligned_indices)/num_samples);
    fprintf('随机方向的样本数: %d (%.1f%%)\n', length(random_indices), 100*length(random_indices)/num_samples);
    
    % 对齐类型的样本夹角统计
    if ~isempty(aligned_indices)
        aligned_angles = all_angles(aligned_indices);
        fprintf('\n与速度方向一致的样本夹角统计:\n');
        fprintf('  最小角度: %.2f°\n', min(aligned_angles));
        fprintf('  平均角度: %.2f°\n', mean(aligned_angles));
        fprintf('  最大角度: %.2f°\n', max(aligned_angles));
        fprintf('  标准差: %.2f°\n', std(aligned_angles));
    end
    
    % 随机类型的样本夹角统计
    if ~isempty(random_indices)
        random_angles = all_angles(random_indices);
        fprintf('\n随机方向的样本夹角统计:\n');
        fprintf('  最小角度: %.2f°\n', min(random_angles));
        fprintf('  平均角度: %.2f°\n', mean(random_angles));
        fprintf('  最大角度: %.2f°\n', max(random_angles));
        fprintf('  标准差: %.2f°\n', std(random_angles));
    end
    
    % 可视化结果
    figure('Position', [100, 100, 1200, 500], 'Name', 'CA模型加速度与速度方向夹角分布');
    
    % 直方图
    subplot(1, 2, 1);
    if ~isempty(aligned_indices)
        histogram(aligned_angles, 0:5:180, 'FaceColor', [0, 0.7, 0], 'FaceAlpha', 0.7, ...
            'DisplayName', '与速度一致');
    end
    hold on;
    if ~isempty(random_indices)
        histogram(random_angles, 0:5:180, 'FaceColor', [0.8, 0, 0], 'FaceAlpha', 0.5, ...
            'DisplayName', '随机方向');
    end
    xlabel('与速度方向的夹角 (度)');
    ylabel('频数');
    title('CA模型加速度与速度方向夹角分布');
    legend('Location', 'best');
    grid on;
    xlim([0, 180]);
    
    % 箱线图
    subplot(1, 2, 2);
    boxplot(all_angles, all_is_aligned, 'Labels', {'随机方向', '与速度一致'}, ...
        'Colors', [0.8, 0, 0; 0, 0.7, 0]);
    ylabel('与速度方向的夹角 (度)');
    title('CA模型两种类型的角度分布');
    grid on;
    
    fprintf('\n=====================================================\n');
    fprintf('  CA模型加速度方向测试完成\n');
    fprintf('=====================================================\n\n');
end