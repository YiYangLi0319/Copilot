%% 增强版双基地雷达数据集生成器
% 统一9维状态建模，合理噪声设计，JSON格式存储
% 新特性：
% 1. 所有模型统一9维状态 [x,y,z,vx,vy,vz,ax,ay,az]
% 2. 基于双基地距离的合理量测噪声
% 3. OU过程建模的时变过程噪声
% 4. 单目标场景
% 5. 详细的JSON格式存储

clear all; close all; clc;

%% ================ 主程序 ================

fprintf('=== 增强版双基地雷达数据集生成器 ===\n\n');

% 运行配置
run_demo = true;        
save_data = true;       
num_scenarios = 1000;     % 场景数量
single_target = true;   % 单目标场景

%% 生成2D数据集
fprintf('生成2D单目标数据集...\n');
config_2d = create_enhanced_radar_config(2);
dataset_2d = generate_enhanced_dataset(config_2d, num_scenarios, single_target, 'enhanced_2d_dataset');

%% 生成3D数据集  
fprintf('生成3D单目标数据集...\n');
config_3d = create_enhanced_radar_config(3);
dataset_3d = generate_enhanced_dataset(config_3d, num_scenarios, single_target, 'enhanced_3d_dataset');

%% 演示可视化
if run_demo
    fprintf('\n可视化演示...\n');
    visualize_enhanced_scenario(dataset_2d.scenarios{1}, '2D增强场景');
    visualize_enhanced_scenario(dataset_3d.scenarios{1}, '3D增强场景');
end

%% 数据集分析
fprintf('\n=== 数据集统计分析 ===\n');
analyze_enhanced_dataset('enhanced_2d_dataset');
analyze_enhanced_dataset('enhanced_3d_dataset');

fprintf('\n增强版数据集生成完成！\n');

%% ================ 配置函数 ================

function config = create_enhanced_radar_config(dimension)
    % 创建增强雷达配置参数
    config = struct();
    config.dimension = dimension;                    % 场景维度 (2D或3D)
    config.state_dimension = 9;                     % 统一9维状态
    config.num_receivers = 3;                       % 接收机数量
    config.simulation_time = 200;                   % 仿真时间(秒)
    config.dt = 1.0;                               % 数据率(秒)
    
    % 接收机分布参数
    config.receiver_area_radius = 8000;            % 接收机分布区域半径(m)
    config.min_receiver_distance = 1000;           % 接收机间最小距离(m)
    
    % 量测噪声参数 - 基于距离的合理建模
    config.range_noise_base = 5.0;                 % 基础距离噪声标准差(m)
    config.range_noise_factor = 0.001;             % 距离噪声因子 (noise = base + factor * range)
    config.angle_noise_base = 0.1 * pi/180;        % 基础角度噪声标准差(rad)
    config.angle_noise_factor = 1e-6;              % 角度噪声距离因子
    config.max_range_noise = 200.0;                % 最大距离噪声标准差(m)
    config.max_angle_noise = 2.0 * pi/180;         % 最大角度噪声标准差(rad)
    
    % 过程噪声参数 - OU过程建模
    config.process_noise_base = 0.5;               % 基础过程噪声标准差（均值水平）
    config.process_noise_ou_tau = 20.0;            % OU过程时间常数(秒)，越大变化越慢
    config.process_noise_ou_sigma = 0.8;           % OU过程噪声强度
    config.process_noise_ou_initial = 0.5;         % OU过程初始值
    
    % 运动模型切换参数
    config.model_change_interval = [20, 40];       % 模型切换时间间隔(秒)
    config.available_models = {'CV', 'CA', 'CT', 'Singer'}; 
    
    % 目标运动范围
    config.initial_position_range = 6000;          % 初始位置范围(m)
    config.initial_velocity_range = 30;            % 初始速度范围(m/s)
    config.max_acceleration = 3.0;                 % 最大加速度(m/s^2)
    config.max_turn_rate = 0.1;                    % 最大转弯率(rad/s)
end

function dataset = generate_enhanced_dataset(config, num_scenarios, single_target, output_dir)
    % 生成增强数据集
    
    if nargin < 4, output_dir = 'enhanced_bistatic_dataset'; end
    
    % 创建输出目录
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    dataset = struct();
    dataset.config = config;
    dataset.scenarios = {};
    dataset.metadata = struct();
    dataset.metadata.creation_time = datestr(now);
    dataset.metadata.num_scenarios = num_scenarios;
    dataset.metadata.single_target = single_target;
    
    fprintf('开始生成 %d 个增强场景...\n', num_scenarios);
    
    for scenario_idx = 1:num_scenarios
        if mod(scenario_idx, 5) == 0 || scenario_idx == 1
            fprintf('  完成 %d/%d 个场景\n', scenario_idx, num_scenarios);
        end
        
        scenario = generate_enhanced_scenario(config, single_target);
        scenario.scenario_id = scenario_idx - 1;
        dataset.scenarios{end+1} = scenario;
        
        % 保存单个场景为JSON
        save_scenario_json(scenario, output_dir, scenario_idx - 1);
    end
    
    % 保存完整数据集
    dataset_file = fullfile(output_dir, 'dataset.mat');
    save(dataset_file, 'dataset');
    
    % 保存数据集元信息为JSON
    dataset_info = struct();
    dataset_info.config = config;
    dataset_info.metadata = dataset.metadata;
    save_json_file(dataset_info, fullfile(output_dir, 'dataset_info.json'));
    
    fprintf('增强数据集已保存至: %s\n', output_dir);
end

function scenario = generate_enhanced_scenario(config, single_target)
    % 生成单个增强场景
    
    % 发射机位置（原点）
    transmitter_pos = zeros(1, config.dimension);
    
    % 生成接收机位置
    receivers_pos = generate_enhanced_receiver_positions(config);
    
    % 初始化场景结构
    scenario = struct();
    scenario.config = config;
    scenario.transmitter_position = transmitter_pos;
    scenario.receivers_positions = receivers_pos;
    scenario.target_info = struct();  % 单目标
    scenario.measurements = {};
    scenario.detailed_records = {};  % 详细记录
    
    time_steps = floor(config.simulation_time / config.dt);
    
    % 生成单目标数据
    [target_info, measurements, detailed_records] = generate_enhanced_target(...
        config, time_steps, transmitter_pos, receivers_pos);
    
    scenario.target_info = target_info;
    scenario.measurements = measurements;
    scenario.detailed_records = detailed_records;
end

function receivers_pos = generate_enhanced_receiver_positions(config)
    % 生成增强的接收机位置
    receivers_pos = {};
    max_attempts = 1000;
    
    for attempt = 1:max_attempts
        temp_receivers = {};
        
        % 生成候选位置
        for i = 1:config.num_receivers
            if config.dimension == 2
                angle = rand() * 2 * pi;
                radius = 2000 + rand() * (config.receiver_area_radius - 2000);
                pos = [radius * cos(angle), radius * sin(angle)];
            else % 3D
                phi = rand() * 2 * pi;      
                theta = pi/6 + rand() * (2*pi/3); % 避免极端仰角
                radius = 2000 + rand() * (config.receiver_area_radius - 2000);
                pos = [radius * sin(theta) * cos(phi), ...
                       radius * sin(theta) * sin(phi), ...
                       radius * cos(theta)];
            end
            temp_receivers{end+1} = pos;
        end
        
        % 检查几何条件
        if check_enhanced_geometry(temp_receivers, config)
            receivers_pos = temp_receivers;
            break;
        end
    end
    
    if isempty(receivers_pos)
        error('无法生成满足几何条件的接收机位置');
    end
end

function is_valid = check_enhanced_geometry(receivers, config)
    % 检查增强几何条件
    is_valid = true;
    
    % 检查最小距离
    for i = 1:length(receivers)
        for j = i+1:length(receivers)
            dist = norm(receivers{i} - receivers{j});
            if dist < config.min_receiver_distance
                is_valid = false;
                return;
            end
        end
    end
    
    % 检查几何精度衰减因子(GDOP)
    if length(receivers) >= 3
        % 简化的GDOP检查：确保接收机不在一条直线或一个平面上
        p1 = receivers{1};
        p2 = receivers{2};
        p3 = receivers{3};
        
        v1 = p2 - p1;
        v2 = p3 - p1;
        
        if config.dimension == 2
            % 2D: 检查叉积
            cross_2d = abs(v1(1)*v2(2) - v1(2)*v2(1));
            if cross_2d < 1e6  % 面积太小
                is_valid = false;
                return;
            end
        else % 3D
            % 3D: 检查三重积
            cross_prod = cross(v1, v2);
            if norm(cross_prod) < 500  % 体积太小
                is_valid = false;
                return;
            end
        end
    end
end

function [target_info, measurements, detailed_records] = generate_enhanced_target(...
    config, time_steps, transmitter_pos, receivers_pos)
    % 生成增强的单目标数据
    
    % 随机初始条件
    pos_range = config.initial_position_range;
    vel_range = config.initial_velocity_range;
    
    if config.dimension == 2
        initial_pos = [(rand() - 0.5) * pos_range, (rand() - 0.5) * pos_range, 0];
        initial_vel = [(rand() - 0.5) * vel_range, (rand() - 0.5) * vel_range, 0];
    else % 3D
        initial_pos = [(rand() - 0.5) * pos_range, (rand() - 0.5) * pos_range, rand() * pos_range/2];
        initial_vel = [(rand() - 0.5) * vel_range, (rand() - 0.5) * vel_range, (rand() - 0.5) * vel_range/2];
    end
    initial_acc = [0, 0, 0];  % 初始加速度为0
    
    % 统一9维状态：[x, y, z, vx, vy, vz, ax, ay, az]
    state = [initial_pos, initial_vel, initial_acc];
    
    % 生成运动模型序列
    model_sequence = generate_enhanced_model_sequence(config, time_steps);
    
    % 生成过程噪声序列（OU过程）
    process_noise_sequence = generate_ou_process_noise(config, time_steps);
    
    % 初始化记录
    target_info = struct();
    target_info.initial_state = state;
    target_info.motion_model_sequence = model_sequence;
    target_info.process_noise_sequence = process_noise_sequence;
    target_info.trajectory = {};
    
    measurements = {};
    detailed_records = {};
    
    % 时间序列仿真
    for t = 1:time_steps
        current_time = (t-1) * config.dt;
        current_pos = state(1:3);  % 3D位置
        current_vel = state(4:6);  % 3D速度
        current_acc = state(7:9);  % 3D加速度
        
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
        
        % 生成测量
        [target_measurements, measurement_noise_info] = generate_enhanced_measurements(...
            current_pos, transmitter_pos, receivers_pos, config);
        
        % 记录测量
        meas_point = struct();
        meas_point.time = current_time;
        meas_point.measurements = target_measurements;
        meas_point.noise_info = measurement_noise_info;
        measurements{end+1} = meas_point;
        
        % 详细记录
        detailed_record = struct();
        detailed_record.time = current_time;
        detailed_record.true_state = state;
        detailed_record.true_position = current_pos;
        detailed_record.true_velocity = current_vel;
        detailed_record.true_acceleration = current_acc;
        detailed_record.motion_model = model_sequence{t}.model;
        detailed_record.model_parameters = model_sequence{t}.params;
        detailed_record.process_noise_std = process_noise_sequence(t);
        detailed_record.measurement_noise_info = measurement_noise_info;
        detailed_record.measurements = target_measurements;
        
        % 计算真实距离和角度（无噪声）
        true_measurements = {};
        for i = 1:length(receivers_pos)
            receiver_pos = receivers_pos{i};
            [true_range, true_azimuth, true_elevation] = compute_true_measurement(...
                current_pos, transmitter_pos, receiver_pos, config.dimension);
            true_measurements{end+1} = [true_range, true_azimuth, true_elevation];
        end
        detailed_record.true_measurements = true_measurements;
        
        detailed_records{end+1} = detailed_record;
        
        % 状态更新
        if t < time_steps
            current_model = model_sequence{t};
            current_noise_std = process_noise_sequence(t);
            
            state = update_enhanced_state(state, current_model, ...
                current_noise_std, config.dt, config);
        end
    end
end

function model_sequence = generate_enhanced_model_sequence(config, time_steps)
    % 生成增强的运动模型序列
    
    model_sequence = cell(1, time_steps);
    current_time = 1;
    available_models = config.available_models;
    
    while current_time <= time_steps
        % 选择运动模型
        model_name = available_models{randi(length(available_models))};
        
        % 生成模型参数
        model_params = generate_enhanced_model_parameters(model_name, config);
        
        % 确定模型持续时间
        duration_range = config.model_change_interval;
        duration = duration_range(1) + rand() * diff(duration_range);
        duration_steps = max(10, round(duration / config.dt));  % 至少10步
        
        % 填充序列
        end_time = min(current_time + duration_steps - 1, time_steps);
        
        for t = current_time:end_time
            model_sequence{t} = struct('model', model_name, 'params', model_params);
        end
        
        current_time = end_time + 1;
    end
end

function model_params = generate_enhanced_model_parameters(model_name, config)
    % 生成增强的模型参数
    model_params = struct();
    
    switch model_name
        case 'CV'
            % 恒速模型：加速度保持为0
            model_params.acceleration = [0, 0, 0];
            
        case 'CA'
            % 恒加速模型：随机加速度
            max_acc = config.max_acceleration;
            if config.dimension == 2
                model_params.acceleration = [(rand() - 0.5) * max_acc, ...
                                           (rand() - 0.5) * max_acc, 0];
            else % 3D
                model_params.acceleration = [(rand() - 0.5) * max_acc, ...
                                           (rand() - 0.5) * max_acc, ...
                                           (rand() - 0.5) * max_acc/2];
            end
            
        case 'CT'
            % 恒转弯模型
            max_turn = config.max_turn_rate;
            model_params.turn_rate = (rand() - 0.5) * max_turn;
            
        case 'Singer'
            % Singer模型
            model_params.alpha = 0.05 + rand() * 0.1;  % 0.05-0.15
            model_params.max_acceleration = config.max_acceleration;
    end
end

function noise_sequence = generate_ou_process_noise(config, time_steps)
    % 生成基于OU过程的过程噪声序列
    % OU过程: da = -(1/tau)*a*dt + sigma*sqrt(dt)*randn()
    % 离散形式: noise(i) = alpha*noise(i-1) + sigma*sqrt(1-alpha^2)*randn()
    
    base_noise = config.process_noise_base;
    tau = config.process_noise_ou_tau;              % 时间常数
    sigma = config.process_noise_ou_sigma;          % 噪声强度  
    dt = config.dt;
    initial_value = config.process_noise_ou_initial;
    
    % OU过程参数
    alpha = exp(-dt/tau);                           % 相关系数
    ou_sigma = sigma * sqrt(1 - alpha^2);           % 调整后的噪声强度
    
    % 初始化噪声序列
    noise_sequence = zeros(1, time_steps);
    noise_sequence(1) = initial_value;
    
    % 生成OU过程
    for i = 2:time_steps
        % OU过程递推公式
        noise_sequence(i) = alpha * noise_sequence(i-1) + ou_sigma * randn();
    end
    
    % 添加基础水平并确保为正值
    noise_sequence = base_noise + noise_sequence;
    
    % 确保噪声在合理范围内
    noise_sequence = max(0.1, min(3.0, abs(noise_sequence)));
    
    fprintf('OU过程参数: tau=%.1fs, sigma=%.2f, alpha=%.3f\n', tau, sigma, alpha);
end

function [measurements, noise_info] = generate_enhanced_measurements(...
    target_pos, transmitter_pos, receivers_pos, config)
    % 生成增强的测量数据
    
    measurements = {};
    noise_info = {};
    
    for i = 1:length(receivers_pos)
        receiver_pos = receivers_pos{i};
        
        % 计算真实测量值
        [true_range, true_azimuth, true_elevation] = compute_true_measurement(...
            target_pos, transmitter_pos, receiver_pos, config.dimension);
        
        % 计算噪声标准差
        range_noise_std = compute_range_noise_std(true_range, config);
        angle_noise_std = compute_angle_noise_std(true_range, config);
        
        % 添加噪声
        noisy_range = true_range + randn() * range_noise_std;
        noisy_azimuth = true_azimuth + randn() * angle_noise_std;
        noisy_elevation = true_elevation + randn() * angle_noise_std;
        
        measurements{end+1} = [noisy_range, noisy_azimuth, noisy_elevation];
        
        % 记录噪声信息
        receiver_noise_info = struct();
        receiver_noise_info.receiver_id = i;
        receiver_noise_info.true_range = true_range;
        receiver_noise_info.true_azimuth = true_azimuth;
        receiver_noise_info.true_elevation = true_elevation;
        receiver_noise_info.range_noise_std = range_noise_std;
        receiver_noise_info.angle_noise_std = angle_noise_std;
        receiver_noise_info.noisy_range = noisy_range;
        receiver_noise_info.noisy_azimuth = noisy_azimuth;
        receiver_noise_info.noisy_elevation = noisy_elevation;
        
        noise_info{end+1} = receiver_noise_info;
    end
end

function [range_sum, azimuth, elevation] = compute_true_measurement(...
    target_pos, tx_pos, rx_pos, dimension)
    % 计算真实测量值（无噪声）
    
    % 将位置扩展到3D（如果是2D的话）
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
    
    % 角度测量（从接收机到目标）
    direction = target_pos - rx_pos;
    
    if dimension == 2
        azimuth = atan2(direction(2), direction(1));
        elevation = 0;
    else % 3D
        azimuth = atan2(direction(2), direction(1));
        direction_norm = norm(direction);
        if direction_norm > 0
            elevation = asin(direction(3) / direction_norm);
        else
            elevation = 0;
        end
    end
end

function range_noise_std = compute_range_noise_std(bistatic_range, config)
    % 计算基于距离的距离噪声标准差
    
    base_noise = config.range_noise_base;
    factor = config.range_noise_factor;
    max_noise = config.max_range_noise;
    
    % 线性增长模型：noise = base + factor * range
    range_noise_std = base_noise + factor * bistatic_range;
    
    % 限制在最大值内
    range_noise_std = min(range_noise_std, max_noise);
end

function angle_noise_std = compute_angle_noise_std(bistatic_range, config)
    % 计算基于距离的角度噪声标准差
    
    base_noise = config.angle_noise_base;
    factor = config.angle_noise_factor;
    max_noise = config.max_angle_noise;
    
    % 平方根增长模型：noise = base + factor * sqrt(range)
    angle_noise_std = base_noise + factor * sqrt(bistatic_range);
    
    % 限制在最大值内
    angle_noise_std = min(angle_noise_std, max_noise);
end

function next_state = update_enhanced_state(state, model_info, noise_std, dt, config)
    % 更新增强的9维状态
    
    model_name = model_info.model;
    model_params = model_info.params;
    
    % 当前状态：[x, y, z, vx, vy, vz, ax, ay, az]
    pos = state(1:3);
    vel = state(4:6);
    acc = state(7:9);
    
    switch model_name
        case 'CV'
            % 恒速模型：加速度为0
            new_acc = [0, 0, 0];
            new_vel = vel;  % 速度保持不变（理想情况）
            new_pos = pos + vel * dt;
            
        case 'CA'
            % 恒加速模型：使用设定的加速度
            new_acc = model_params.acceleration;
            new_vel = vel + new_acc * dt;
            new_pos = pos + vel * dt + 0.5 * new_acc * dt^2;
            
        case 'CT'
            % 恒转弯模型（主要在水平面）
            omega = model_params.turn_rate;
            new_acc = [0, 0, 0];  % 转弯不改变加速度
            
            if config.dimension == 2 || abs(omega) > 1e-6
                % 在XY平面转弯
                cos_wt = cos(omega * dt);
                sin_wt = sin(omega * dt);
                
                % 旋转速度向量
                new_vel = vel;
                new_vel(1) = cos_wt * vel(1) - sin_wt * vel(2);
                new_vel(2) = sin_wt * vel(1) + cos_wt * vel(2);
                % Z方向速度不变
            else
                new_vel = vel;
            end
            
            new_pos = pos + vel * dt;
            
        case 'Singer'
            % Singer模型：随机机动
            alpha = model_params.alpha;
            max_acc = model_params.max_acceleration;
            
            % 生成随机机动加速度
            maneuver_acc = randn(1, 3) * alpha * max_acc;
            if config.dimension == 2
                maneuver_acc(3) = 0;  % 2D场景Z方向无机动
            end
            
            new_acc = acc * exp(-alpha * dt) + maneuver_acc;
            new_vel = vel + new_acc * dt;
            new_pos = pos + vel * dt + 0.5 * new_acc * dt^2;
    end
    
    % 添加过程噪声
    process_noise = generate_process_noise_9d(noise_std, config.dimension);
    
    % 组合新状态
    next_state = [new_pos + process_noise(1:3), ...
                  new_vel + process_noise(4:6), ...
                  new_acc + process_noise(7:9)];
    
    % 对于2D场景，Z相关分量置零
    if config.dimension == 2
        next_state([3, 6, 9]) = 0;
    end
end

function noise = generate_process_noise_9d(noise_std, dimension)
    % 生成9维过程噪声
    
    noise = randn(1, 9) * noise_std;
    
    % 不同状态分量使用不同的噪声强度
    noise(1:3) = noise(1:3) * 1.0;    % 位置噪声
    noise(4:6) = noise(4:6) * 0.5;    % 速度噪声
    noise(7:9) = noise(7:9) * 0.2;    % 加速度噪声
    
    % 2D场景Z方向噪声为0
    if dimension == 2
        noise([3, 6, 9]) = 0;
    end
end

function save_scenario_json(scenario, output_dir, scenario_id)
    % 保存场景为JSON格式
    
    filename = fullfile(output_dir, sprintf('scenario_%04d.json', scenario_id));
    
    % 转换为JSON友好格式
    json_scenario = convert_to_json_format(scenario);
    
    % 保存JSON文件
    save_json_file(json_scenario, filename);
end

function json_data = convert_to_json_format(matlab_data)
    % 将MATLAB结构转换为JSON友好格式
    
    if isstruct(matlab_data)
        json_data = struct();
        fields = fieldnames(matlab_data);
        for i = 1:length(fields)
            field_name = fields{i};
            field_value = matlab_data.(field_name);
            json_data.(field_name) = convert_to_json_format(field_value);
        end
    elseif iscell(matlab_data)
        json_data = cell(size(matlab_data));
        for i = 1:numel(matlab_data)
            json_data{i} = convert_to_json_format(matlab_data{i});
        end
    elseif isnumeric(matlab_data)
        json_data = matlab_data;
    else
        json_data = matlab_data;
    end
end

function save_json_file(data, filename)
    % 保存数据为JSON文件
    
    try
        % 尝试使用jsonencode（R2016b及以上）
        json_str = jsonencode(data, 'PrettyPrint', true);
        fid = fopen(filename, 'w', 'n', 'UTF-8');
        fprintf(fid, '%s', json_str);
        fclose(fid);
    catch
        % 如果jsonencode不可用，保存为.mat文件
        [filepath, name, ~] = fileparts(filename);
        mat_filename = fullfile(filepath, [name, '.mat']);
        save(mat_filename, 'data');
        fprintf('警告：JSON编码不可用，保存为MAT格式：%s\n', mat_filename);
    end
end

%% ================ 可视化函数 ================

function visualize_enhanced_scenario(scenario, title_str)
    % 可视化增强场景
    
    if nargin < 2, title_str = '增强雷达场景'; end
    
    config = scenario.config;
    dimension = config.dimension;
    
    figure('Position', [100, 100, 1400, 1000]);
    
    if dimension == 2
        visualize_2d_enhanced(scenario, title_str);
    else
        visualize_3d_enhanced(scenario, title_str);
    end
end

function visualize_2d_enhanced(scenario, title_str)
    % 2D增强可视化
    
    % 子图1: 轨迹和雷达配置
    subplot(2, 4, 1);
    plot_trajectory_and_radars(scenario);
    
    % 子图2: 运动模型时序
    subplot(2, 4, 2);
    plot_motion_model_sequence(scenario);
    
    % 子图3: 过程噪声时序
    subplot(2, 4, 3);
    plot_process_noise_sequence(scenario);
    
    % 子图4: 状态分量时序
    subplot(2, 4, 4);
    plot_state_components(scenario);
    
    % 子图5: 距离测量
    subplot(2, 4, 5);
    plot_range_measurements_enhanced(scenario);
    
    % 子图6: 角度测量
    subplot(2, 4, 6);
    plot_angle_measurements_enhanced(scenario);
    
    % 子图7: OU过程分析
    subplot(2, 4, 7);
    plot_ou_process_analysis(scenario);
    
    % 子图8: 测量噪声分析
    subplot(2, 4, 8);
    plot_measurement_noise_analysis(scenario);
    
    sgtitle(title_str, 'FontSize', 16, 'FontWeight', 'bold');
end

function plot_trajectory_and_radars(scenario)
    % 绘制轨迹和雷达配置
    hold on;
    
    % 发射机
    tx_pos = scenario.transmitter_position;
    plot(tx_pos(1), tx_pos(2), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'red', 'DisplayName', '发射机');
    
    % 接收机
    for i = 1:length(scenario.receivers_positions)
        rx_pos = scenario.receivers_positions{i};
        plot(rx_pos(1), rx_pos(2), 'b^', 'MarkerSize', 12, 'MarkerFaceColor', 'blue', ...
             'DisplayName', sprintf('RX%d', i));
    end
    
    % 目标轨迹
    trajectory = scenario.target_info.trajectory;
    positions = zeros(length(trajectory), 2);
    for j = 1:length(trajectory)
        positions(j, :) = trajectory{j}.position(1:2);
    end
    
    plot(positions(:, 1), positions(:, 2), 'g-', 'LineWidth', 2.5, 'DisplayName', '目标轨迹');
    
    % 起点和终点
    plot(positions(1, 1), positions(1, 2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'green');
    plot(positions(end, 1), positions(end, 2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
    
    legend('Location', 'best');
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('轨迹和雷达配置');
    axis equal;
end

function plot_motion_model_sequence(scenario)
    % 绘制运动模型序列
    hold on;
    
    trajectory = scenario.target_info.trajectory;
    model_colors = containers.Map({'CV', 'CA', 'CT', 'Singer'}, ...
                                  {'red', 'green', 'blue', 'magenta'});
    
    times = zeros(1, length(trajectory));
    for j = 1:length(trajectory)
        times(j) = trajectory{j}.time;
        model = trajectory{j}.motion_model;
        
        if isKey(model_colors, model)
            color = model_colors(model);
        else
            color = 'black';
        end
        
        % 绘制模型段
        if j < length(trajectory)
            plot([times(j), times(j+1)], [1, 1], 'Color', color, 'LineWidth', 6);
        end
    end
    
    xlabel('时间 (s)');
    ylabel('模型');
    title('运动模型时序');
    grid on;
    ylim([0.5, 1.5]);
    
    % 添加图例
    legend_entries = keys(model_colors);
    legend_colors = values(model_colors);
    for k = 1:length(legend_entries)
        plot(NaN, NaN, 'Color', legend_colors{k}, 'LineWidth', 4, ...
             'DisplayName', legend_entries{k});
    end
    legend('Location', 'best');
end

function plot_process_noise_sequence(scenario)
    % 绘制过程噪声序列
    
    trajectory = scenario.target_info.trajectory;
    times = zeros(1, length(trajectory));
    noise_stds = zeros(1, length(trajectory));
    
    for j = 1:length(trajectory)
        times(j) = trajectory{j}.time;
        % 从详细记录中获取噪声信息
        if j <= length(scenario.detailed_records)
            noise_stds(j) = scenario.detailed_records{j}.process_noise_std;
        end
    end
    
    plot(times, noise_stds, 'b-', 'LineWidth', 2, 'DisplayName', '过程噪声');
    
    grid on;
    xlabel('时间 (s)');
    ylabel('噪声标准差');
    title('过程噪声时变特性');
    legend('Location', 'best');
end

function plot_state_components(scenario)
    % 绘制状态分量
    hold on;
    
    trajectory = scenario.target_info.trajectory;
    times = zeros(1, length(trajectory));
    positions = zeros(length(trajectory), 3);
    
    for j = 1:length(trajectory)
        times(j) = trajectory{j}.time;
        positions(j, :) = trajectory{j}.position;
    end
    
    plot(times, positions(:, 1), 'r-', 'LineWidth', 2, 'DisplayName', 'X位置');
    plot(times, positions(:, 2), 'g-', 'LineWidth', 2, 'DisplayName', 'Y位置');
    if scenario.config.dimension == 3
        plot(times, positions(:, 3), 'b-', 'LineWidth', 2, 'DisplayName', 'Z位置');
    end
    
    grid on;
    xlabel('时间 (s)');
    ylabel('位置 (m)');
    title('位置分量');
    legend('Location', 'best');
end

function plot_range_measurements_enhanced(scenario)
    % 绘制增强的距离测量
    hold on;
    
    measurements = scenario.measurements;
    detailed_records = scenario.detailed_records;
    
    times = zeros(1, length(measurements));
    true_ranges = zeros(1, length(measurements));
    noisy_ranges = zeros(1, length(measurements));
    
    for j = 1:length(measurements)
        times(j) = measurements{j}.time;
        noisy_ranges(j) = measurements{j}.measurements{1}(1);  % 第一个接收机
        
        if j <= length(detailed_records)
            true_ranges(j) = detailed_records{j}.true_measurements{1}(1);
        end
    end
    
    plot(times, true_ranges, 'r-', 'LineWidth', 2, 'DisplayName', '真实距离');
    plot(times, noisy_ranges, 'b--', 'LineWidth', 1.5, 'DisplayName', '带噪距离');
    
    grid on;
    xlabel('时间 (s)');
    ylabel('双基地距离和 (m)');
    title('距离测量对比');
    legend('Location', 'best');
end

function plot_angle_measurements_enhanced(scenario)
    % 绘制增强的角度测量
    hold on;
    
    measurements = scenario.measurements;
    detailed_records = scenario.detailed_records;
    
    times = zeros(1, length(measurements));
    true_azimuths = zeros(1, length(measurements));
    noisy_azimuths = zeros(1, length(measurements));
    
    for j = 1:length(measurements)
        times(j) = measurements{j}.time;
        noisy_azimuths(j) = measurements{j}.measurements{1}(2) * 180/pi;
        
        if j <= length(detailed_records)
            true_azimuths(j) = detailed_records{j}.true_measurements{1}(2) * 180/pi;
        end
    end
    
    plot(times, true_azimuths, 'r-', 'LineWidth', 2, 'DisplayName', '真实方位角');
    plot(times, noisy_azimuths, 'b--', 'LineWidth', 1.5, 'DisplayName', '带噪方位角');
    
    grid on;
    xlabel('时间 (s)');
    ylabel('方位角 (度)');
    title('角度测量对比');
    legend('Location', 'best');
end

function plot_ou_process_analysis(scenario)
    % 绘制OU过程分析
    
    detailed_records = scenario.detailed_records;
    
    % 提取过程噪声序列
    process_noises = [];
    for j = 1:length(detailed_records)
        process_noises = [process_noises, detailed_records{j}.process_noise_std];
    end
    
    if length(process_noises) < 10
        text(0.5, 0.5, '数据不足', 'HorizontalAlignment', 'center');
        return;
    end
    
    % 计算自相关函数
    max_lag = min(20, floor(length(process_noises)/4));
    autocorr_values = zeros(1, max_lag+1);
    process_centered = process_noises - mean(process_noises);
    
    for lag = 0:max_lag
        if lag == 0
            autocorr_values(lag+1) = 1;
        else
            autocorr_values(lag+1) = corr(process_centered(1:end-lag)', process_centered(1+lag:end)');
        end
    end
    
    % 绘制自相关函数
    lags = 0:max_lag;
    plot(lags, autocorr_values, 'bo-', 'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', '实际自相关');
    
    hold on;
    
    % 理论OU过程自相关函数
    config = scenario.config;
    if isfield(config, 'process_noise_ou_tau') && isfield(config, 'dt')
        tau = config.process_noise_ou_tau;
        dt = config.dt;
        alpha = exp(-dt/tau);
        
        theoretical_autocorr = alpha.^lags;
        plot(lags, theoretical_autocorr, 'r--', 'LineWidth', 2, 'DisplayName', '理论OU过程');
        
        % 显示参数
        text(max_lag*0.6, max(autocorr_values)*0.8, ...
             sprintf('τ=%.1fs\nα=%.3f', tau, alpha), ...
             'FontSize', 10, 'BackgroundColor', 'white');
    end
    
    grid on;
    xlabel('滞后步数');
    ylabel('自相关系数');
    title('OU过程自相关分析');
    legend('Location', 'best');
    ylim([-0.1, 1.1]);
end

function plot_measurement_noise_analysis(scenario)
    % 绘制测量噪声分析
    hold on;
    
    detailed_records = scenario.detailed_records;
    
    times = zeros(1, length(detailed_records));
    range_noise_stds = zeros(1, length(detailed_records));
    angle_noise_stds = zeros(1, length(detailed_records));
    
    for j = 1:length(detailed_records)
        times(j) = detailed_records{j}.time;
        if ~isempty(detailed_records{j}.measurement_noise_info)
            range_noise_stds(j) = detailed_records{j}.measurement_noise_info{1}.range_noise_std;
            angle_noise_stds(j) = detailed_records{j}.measurement_noise_info{1}.angle_noise_std * 180/pi;
        end
    end
    
    yyaxis left;
    plot(times, range_noise_stds, 'b-', 'LineWidth', 2);
    ylabel('距离噪声标准差 (m)', 'Color', 'b');
    
    yyaxis right;
    plot(times, angle_noise_stds, 'r-', 'LineWidth', 2);
    ylabel('角度噪声标准差 (度)', 'Color', 'r');
    
    grid on;
    xlabel('时间 (s)');
    title('测量噪声水平');
end

function plot_velocity_acceleration(scenario)
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
    
    % 计算速度和加速度的幅值
    vel_magnitude = sqrt(sum(velocities.^2, 2));
    acc_magnitude = sqrt(sum(accelerations.^2, 2));
    
    yyaxis left;
    plot(times, vel_magnitude, 'b-', 'LineWidth', 2);
    ylabel('速度幅值 (m/s)', 'Color', 'b');
    
    yyaxis right;
    plot(times, acc_magnitude, 'r-', 'LineWidth', 2);
    ylabel('加速度幅值 (m/s²)', 'Color', 'r');
    
    grid on;
    xlabel('时间 (s)');
    title('速度和加速度');
end

function visualize_3d_enhanced(scenario, title_str)
    % 3D增强可视化
    
    % 3D轨迹图
    subplot(1, 2, 1);
    plot_3d_trajectory_and_radars(scenario);
    
    % 综合分析
    subplot(1, 2, 2);
    plot_comprehensive_analysis(scenario);
    
    sgtitle(title_str, 'FontSize', 16, 'FontWeight', 'bold');
end

function plot_3d_trajectory_and_radars(scenario)
    % 绘制3D轨迹和雷达
    hold on;
    
    % 发射机
    tx_pos = scenario.transmitter_position;
    scatter3(tx_pos(1), tx_pos(2), tx_pos(3), 150, 'red', 's', 'filled', 'DisplayName', '发射机');
    
    % 接收机
    for i = 1:length(scenario.receivers_positions)
        rx_pos = scenario.receivers_positions{i};
        scatter3(rx_pos(1), rx_pos(2), rx_pos(3), 100, 'blue', '^', 'filled', ...
                 'DisplayName', sprintf('RX%d', i));
    end
    
    % 目标轨迹
    trajectory = scenario.target_info.trajectory;
    positions = zeros(length(trajectory), 3);
    for j = 1:length(trajectory)
        positions(j, :) = trajectory{j}.position;
    end
    
    plot3(positions(:, 1), positions(:, 2), positions(:, 3), ...
          'g-', 'LineWidth', 2.5, 'DisplayName', '目标轨迹');
    
    % 起点和终点
    scatter3(positions(1, 1), positions(1, 2), positions(1, 3), ...
             80, 'green', 'o', 'filled');
    scatter3(positions(end, 1), positions(end, 2), positions(end, 3), ...
             80, 'red', 'o', 'filled');
    
    legend('Location', 'best');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D雷达场景');
    grid on;
    view(3);
end

function plot_comprehensive_analysis(scenario)
    % 绘制综合分析
    
    % 运动模型统计
    trajectory = scenario.target_info.trajectory;
    model_counts = containers.Map();
    
    for j = 1:length(trajectory)
        model = trajectory{j}.motion_model;
        if isKey(model_counts, model)
            model_counts(model) = model_counts(model) + 1;
        else
            model_counts(model) = 1;
        end
    end
    
    models = keys(model_counts);
    counts = cell2mat(values(model_counts));
    
    bar(categorical(models), counts);
    xlabel('运动模型');
    ylabel('使用次数');
    title('运动模型使用统计');
    grid on;
end

%% ================ 数据分析函数 ================

function analyze_enhanced_dataset(dataset_dir)
    % 分析增强数据集
    
    dataset_file = fullfile(dataset_dir, 'dataset.mat');
    
    if ~exist(dataset_file, 'file')
        fprintf('数据集文件不存在: %s\n', dataset_file);
        return;
    end
    
    load(dataset_file, 'dataset');
    
    fprintf('\n=== %s 分析结果 ===\n', dataset_dir);
    fprintf('维度: %dD\n', dataset.config.dimension);
    fprintf('场景数量: %d\n', length(dataset.scenarios));
    fprintf('仿真时间: %.1fs\n', dataset.config.simulation_time);
    fprintf('状态维度: %d维（统一建模）\n', dataset.config.state_dimension);
    
    % 分析第一个场景
    if ~isempty(dataset.scenarios)
        scenario = dataset.scenarios{1};
        
        fprintf('\n场景结构分析:\n');
        fprintf('  轨迹点数: %d\n', length(scenario.target_info.trajectory));
        fprintf('  测量点数: %d\n', length(scenario.measurements));
        fprintf('  详细记录数: %d\n', length(scenario.detailed_records));
        
        % 运动模型统计
        model_stats = containers.Map();
        trajectory = scenario.target_info.trajectory;
        for j = 1:length(trajectory)
            model = trajectory{j}.motion_model;
            if isKey(model_stats, model)
                model_stats(model) = model_stats(model) + 1;
            else
                model_stats(model) = 1;
            end
        end
        
        fprintf('\n运动模型使用统计:\n');
        models = keys(model_stats);
        for i = 1:length(models)
            fprintf('  %s: %d 次 (%.1f%%)\n', models{i}, model_stats(models{i}), ...
                    100 * model_stats(models{i}) / length(trajectory));
        end
        
        % 噪声水平分析
        if ~isempty(scenario.detailed_records)
            analyze_noise_levels(scenario.detailed_records, dataset.config);
        end
    end
    
    fprintf('JSON文件保存位置: %s/*.json\n', dataset_dir);
end

function analyze_noise_levels(detailed_records, config)
    % 分析噪声水平（包括OU过程特性）
    
    range_noises = [];
    angle_noises = [];
    process_noises = [];
    
    for i = 1:length(detailed_records)
        record = detailed_records{i};
        
        process_noises = [process_noises, record.process_noise_std];
        
        if ~isempty(record.measurement_noise_info)
            range_noises = [range_noises, record.measurement_noise_info{1}.range_noise_std];
            angle_noises = [angle_noises, record.measurement_noise_info{1}.angle_noise_std * 180/pi];
        end
    end
    
    fprintf('\n噪声水平统计:\n');
    
    % OU过程噪声分析
    fprintf('  OU过程噪声: %.3f ± %.3f (范围: %.3f - %.3f)\n', ...
            mean(process_noises), std(process_noises), min(process_noises), max(process_noises));
    
    % 计算OU过程的自相关系数
    if length(process_noises) > 10
        process_centered = process_noises - mean(process_noises);
        autocorr_1 = corr(process_centered(1:end-1)', process_centered(2:end)');
        
        fprintf('    一阶自相关系数: %.3f', autocorr_1);
        
        % 与理论值比较
        if isfield(config, 'process_noise_ou_tau') && isfield(config, 'dt')
            tau = config.process_noise_ou_tau;
            dt = config.dt;
            theoretical_alpha = exp(-dt/tau);
            fprintf(' (理论值: %.3f)\n', theoretical_alpha);
            
            % OU过程参数估计
            if abs(autocorr_1) > 0.01
                estimated_tau = -dt / log(abs(autocorr_1));
                fprintf('    估计时间常数: %.1fs (设定值: %.1fs)\n', estimated_tau, tau);
            end
        else
            fprintf('\n');
        end
    end
    
    % 测量噪声分析
    if ~isempty(range_noises)
        fprintf('  距离噪声: %.2fm ± %.2fm (范围: %.2f - %.2fm)\n', ...
                mean(range_noises), std(range_noises), min(range_noises), max(range_noises));
    end
    
    if ~isempty(angle_noises)
        fprintf('  角度噪声: %.3f° ± %.3f° (范围: %.3f - %.3f°)\n', ...
                mean(angle_noises), std(angle_noises), min(angle_noises), max(angle_noises));
    end
end