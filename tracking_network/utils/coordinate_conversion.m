%% =====================================================================
%  坐标转换工具函数
%  Coordinate Conversion Utilities
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  功能：
%  1. 球坐标与笛卡尔坐标转换
%  2. 双基地雷达几何计算
%  3. 坐标系变换
%  =====================================================================

%% ================ 球坐标转笛卡尔坐标 ================

function [x, y, z] = spherical_to_cartesian(range, azimuth, elevation)
    % 球坐标转笛卡尔坐标
    %
    % 输入:
    %   range - 距离 (m)
    %   azimuth - 方位角 (rad)
    %   elevation - 仰角 (rad)
    %
    % 输出:
    %   x, y, z - 笛卡尔坐标 (m)
    
    x = range .* cos(elevation) .* cos(azimuth);
    y = range .* cos(elevation) .* sin(azimuth);
    z = range .* sin(elevation);
end

%% ================ 笛卡尔坐标转球坐标 ================

function [range, azimuth, elevation] = cartesian_to_spherical(x, y, z)
    % 笛卡尔坐标转球坐标
    %
    % 输入:
    %   x, y, z - 笛卡尔坐标 (m)
    %
    % 输出:
    %   range - 距离 (m)
    %   azimuth - 方位角 (rad)
    %   elevation - 仰角 (rad)
    
    range = sqrt(x.^2 + y.^2 + z.^2);
    azimuth = atan2(y, x);
    elevation = atan2(z, sqrt(x.^2 + y.^2));
end

%% ================ 双基地雷达量测计算 ================

function [bistatic_range, azimuth, elevation] = compute_bistatic_measurement(...
    target_pos, tx_pos, rx_pos, add_noise, noise_std)
    % 计算双基地雷达量测
    %
    % 输入:
    %   target_pos - 目标位置 [x, y, z]
    %   tx_pos - 发射机位置 [x, y, z]
    %   rx_pos - 接收机位置 [x, y, z]
    %   add_noise - 是否添加噪声（默认false）
    %   noise_std - 噪声标准差 [range_std, azimuth_std, elevation_std]
    %
    % 输出:
    %   bistatic_range - 双基地距离 (m)
    %   azimuth - 方位角 (rad)
    %   elevation - 仰角 (rad)
    
    if nargin < 4
        add_noise = false;
    end
    if nargin < 5
        noise_std = [1.0, 0.01, 0.01];  % 默认噪声标准差
    end
    
    % 计算发射机到目标的距离
    tx_to_target = norm(target_pos - tx_pos);
    
    % 计算目标到接收机的距离
    target_to_rx = norm(target_pos - rx_pos);
    
    % 双基地距离
    bistatic_range = tx_to_target + target_to_rx;
    
    % 计算接收机处的方位角和仰角
    relative_pos = target_pos - rx_pos;
    [~, azimuth, elevation] = cartesian_to_spherical(...
        relative_pos(1), relative_pos(2), relative_pos(3));
    
    % 添加噪声
    if add_noise
        bistatic_range = bistatic_range + randn() * noise_std(1);
        azimuth = azimuth + randn() * noise_std(2);
        elevation = elevation + randn() * noise_std(3);
    end
end

%% ================ 坐标系旋转 ================

function pos_rotated = rotate_coordinates(pos, rotation_matrix)
    % 坐标系旋转
    %
    % 输入:
    %   pos - 位置向量 [x, y, z] 或 [N x 3]
    %   rotation_matrix - 3x3 旋转矩阵
    %
    % 输出:
    %   pos_rotated - 旋转后的位置
    
    if size(pos, 1) == 1
        pos_rotated = (rotation_matrix * pos')';
    else
        pos_rotated = (rotation_matrix * pos')';
    end
end

%% ================ 生成旋转矩阵 ================

function R = rotation_matrix_xyz(roll, pitch, yaw)
    % 生成XYZ欧拉角旋转矩阵
    %
    % 输入:
    %   roll - 滚转角 (rad)
    %   pitch - 俯仰角 (rad)
    %   yaw - 偏航角 (rad)
    %
    % 输出:
    %   R - 3x3旋转矩阵
    
    % 绕X轴旋转
    Rx = [1, 0, 0;
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];
    
    % 绕Y轴旋转
    Ry = [cos(pitch), 0, sin(pitch);
          0, 1, 0;
          -sin(pitch), 0, cos(pitch)];
    
    % 绕Z轴旋转
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw), 0;
          0, 0, 1];
    
    % 组合旋转
    R = Rz * Ry * Rx;
end

%% ================ ENU转NED坐标系 ================

function pos_ned = enu_to_ned(pos_enu)
    % ENU (East-North-Up) 转 NED (North-East-Down)
    %
    % 输入:
    %   pos_enu - ENU坐标 [E, N, U]
    %
    % 输出:
    %   pos_ned - NED坐标 [N, E, D]
    
    pos_ned = [pos_enu(2), pos_enu(1), -pos_enu(3)];
end

%% ================ NED转ENU坐标系 ================

function pos_enu = ned_to_enu(pos_ned)
    % NED (North-East-Down) 转 ENU (East-North-Up)
    %
    % 输入:
    %   pos_ned - NED坐标 [N, E, D]
    %
    % 输出:
    %   pos_enu - ENU坐标 [E, N, U]
    
    pos_enu = [pos_ned(2), pos_ned(1), -pos_ned(3)];
end

%% ================ 地心坐标转换（简化版）================

function [lat, lon, alt] = ecef_to_lla(x, y, z)
    % ECEF转经纬度高度（简化算法）
    %
    % 输入:
    %   x, y, z - ECEF坐标 (m)
    %
    % 输出:
    %   lat - 纬度 (deg)
    %   lon - 经度 (deg)
    %   alt - 高度 (m)
    
    % WGS84参数
    a = 6378137.0;  % 地球长半轴
    e2 = 0.00669437999014;  % 第一偏心率平方
    
    % 经度
    lon = atan2(y, x) * 180 / pi;
    
    % 纬度（迭代算法）
    p = sqrt(x^2 + y^2);
    lat = atan2(z, p * (1 - e2));
    
    for iter = 1:5
        N = a / sqrt(1 - e2 * sin(lat)^2);
        lat = atan2(z + e2 * N * sin(lat), p);
    end
    
    lat = lat * 180 / pi;
    
    % 高度
    N = a / sqrt(1 - e2 * sin(lat * pi / 180)^2);
    alt = p / cos(lat * pi / 180) - N;
end

%% ================ 距离方位角仰角转笛卡尔（相对坐标）================

function pos = rae_to_cartesian_relative(range, azimuth, elevation, reference_pos)
    % 距离-方位角-仰角转相对笛卡尔坐标
    %
    % 输入:
    %   range - 距离 (m)
    %   azimuth - 方位角 (rad)
    %   elevation - 仰角 (rad)
    %   reference_pos - 参考点位置 [x, y, z]
    %
    % 输出:
    %   pos - 目标绝对位置 [x, y, z]
    
    [x_rel, y_rel, z_rel] = spherical_to_cartesian(range, azimuth, elevation);
    pos = reference_pos + [x_rel, y_rel, z_rel];
end

%% ================ 角度归一化 ================

function angle_norm = normalize_angle(angle)
    % 将角度归一化到 [-π, π]
    %
    % 输入:
    %   angle - 角度 (rad)
    %
    % 输出:
    %   angle_norm - 归一化后的角度 (rad)
    
    angle_norm = mod(angle + pi, 2*pi) - pi;
end

%% ================ 角度差计算 ================

function angle_diff = compute_angle_difference(angle1, angle2)
    % 计算两个角度的最小差值
    %
    % 输入:
    %   angle1, angle2 - 角度 (rad)
    %
    % 输出:
    %   angle_diff - 角度差 (rad)，范围 [-π, π]
    
    diff = angle1 - angle2;
    angle_diff = normalize_angle(diff);
end

%% ================ 测试函数 ================

function test_coordinate_conversion()
    % 测试坐标转换函数
    
    fprintf('测试坐标转换函数...\n\n');
    
    % 测试1: 球坐标与笛卡尔坐标互转
    fprintf('测试1: 球坐标 <-> 笛卡尔坐标\n');
    range = 100;
    azimuth = pi/4;
    elevation = pi/6;
    
    [x, y, z] = spherical_to_cartesian(range, azimuth, elevation);
    fprintf('  球坐标 (%.2f, %.2f°, %.2f°) -> 笛卡尔 (%.2f, %.2f, %.2f)\n', ...
        range, azimuth*180/pi, elevation*180/pi, x, y, z);
    
    [range2, azimuth2, elevation2] = cartesian_to_spherical(x, y, z);
    fprintf('  笛卡尔 (%.2f, %.2f, %.2f) -> 球坐标 (%.2f, %.2f°, %.2f°)\n', ...
        x, y, z, range2, azimuth2*180/pi, elevation2*180/pi);
    fprintf('  误差: %.6f\n\n', norm([range-range2, azimuth-azimuth2, elevation-elevation2]));
    
    % 测试2: 双基地量测计算
    fprintf('测试2: 双基地雷达量测\n');
    target_pos = [1000, 2000, 500];
    tx_pos = [0, 0, 0];
    rx_pos = [5000, 0, 0];
    
    [bistatic_range, az, el] = compute_bistatic_measurement(...
        target_pos, tx_pos, rx_pos, false);
    fprintf('  目标位置: [%.0f, %.0f, %.0f]\n', target_pos);
    fprintf('  发射机位置: [%.0f, %.0f, %.0f]\n', tx_pos);
    fprintf('  接收机位置: [%.0f, %.0f, %.0f]\n', rx_pos);
    fprintf('  双基地距离: %.2f m\n', bistatic_range);
    fprintf('  方位角: %.2f°\n', az*180/pi);
    fprintf('  仰角: %.2f°\n\n', el*180/pi);
    
    % 测试3: 坐标系旋转
    fprintf('测试3: 坐标系旋转\n');
    pos = [1, 0, 0];
    R = rotation_matrix_xyz(0, 0, pi/2);  % 绕Z轴旋转90度
    pos_rotated = rotate_coordinates(pos, R);
    fprintf('  原始位置: [%.2f, %.2f, %.2f]\n', pos);
    fprintf('  绕Z轴旋转90°后: [%.2f, %.2f, %.2f]\n\n', pos_rotated);
    
    % 测试4: 角度归一化
    fprintf('测试4: 角度归一化\n');
    angles = [0, pi/2, pi, 3*pi/2, 2*pi, 5*pi/2, -pi/2];
    for angle = angles
        angle_norm = normalize_angle(angle);
        fprintf('  %.2f° -> %.2f°\n', angle*180/pi, angle_norm*180/pi);
    end
    
    fprintf('\n所有测试完成！\n');
end
