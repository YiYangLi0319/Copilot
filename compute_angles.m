% MATLAB 程序：计算目标航向及入射/出射角度
% 功能：
% 1. 如果目标位置坐标只有一个，则默认正北方向运动
% 2. 计算并打印发射站到目标、接收站到目标的距离

clear; clc;

% --- 用户输入部分 ---
% 发射站位置坐标 [纬度, 经度, 高度] (单位: 度, 度, 米)
% tx_lla = [-22.548, 90.134, 35778880];
tx_lla = [-4.885, 90.152, 35778800];
% tx_lla = [-4.62, 90.117, 35778710];
% 目标多个位置坐标 [纬度, 经度, 高度] (单位: 度, 度, 米)
% 至少需要两个点来估算航向，否则无法确定运动方向。
% 如果只有一个点，将默认为正北方向运动
target_lla = [
%     31.256, 117.251, 6000;
    % 如果需要多个点，可以取消注释下面的行
%      31.448, 117.249, 6000;
    30.0537, 117.207, 6000;
    30.3975, 117.218, 6000;
%     30.51, 117.14, 5000;
]; 

% 接收站位置坐标 [纬度, 经度, 高度] (单位: 度, 度, 米)
rx_lla = [31.9922, 116.988, 70]; 
rx_lla = [32.3969,118.8105,51];



% --- WGS84 常量 ---
[a, f, e2] = wgs84_constants();

% --- 1. 估算目标航向角 ---
num_target_points = size(target_lla, 1);

if num_target_points == 1
    % 功能1：如果只有一个目标点，默认航向为正北方向
    fprintf('注意：只有一个目标点，默认航向为正北方向。\n');
    heading_vec_enu_normalized = [0, 1, 0]; % 正北方向在ENU中为[0,1,0]
else
    % 将第一个和最后一个目标点的LLA坐标转换为ECEF坐标
    [x1_ecef, y1_ecef, z1_ecef] = lla2ecef(target_lla(1,1), target_lla(1,2), target_lla(1,3));
    [xN_ecef, yN_ecef, zN_ecef] = lla2ecef(target_lla(end,1), target_lla(end,2), target_lla(end,3));

    % 定义一个局部ENU参考点
    ref_lat_for_heading = target_lla(1,1);
    ref_lon_for_heading = target_lla(1,2);

    % 计算从第一个点到最后一个点的ECEF坐标差异向量
    delta_x_ecef_heading = xN_ecef - x1_ecef;
    delta_y_ecef_heading = yN_ecef - y1_ecef;
    delta_z_ecef_heading = zN_ecef - z1_ecef;

    % 将ECEF差异向量转换到ENU坐标系中
    [head_east, head_north, head_up] = ecef2enu_vector(delta_x_ecef_heading, delta_y_ecef_heading, delta_z_ecef_heading, ...
                                                        ref_lat_for_heading, ref_lon_for_heading);

    % 航向主要由水平运动决定
    heading_vec_enu_horizontal = [head_east, head_north, 0];
    if norm(heading_vec_enu_horizontal) < 1e-6
        fprintf('注意：目标起点和终点坐标过于接近，使用默认正北方向作为航向。\n');
        heading_vec_enu_normalized = [0, 1, 0];
    else
        heading_vec_enu_normalized = heading_vec_enu_horizontal / norm(heading_vec_enu_horizontal);
    end
end

% --- 定义目标局部机体坐标系（Body Frame）---
% X轴（机头）：与估算的航向方向一致
X_body_axis_enu = heading_vec_enu_normalized;

% Z轴（垂直机身向上）：假设与当地的地理"向上"方向对齐
Z_body_axis_enu = [0, 0, 1];

% Y轴（机身左侧）：根据右手定则确定
Y_body_axis_enu = cross(Z_body_axis_enu, X_body_axis_enu);
Y_body_axis_enu = Y_body_axis_enu / norm(Y_body_axis_enu);

% 构建从ENU坐标系到目标机体坐标系的旋转变换矩阵
T_body_from_enu = [
    X_body_axis_enu;
    Y_body_axis_enu;
    Z_body_axis_enu
];

% --- 将发射站和接收站的LLA坐标转换为ECEF坐标 ---
[tx_x_ecef, tx_y_ecef, tx_z_ecef] = lla2ecef(tx_lla(1), tx_lla(2), tx_lla(3));
[rx_x_ecef, rx_y_ecef, rx_z_ecef] = lla2ecef(rx_lla(1), rx_lla(2), rx_lla(3));

% --- 遍历每个目标点，计算入射和出射角度 ---
incident_angles = zeros(num_target_points, 2); % [方位角, 俯仰角]
exit_angles = zeros(num_target_points, 2);     % [方位角, 俯仰角]
distances_tx_target = zeros(num_target_points, 1); % 发射站到目标距离
distances_rx_target = zeros(num_target_points, 1); % 接收站到目标距离

fprintf('\n=== 计算结果 ===\n');
fprintf('估算的航向向量（ENU）：[%.4f, %.4f, %.4f]\n', heading_vec_enu_normalized(1), heading_vec_enu_normalized(2), heading_vec_enu_normalized(3));
fprintf('估算的航向角（相对于正北，顺时针为正）：%.2f 度\n', rad2deg(atan2(heading_vec_enu_normalized(1), heading_vec_enu_normalized(2)))); 
fprintf('==================\n\n');

for i = 1:num_target_points
    current_target_lla = target_lla(i, :);
    [target_x_ecef, target_y_ecef, target_z_ecef] = lla2ecef(current_target_lla(1), current_target_lla(2), current_target_lla(3));

    % 功能2：计算发射站到目标的距离
    delta_tx_target = [tx_x_ecef - target_x_ecef, tx_y_ecef - target_y_ecef, tx_z_ecef - target_z_ecef];
    distances_tx_target(i) = norm(delta_tx_target);
    
    % 功能2：计算接收站到目标的距离
    delta_rx_target = [rx_x_ecef - target_x_ecef, rx_y_ecef - target_y_ecef, rx_z_ecef - target_z_ecef];
    distances_rx_target(i) = norm(delta_rx_target);

    % 定义以当前目标点为原点的局部ENU参考坐标系
    ref_lat_target = current_target_lla(1);
    ref_lon_target = current_target_lla(2);

    % --- 计算入射角 (发射站 -> 目标) ---
    vec_tx_target_ecef = [tx_x_ecef - target_x_ecef, ...
                          tx_y_ecef - target_y_ecef, ...
                          tx_z_ecef - target_z_ecef];

    [inc_e, inc_n, inc_u] = ecef2enu_vector(vec_tx_target_ecef(1), vec_tx_target_ecef(2), vec_tx_target_ecef(3), ...
                                            ref_lat_target, ref_lon_target);
    vec_tx_target_enu = [inc_e, inc_n, inc_u];

    % 转换到目标机体坐标系
    vec_tx_target_body = (T_body_from_enu * vec_tx_target_enu')';

    % 计算入射方位角和俯仰角
    [inc_az, inc_el] = calculate_angles_from_body_vector(vec_tx_target_body);
    incident_angles(i, :) = [inc_az, inc_el];

    % --- 计算出射角 (目标 -> 接收站) ---
    vec_target_rx_ecef = [rx_x_ecef - target_x_ecef, ...
                          rx_y_ecef - target_y_ecef, ...
                          rx_z_ecef - target_z_ecef];

    [ext_e, ext_n, ext_u] = ecef2enu_vector(vec_target_rx_ecef(1), vec_target_rx_ecef(2), vec_target_rx_ecef(3), ...
                                            ref_lat_target, ref_lon_target);
    vec_target_rx_enu = [ext_e, ext_n, ext_u];

    % 转换到目标机体坐标系
    vec_target_rx_body = (T_body_from_enu * vec_target_rx_enu')';

    % 计算出射方位角和俯仰角
    [ext_az, ext_el] = calculate_angles_from_body_vector(vec_target_rx_body);
    exit_angles(i, :) = [ext_az, ext_el];

    % 功能2：打印距离信息
    fprintf('目标点 %d (%.4f°N, %.4f°E, %.1fm):\n', i, current_target_lla(1), current_target_lla(2), current_target_lla(3));
    fprintf('  发射站到目标的距离: %.2f 米 (%.2f 公里)\n', distances_tx_target(i), distances_tx_target(i)/1000);
    fprintf('  接收站到目标的距离: %.2f 米 (%.2f 公里)\n', distances_rx_target(i), distances_rx_target(i)/1000);
    fprintf('  入射角 (Tx->Target): 方位角 = %.2f°, 俯仰角 = %.2f°\n', inc_az, inc_el);
    fprintf('  出射角 (Target->Rx): 方位角 = %.2f°, 俯仰角 = %.2f°\n\n', ext_az, ext_el);
end

% 如果有多个目标点，显示距离统计信息
if num_target_points > 1
    fprintf('=== 距离统计信息 ===\n');
    fprintf('发射站到目标距离：最小 %.2f 米，最大 %.2f 米，平均 %.2f 米\n', ...
            min(distances_tx_target), max(distances_tx_target), mean(distances_tx_target));
    fprintf('接收站到目标距离：最小 %.2f 米，最大 %.2f 米，平均 %.2f 米\n', ...
            min(distances_rx_target), max(distances_rx_target), mean(distances_rx_target));
    fprintf('==================\n\n');
end

fprintf('=== 重要说明 ===\n');
fprintf('1. 航向角是根据第一个和最后一个目标点估算的整体方向\n');
fprintf('2. 如果只有一个目标点，默认使用正北方向作为航向\n');
fprintf('3. 目标机体Z轴假定为当地地理"向上"方向\n');
fprintf('4. 方位角：机头方向为0°，逆时针为正，范围[0°, 360°)\n');
fprintf('5. 俯仰角：XOY平面为基准，向上为正，向下为负，范围[-90°, 90°]\n');
fprintf('================\n');

% --- 辅助函数定义 ---

% 函数：获取WGS84椭球常量
function [a, f, e2] = wgs84_constants()
    a = 6378137.0;        % 半长轴 (m)
    f = 1/298.257223563;  % 扁率
    e2 = 2*f - f^2;       % 第一偏心率平方
end

% 函数：将LLA坐标转换为ECEF坐标
function [X, Y, Z] = lla2ecef(lat_deg, lon_deg, alt_m)
    [a, ~, e2] = wgs84_constants();
    lat_rad = deg2rad(lat_deg);
    lon_rad = deg2rad(lon_deg);

    N = a ./ sqrt(1 - e2 * sin(lat_rad).^2); % 卯酉圈曲率半径
    X = (N + alt_m) .* cos(lat_rad) .* cos(lon_rad);
    Y = (N + alt_m) .* cos(lat_rad) .* sin(lon_rad);
    Z = (N .* (1 - e2) + alt_m) .* sin(lat_rad);
end

% 函数：将ECEF向量转换为ENU分量
function [east, north, up] = ecef2enu_vector(dx, dy, dz, ref_lat_deg, ref_lon_deg)
    ref_lat_rad = deg2rad(ref_lat_deg);
    ref_lon_rad = deg2rad(ref_lon_deg);

    east = -sin(ref_lon_rad) * dx + cos(ref_lon_rad) * dy;
    north = -sin(ref_lat_rad) * cos(ref_lon_rad) * dx - sin(ref_lat_rad) * sin(ref_lon_rad) * dy + cos(ref_lat_rad) * dz;
    up = cos(ref_lat_rad) * cos(ref_lon_rad) * dx + cos(ref_lat_rad) * sin(ref_lon_rad) * dy + sin(ref_lat_rad) * dz;
end

% 函数：从机体坐标系向量计算方位角和俯仰角
function [azimuth_deg, elevation_deg] = calculate_angles_from_body_vector(vector_body)
    Vx = vector_body(1);
    Vy = vector_body(2);
    Vz = vector_body(3);

    % 方位角计算
    azimuth_rad = atan2(Vy, Vx);
    azimuth_deg = rad2deg(azimuth_rad);

    % 将方位角归一化到 [0, 360) 范围
    if azimuth_deg < 0
        azimuth_deg = azimuth_deg + 360;
    end

    % 俯仰角计算
    horizontal_projection = sqrt(Vx^2 + Vy^2);
    
    if horizontal_projection < 1e-9
        if Vz > 0
            elevation_deg = 90;   % 垂直向上
        elseif Vz < 0
            elevation_deg = -90;  % 垂直向下
        else
            elevation_deg = 0;    % 零向量
        end
    else
        elevation_rad = atan2(Vz, horizontal_projection);
        elevation_deg = rad2deg(elevation_rad);
    end
end