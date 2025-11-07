%% =====================================================================
%  可视化工具函数
%  Visualization Tools
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  功能：
%  1. 轨迹可视化（2D/3D）
%  2. 注意力权重热图
%  3. 机动检测时间线
%  4. 误差分析图表
%  5. 性能对比图
%  =====================================================================

%% ================ 绘制3D轨迹 ================

function fig = plot_trajectory_3d(trajectories, labels, title_text, options)
    % 绘制3D轨迹
    %
    % 输入:
    %   trajectories - cell数组，每个元素是一条轨迹 [N x 3]
    %   labels - cell数组，轨迹标签
    %   title_text - 图表标题
    %   options - 可选参数结构体
    
    if nargin < 3
        title_text = '3D Trajectory';
    end
    if nargin < 4
        options = struct();
    end
    
    % 默认选项
    if ~isfield(options, 'colors')
        options.colors = lines(length(trajectories));
    end
    if ~isfield(options, 'line_width')
        options.line_width = 2;
    end
    if ~isfield(options, 'marker_size')
        options.marker_size = 8;
    end
    if ~isfield(options, 'show_start_end')
        options.show_start_end = true;
    end
    
    fig = figure('Name', title_text, 'Position', [100, 100, 1000, 800]);
    hold on;
    
    % 绘制每条轨迹
    for i = 1:length(trajectories)
        traj = trajectories{i};
        color = options.colors(i, :);
        
        % 绘制轨迹线
        plot3(traj(:, 1), traj(:, 2), traj(:, 3), ...
            'Color', color, 'LineWidth', options.line_width, ...
            'DisplayName', labels{i});
        
        % 标记起点和终点
        if options.show_start_end
            % 起点
            plot3(traj(1, 1), traj(1, 2), traj(1, 3), ...
                'o', 'MarkerSize', options.marker_size, ...
                'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', ...
                'HandleVisibility', 'off');
            % 终点
            plot3(traj(end, 1), traj(end, 2), traj(end, 3), ...
                's', 'MarkerSize', options.marker_size, ...
                'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', ...
                'HandleVisibility', 'off');
        end
    end
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(title_text);
    legend('Location', 'best');
    grid on;
    axis equal;
    view(3);
    hold off;
end

%% ================ 绘制2D轨迹 ================

function fig = plot_trajectory_2d(trajectories, labels, title_text, options)
    % 绘制2D轨迹
    %
    % 输入:
    %   trajectories - cell数组，每个元素是一条轨迹 [N x 2]或[N x 3]
    %   labels - cell数组，轨迹标签
    %   title_text - 图表标题
    %   options - 可选参数结构体
    
    if nargin < 3
        title_text = '2D Trajectory';
    end
    if nargin < 4
        options = struct();
    end
    
    % 默认选项
    if ~isfield(options, 'colors')
        options.colors = lines(length(trajectories));
    end
    if ~isfield(options, 'line_width')
        options.line_width = 2;
    end
    if ~isfield(options, 'marker_size')
        options.marker_size = 8;
    end
    if ~isfield(options, 'show_start_end')
        options.show_start_end = true;
    end
    
    fig = figure('Name', title_text, 'Position', [100, 100, 1000, 800]);
    hold on;
    
    % 绘制每条轨迹
    for i = 1:length(trajectories)
        traj = trajectories{i};
        color = options.colors(i, :);
        
        % 绘制轨迹线（只使用X-Y坐标）
        plot(traj(:, 1), traj(:, 2), ...
            'Color', color, 'LineWidth', options.line_width, ...
            'DisplayName', labels{i});
        
        % 标记起点和终点
        if options.show_start_end
            % 起点
            plot(traj(1, 1), traj(1, 2), ...
                'o', 'MarkerSize', options.marker_size, ...
                'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', ...
                'HandleVisibility', 'off');
            % 终点
            plot(traj(end, 1), traj(end, 2), ...
                's', 'MarkerSize', options.marker_size, ...
                'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', ...
                'HandleVisibility', 'off');
        end
    end
    
    xlabel('X (m)');
    ylabel('Y (m)');
    title(title_text);
    legend('Location', 'best');
    grid on;
    axis equal;
    hold off;
end

%% ================ 绘制注意力权重热图 ================

function fig = plot_attention_heatmap(attention_weights, title_text, options)
    % 绘制注意力权重热图
    %
    % 输入:
    %   attention_weights - 注意力权重矩阵 [seq_len x seq_len]
    %   title_text - 图表标题
    %   options - 可选参数结构体
    
    if nargin < 2
        title_text = 'Attention Weights Heatmap';
    end
    if nargin < 3
        options = struct();
    end
    
    % 默认选项
    if ~isfield(options, 'colormap')
        options.colormap = 'hot';
    end
    if ~isfield(options, 'show_values')
        options.show_values = false;
    end
    
    fig = figure('Name', title_text, 'Position', [100, 100, 800, 700]);
    
    imagesc(attention_weights);
    colorbar;
    colormap(options.colormap);
    
    xlabel('Key Position (Time Step)');
    ylabel('Query Position (Time Step)');
    title(title_text);
    axis equal tight;
    
    % 显示数值（如果权重矩阵不太大）
    if options.show_values && size(attention_weights, 1) <= 20
        [rows, cols] = size(attention_weights);
        for i = 1:rows
            for j = 1:cols
                text(j, i, sprintf('%.2f', attention_weights(i, j)), ...
                    'HorizontalAlignment', 'center', ...
                    'Color', 'w', 'FontSize', 8);
            end
        end
    end
end

%% ================ 绘制机动检测时间线 ================

function fig = plot_maneuver_timeline(time_steps, true_maneuvers, ...
    pred_maneuvers, title_text)
    % 绘制机动检测时间线
    %
    % 输入:
    %   time_steps - 时间步 [N x 1]
    %   true_maneuvers - 真实机动模式 [N x 1]
    %   pred_maneuvers - 预测机动模式 [N x 1]
    %   title_text - 图表标题
    
    if nargin < 4
        title_text = 'Maneuver Detection Timeline';
    end
    
    maneuver_names = {'CV', 'CA', 'CT', 'Singer'};
    colors = [0.2, 0.6, 1.0; ...  % CV - 蓝色
              1.0, 0.5, 0.2; ...  % CA - 橙色
              0.2, 0.8, 0.4; ...  % CT - 绿色
              0.8, 0.2, 0.6];     % Singer - 紫色
    
    fig = figure('Name', title_text, 'Position', [100, 100, 1200, 600]);
    
    % 上半部分：真实机动模式
    subplot(2, 1, 1);
    hold on;
    for t = 1:length(time_steps)
        maneuver = true_maneuvers(t);
        if maneuver >= 1 && maneuver <= 4
            color = colors(maneuver, :);
            plot(time_steps(t), 1, 's', 'MarkerSize', 10, ...
                'MarkerFaceColor', color, 'MarkerEdgeColor', 'k');
        end
    end
    ylim([0.5, 1.5]);
    xlim([min(time_steps), max(time_steps)]);
    ylabel('真实模式');
    title('真实机动模式');
    yticks(1);
    yticklabels({''});
    grid on;
    
    % 添加图例
    legend_handles = [];
    for i = 1:4
        h = plot(NaN, NaN, 's', 'MarkerSize', 10, ...
            'MarkerFaceColor', colors(i, :), 'MarkerEdgeColor', 'k', ...
            'DisplayName', maneuver_names{i});
        legend_handles = [legend_handles; h];
    end
    legend(legend_handles, 'Location', 'eastoutside');
    hold off;
    
    % 下半部分：预测机动模式
    subplot(2, 1, 2);
    hold on;
    for t = 1:length(time_steps)
        true_maneuver = true_maneuvers(t);
        pred_maneuver = pred_maneuvers(t);
        
        if pred_maneuver >= 1 && pred_maneuver <= 4
            color = colors(pred_maneuver, :);
            
            % 如果预测正确，使用实心；否则使用空心
            if pred_maneuver == true_maneuver
                plot(time_steps(t), 1, 's', 'MarkerSize', 10, ...
                    'MarkerFaceColor', color, 'MarkerEdgeColor', 'k');
            else
                plot(time_steps(t), 1, 's', 'MarkerSize', 10, ...
                    'MarkerFaceColor', 'none', 'MarkerEdgeColor', color, ...
                    'LineWidth', 2);
            end
        end
    end
    ylim([0.5, 1.5]);
    xlim([min(time_steps), max(time_steps)]);
    xlabel('Time Step');
    ylabel('预测模式');
    title('预测机动模式（实心=正确，空心=错误）');
    yticks(1);
    yticklabels({''});
    grid on;
    hold off;
end

%% ================ 绘制误差分布直方图 ================

function fig = plot_error_histogram(errors, title_text, options)
    % 绘制误差分布直方图
    %
    % 输入:
    %   errors - 误差数组 [N x 1]
    %   title_text - 图表标题
    %   options - 可选参数结构体
    
    if nargin < 2
        title_text = 'Error Distribution';
    end
    if nargin < 3
        options = struct();
    end
    
    % 默认选项
    if ~isfield(options, 'num_bins')
        options.num_bins = 30;
    end
    if ~isfield(options, 'show_stats')
        options.show_stats = true;
    end
    
    fig = figure('Name', title_text, 'Position', [100, 100, 800, 600]);
    
    histogram(errors, options.num_bins, 'FaceColor', [0.3, 0.6, 0.9], ...
        'EdgeColor', 'k');
    
    xlabel('Error');
    ylabel('Frequency');
    title(title_text);
    grid on;
    
    % 显示统计信息
    if options.show_stats
        mean_error = mean(errors);
        std_error = std(errors);
        median_error = median(errors);
        
        % 添加统计线
        hold on;
        y_lim = ylim;
        plot([mean_error, mean_error], y_lim, 'r--', 'LineWidth', 2, ...
            'DisplayName', sprintf('Mean: %.3f', mean_error));
        plot([median_error, median_error], y_lim, 'g--', 'LineWidth', 2, ...
            'DisplayName', sprintf('Median: %.3f', median_error));
        legend('Location', 'best');
        hold off;
        
        % 添加文本注释
        text_str = sprintf('Mean: %.3f\nStd: %.3f\nMedian: %.3f', ...
            mean_error, std_error, median_error);
        annotation('textbox', [0.15, 0.75, 0.2, 0.15], 'String', text_str, ...
            'FitBoxToText', 'on', 'BackgroundColor', 'w', 'EdgeColor', 'k');
    end
end

%% ================ 绘制混淆矩阵 ================

function fig = plot_confusion_matrix(conf_mat, class_names, title_text)
    % 绘制混淆矩阵
    %
    % 输入:
    %   conf_mat - 混淆矩阵 [num_classes x num_classes]
    %   class_names - 类别名称 cell数组
    %   title_text - 图表标题
    
    if nargin < 2
        class_names = arrayfun(@(x) sprintf('Class %d', x), ...
            1:size(conf_mat, 1), 'UniformOutput', false);
    end
    if nargin < 3
        title_text = 'Confusion Matrix';
    end
    
    fig = figure('Name', title_text, 'Position', [100, 100, 700, 600]);
    
    imagesc(conf_mat);
    colorbar;
    colormap('hot');
    
    xlabel('Predicted Class');
    ylabel('True Class');
    title(title_text);
    
    % 设置刻度标签
    num_classes = length(class_names);
    xticks(1:num_classes);
    xticklabels(class_names);
    yticks(1:num_classes);
    yticklabels(class_names);
    
    % 添加数值标注
    for i = 1:num_classes
        for j = 1:num_classes
            text(j, i, num2str(conf_mat(i, j)), ...
                'HorizontalAlignment', 'center', ...
                'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
        end
    end
    
    axis equal tight;
end

%% ================ 绘制损失曲线 ================

function fig = plot_loss_curves(train_loss, val_loss, title_text)
    % 绘制训练和验证损失曲线
    %
    % 输入:
    %   train_loss - 训练损失 [num_epochs x 1]
    %   val_loss - 验证损失 [num_epochs x 1]
    %   title_text - 图表标题
    
    if nargin < 3
        title_text = 'Training and Validation Loss';
    end
    
    fig = figure('Name', title_text, 'Position', [100, 100, 1000, 600]);
    
    epochs = 1:length(train_loss);
    
    plot(epochs, train_loss, 'b-', 'LineWidth', 2, 'DisplayName', 'Training Loss');
    hold on;
    plot(epochs, val_loss, 'r-', 'LineWidth', 2, 'DisplayName', 'Validation Loss');
    
    xlabel('Epoch');
    ylabel('Loss');
    title(title_text);
    legend('Location', 'best');
    grid on;
    
    % 标记最佳验证损失
    [min_val_loss, min_epoch] = min(val_loss);
    plot(min_epoch, min_val_loss, 'r*', 'MarkerSize', 15, ...
        'DisplayName', sprintf('Best Val Loss: %.4f @ Epoch %d', ...
        min_val_loss, min_epoch));
    
    hold off;
end

%% ================ 绘制性能对比柱状图 ================

function fig = plot_performance_comparison(methods, metrics, metric_names, title_text)
    % 绘制不同方法的性能对比柱状图
    %
    % 输入:
    %   methods - 方法名称 cell数组
    %   metrics - 指标矩阵 [num_methods x num_metrics]
    %   metric_names - 指标名称 cell数组
    %   title_text - 图表标题
    
    if nargin < 4
        title_text = 'Performance Comparison';
    end
    
    fig = figure('Name', title_text, 'Position', [100, 100, 1000, 600]);
    
    bar(metrics);
    
    xlabel('Method');
    ylabel('Score');
    title(title_text);
    legend(metric_names, 'Location', 'best');
    
    xticks(1:length(methods));
    xticklabels(methods);
    xtickangle(45);
    
    grid on;
    
    % 添加数值标注
    [num_methods, num_metrics] = size(metrics);
    for i = 1:num_methods
        for j = 1:num_metrics
            text(i + (j - (num_metrics+1)/2) * 0.15, metrics(i, j), ...
                sprintf('%.2f', metrics(i, j)), ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'bottom', 'FontSize', 8);
        end
    end
end

%% ================ 绘制速度和加速度时间序列 ================

function fig = plot_velocity_acceleration(time_steps, velocities, ...
    accelerations, title_text)
    % 绘制速度和加速度时间序列
    %
    % 输入:
    %   time_steps - 时间步 [N x 1]
    %   velocities - 速度 [N x 3]
    %   accelerations - 加速度 [N x 3]
    %   title_text - 图表标题
    
    if nargin < 4
        title_text = 'Velocity and Acceleration Time Series';
    end
    
    fig = figure('Name', title_text, 'Position', [100, 100, 1200, 800]);
    
    % 速度
    subplot(2, 1, 1);
    plot(time_steps, velocities(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Vx');
    hold on;
    plot(time_steps, velocities(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Vy');
    plot(time_steps, velocities(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Vz');
    plot(time_steps, vecnorm(velocities, 2, 2), 'k--', 'LineWidth', 2, ...
        'DisplayName', '‖V‖');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Components');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 加速度
    subplot(2, 1, 2);
    plot(time_steps, accelerations(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Ax');
    hold on;
    plot(time_steps, accelerations(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Ay');
    plot(time_steps, accelerations(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Az');
    plot(time_steps, vecnorm(accelerations, 2, 2), 'k--', 'LineWidth', 2, ...
        'DisplayName', '‖A‖');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s²)');
    title('Acceleration Components');
    legend('Location', 'best');
    grid on;
    hold off;
end

%% ================ 保存所有图表 ================

function save_all_figures(output_dir, prefix)
    % 保存所有打开的图表
    %
    % 输入:
    %   output_dir - 输出目录
    %   prefix - 文件名前缀（可选）
    
    if nargin < 2
        prefix = 'fig';
    end
    
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    % 获取所有图表句柄
    figs = findall(0, 'Type', 'figure');
    
    fprintf('保存 %d 个图表到 %s...\n', length(figs), output_dir);
    
    for i = 1:length(figs)
        fig = figs(i);
        
        % 生成文件名
        if isempty(fig.Name)
            filename = sprintf('%s_%d.png', prefix, fig.Number);
        else
            % 清理文件名中的非法字符
            clean_name = regexprep(fig.Name, '[^a-zA-Z0-9_]', '_');
            filename = sprintf('%s_%s.png', prefix, clean_name);
        end
        
        filepath = fullfile(output_dir, filename);
        saveas(fig, filepath);
        fprintf('  已保存: %s\n', filename);
    end
    
    fprintf('所有图表已保存！\n');
end

%% ================ 测试函数 ================

function test_visualization_tools()
    % 测试可视化工具
    
    fprintf('测试可视化工具...\n\n');
    
    % 生成测试数据
    t = linspace(0, 10, 100)';
    
    % 测试1: 3D轨迹
    fprintf('1. 测试3D轨迹绘制\n');
    traj1 = [100*cos(t), 100*sin(t), 10*t];
    traj2 = [100*cos(t+pi/4), 100*sin(t+pi/4), 10*t+50];
    plot_trajectory_3d({traj1, traj2}, {'轨迹1', '轨迹2'}, '3D轨迹测试');
    fprintf('   完成！\n\n');
    
    % 测试2: 注意力热图
    fprintf('2. 测试注意力热图\n');
    attn = rand(50, 50);
    attn = attn ./ sum(attn, 2);  % 归一化
    plot_attention_heatmap(attn, '注意力热图测试');
    fprintf('   完成！\n\n');
    
    % 测试3: 机动检测时间线
    fprintf('3. 测试机动检测时间线\n');
    true_maneuvers = [ones(25,1); 2*ones(25,1); 3*ones(25,1); 4*ones(25,1)];
    pred_maneuvers = true_maneuvers + (rand(100,1) > 0.8) .* randi([-1,1], 100, 1);
    pred_maneuvers = max(1, min(4, pred_maneuvers));
    plot_maneuver_timeline(1:100, true_maneuvers, pred_maneuvers, '机动检测时间线测试');
    fprintf('   完成！\n\n');
    
    % 测试4: 误差分布
    fprintf('4. 测试误差分布直方图\n');
    errors = randn(1000, 1) * 5 + 2;
    plot_error_histogram(errors, '误差分布测试');
    fprintf('   完成！\n\n');
    
    % 测试5: 混淆矩阵
    fprintf('5. 测试混淆矩阵\n');
    conf_mat = [85, 5, 5, 5; 10, 75, 10, 5; 5, 10, 80, 5; 8, 7, 5, 80];
    plot_confusion_matrix(conf_mat, {'CV', 'CA', 'CT', 'Singer'}, '混淆矩阵测试');
    fprintf('   完成！\n\n');
    
    fprintf('所有测试完成！\n');
end
