%% =====================================================================
%  机动感知跟踪网络 - 主入口
%  Attention-Based Maneuver-Aware Tracking Network - Main Entry Point
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  这是tracking_network框架的主入口文件
%  提供快速访问各个功能模块的菜单
%  =====================================================================

function tracking_network_main()
    clear all; close all; clc;
    
    fprintf('=====================================================\n');
    fprintf('  机动感知跟踪网络\n');
    fprintf('  Attention-Based Maneuver-Aware Tracking Network\n');
    fprintf('=====================================================\n\n');
    
    % 显示菜单
    while true
        fprintf('请选择操作：\n');
        fprintf('  1. 快速入门 (Quick Start)\n');
        fprintf('  2. 运行完整示例 (Complete Pipeline)\n');
        fprintf('  3. 测试所有组件 (Test Components)\n');
        fprintf('  4. 加载数据集 (Load Dataset)\n');
        fprintf('  5. 训练网络 (Train Network)\n');
        fprintf('  6. 评估模型 (Evaluate Model)\n');
        fprintf('  7. 可视化工具演示 (Visualization Demo)\n');
        fprintf('  8. 查看文档 (View Documentation)\n');
        fprintf('  9. 关于 (About)\n');
        fprintf('  0. 退出 (Exit)\n\n');
        
        choice = input('输入选项 (0-9): ', 's');
        fprintf('\n');
        
        switch choice
            case '1'
                run_quick_start();
            case '2'
                run_complete_pipeline();
            case '3'
                run_component_tests();
            case '4'
                load_dataset_demo();
            case '5'
                train_network_demo();
            case '6'
                evaluate_model_demo();
            case '7'
                visualization_demo();
            case '8'
                view_documentation();
            case '9'
                show_about();
            case '0'
                fprintf('退出程序。再见！\n\n');
                break;
            otherwise
                fprintf('无效选项，请重新选择。\n\n');
        end
        
        if ~strcmp(choice, '0')
            fprintf('\n按回车键返回菜单...\n');
            pause;
            clc;
        end
    end
end

%% ================ 功能函数 ================

function run_quick_start()
    fprintf('=== 运行快速入门指南 ===\n\n');
    try
        quick_start_guide;
    catch ME
        fprintf('错误: %s\n', ME.message);
    end
end

function run_complete_pipeline()
    fprintf('=== 运行完整示例流程 ===\n\n');
    try
        example_complete_pipeline;
    catch ME
        fprintf('错误: %s\n', ME.message);
    end
end

function run_component_tests()
    fprintf('=== 运行组件测试 ===\n\n');
    try
        test_all_components;
    catch ME
        fprintf('错误: %s\n', ME.message);
    end
end

function load_dataset_demo()
    fprintf('=== 数据加载演示 ===\n\n');
    
    dataset_path = '../enhanced_2d_dataset_v2';
    
    if ~exist(dataset_path, 'dir')
        fprintf('错误：数据集不存在！\n');
        fprintf('请先运行 radar_simulation_v3.m 生成数据集\n');
        return;
    end
    
    fprintf('数据集路径: %s\n', dataset_path);
    fprintf('正在加载数据...\n');
    
    try
        config = struct();
        config.sequence_length = 50;
        config.stride = 10;
        
        data_loader = data_loader_maneuver_tracking(dataset_path, config);
        
        fprintf('\n加载成功！\n');
        fprintf('数据统计：\n');
        fprintf('  样本数: %d\n', size(data_loader.normalized_samples, 1));
        fprintf('  场景数: %d\n', length(data_loader.scenarios));
        fprintf('  特征维度: [%d x %d x %d x %d]\n', ...
            size(data_loader.normalized_samples));
        
    catch ME
        fprintf('加载失败: %s\n', ME.message);
    end
end

function train_network_demo()
    fprintf('=== 训练网络演示 ===\n\n');
    
    fprintf('这是一个交互式训练示例\n');
    fprintf('实际训练需要较长时间，建议使用完整示例脚本\n\n');
    
    dataset_path = input('输入数据集路径 [../enhanced_2d_dataset_v2]: ', 's');
    if isempty(dataset_path)
        dataset_path = '../enhanced_2d_dataset_v2';
    end
    
    if ~exist(dataset_path, 'dir')
        fprintf('错误：数据集不存在！\n');
        return;
    end
    
    fprintf('\n配置训练参数：\n');
    num_epochs = input('训练轮数 [10]: ');
    if isempty(num_epochs), num_epochs = 10; end
    
    batch_size = input('批次大小 [32]: ');
    if isempty(batch_size), batch_size = 32; end
    
    fprintf('\n准备开始训练...\n');
    fprintf('  数据集: %s\n', dataset_path);
    fprintf('  轮数: %d\n', num_epochs);
    fprintf('  批次大小: %d\n\n', batch_size);
    
    user_confirm = input('确认开始训练？(y/n): ', 's');
    
    if strcmpi(user_confirm, 'y')
        try
            config = struct();
            config.num_epochs = num_epochs;
            config.batch_size = batch_size;
            
            train_results = train_maneuver_tracking(dataset_path, config);
            fprintf('\n训练完成！\n');
            
        catch ME
            fprintf('训练失败: %s\n', ME.message);
        end
    else
        fprintf('已取消训练\n');
    end
end

function evaluate_model_demo()
    fprintf('=== 模型评估演示 ===\n\n');
    
    model_dir = 'trained_models';
    
    if ~exist(model_dir, 'dir')
        fprintf('错误：未找到训练好的模型目录\n');
        fprintf('请先训练网络\n');
        return;
    end
    
    % 列出可用模型
    model_files = dir(fullfile(model_dir, 'trained_model_*.mat'));
    
    if isempty(model_files)
        fprintf('错误：没有找到训练好的模型\n');
        return;
    end
    
    fprintf('找到 %d 个模型文件：\n', length(model_files));
    for i = 1:length(model_files)
        fprintf('  %d. %s\n', i, model_files(i).name);
    end
    
    model_idx = input(sprintf('\n选择模型 (1-%d): ', length(model_files)));
    
    if model_idx >= 1 && model_idx <= length(model_files)
        model_file = fullfile(model_dir, model_files(model_idx).name);
        
        dataset_path = input('输入测试数据集路径 [../enhanced_2d_dataset_v2]: ', 's');
        if isempty(dataset_path)
            dataset_path = '../enhanced_2d_dataset_v2';
        end
        
        fprintf('\n开始评估...\n');
        
        try
            config = struct();
            config.num_visualization_samples = 3;
            
            eval_results = evaluate_tracking_network(model_file, dataset_path, config);
            
            fprintf('\n评估完成！\n');
            
        catch ME
            fprintf('评估失败: %s\n', ME.message);
        end
    else
        fprintf('无效的选择\n');
    end
end

function visualization_demo()
    fprintf('=== 可视化工具演示 ===\n\n');
    
    addpath('utils');
    
    fprintf('生成示例可视化...\n\n');
    
    % 1. 轨迹可视化
    fprintf('1. 生成3D轨迹...\n');
    t = linspace(0, 10, 100)';
    traj1 = [100*cos(t), 100*sin(t), 10*t];
    traj2 = [100*cos(t+pi/6), 100*sin(t+pi/6), 10*t+20];
    
    plot_trajectory_3d({traj1, traj2}, {'轨迹1', '轨迹2'}, '3D轨迹示例');
    
    % 2. 注意力热图
    fprintf('2. 生成注意力热图...\n');
    attn = rand(50, 50);
    attn = attn ./ sum(attn, 2);
    
    plot_attention_heatmap(attn, '注意力权重示例');
    
    % 3. 机动检测
    fprintf('3. 生成机动检测时间线...\n');
    time_steps = 1:100;
    true_maneuvers = [ones(25,1); 2*ones(25,1); 3*ones(25,1); 4*ones(25,1)];
    pred_maneuvers = true_maneuvers + (rand(100,1) > 0.9) .* randi([-1,1], 100, 1);
    pred_maneuvers = max(1, min(4, pred_maneuvers));
    
    plot_maneuver_timeline(time_steps, true_maneuvers, pred_maneuvers, '机动检测示例');
    
    % 4. 混淆矩阵
    fprintf('4. 生成混淆矩阵...\n');
    conf_mat = [85, 5, 5, 5; 10, 75, 10, 5; 5, 10, 80, 5; 8, 7, 5, 80];
    
    plot_confusion_matrix(conf_mat, {'CV', 'CA', 'CT', 'Singer'}, '混淆矩阵示例');
    
    fprintf('\n可视化演示完成！请查看生成的图表。\n');
end

function view_documentation()
    fprintf('=== 查看文档 ===\n\n');
    
    readme_file = 'README_tracking_network.md';
    
    if exist(readme_file, 'file')
        fprintf('文档位置: %s\n\n', readme_file);
        
        % 尝试在系统编辑器中打开
        if ispc
            system(['notepad ' readme_file]);
        elseif ismac
            system(['open ' readme_file]);
        else
            system(['xdg-open ' readme_file ' &']);
        end
        
        fprintf('文档已在系统编辑器中打开\n');
        fprintf('如果没有自动打开，请手动打开 %s\n', readme_file);
    else
        fprintf('错误：文档文件不存在\n');
    end
end

function show_about()
    fprintf('=====================================================\n');
    fprintf('  机动感知跟踪网络\n');
    fprintf('  Attention-Based Maneuver-Aware Tracking Network\n');
    fprintf('=====================================================\n\n');
    
    fprintf('版本: 1.0.0\n');
    fprintf('发布日期: 2025-11-05\n');
    fprintf('作者: YiYangLi0319 with Copilot AI Assistant\n\n');
    
    fprintf('功能特性:\n');
    fprintf('  • LSTM/GRU时序特征提取\n');
    fprintf('  • Self-Attention机制\n');
    fprintf('  • Maneuver-Aware Attention\n');
    fprintf('  • 双分支输出（状态估计 + 机动分类）\n');
    fprintf('  • 支持2D/3D数据集\n');
    fprintf('  • 多接收机融合\n\n');
    
    fprintf('主要文件:\n');
    fprintf('  • data_loader_maneuver_tracking.m - 数据加载\n');
    fprintf('  • maneuver_aware_tracking_network.m - 网络架构\n');
    fprintf('  • attention_modules.m - 注意力机制\n');
    fprintf('  • train_maneuver_tracking.m - 训练脚本\n');
    fprintf('  • evaluate_tracking_network.m - 评估脚本\n');
    fprintf('  • utils/ - 工具函数库\n\n');
    
    fprintf('示例脚本:\n');
    fprintf('  • quick_start_guide.m - 快速入门\n');
    fprintf('  • example_complete_pipeline.m - 完整流程\n');
    fprintf('  • test_all_components.m - 组件测试\n\n');
    
    fprintf('统计信息:\n');
    fprintf('  • 代码总行数: ~5,400\n');
    fprintf('  • MATLAB文件: 12个\n');
    fprintf('  • 测试套件: 6个\n\n');
    
    fprintf('项目地址:\n');
    fprintf('  https://github.com/YiYangLi0319/Copilot\n\n');
    
    fprintf('许可证: MIT\n\n');
end

%% ================ 启动程序 ================

% 当直接运行此文件时，自动启动主菜单
if ~nargout
    tracking_network_main();
end
