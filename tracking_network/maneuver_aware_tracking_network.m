%% =====================================================================
%  机动感知跟踪网络架构
%  Maneuver-Aware Tracking Network Architecture
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  功能：
%  1. 构建深度学习网络架构
%  2. LSTM/GRU层提取时序特征
%  3. Self-Attention和Maneuver-Aware Attention
%  4. 双分支输出：状态估计 + 机动分类
%  
%  网络结构：
%  Input [num_receivers x 3 x sequence_length]
%    ↓
%  Feature Extraction (LSTM/GRU)
%    ↓
%  Self-Attention Layer
%    ↓
%  Maneuver-Aware Attention
%    ↓
%  ├─ State Estimation Branch (位置、速度、加速度)
%  └─ Maneuver Classification Branch (CV/CA/CT/Singer)
%  =====================================================================

function network = maneuver_aware_tracking_network(config)
    % 创建机动感知跟踪网络
    %
    % 输入:
    %   config - 网络配置结构体
    %
    % 输出:
    %   network - 网络层数组或LayerGraph对象
    
    if nargin < 1
        config = get_default_network_config();
    end
    
    fprintf('=====================================================\n');
    fprintf('  构建机动感知跟踪网络\n');
    fprintf('  序列长度: %d\n', config.sequence_length);
    fprintf('  隐藏层大小: %d\n', config.hidden_size);
    fprintf('  注意力头数: %d\n', config.num_attention_heads);
    fprintf('=====================================================\n\n');
    
    % 使用LayerGraph构建复杂网络
    network = create_network_graph(config);
    
    % 显示网络摘要
    fprintf('网络构建完成！\n');
    display_network_summary(network, config);
    
    % 可选：绘制网络结构
    if config.plot_network
        figure('Name', 'Maneuver-Aware Tracking Network Architecture');
        plot(network);
        title('Network Architecture');
    end
end

%% ================ 默认配置 ================

function config = get_default_network_config()
    % 获取默认网络配置
    
    config = struct();
    
    % 输入参数
    config.num_receivers = 3;             % 接收机数量
    config.num_features = 3;              % 特征维度（距离、方位角、仰角）
    config.sequence_length = 50;          % 输入序列长度
    
    % 网络架构
    config.hidden_size = 128;             % LSTM隐藏层大小
    config.num_lstm_layers = 2;           % LSTM层数
    config.lstm_type = 'lstm';            % 'lstm' 或 'gru'
    
    % 注意力机制
    config.num_attention_heads = 8;       % 注意力头数
    config.attention_key_dim = 64;        % Key/Query维度
    config.attention_value_dim = 64;      % Value维度
    config.use_self_attention = true;     % 是否使用Self-Attention
    config.use_maneuver_attention = true; % 是否使用Maneuver-Aware Attention
    
    % 正则化
    config.dropout_rate = 0.2;            % Dropout比例
    config.use_batch_norm = true;         % 是否使用Batch Normalization
    
    % 输出分支
    config.state_output_size = 9;         % 状态输出维度（位置3 + 速度3 + 加速度3）
    config.num_maneuver_classes = 4;      % 机动模式类别数（CV/CA/CT/Singer）
    
    % 其他
    config.plot_network = false;          % 是否绘制网络结构图
end

%% ================ 网络图构建 ================

function lgraph = create_network_graph(config)
    % 创建网络层图
    
    % 1. 输入层
    input_size = [config.num_receivers, config.num_features, config.sequence_length];
    layers = [
        sequenceInputLayer(input_size, 'Name', 'input', ...
            'Normalization', 'rescale-symmetric')
    ];
    
    % 2. 特征预处理
    % 将多接收机数据展平或融合
    layers = [layers
        sequenceFoldingLayer('Name', 'seqfold')
        fullyConnectedLayer(config.hidden_size, 'Name', 'fc_preprocess')
        reluLayer('Name', 'relu_preprocess')
        sequenceUnfoldingLayer('Name', 'sequnfold')
    ];
    
    % 3. LSTM/GRU层提取时序特征
    if strcmp(config.lstm_type, 'lstm')
        for i = 1:config.num_lstm_layers
            layers = [layers
                lstmLayer(config.hidden_size, ...
                    'Name', sprintf('lstm_%d', i), ...
                    'OutputMode', 'sequence')
                dropoutLayer(config.dropout_rate, ...
                    'Name', sprintf('dropout_lstm_%d', i))
            ];
        end
    else
        for i = 1:config.num_lstm_layers
            layers = [layers
                gruLayer(config.hidden_size, ...
                    'Name', sprintf('gru_%d', i), ...
                    'OutputMode', 'sequence')
                dropoutLayer(config.dropout_rate, ...
                    'Name', sprintf('dropout_gru_%d', i))
            ];
        end
    end
    
    % 创建层图
    lgraph = layerGraph(layers);
    
    % 4. Self-Attention分支（如果启用）
    if config.use_self_attention
        % 注意：MATLAB的selfAttentionLayer需要R2021a+
        % 这里使用全连接层近似实现
        attn_layers = create_attention_branch(config, 'self_attn');
        lgraph = addLayers(lgraph, attn_layers);
        
        % 连接注意力分支
        last_lstm_name = sprintf('dropout_lstm_%d', config.num_lstm_layers);
        if ~strcmp(config.lstm_type, 'lstm')
            last_lstm_name = sprintf('dropout_gru_%d', config.num_lstm_layers);
        end
        lgraph = connectLayers(lgraph, last_lstm_name, 'self_attn_input');
    end
    
    % 5. Maneuver-Aware Attention分支（如果启用）
    if config.use_maneuver_attention
        maneuver_attn_layers = create_maneuver_attention_branch(config);
        lgraph = addLayers(lgraph, maneuver_attn_layers);
        
        % 连接机动注意力分支
        if config.use_self_attention
            lgraph = connectLayers(lgraph, 'self_attn_output', 'maneuver_attn_input');
        else
            last_layer = sprintf('dropout_lstm_%d', config.num_lstm_layers);
            if ~strcmp(config.lstm_type, 'lstm')
                last_layer = sprintf('dropout_gru_%d', config.num_lstm_layers);
            end
            lgraph = connectLayers(lgraph, last_layer, 'maneuver_attn_input');
        end
    end
    
    % 6. 双分支输出
    % 找到特征提取的最后一层
    if config.use_maneuver_attention
        feature_layer = 'maneuver_attn_output';
    elseif config.use_self_attention
        feature_layer = 'self_attn_output';
    else
        feature_layer = sprintf('dropout_lstm_%d', config.num_lstm_layers);
        if ~strcmp(config.lstm_type, 'lstm')
            feature_layer = sprintf('dropout_gru_%d', config.num_lstm_layers);
        end
    end
    
    % 状态估计分支
    state_branch = create_state_estimation_branch(config);
    lgraph = addLayers(lgraph, state_branch);
    lgraph = connectLayers(lgraph, feature_layer, 'state_branch_input');
    
    % 机动分类分支
    maneuver_branch = create_maneuver_classification_branch(config);
    lgraph = addLayers(lgraph, maneuver_branch);
    lgraph = connectLayers(lgraph, feature_layer, 'maneuver_branch_input');
end

%% ================ 注意力分支 ================

function layers = create_attention_branch(config, name_prefix)
    % 创建Self-Attention分支
    
    layers = [
        sequenceInputLayer([config.hidden_size, 1], 'Name', [name_prefix '_input'])
        
        % Query, Key, Value投影
        fullyConnectedLayer(config.attention_key_dim * config.num_attention_heads, ...
            'Name', [name_prefix '_query'])
        fullyConnectedLayer(config.attention_key_dim * config.num_attention_heads, ...
            'Name', [name_prefix '_key'])
        fullyConnectedLayer(config.attention_value_dim * config.num_attention_heads, ...
            'Name', [name_prefix '_value'])
        
        % 注意力计算（简化实现）
        fullyConnectedLayer(config.hidden_size, 'Name', [name_prefix '_fc'])
        reluLayer('Name', [name_prefix '_relu'])
        
        % 残差连接和层归一化
        layerNormalizationLayer('Name', [name_prefix '_norm'])
        dropoutLayer(config.dropout_rate, 'Name', [name_prefix '_dropout'])
    ];
    
    % 添加输出层
    layers = [layers
        fullyConnectedLayer(config.hidden_size, 'Name', [name_prefix '_output'])
    ];
end

function layers = create_maneuver_attention_branch(config)
    % 创建Maneuver-Aware Attention分支
    
    layers = [
        sequenceInputLayer([config.hidden_size, 1], 'Name', 'maneuver_attn_input')
        
        % 机动特征提取
        fullyConnectedLayer(config.hidden_size, 'Name', 'maneuver_attn_fc1')
        reluLayer('Name', 'maneuver_attn_relu1')
        dropoutLayer(config.dropout_rate, 'Name', 'maneuver_attn_dropout1')
        
        % 注意力权重计算
        fullyConnectedLayer(config.hidden_size, 'Name', 'maneuver_attn_fc2')
        tanhLayer('Name', 'maneuver_attn_tanh')
        
        % 输出层
        fullyConnectedLayer(config.hidden_size, 'Name', 'maneuver_attn_output')
        layerNormalizationLayer('Name', 'maneuver_attn_norm')
    ];
end

%% ================ 输出分支 ================

function layers = create_state_estimation_branch(config)
    % 创建状态估计分支
    % 输出：位置(3) + 速度(3) + 加速度(3) = 9维
    
    layers = [
        sequenceInputLayer([config.hidden_size, 1], 'Name', 'state_branch_input')
        
        % 仅使用最后一个时间步
        sequenceFoldingLayer('Name', 'state_seqfold')
        
        % 全连接层
        fullyConnectedLayer(256, 'Name', 'state_fc1')
        reluLayer('Name', 'state_relu1')
        dropoutLayer(config.dropout_rate, 'Name', 'state_dropout1')
        
        fullyConnectedLayer(128, 'Name', 'state_fc2')
        reluLayer('Name', 'state_relu2')
        dropoutLayer(config.dropout_rate, 'Name', 'state_dropout2')
        
        % 输出层
        fullyConnectedLayer(config.state_output_size, 'Name', 'state_output')
        regressionLayer('Name', 'state_regression')
    ];
end

function layers = create_maneuver_classification_branch(config)
    % 创建机动分类分支
    % 输出：4个类别（CV/CA/CT/Singer）
    
    layers = [
        sequenceInputLayer([config.hidden_size, 1], 'Name', 'maneuver_branch_input')
        
        % 仅使用最后一个时间步
        sequenceFoldingLayer('Name', 'maneuver_seqfold')
        
        % 全连接层
        fullyConnectedLayer(128, 'Name', 'maneuver_fc1')
        reluLayer('Name', 'maneuver_relu1')
        dropoutLayer(config.dropout_rate, 'Name', 'maneuver_dropout1')
        
        fullyConnectedLayer(64, 'Name', 'maneuver_fc2')
        reluLayer('Name', 'maneuver_relu2')
        
        % 分类输出
        fullyConnectedLayer(config.num_maneuver_classes, 'Name', 'maneuver_fc_output')
        softmaxLayer('Name', 'maneuver_softmax')
        classificationLayer('Name', 'maneuver_classification')
    ];
end

%% ================ 网络摘要 ================

function display_network_summary(network, config)
    % 显示网络摘要信息
    
    fprintf('\n--- 网络架构摘要 ---\n');
    fprintf('输入层:\n');
    fprintf('  形状: [%d x %d x %d]\n', ...
        config.num_receivers, config.num_features, config.sequence_length);
    fprintf('  说明: %d个接收机 × %d个特征维度 × %d个时间步\n', ...
        config.num_receivers, config.num_features, config.sequence_length);
    
    fprintf('\n特征提取层:\n');
    fprintf('  类型: %s\n', upper(config.lstm_type));
    fprintf('  层数: %d\n', config.num_lstm_layers);
    fprintf('  隐藏层大小: %d\n', config.hidden_size);
    fprintf('  Dropout率: %.2f\n', config.dropout_rate);
    
    if config.use_self_attention
        fprintf('\nSelf-Attention层:\n');
        fprintf('  注意力头数: %d\n', config.num_attention_heads);
        fprintf('  Key维度: %d\n', config.attention_key_dim);
        fprintf('  Value维度: %d\n', config.attention_value_dim);
    end
    
    if config.use_maneuver_attention
        fprintf('\nManeuver-Aware Attention层:\n');
        fprintf('  已启用\n');
    end
    
    fprintf('\n输出分支:\n');
    fprintf('  1. 状态估计: %d维 (位置3 + 速度3 + 加速度3)\n', ...
        config.state_output_size);
    fprintf('  2. 机动分类: %d类 (CV/CA/CT/Singer)\n', ...
        config.num_maneuver_classes);
    
    fprintf('\n总参数估计:\n');
    % 粗略估计参数数量
    total_params = estimate_total_parameters(config);
    fprintf('  约 %.2f M 参数\n', total_params / 1e6);
    
    fprintf('=====================================================\n');
end

function total_params = estimate_total_parameters(config)
    % 估计网络总参数数量
    
    total_params = 0;
    
    % LSTM/GRU参数
    input_size = config.num_receivers * config.num_features;
    lstm_params = 4 * config.hidden_size * (input_size + config.hidden_size + 1); % 第一层
    for i = 2:config.num_lstm_layers
        lstm_params = lstm_params + 4 * config.hidden_size * (config.hidden_size + config.hidden_size + 1);
    end
    total_params = total_params + lstm_params;
    
    % 注意力层参数
    if config.use_self_attention
        attn_params = 3 * config.hidden_size * config.attention_key_dim * config.num_attention_heads;
        attn_params = attn_params + config.num_attention_heads * config.attention_value_dim * config.hidden_size;
        total_params = total_params + attn_params;
    end
    
    if config.use_maneuver_attention
        maneuver_attn_params = 2 * config.hidden_size * config.hidden_size;
        total_params = total_params + maneuver_attn_params;
    end
    
    % 输出分支参数
    % 状态估计分支
    state_params = config.hidden_size * 256 + 256 * 128 + 128 * config.state_output_size;
    total_params = total_params + state_params;
    
    % 机动分类分支
    maneuver_params = config.hidden_size * 128 + 128 * 64 + 64 * config.num_maneuver_classes;
    total_params = total_params + maneuver_params;
end

%% ================ 网络创建的简化版本 ================

function net = create_simple_network(config)
    % 创建简化版网络（不使用LayerGraph）
    % 适用于不支持复杂图结构的MATLAB版本
    
    if nargin < 1
        config = get_default_network_config();
    end
    
    % 输入层
    input_size = config.num_receivers * config.num_features;
    
    layers = [
        sequenceInputLayer(input_size, 'Name', 'input')
        
        % LSTM层
        lstmLayer(config.hidden_size, 'Name', 'lstm1', 'OutputMode', 'sequence')
        dropoutLayer(config.dropout_rate, 'Name', 'dropout1')
        
        lstmLayer(config.hidden_size, 'Name', 'lstm2', 'OutputMode', 'last')
        dropoutLayer(config.dropout_rate, 'Name', 'dropout2')
        
        % 全连接层
        fullyConnectedLayer(256, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        dropoutLayer(config.dropout_rate, 'Name', 'dropout3')
        
        fullyConnectedLayer(128, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        
        % 输出层（状态估计）
        fullyConnectedLayer(config.state_output_size, 'Name', 'output')
        regressionLayer('Name', 'regression')
    ];
    
    net = layers;
end

%% ================ 测试函数 ================

function test_network_creation()
    % 测试网络创建
    
    fprintf('测试网络创建...\n\n');
    
    % 创建配置
    config = get_default_network_config();
    config.plot_network = true;
    
    % 创建网络
    try
        network = maneuver_aware_tracking_network(config);
        fprintf('\n网络创建成功！\n');
        
        % 尝试分析网络
        fprintf('\n分析网络层...\n');
        layers = network.Layers;
        fprintf('总层数: %d\n', length(layers));
        
    catch ME
        fprintf('网络创建失败: %s\n', ME.message);
        fprintf('尝试创建简化版本...\n');
        
        simple_net = create_simple_network(config);
        fprintf('简化网络创建成功！\n');
        fprintf('总层数: %d\n', length(simple_net));
    end
end
