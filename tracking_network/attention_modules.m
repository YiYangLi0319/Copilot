%% =====================================================================
%  注意力机制模块 - 机动感知跟踪网络
%  Attention Modules for Maneuver-Aware Tracking Network
%  =====================================================================
%  作者: Copilot AI Assistant
%  日期: 2025-11-05
%  
%  功能：
%  1. Self-Attention: 标准多头自注意力机制
%  2. Maneuver-Aware Attention: 针对机动检测的特殊注意力
%  3. Positional Encoding: 位置编码
%  
%  注意：MATLAB深度学习工具箱中的注意力层实现
%  =====================================================================

function attention = attention_modules()
    % 创建注意力模块函数集合
    
    attention = struct();
    attention.create_self_attention = @create_self_attention_layer;
    attention.create_maneuver_attention = @create_maneuver_attention_layer;
    attention.add_positional_encoding = @add_positional_encoding;
    attention.compute_attention_weights = @compute_attention_weights;
end

%% ================ Self-Attention Layer ================

function layers = create_self_attention_layer(input_size, num_heads, key_dim, name_prefix)
    % 创建Self-Attention层
    %
    % 输入:
    %   input_size - 输入特征维度
    %   num_heads - 注意力头数
    %   key_dim - 每个头的维度
    %   name_prefix - 层名称前缀
    %
    % 输出:
    %   layers - 注意力层数组
    
    if nargin < 4
        name_prefix = 'self_attn';
    end
    
    % 注意：MATLAB R2021a+支持selfAttentionLayer
    % 对于较早版本，需要自定义实现
    
    layers = [
        % 多头自注意力层
        % selfAttentionLayer(num_heads, key_dim, 'Name', [name_prefix '_mha'])
        
        % 使用全连接层近似实现注意力机制
        % Query, Key, Value投影
        fullyConnectedLayer(num_heads * key_dim, 'Name', [name_prefix '_query'])
        fullyConnectedLayer(num_heads * key_dim, 'Name', [name_prefix '_key'])
        fullyConnectedLayer(num_heads * key_dim, 'Name', [name_prefix '_value'])
        
        % 注意力计算（通过自定义层实现）
        % 这里使用LayerNormalization和Dropout作为占位符
        layerNormalizationLayer('Name', [name_prefix '_norm'])
        dropoutLayer(0.1, 'Name', [name_prefix '_dropout'])
        
        % 输出投影
        fullyConnectedLayer(input_size, 'Name', [name_prefix '_output'])
    ];
end

%% ================ Maneuver-Aware Attention Layer ================

function layers = create_maneuver_attention_layer(input_size, hidden_size, name_prefix)
    % 创建Maneuver-Aware Attention层
    %
    % 这个注意力机制专门用于识别机动模式变化
    % 通过学习时序模式的突变来检测机动
    %
    % 输入:
    %   input_size - 输入特征维度
    %   hidden_size - 隐藏层维度
    %   name_prefix - 层名称前缀
    %
    % 输出:
    %   layers - 注意力层数组
    
    if nargin < 3
        name_prefix = 'maneuver_attn';
    end
    
    layers = [
        % 特征提取
        fullyConnectedLayer(hidden_size, 'Name', [name_prefix '_fc1'])
        reluLayer('Name', [name_prefix '_relu1'])
        dropoutLayer(0.2, 'Name', [name_prefix '_dropout1'])
        
        % 注意力分数计算
        fullyConnectedLayer(hidden_size, 'Name', [name_prefix '_fc2'])
        tanhLayer('Name', [name_prefix '_tanh'])
        
        % 输出注意力权重
        fullyConnectedLayer(1, 'Name', [name_prefix '_score'])
        softmaxLayer('Name', [name_prefix '_softmax'])
    ];
end

%% ================ Positional Encoding ================

function encoded_input = add_positional_encoding(input_sequence, d_model, max_len)
    % 添加位置编码到输入序列
    %
    % 输入:
    %   input_sequence - [batch_size x sequence_length x d_model]
    %   d_model - 特征维度
    %   max_len - 最大序列长度
    %
    % 输出:
    %   encoded_input - 添加位置编码后的输入
    
    if nargin < 3
        max_len = 1000;
    end
    
    [batch_size, seq_len, ~] = size(input_sequence);
    
    % 生成位置编码
    position = (0:seq_len-1)';
    div_term = exp((0:2:d_model-1) * -(log(10000.0) / d_model));
    
    pe = zeros(seq_len, d_model);
    pe(:, 1:2:end) = sin(position * div_term);
    pe(:, 2:2:end) = cos(position * div_term);
    
    % 扩展到批次维度
    pe_batch = repmat(reshape(pe, [1, seq_len, d_model]), [batch_size, 1, 1]);
    
    % 添加位置编码
    encoded_input = input_sequence + pe_batch;
end

%% ================ Attention Weight Computation ================

function [attention_weights, context] = compute_attention_weights(query, key, value, mask)
    % 计算注意力权重和上下文向量
    %
    % 输入:
    %   query - [batch_size x seq_len_q x d_k]
    %   key - [batch_size x seq_len_k x d_k]
    %   value - [batch_size x seq_len_v x d_v]
    %   mask - 可选的掩码矩阵
    %
    % 输出:
    %   attention_weights - [batch_size x seq_len_q x seq_len_k]
    %   context - [batch_size x seq_len_q x d_v]
    
    if nargin < 4
        mask = [];
    end
    
    [batch_size, seq_len_q, d_k] = size(query);
    [~, seq_len_k, ~] = size(key);
    [~, ~, d_v] = size(value);
    
    % 计算注意力分数: Q * K^T / sqrt(d_k)
    scores = zeros(batch_size, seq_len_q, seq_len_k);
    for b = 1:batch_size
        scores(b, :, :) = query(b, :, :) * key(b, :, :)' / sqrt(d_k);
    end
    
    % 应用掩码（如果有）
    if ~isempty(mask)
        scores = scores + mask * (-1e9);
    end
    
    % Softmax归一化
    attention_weights = zeros(size(scores));
    for b = 1:batch_size
        for i = 1:seq_len_q
            attention_weights(b, i, :) = softmax(squeeze(scores(b, i, :)));
        end
    end
    
    % 计算上下文向量: Attention * V
    context = zeros(batch_size, seq_len_q, d_v);
    for b = 1:batch_size
        context(b, :, :) = squeeze(attention_weights(b, :, :)) * squeeze(value(b, :, :));
    end
end

%% ================ Multi-Head Attention ================

function [output, attention_weights] = multi_head_attention(input, num_heads, d_model, d_k, d_v)
    % 多头注意力机制
    %
    % 输入:
    %   input - [batch_size x seq_len x d_model]
    %   num_heads - 注意力头数
    %   d_model - 模型维度
    %   d_k - Key/Query维度
    %   d_v - Value维度
    %
    % 输出:
    %   output - [batch_size x seq_len x d_model]
    %   attention_weights - 各头的注意力权重
    
    [batch_size, seq_len, ~] = size(input);
    
    % 初始化权重矩阵（在实际训练中这些是可学习参数）
    W_q = randn(d_model, num_heads * d_k) * 0.01;
    W_k = randn(d_model, num_heads * d_k) * 0.01;
    W_v = randn(d_model, num_heads * d_v) * 0.01;
    W_o = randn(num_heads * d_v, d_model) * 0.01;
    
    % 线性投影
    Q = reshape(input, [], d_model) * W_q;
    K = reshape(input, [], d_model) * W_k;
    V = reshape(input, [], d_model) * W_v;
    
    % 重塑为多头格式
    Q = reshape(Q, batch_size, seq_len, num_heads, d_k);
    K = reshape(K, batch_size, seq_len, num_heads, d_k);
    V = reshape(V, batch_size, seq_len, num_heads, d_v);
    
    % 对每个头计算注意力
    attention_weights = cell(1, num_heads);
    head_outputs = zeros(batch_size, seq_len, num_heads, d_v);
    
    for h = 1:num_heads
        Q_h = squeeze(Q(:, :, h, :));
        K_h = squeeze(K(:, :, h, :));
        V_h = squeeze(V(:, :, h, :));
        
        [attn_weights, context] = compute_attention_weights(Q_h, K_h, V_h);
        attention_weights{h} = attn_weights;
        head_outputs(:, :, h, :) = context;
    end
    
    % 连接所有头的输出
    concat_output = reshape(head_outputs, batch_size, seq_len, num_heads * d_v);
    
    % 输出线性投影
    output = reshape(concat_output, [], num_heads * d_v) * W_o;
    output = reshape(output, batch_size, seq_len, d_model);
end

%% ================ Scaled Dot-Product Attention ================

function [output, attention_weights] = scaled_dot_product_attention(Q, K, V, mask, dropout_rate)
    % 缩放点积注意力
    %
    % 输入:
    %   Q - Query矩阵 [batch_size x seq_len_q x d_k]
    %   K - Key矩阵 [batch_size x seq_len_k x d_k]
    %   V - Value矩阵 [batch_size x seq_len_v x d_v]
    %   mask - 可选的掩码
    %   dropout_rate - Dropout比率
    %
    % 输出:
    %   output - 输出 [batch_size x seq_len_q x d_v]
    %   attention_weights - 注意力权重
    
    if nargin < 4
        mask = [];
    end
    if nargin < 5
        dropout_rate = 0.0;
    end
    
    d_k = size(Q, 3);
    
    % 计算注意力权重
    [attention_weights, output] = compute_attention_weights(Q, K, V, mask);
    
    % 应用Dropout（训练时）
    if dropout_rate > 0
        dropout_mask = rand(size(attention_weights)) > dropout_rate;
        attention_weights = attention_weights .* dropout_mask / (1 - dropout_rate);
    end
end

%% ================ 辅助函数：创建因果掩码 ================

function mask = create_causal_mask(seq_len)
    % 创建因果掩码（用于自回归模型）
    % 防止模型看到未来的信息
    %
    % 输入:
    %   seq_len - 序列长度
    %
    % 输出:
    %   mask - [seq_len x seq_len] 掩码矩阵
    
    mask = triu(ones(seq_len, seq_len), 1);
    mask(mask == 1) = -Inf;
    mask(mask == 0) = 0;
end

%% ================ 辅助函数：创建填充掩码 ================

function mask = create_padding_mask(seq_lengths, max_len)
    % 创建填充掩码
    % 用于处理变长序列
    %
    % 输入:
    %   seq_lengths - 每个样本的实际长度 [batch_size x 1]
    %   max_len - 最大序列长度
    %
    % 输出:
    %   mask - [batch_size x max_len] 掩码矩阵
    
    batch_size = length(seq_lengths);
    mask = zeros(batch_size, max_len);
    
    for i = 1:batch_size
        if seq_lengths(i) < max_len
            mask(i, seq_lengths(i)+1:end) = -Inf;
        end
    end
end

%% ================ 注意力可视化函数 ================

function visualize_attention_weights(attention_weights, time_labels, title_text)
    % 可视化注意力权重热图
    %
    % 输入:
    %   attention_weights - [seq_len x seq_len] 注意力权重矩阵
    %   time_labels - 可选的时间标签
    %   title_text - 图表标题
    
    if nargin < 2
        time_labels = [];
    end
    if nargin < 3
        title_text = 'Attention Weights Heatmap';
    end
    
    figure('Name', title_text);
    imagesc(attention_weights);
    colorbar;
    colormap('hot');
    
    xlabel('Key Position');
    ylabel('Query Position');
    title(title_text);
    
    if ~isempty(time_labels)
        xticks(1:length(time_labels));
        xticklabels(time_labels);
        yticks(1:length(time_labels));
        yticklabels(time_labels);
    end
    
    axis equal tight;
end

%% ================ 注意力分析函数 ================

function analysis = analyze_attention_patterns(attention_weights, threshold)
    % 分析注意力模式
    %
    % 输入:
    %   attention_weights - [num_heads x seq_len x seq_len] 或 [seq_len x seq_len]
    %   threshold - 注意力阈值
    %
    % 输出:
    %   analysis - 分析结果结构体
    
    if nargin < 2
        threshold = 0.1;
    end
    
    analysis = struct();
    
    % 检查维度
    if ndims(attention_weights) == 3
        % 多头注意力
        [num_heads, seq_len, ~] = size(attention_weights);
        analysis.num_heads = num_heads;
        analysis.sequence_length = seq_len;
        
        % 计算每个头的平均注意力
        analysis.avg_attention_per_head = zeros(num_heads, 1);
        for h = 1:num_heads
            analysis.avg_attention_per_head(h) = mean(attention_weights(h, :, :), 'all');
        end
        
        % 聚合所有头的注意力
        aggregated_attention = squeeze(mean(attention_weights, 1));
    else
        % 单头注意力
        aggregated_attention = attention_weights;
        seq_len = size(attention_weights, 1);
        analysis.sequence_length = seq_len;
    end
    
    % 计算注意力统计
    analysis.mean_attention = mean(aggregated_attention, 'all');
    analysis.std_attention = std(aggregated_attention, 0, 'all');
    analysis.max_attention = max(aggregated_attention, [], 'all');
    analysis.min_attention = min(aggregated_attention, [], 'all');
    
    % 识别强注意力连接
    strong_connections = aggregated_attention > threshold;
    analysis.num_strong_connections = sum(strong_connections, 'all');
    analysis.strong_connection_ratio = analysis.num_strong_connections / numel(aggregated_attention);
    
    % 计算每个位置的注意力分布
    analysis.attention_per_query = sum(aggregated_attention, 2);
    analysis.attention_per_key = sum(aggregated_attention, 1)';
    
    % 识别关键时间步
    [~, analysis.most_attended_positions] = maxk(analysis.attention_per_key, min(5, seq_len));
    [~, analysis.most_attending_positions] = maxk(analysis.attention_per_query, min(5, seq_len));
end

%% ================ 测试函数 ================

function test_attention_modules()
    % 测试注意力模块的基本功能
    
    fprintf('测试注意力模块...\n\n');
    
    % 测试参数
    batch_size = 4;
    seq_len = 50;
    d_model = 128;
    num_heads = 8;
    d_k = 16;
    d_v = 16;
    
    % 生成随机输入
    input_seq = randn(batch_size, seq_len, d_model);
    
    % 测试位置编码
    fprintf('1. 测试位置编码...\n');
    encoded_input = add_positional_encoding(input_seq, d_model);
    fprintf('   输入形状: [%d x %d x %d]\n', size(input_seq));
    fprintf('   编码后形状: [%d x %d x %d]\n', size(encoded_input));
    fprintf('   完成！\n\n');
    
    % 测试注意力权重计算
    fprintf('2. 测试注意力权重计算...\n');
    Q = randn(batch_size, seq_len, d_k);
    K = randn(batch_size, seq_len, d_k);
    V = randn(batch_size, seq_len, d_v);
    
    [attn_weights, context] = compute_attention_weights(Q, K, V);
    fprintf('   注意力权重形状: [%d x %d x %d]\n', size(attn_weights));
    fprintf('   上下文向量形状: [%d x %d x %d]\n', size(context));
    fprintf('   完成！\n\n');
    
    % 测试多头注意力
    fprintf('3. 测试多头注意力...\n');
    [mha_output, mha_weights] = multi_head_attention(input_seq, num_heads, d_model, d_k, d_v);
    fprintf('   输出形状: [%d x %d x %d]\n', size(mha_output));
    fprintf('   注意力头数: %d\n', length(mha_weights));
    fprintf('   完成！\n\n');
    
    % 测试掩码
    fprintf('4. 测试因果掩码...\n');
    causal_mask = create_causal_mask(seq_len);
    fprintf('   掩码形状: [%d x %d]\n', size(causal_mask));
    fprintf('   完成！\n\n');
    
    % 测试注意力分析
    fprintf('5. 测试注意力分析...\n');
    test_attention = rand(seq_len, seq_len);
    test_attention = test_attention ./ sum(test_attention, 2);  % 归一化
    analysis = analyze_attention_patterns(test_attention, 0.05);
    fprintf('   平均注意力: %.4f\n', analysis.mean_attention);
    fprintf('   强连接数量: %d (%.1f%%)\n', analysis.num_strong_connections, ...
        analysis.strong_connection_ratio * 100);
    fprintf('   完成！\n\n');
    
    fprintf('所有测试完成！\n');
end
