# 机动感知跟踪网络 (Attention-Based Maneuver-Aware Tracking Network)

基于注意力机制的机动感知跟踪网络实现，用于复现相关论文，利用 `radar_simulation_v3.m` 生成的数据集进行训练和评估。

## 项目概述

本项目实现了一个深度学习框架，用于机动目标跟踪，主要特点包括：

- **LSTM/GRU时序特征提取**：捕捉雷达量测序列的时间依赖关系
- **Self-Attention机制**：全局时序关注，增强长期依赖建模
- **Maneuver-Aware Attention**：专门用于识别机动模式变化
- **双分支输出**：
  - 状态估计分支：预测位置、速度、加速度
  - 机动分类分支：识别机动模式（CV/CA/CT/Singer）
- **多接收机融合**：利用3个双基地雷达接收机的量测信息
- **2D/3D兼容**：支持2D和3D数据集

## 目录结构

```
tracking_network/
├── data_loader_maneuver_tracking.m       # 数据加载和预处理
├── maneuver_aware_tracking_network.m     # 网络架构定义
├── attention_modules.m                   # 注意力机制实现
├── train_maneuver_tracking.m             # 训练脚本
├── evaluate_tracking_network.m           # 评估脚本
├── utils/
│   ├── coordinate_conversion.m           # 坐标转换工具
│   ├── metrics_computation.m             # 性能指标计算
│   └── visualization_tools.m             # 可视化工具
└── README_tracking_network.md            # 本文档
```

## 依赖环境

### 软件要求

- **MATLAB**: R2020a或更高版本
- **深度学习工具箱** (Deep Learning Toolbox)
- **统计和机器学习工具箱** (Statistics and Machine Learning Toolbox)
- 可选：**并行计算工具箱** (Parallel Computing Toolbox) - 用于GPU加速

### 数据要求

需要先运行 `radar_simulation_v3.m` 生成数据集：
- 2D数据集：`enhanced_2d_dataset_v2/`
- 3D数据集：`enhanced_3d_dataset_v2/`

## 快速开始

### 1. 生成数据集

```matlab
% 运行雷达仿真脚本生成数据集
radar_simulation_v3
```

这将在当前目录下生成：
- `enhanced_2d_dataset_v2/` - 包含20个2D场景
- `enhanced_3d_dataset_v2/` - 包含20个3D场景

### 2. 加载数据

```matlab
% 切换到tracking_network目录
cd tracking_network

% 创建数据加载器配置
config = struct();
config.sequence_length = 50;      % 输入序列长度
config.stride = 10;                % 滑动窗口步长
config.train_ratio = 0.7;          % 训练集比例
config.val_ratio = 0.15;           % 验证集比例
config.test_ratio = 0.15;          % 测试集比例

% 加载数据
data_loader = data_loader_maneuver_tracking('../enhanced_2d_dataset_v2', config);
```

### 3. 训练网络

```matlab
% 配置训练参数
train_config = struct();
train_config.batch_size = 32;
train_config.num_epochs = 100;
train_config.learning_rate = 0.001;
train_config.hidden_size = 128;
train_config.num_lstm_layers = 2;

% 开始训练
train_results = train_maneuver_tracking('../enhanced_2d_dataset_v2', train_config);
```

### 4. 评估模型

```matlab
% 配置评估参数
eval_config = struct();
eval_config.num_visualization_samples = 5;
eval_config.plot_trajectories = true;
eval_config.plot_attention_weights = true;

% 评估模型
eval_results = evaluate_tracking_network(...
    'trained_models/trained_model_20251105_120000.mat', ...
    '../enhanced_2d_dataset_v2', ...
    eval_config);
```

## 详细使用说明

### 网络配置参数

#### 数据配置
- `sequence_length`: 输入序列长度（默认：50）
- `stride`: 滑动窗口步长（默认：10）
- `normalize_method`: 归一化方法，'minmax'或'zscore'（默认：'minmax'）

#### 网络架构配置
- `hidden_size`: LSTM隐藏层大小（默认：128）
- `num_lstm_layers`: LSTM层数（默认：2）
- `lstm_type`: 'lstm'或'gru'（默认：'lstm'）
- `num_attention_heads`: 注意力头数（默认：8）
- `dropout_rate`: Dropout比例（默认：0.2）

#### 训练配置
- `batch_size`: 批次大小（默认：32）
- `num_epochs`: 训练轮数（默认：100）
- `learning_rate`: 学习率（默认：0.001）
- `state_loss_weight`: 状态损失权重（默认：1.0）
- `maneuver_loss_weight`: 机动损失权重（默认：0.5）

### 损失函数

网络使用加权组合损失函数：

```
Total Loss = α × State Loss + β × Maneuver Loss
```

其中：
- **State Loss**: 状态估计的均方误差（MSE）
  - 包括位置、速度、加速度的误差
- **Maneuver Loss**: 机动分类的交叉熵损失
  - 4类分类问题（CV/CA/CT/Singer）

### 性能指标

#### 状态估计指标
- **Position RMSE**: 位置均方根误差（米）
- **Velocity RMSE**: 速度均方根误差（米/秒）
- **Acceleration RMSE**: 加速度均方根误差（米/秒²）

#### 机动分类指标
- **Accuracy**: 分类准确率
- **Precision**: 精确率（各类别）
- **Recall**: 召回率（各类别）
- **F1 Score**: F1分数（各类别）
- **Confusion Matrix**: 混淆矩阵

## 高级功能

### 1. 自定义网络架构

```matlab
% 创建自定义网络配置
net_config = struct();
net_config.hidden_size = 256;              % 更大的隐藏层
net_config.num_lstm_layers = 3;            % 更多LSTM层
net_config.use_self_attention = true;      % 启用Self-Attention
net_config.use_maneuver_attention = true;  % 启用Maneuver-Aware Attention

% 构建网络
network = maneuver_aware_tracking_network(net_config);
```

### 2. 注意力权重分析

```matlab
% 加载注意力模块
attention = attention_modules();

% 分析注意力模式
analysis = attention.compute_attention_weights(Q, K, V);

% 可视化注意力权重
attention.visualize_attention_weights(attn_weights, time_labels, '注意力分析');
```

### 3. 自定义可视化

```matlab
% 使用可视化工具
addpath('utils');

% 绘制3D轨迹对比
plot_trajectory_3d({true_traj, pred_traj}, {'真实', '预测'}, '轨迹对比');

% 绘制机动检测时间线
plot_maneuver_timeline(time_steps, true_maneuvers, pred_maneuvers);

% 绘制注意力热图
plot_attention_heatmap(attention_weights, '注意力权重');
```

### 4. 批量评估

```matlab
% 评估多个模型
model_files = {
    'trained_models/model_epoch_50.mat'
    'trained_models/model_epoch_100.mat'
    'trained_models/model_final.mat'
};

results = cell(length(model_files), 1);
for i = 1:length(model_files)
    results{i} = evaluate_tracking_network(model_files{i}, dataset_path, config);
end

% 比较性能
compare_model_performance(results);
```

## 实验结果示例

### 预期性能（参考）

基于2D数据集的典型结果：

| 指标 | 数值 |
|------|------|
| 位置RMSE | ~5-10 m |
| 速度RMSE | ~2-5 m/s |
| 加速度RMSE | ~1-3 m/s² |
| 机动分类准确率 | ~80-90% |

### 可视化输出

训练和评估过程会生成以下可视化：

1. **训练曲线**：训练和验证损失随epoch的变化
2. **轨迹对比图**：真实轨迹vs预测轨迹（2D/3D）
3. **注意力热图**：时序注意力权重分布
4. **机动检测时间线**：机动模式切换的检测情况
5. **误差分布图**：位置、速度、加速度误差的统计分布
6. **混淆矩阵**：机动分类的详细结果

## 故障排除

### 常见问题

#### 1. 内存不足

**问题**：训练时出现"Out of Memory"错误

**解决方案**：
- 减小批次大小：`config.batch_size = 16`
- 减少序列长度：`config.sequence_length = 30`
- 使用更小的网络：`config.hidden_size = 64`

#### 2. 训练不收敛

**问题**：损失值不下降或波动剧烈

**解决方案**：
- 降低学习率：`config.learning_rate = 0.0001`
- 增加梯度裁剪：`config.gradient_threshold = 0.5`
- 检查数据归一化是否正确
- 调整损失权重比例

#### 3. GPU内存问题

**问题**：GPU内存不足

**解决方案**：
```matlab
% 使用CPU训练
options.ExecutionEnvironment = 'cpu';

% 或者启用混合精度训练（R2020a+）
options.ExecutionEnvironment = 'auto';
```

#### 4. 数据加载错误

**问题**：找不到数据集文件

**解决方案**：
- 确保已运行 `radar_simulation_v3.m` 生成数据
- 检查数据集路径是否正确
- 确认数据集目录结构完整

## 进阶使用

### 1. 数据增强

```matlab
% 在data_loader中添加数据增强
function augmented_samples = augment_data(samples)
    % 添加噪声
    noise = randn(size(samples)) * 0.1;
    augmented_samples = samples + noise;
    
    % 时间缩放
    % scale_factor = 0.8 + rand() * 0.4;
    % ...
end
```

### 2. 迁移学习

```matlab
% 加载预训练模型
pretrained_model = load('trained_models/pretrained_2d.mat');

% 微调网络
fine_tune_config = config;
fine_tune_config.learning_rate = 0.0001;  % 较小的学习率
fine_tune_config.num_epochs = 20;          % 较少的epoch

% 在新数据集上微调
results = train_maneuver_tracking(new_dataset_path, fine_tune_config);
```

### 3. 集成学习

```matlab
% 训练多个模型
num_models = 5;
models = cell(num_models, 1);

for i = 1:num_models
    config.random_seed = 42 + i;
    models{i} = train_maneuver_tracking(dataset_path, config);
end

% 集成预测
ensemble_predictions = ensemble_predict(models, test_data);
```

## 性能优化建议

### 1. 网络架构优化
- 使用GRU代替LSTM可以减少参数量和训练时间
- 适当减少注意力头数可以加速训练
- 考虑使用残差连接改善梯度流动

### 2. 训练优化
- 使用学习率预热和余弦衰减
- 实施早停策略避免过拟合
- 使用梯度累积处理大批次

### 3. 数据优化
- 增加数据量可以显著提升性能
- 平衡各机动模式的样本分布
- 使用在线数据增强

## 引用

如果使用本实现，请引用相关论文：

```bibtex
@article{attention_maneuver_tracking,
  title={Attention-Based Maneuver-Aware Tracking Network for Maneuvering Target Tracking},
  author={...},
  journal={...},
  year={2024}
}
```

## 许可证

本项目遵循MIT许可证。

## 贡献

欢迎提交Issue和Pull Request！

## 联系方式

- 作者：YiYangLi0319
- 项目地址：https://github.com/YiYangLi0319/Copilot

## 更新日志

### v1.0.0 (2025-11-05)
- 初始版本发布
- 实现完整的注意力机制跟踪网络
- 支持2D/3D数据集
- 提供完整的训练和评估流程
- 包含丰富的可视化工具

## 附录

### A. 网络架构详细说明

网络采用编码器-解码器架构：

```
Input Layer (3 receivers × 3 features × 50 time steps)
    ↓
Feature Preprocessing (Fully Connected)
    ↓
LSTM Layers (2 layers, 128 hidden units)
    ↓
Self-Attention Layer (8 heads)
    ↓
Maneuver-Aware Attention Layer
    ↓
    ├─→ State Estimation Branch
    │     ├─ FC Layer (256 units)
    │     ├─ FC Layer (128 units)
    │     └─ Output (9-dim: pos + vel + acc)
    │
    └─→ Maneuver Classification Branch
          ├─ FC Layer (128 units)
          ├─ FC Layer (64 units)
          └─ Output (4 classes: CV/CA/CT/Singer)
```

### B. 数据格式说明

#### 输入格式
```matlab
% 输入样本
samples: [N × 3 × 3 × 50]
% N: 样本数
% 3: 接收机数量
% 3: 特征维度（距离、方位角、仰角）
% 50: 时间步数
```

#### 输出格式
```matlab
% 状态标签
labels_state: [N × 9]
% 9维：位置(3) + 速度(3) + 加速度(3)

% 机动标签
labels_maneuver: [N × 1]
% 1=CV, 2=CA, 3=CT, 4=Singer
```

### C. 常用函数参考

#### 数据加载
```matlab
data_loader_maneuver_tracking(dataset_path, config)
```

#### 网络构建
```matlab
maneuver_aware_tracking_network(net_config)
```

#### 训练
```matlab
train_maneuver_tracking(dataset_path, train_config)
```

#### 评估
```matlab
evaluate_tracking_network(model_file, dataset_path, eval_config)
```

### D. 配置模板

完整的配置模板见各脚本中的 `get_default_config()` 函数。

---

**最后更新**: 2025-11-05
