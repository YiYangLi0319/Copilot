function [x_updated, P_updated, innovation, likelihood] = ...
    model_update(x_pred, P_pred, z, R, data, imm_config)
    % 标准卡尔曼滤波更新步骤（数值稳定版）
    
    % 量测矩阵
    H = get_measurement_matrix(x_pred, data);
    
    % 新息
    z_pred = H * x_pred;
    innovation = z - z_pred;
    
    % 新息协方差（添加正则化）
    S = H * P_pred * H' + R;
    
    % 确保对称性和正定性
    S = (S + S') / 2;
    
    % 添加正则化项（防止奇异）
    meas_dim = size(S, 1);
    reg_factor = 1e-4 * trace(S) / meas_dim;
    if reg_factor < 1e-6
        reg_factor = 1e-6;
    end
    S = S + eye(meas_dim) * reg_factor;
    
    % 使用Cholesky分解检查正定性
    try
        L = chol(S, 'lower');
        % 使用Cholesky分解求解卡尔曼增益
        K = (P_pred * H') / S;
    catch
        % 如果失败，使用SVD分解
        warning('S矩阵非正定，使用SVD修正');
        [U, Sigma, V] = svd(S);
        Sigma_reg = max(Sigma, reg_factor);
        S = U * Sigma_reg * V';
        K = (P_pred * H') / S;
    end
    
    % 状态更新
    x_updated = x_pred + K * innovation;
    
    % 协方差更新（Joseph形式，数值稳定）
    I_KH = eye(size(K, 1)) - K * H;
    P_updated = I_KH * P_pred * I_KH' + K * R * K';
    P_updated = (P_updated + P_updated') / 2;
    
    % 添加小的正则化确保正定
    P_updated = P_updated + eye(size(P_updated)) * 1e-8;
    
    % 似然（使用数值稳定的计算方式）
    try
        % 使用对数似然避免下溢
        log_likelihood = -0.5 * (length(innovation) * log(2*pi) + ...
                                 log(det(S)) + ...
                                 innovation' * (S \ innovation));
        likelihood = exp(log_likelihood);
    catch
        % 如果计算失败，使用近似值
        likelihood = exp(-0.5 * norm(innovation)^2 / (trace(S) / length(innovation)));
    end
    
    % 防止数值下溢
    if likelihood < 1e-100 || isnan(likelihood) || isinf(likelihood)
        likelihood = 1e-100;
    end
end

function R_adaptive = adaptive_measurement_noise_estimation(...
    imm_state, model_idx, z_k, data, imm_config, k)
    % 论文核心算法：在线估计量测噪声协方差R（数值稳定版）
    
    model = imm_state.models{model_idx};
    window_size = imm_config.R_adaptation.window_size;
    forgetting_factor = imm_config.R_adaptation.forgetting_factor;
    
    % 计算当前新息
    H = get_measurement_matrix(model.x_pred, data);
    innovation = z_k - H * model.x_pred;
    
    % 更新新息历史（滑动窗口）
    if isempty(model.innovation_history)
        model.innovation_history = innovation;
    else
        model.innovation_history = [model.innovation_history, innovation];
        if size(model.innovation_history, 2) > window_size
            model.innovation_history = model.innovation_history(:, end-window_size+1:end);
        end
    end
    
    % 保存回状态
    imm_state.models{model_idx}.innovation_history = model.innovation_history;
    
    % 如果样本不足，使用初始R
    if size(model.innovation_history, 2) < imm_config.R_adaptation.min_samples
        R_adaptive = model.R_adaptive;
        return;
    end
    
    % 计算样本协方差（带遗忘因子）
    num_samples = size(model.innovation_history, 2);
    weights = forgetting_factor.^(num_samples-1:-1:0);
    weights = weights / sum(weights);
    
    % 加权均值
    innovation_mean = model.innovation_history * weights(:);
    
    % 加权协方差
    R_sample = zeros(size(model.R_adaptive));
    for i = 1:num_samples
        diff = model.innovation_history(:, i) - innovation_mean;
        R_sample = R_sample + weights(i) * (diff * diff');
    end
    
    % 减去预测协方差的贡献（根据论文公式）
    S = H * model.P_pred * H';
    R_adaptive = R_sample - S;
    
    % 确保正定性（关键改进）
    R_adaptive = (R_adaptive + R_adaptive') / 2;
    
    % 使用特征值分解确保正定
    [V, D] = eig(R_adaptive);
    D_diag = diag(D);
    
    % 设置最小特征值（基于初始R的量级）
    min_eigenvalue = 1e-4 * trace(model.R_adaptive) / length(D_diag);
    if min_eigenvalue < 1e-6
        min_eigenvalue = 1e-6;
    end
    
    D_diag = max(D_diag, min_eigenvalue);
    R_adaptive = V * diag(D_diag) * V';
    
    % 确保对称性
    R_adaptive = (R_adaptive + R_adaptive') / 2;
    
    % 平滑更新（避免剧烈变化）
    alpha_smooth = 0.8;  % 增加平滑系数
    R_adaptive = alpha_smooth * model.R_adaptive + (1 - alpha_smooth) * R_adaptive;
    
    % 最终检查：如果仍然有问题，回退到上一时刻的R
    if any(isnan(R_adaptive(:))) || any(isinf(R_adaptive(:)))
        warning('R_adaptive contains NaN or Inf, using previous R');
        R_adaptive = model.R_adaptive;
    end
    
    % 保存到模型状态
    imm_state.models{model_idx}.R_adaptive = R_adaptive;
end

function imm_state = initialize_imm_state(data, imm_config)
    % 初始化IMM状态（改进版）
    
    imm_state = struct();
    imm_state.mu = imm_config.mu0;
    imm_state.models = cell(1, imm_config.num_models);
    
    % 初始状态估计
    x0 = initialize_state_from_measurement(data);
    
    % 初始协方差
    state_dim = length(x0);
    if state_dim == 6
        P0 = diag([1000, 1000, 100, 100, 10, 10]);
    else
        P0 = diag([1000, 1000, 1000, 100, 100, 100, 10, 10, 10]);
    end
    
    % 初始化量测噪声协方差（基于物理参数）
    meas_dim = size(data.measurements, 1);
    num_receivers = data.num_receivers;
    
    % 每个接收机的量测噪声（更合理的初值）
    sigma_range = 10.0;  % 距离标准差 (m)
    sigma_azimuth = 0.01;  % 方位角标准差 (rad, ~0.57度)
    sigma_elevation = 0.01;  % 俯仰角标准差 (rad)
    
    R0 = zeros(meas_dim, meas_dim);
    for r = 1:num_receivers
        idx_start = (r-1) * 3 + 1;
        R0(idx_start:idx_start+2, idx_start:idx_start+2) = diag([
            sigma_range^2,
            sigma_azimuth^2,
            sigma_elevation^2
        ]);
    end
    
    % 初始化每个模型
    for j = 1:imm_config.num_models
        model_state = struct();
        model_state.x_mixed = x0;
        model_state.P_mixed = P0;
        model_state.x_pred = x0;
        model_state.P_pred = P0;
        model_state.x_updated = x0;
        model_state.P_updated = P0;
        model_state.likelihood = 1.0;
        model_state.innovation = [];
        
        % 使用改进的初始R
        model_state.R_adaptive = R0;
        model_state.innovation_history = [];
        
        imm_state.models{j} = model_state;
    end
end

function is_good = check_matrix_condition(M, threshold)
    % 检查矩阵条件数
    if nargin < 2
        threshold = 1e10;
    end
    
    cond_num = cond(M);
    is_good = (cond_num < threshold) && ~any(isnan(M(:))) && ~any(isinf(M(:)));
end
