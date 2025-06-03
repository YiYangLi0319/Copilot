% 参数设置
c = 3e8;                    % 光速 (m/s)
% Tx_pos = [5e3,11e3,10e3;]    % 发射机位置 (m)
          % 8.5e3,8e3,5e3;
          % 11e3,4e3,1e3];
% Rx_pos = [7e3,9.5e3,2.5e3;  % 接收机位置 (m)
%           10e3,6.5e3,4.5e3;
%           12e3,2e3,5e3;
%           14e3,1e3,7.5e3];
% Tx_pos = [0,0,0];    % 发射机位置 (m)
% 
% Rx_pos = [200*sin(0:pi/8:pi/2).*cos(0:pi/12:pi/3);
%           200*sin(0:pi/8:pi/2).*sin(0:pi/12:pi/3); 
%           200*cos(0:pi/8:pi/2);                  ]';
% u_true = [-1000;1000;1000];     % 真实目标位置 (m)
Tx_pos = [
    1000,1000,500];    % 发射机位置 (m)

d = 500;
Rx_pos = [0,0,d;
          0,0,-d;
          d,0,0;
          -d,0,0;
          0,d,0;
          0,-d,0;];

u_true = [250;-250;250];     % 真实目标位置 (m)
M = size(Tx_pos,1);          % 发射机数量
N = size(Rx_pos,1);          % 接收机数量
numMC = 2000;                % 蒙特卡洛仿真次数

% 仿真参数设置
% sigma_t_values = 0.05:0.05:0.5;    % 时间测量噪声 (ns)
% sigma_AOA_fixed = 0.02;      % 固定角度噪声 (度)
% sigma_AOA_values = 0.01:0.01:0.1; % 角度噪声范围 (度)
% sigma_t_values = 5:5:50;    % 时间测量噪声 (ns)
sigma_r_db = -40:5:20;
sigma_t_values = db2mag(sigma_r_db)/3e8*1e9;    % 时间测量噪声 (ns)
sigma_AOA_fixed = 0.02;       % 固定角度噪声 (度)
sigma_AOA_values = 1:0.5:3; % 角度噪声范围 (度)

% 预存储结果
MSE_hybrid = zeros(length(sigma_t_values),1);
CRLB_hybrid = zeros(length(sigma_t_values),1);
MSE_AOA = zeros(length(sigma_AOA_values),1);
CRLB_AOA = zeros(length(sigma_AOA_values),1);

%% 仿真1：固定角度噪声，变化BR噪声
for idx = 1:length(sigma_t_values)
    sigma_t = sigma_t_values(idx)*1e-9; % 转换为秒
    sigma_r = c*sigma_t;               % BR噪声标准差 (m)
    sigma_AOA = deg2rad(sigma_AOA_fixed); % 转换为弧度
    
    % 预存储估计结果
    estimates = zeros(3,numMC);
    
    % 蒙特卡洛仿真
    for mc = 1:numMC
        % 生成测量噪声
        noise_r = sigma_r * randn(M,N);
        noise_theta = sigma_AOA * randn(N,1);
        noise_phi = sigma_AOA * randn(N,1);
        
        % 生成带噪声的测量值
        [BR_meas, AOA_theta_meas, AOA_phi_meas] = generate_measurements(u_true, Tx_pos, Rx_pos, noise_r, noise_theta, noise_phi);
        
        % 两阶段定位算法
        u_est = two_stage_estimator(BR_meas, AOA_theta_meas, AOA_phi_meas, Tx_pos, Rx_pos, sigma_r, sigma_AOA);
        
        estimates(:,mc) = u_est;
    end
    
    % 计算MSE
    MSE_hybrid(idx) = mean(sum((estimates - u_true).^2,1));
    
    % 计算CRLB
    CRLB_hybrid(idx) = trace(compute_CRLB(u_true, Tx_pos, Rx_pos, sigma_r, sigma_AOA));
end

%% 仿真2：固定BR噪声，变化角度噪声
% sigma_t_fixed = 40e-9;       % 固定时间噪声 (40ns)
% sigma_r_fixed = c*sigma_t_fixed; % 固定BR噪声 (m)
% 
% for idx = 1:length(sigma_AOA_values)
%     sigma_AOA = deg2rad(sigma_AOA_values(idx)); % 转换为弧度
%     
%     % 预存储估计结果
%     estimates = zeros(3,numMC);
%     
%     % 蒙特卡洛仿真
%     for mc = 1:numMC
%         % 生成测量噪声
%         noise_r = sigma_r_fixed * randn(M,N);
%         noise_theta = sigma_AOA * randn(N,1);
%         noise_phi = sigma_AOA * randn(N,1);
%         
%         % 生成带噪声的测量值
%         [BR_meas, AOA_theta_meas, AOA_phi_meas] = generate_measurements(u_true, Tx_pos, Rx_pos, noise_r, noise_theta, noise_phi);
%         
%         % 两阶段定位算法
%         u_est = two_stage_estimator(BR_meas, AOA_theta_meas, AOA_phi_meas, Tx_pos, Rx_pos, sigma_r_fixed, sigma_AOA);
%         
%         estimates(:,mc) = u_est;
%     end
%     
%     % 计算MSE
%     MSE_AOA(idx) = mean(sum((estimates - u_true).^2,1));
%     
%     % 计算CRLB
%     CRLB_AOA(idx) = trace(compute_CRLB(u_true, Tx_pos, Rx_pos, sigma_r_fixed, sigma_AOA));
% end

%% 绘制结果
% figure('Position',[100 100 800 600])

% 图1：BR噪声变化
% subplot(2,1,1)
figure;
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_hybrid),'b-o','LineWidth',2,'MarkerSize',8)
hold on
plot(db(sigma_t_values*1e-9*3e8), 10*log10(CRLB_hybrid),'r--','LineWidth',2)
grid on
xlabel('Time Measurement Noise \sigma_t (ns)')
ylabel('10log(MSE (m^2))')
legend('Proposed Method','CRLB','Location','northwest')
title('Localization Performance vs BR Measurement Noise')

% 图2：AOA噪声变化
% subplot(2,1,2)
% semilogy(sigma_AOA_values, 10*log10(MSE_AOA),'b-o','LineWidth',2,'MarkerSize',8)
% hold on
% semilogy(sigma_AOA_values, 10*log10(CRLB_AOA),'r--','LineWidth',2)
% grid on
% xlabel('AOA Measurement Noise \sigma_{AOA} (degree)')
% ylabel('10log(MSE (m^2))')
% legend('Proposed Method','CRLB','Location','northwest')
% title('Localization Performance vs AOA Measurement Noise')

%% 辅助函数
function [BR_meas, AOA_theta_meas, AOA_phi_meas] = generate_measurements(u, Tx, Rx, noise_r, noise_theta, noise_phi)
    M = size(Tx,1);
    N = size(Rx,1);
    BR_meas = zeros(M,N);
    
    for i = 1:M
        for j = 1:N
            d_ij = norm(Tx(i,:)-Rx(j,:));
            r_i = norm(u - Tx(i,:)');
            r_j = norm(u - Rx(j,:)');
            BR_true = r_i + r_j - d_ij;
            BR_meas(i,j) = BR_true + noise_r(i,j);
        end
    end
    
    AOA_theta_meas = zeros(N,1);
    AOA_phi_meas = zeros(N,1);
    for j = 1:N
        dx = u(1) - Rx(j,1);
        dy = u(2) - Rx(j,2);
        dz = u(3) - Rx(j,3);
        AOA_theta_true = atan2(dy, dx);
        AOA_phi_true = asin(dz/norm(u - Rx(j,:)'));
        AOA_theta_meas(j) = AOA_theta_true + noise_theta(j);
        AOA_phi_meas(j) = AOA_phi_true + noise_phi(j);
    end
end

function CRLB = compute_CRLB(u, Tx, Rx, sigma_r, sigma_AOA)
    M = size(Tx,1);
    N = size(Rx,1);
    
    % 计算Fisher信息矩阵
    FIM = zeros(3);
    
    % BR部分的Fisher信息
    for i = 1:M
        for j = 1:N
            r_i = u - Tx(i,:)';
            r_j = u - Rx(j,:)';
            dr_dx = (r_i/norm(r_i) + r_j/norm(r_j))';
            FIM = FIM + (dr_dx'*dr_dx)/(sigma_r^2);
        end
    end
    
    % 方位角部分的Fisher信息
    for j = 1:N
        r_j = u - Rx(j,:)';
        dx = r_j(1); dy = r_j(2); dz = r_j(3);
        rho_xy = norm(r_j(1:2));
        
        % 方位角导数
        dtheta_dx = -dy/(dx^2 + dy^2);
        dtheta_dy = dx/(dx^2 + dy^2);
        dtheta_dz = 0;
        grad_theta = [dtheta_dx; dtheta_dy; dtheta_dz];
        
        
        FIM = FIM +(grad_theta*grad_theta')/(sigma_AOA^2) ;     

    end
    
    CRLB = inv(FIM);
end

function u_est = two_stage_estimator(BR_meas, AOA_theta, AOA_phi, Tx, Rx, sigma_r, sigma_AOA)
    M = size(Tx,1);
    N = size(Rx,1);
    
    % 第一阶段：构建伪线性方程
    h1 = [];
    G1 = [];
    
    % BR测量部分
    for i = 1:M
        for j = 1:N
            d_ij = norm(Tx(i,:) - Rx(j,:));
            term = BR_meas(i,j)^2 + 2*BR_meas(i,j)*d_ij + 2*(Rx(j,:)-Tx(i,:))*Rx(j,:)';
            h1 = [h1; term];
            
            G_row = [2*(Rx(j,:)-Tx(i,:)), zeros(1,j-1), 2*(BR_meas(i,j)+d_ij), zeros(1,N-j)];
            G1 = [G1; G_row];
        end
    end

    % AOA测量部分
    for j = 1:N
        % 方位角
        vj = [cos(AOA_theta(j)); sin(AOA_theta(j)); 0];
        wj = [-sin(AOA_theta(j)); cos(AOA_theta(j)); 0];
        h1 = [h1; wj'*Rx(j,:)'];
        G_row = [wj', zeros(1,N)];
        G1 = [G1; G_row];
        
        % 俯仰角
        kj = [0; 0; 1];
        h1 = [h1; kj'*Rx(j,:)'];
        G_row = [kj', zeros(1,j-1), -sin(AOA_phi(j)), zeros(1,N-j)];
        G1 = [G1; G_row];
    end
    
    % 第一阶段WLS估计
    W1 = inv(diag([repmat(1/sigma_r^2, M*N, 1); 
              repmat(1/sigma_AOA^2, N, 1);  
              repmat(1/sigma_AOA^2, N, 1)]));
    % 使用第一阶段估计的 u_hat 计算 B1 和 W1
    
    Psi = inv(G1'*W1*G1) * (G1'*W1*h1);
    u_hat = Psi(1:3);
    r_hat = Psi(4:end);
    for i = 1:2
        W1 = compute_W1(u_hat, Tx, Rx, BR_meas, AOA_theta, AOA_phi, sigma_r, sigma_AOA);
        Psi = inv(G1'*W1*G1) * (G1'*W1*h1);
        u_hat = Psi(1:3);
        r_hat = Psi(4:end);
        % temp=[-50;50;50];
        % sum((u_hat - temp).^2)
    end
    % 第二阶段：误差修正
    h2 = [];
    G2 = [];
    % h2(1:3) = u_hat;
    % G2(1:3,:) = -eye(3);
    
    % for j = 1:N
    %     h2(3+j) = r_hat(j)^2 - norm(u_hat - Rx(j,:)')^2;
    %     G2(3+j,:) = -2*(u_hat - Rx(j,:)')';
    % end
    for j = 1:N
        term = r_hat(j)^2 - norm(u_hat - Rx(j,:)')^2;
        h2 = [h2; term];
        G2_row = [ -2*(u_hat - Rx(j,:)')' ];
        G2 = [G2; G2_row];
    end
    h2 = [zeros(3,1); h2];
    G2 = [ -eye(3); G2 ];
    % W2 = diag([ones(3,1); 1./(4*r_hat.^2*sigma_r^2)]);
    B_psi = 2 * diag(r_hat);
    B2 = blkdiag(eye(3), B_psi);
    inv_B2 = blkdiag(eye(3), diag(1./(2*r_hat))); % B_psi 的逆是对角元素取倒数
    cov_Psi = inv(G1' * W1 * G1); % 第一阶段协方差矩阵
    W2 = inv_B2' * inv(cov_Psi) * inv_B2; % 论文公式
    delta_u = inv(G2'*W2*G2) * (G2'*W2*h2);
    
    u_est = u_hat - delta_u;





% % WLS求解第二阶段
% W2 = eye(size(G2,1));  % 初始权重矩阵（简化为单位矩阵）
% delta_u = (G2' * W2 * G2) \ (G2' * W2 * h2);
% u_final = u_hat - delta_u(1:3);


end

function W1 = compute_W1(u, Tx, Rx, BR_meas, AOA_theta, AOA_phi, sigma_r, sigma_AOA)
    M = size(Tx,1);
    N = size(Rx,1);
    
    % 1. 计算 B_r
    r_t = zeros(M,1);
    for i = 1:M
        r_t(i) = norm(u - Tx(i,:)'); % 目标到发射机的距离
    end
    B_r = 2 * kron(eye(N), diag(r_t));
    
    % 2. 计算 B_theta
    B_theta = zeros(N);
    for j = 1:N
        vj = [cos(AOA_theta(j)); sin(AOA_theta(j)); 0];
        B_theta(j,j) = vj' * (u - Rx(j,:)');
    end
    
    % 3. 计算 B_phi
    B_phi = zeros(N);
    for j = 1:N
        r_j = norm(u - Rx(j,:)');
        B_phi(j,j) = cos(AOA_phi(j)) * r_j;
    end
    
    % 4. 构造 B1
    B1 = blkdiag(B_r,B_theta, B_phi);
    
    % 5. 计算 Q_m
    Q_r = sigma_r^2 * eye(M*N);
    Q_theta = sigma_AOA^2 * eye(N);
    Q_phi = sigma_AOA^2 * eye(N);
    Q_m = blkdiag(Q_r,Q_theta, Q_phi);
    
    % 6. 计算 W1
    inv_B1 = inv(B1);
    W1 = inv_B1' * inv(Q_m) * inv_B1;
end