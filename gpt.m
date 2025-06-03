% 参数设置
c = 3e8;                    % 光速 (m/s)

% 发射站和接收站布局
Tx_pos = [1000,1000,500];    % 发射机位置 (m)

% 接收站布局 - 立方体顶点
d = 500; 
Rx_pos = [0,0,d;
           0,0,-d;
           d,0,0;
           -d,0,0;
           0,d,0;
           0,-d,0];

% 目标位置
u_true = [250;-250;250];     % 真实目标位置 (m)

%% 场景可视化
figure
scatter3(Tx_pos(1),Tx_pos(2),Tx_pos(3),100,'red','filled');hold on;
scatter3(Rx_pos(:,1),Rx_pos(:,2),Rx_pos(:,3),80,'blue','filled');
scatter3(u_true(1),u_true(2),u_true(3),120,'green','filled');
legend('发射机', '接收机', '目标');
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('定位系统几何配置');

M = size(Tx_pos,1);          % 发射机数量
N = size(Rx_pos,1);          % 接收机数量
numMC = 50;                  % 蒙特卡洛仿真次数（增加稳定性）

% 仿真参数设置
sigma_r_db = -40:5:5;
sigma_t_values = db2mag(sigma_r_db)/3e8*1e9;

% 角度噪声设置 - 增大俯仰角噪声
sigma_azimuth_fixed = 0.02;     % 方位角噪声 (度)
sigma_elevation_fixed = 0.1;    % 俯仰角噪声 (度) - 是方位角噪声的5倍

% 预存储结果
MSE_twostage = zeros(length(sigma_t_values),1);
MSE_classic_wls = zeros(length(sigma_t_values),1);
MSE_nonlinear = zeros(length(sigma_t_values),1);
CRLB_hybrid = zeros(length(sigma_t_values),1);

MSE_xyz_twostage = zeros(length(sigma_t_values),3);  % 分别记录x,y,z方向的MSE
MSE_xyz_classic_wls = zeros(length(sigma_t_values),3);
MSE_xyz_nonlinear = zeros(length(sigma_t_values),3);

%% 仿真：固定角度噪声，变化BR噪声
for idx = 1:length(sigma_t_values)
    sigma_t = sigma_t_values(idx)*1e-9;         % 转换为秒
    sigma_r = c*sigma_t;                        % BR噪声标准差 (m)
    sigma_azimuth = deg2rad(sigma_azimuth_fixed);    % 转换为弧度
    sigma_elevation = deg2rad(sigma_elevation_fixed); % 转换为弧度
    
    % 预存储估计结果
    estimates_twostage = zeros(3,numMC);
    estimates_classic_wls = zeros(3,numMC);
    estimates_nonlinear = zeros(3,numMC);
    
    % 蒙特卡洛仿真
    for mc = 1:numMC
        % 生成测量噪声
        noise_r = sigma_r * randn(M,N);
        noise_azimuth = sigma_azimuth * randn(N,1);
        noise_elevation = sigma_elevation * randn(N,1);
        
        % 生成带噪声的测量值
        [BR_meas, AOA_theta_meas, AOA_phi_meas] = generate_measurements(u_true, Tx_pos, Rx_pos, noise_r, noise_azimuth, noise_elevation);
        
        % 方法1: 两阶段定位算法
        u_est_twostage = two_stage_estimator(BR_meas, AOA_theta_meas, AOA_phi_meas, Tx_pos, Rx_pos, sigma_r, sigma_azimuth, sigma_elevation);
        estimates_twostage(:,mc) = u_est_twostage;
        
        % 方法2: 经典加权最小二乘
        u_est_classic_wls = classic_weighted_ls(BR_meas, AOA_theta_meas, AOA_phi_meas, Tx_pos, Rx_pos, sigma_r, sigma_azimuth, sigma_elevation);
        estimates_classic_wls(:,mc) = u_est_classic_wls;
        
        % 方法3: 非线性优化
        options = optimoptions('lsqnonlin','Display','off','MaxIterations',50);
        fun = @(u) br_aoa_residuals(u, Tx_pos, Rx_pos, BR_meas, AOA_theta_meas, AOA_phi_meas, sigma_r, sigma_azimuth, sigma_elevation);
        u_est_nonlinear = lsqnonlin(fun, u_est_twostage, [], [], options);
        estimates_nonlinear(:,mc) = u_est_nonlinear;
    end
    
    % 计算总体MSE
    MSE_twostage(idx) = mean(sum((estimates_twostage - u_true).^2,1));
    MSE_classic_wls(idx) = mean(sum((estimates_classic_wls - u_true).^2,1));
    MSE_nonlinear(idx) = mean(sum((estimates_nonlinear - u_true).^2,1));
    
    % 计算各方向MSE - 两阶段法
    MSE_xyz_twostage(idx,1) = mean((estimates_twostage(1,:) - u_true(1)).^2);
    MSE_xyz_twostage(idx,2) = mean((estimates_twostage(2,:) - u_true(2)).^2);
    MSE_xyz_twostage(idx,3) = mean((estimates_twostage(3,:) - u_true(3)).^2);
    
    % 计算各方向MSE - 经典WLS
    MSE_xyz_classic_wls(idx,1) = mean((estimates_classic_wls(1,:) - u_true(1)).^2);
    MSE_xyz_classic_wls(idx,2) = mean((estimates_classic_wls(2,:) - u_true(2)).^2);
    MSE_xyz_classic_wls(idx,3) = mean((estimates_classic_wls(3,:) - u_true(3)).^2);
    
    % 计算各方向MSE - 非线性优化
    MSE_xyz_nonlinear(idx,1) = mean((estimates_nonlinear(1,:) - u_true(1)).^2);
    MSE_xyz_nonlinear(idx,2) = mean((estimates_nonlinear(2,:) - u_true(2)).^2);
    MSE_xyz_nonlinear(idx,3) = mean((estimates_nonlinear(3,:) - u_true(3)).^2);
    
    % 计算CRLB (考虑不同的角度噪声)
    CRLB_hybrid(idx) = trace(compute_CRLB(u_true, Tx_pos, Rx_pos, sigma_r, sigma_azimuth, sigma_elevation));
    
    % 显示进度
    fprintf('完成仿真 %d/%d, MSE(两阶段/经典WLS/非线性) = %.2e/%.2e/%.2e dB\n', idx, length(sigma_t_values), ...
            10*log10(MSE_twostage(idx)), 10*log10(MSE_classic_wls(idx)), 10*log10(MSE_nonlinear(idx)));
end

%% 绘制结果
figure('Position',[100 100 1200 900])

% 图1：总体性能比较
subplot(2,2,1)
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_twostage),'k-o','LineWidth',2,'MarkerSize',8)
hold on
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_classic_wls),'b-s','LineWidth',2,'MarkerSize',8)
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_nonlinear),'g-d','LineWidth',2,'MarkerSize',8)
plot(db(sigma_t_values*1e-9*3e8), 10*log10(CRLB_hybrid),'r--','LineWidth',2)
grid on
xlabel('测量噪声 \sigma_r (dB)')
ylabel('MSE (dB)')
legend('两阶段方法','经典WLS','非线性优化','CRLB','Location','northwest')
title(['总体定位性能比较 (俯仰角噪声=',num2str(sigma_elevation_fixed),'°)'])

% 图2：X方向性能比较
subplot(2,2,2)
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_xyz_twostage(:,1)),'k-o','LineWidth',2,'MarkerSize',6)
hold on
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_xyz_classic_wls(:,1)),'b-s','LineWidth',2,'MarkerSize',6)
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_xyz_nonlinear(:,1)),'g-d','LineWidth',2,'MarkerSize',6)
grid on
xlabel('测量噪声 \sigma_r (dB)')
ylabel('X方向MSE (dB)')
legend('两阶段方法','经典WLS','非线性优化','Location','northwest')
title('X方向定位性能')

% 图3：Y方向性能比较
subplot(2,2,3)
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_xyz_twostage(:,2)),'k-o','LineWidth',2,'MarkerSize',6)
hold on
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_xyz_classic_wls(:,2)),'b-s','LineWidth',2,'MarkerSize',6)
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_xyz_nonlinear(:,2)),'g-d','LineWidth',2,'MarkerSize',6)
grid on
xlabel('测量噪声 \sigma_r (dB)')
ylabel('Y方向MSE (dB)')
legend('两阶段方法','经典WLS','非线性优化','Location','northwest')
title('Y方向定位性能')

% 图4：Z方向性能比较
subplot(2,2,4)
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_xyz_twostage(:,3)),'k-o','LineWidth',2,'MarkerSize',6)
hold on
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_xyz_classic_wls(:,3)),'b-s','LineWidth',2,'MarkerSize',6)
plot(db(sigma_t_values*1e-9*3e8), 10*log10(MSE_xyz_nonlinear(:,3)),'g-d','LineWidth',2,'MarkerSize',6)
grid on
xlabel('测量噪声 \sigma_r (dB)')
ylabel('Z方向MSE (dB)')
legend('两阶段方法','经典WLS','非线性优化','Location','northwest')
title('Z方向定位性能 (受俯仰角噪声影响)')

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

function CRLB = compute_CRLB(u, Tx, Rx, sigma_r, sigma_azimuth, sigma_elevation)
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
    
    % AOA部分的Fisher信息 - 分别考虑方位角和俯仰角
    for j = 1:N
        r_j = u - Rx(j,:)';
        dx = r_j(1); dy = r_j(2); dz = r_j(3);
        rho_xy = norm(r_j(1:2));
        
        % 方位角导数
        dtheta_dx = -dy/(dx^2 + dy^2);
        dtheta_dy = dx/(dx^2 + dy^2);
        dtheta_dz = 0;
        grad_theta = [dtheta_dx; dtheta_dy; dtheta_dz];
        
        % 俯仰角导数
        dphi_dx = (-dx*dz)/(norm(r_j)^2*rho_xy);
        dphi_dy = (-dy*dz)/(norm(r_j)^2*rho_xy);
        dphi_dz = rho_xy/(norm(r_j)^2);
        grad_phi = [dphi_dx; dphi_dy; dphi_dz];
        
        % 分别考虑不同噪声级别的贡献
        FIM = FIM + (grad_theta*grad_theta')/(sigma_azimuth^2) + ...
                    (grad_phi*grad_phi')/(sigma_elevation^2);
    end
    
    % 添加微小正则化以避免奇异矩阵
    FIM = FIM + 1e-10*eye(3);
    
    CRLB = inv(FIM);
end

function u_est = two_stage_estimator(BR_meas, AOA_theta, AOA_phi, Tx, Rx, sigma_r, sigma_azimuth, sigma_elevation)
    M = size(Tx,1);
    N = size(Rx,1);
    
    % 第一阶段：构建伪线性方程
    h1 = [];
    G1 = [];
    
    % BR测量部分
    d_ij = zeros(M,N);
    for i = 1:M
        for j = 1:N
            d_ij(i,j) = norm(Tx(i,:) - Rx(j,:));
            term = BR_meas(i,j)^2 + 2*BR_meas(i,j)*d_ij(i,j) + 2*(Rx(j,:)-Tx(i,:))*Rx(j,:)';
            h1 = [h1; term];
            
            G_row = [2*(Rx(j,:)-Tx(i,:)), zeros(1,j-1), 2*(BR_meas(i,j)+d_ij(i,j)), zeros(1,N-j)];
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
    
    % 构造A矩阵
    A  = [-G1,h1];
    
    % 考虑不同角度噪声的权重矩阵
    Q  = diag([repmat(1/sigma_r^2, M*N, 1); 
               repmat(1/sigma_azimuth^2, N, 1);
               repmat(1/sigma_elevation^2, N, 1)]);
    W1 = inv(Q);
    
    % 初始化估计值
    rd_hat = [reshape(BR_meas,[M*N,1]); AOA_theta; AOA_phi];
    
    % 迭代优化权重矩阵
    for k = 1:15
        % 构建Omega矩阵
        Omega_11 = zeros(N);
        for i = 1:N
            Omega_11(i,i) = W1(i,i)*Q(i,i)+cos(AOA_phi(i))^2*Q(M*N+N+i)*W1(M*N+N+i,M*N+N+i);
        end
        
        Omega_12 = zeros(N,1);
        for i = 1:N
            Omega_12(i) = 2*Q(i,i)*W1(i,i)*rd_hat(i);
        end
        
        Omega_21 = Omega_12';
        
        Omega_22 = 0;
        for i = 1:N
            Omega_22 = Omega_22 + Q(i,i)*W1(i,i)*4*(rd_hat(i))^2 + ([cos(AOA_theta(i)),sin(AOA_theta(i)),0]*Rx(i,:)')^2*Q(M*N+i)*W1(M*N+i,M*N+i);
        end
        
        OmegaTilde = [Omega_11,Omega_12;Omega_21,Omega_22];

        Omega_1 = zeros(3,3);
        Omega_1(1,1) = sum(cos(AOA_theta).^2)*Q(M*N+1)*W1(M*N+1,M*N+1);
        Omega_1(1,2) = sum(cos(AOA_theta).*sin(AOA_theta))*Q(M*N+1)*W1(M*N+1,M*N+1);
        Omega_1(2,1) = Omega_1(1,2);
        Omega_1(2,2) = sum(sin(AOA_theta).^2)*Q(M*N+1)*W1(M*N+1,M*N+1);

        Omega_2 = zeros(3,M*N+1);
        for i = 1:N
            Omega_2(1,M*N+1) = Omega_2(1,M*N+1)+[cos(AOA_theta(i)),sin(AOA_theta(i)),0]*Rx(i,:)'*cos(AOA_theta(i))*Q(M*N+i)*W1(M*N+i,M*N+i);
            Omega_2(2,M*N+1) = Omega_2(2,M*N+1)+[cos(AOA_theta(i)),sin(AOA_theta(i)),0]*Rx(i,:)'*sin(AOA_theta(i))*Q(M*N+i)*W1(M*N+i,M*N+i);
        end
        
        Omega_3 = Omega_2';

        Omega = [Omega_1,Omega_2;
                 Omega_3,OmegaTilde];
        
        % 添加正则化以提高数值稳定性
        At = A' * W1 * A + 1e-4*eye(size(A,2));
        
        X = inv(At)*Omega;
        
        % 找到最大特征值对应的特征向量
        [E,F] = eig(X);
        [~,idx] = max(diag(F));
        v2 = E(:,idx);
        v2 = v2/v2(end);
        
        psi1 = v2(1:end-1);
        u_hat = psi1(1:3);
        r_hat = psi1(4:3+N);  % 只取N个，对应接收机数量
        rd_hat = psi1;
        
        % 更新 W1
        % 1. 计算 B_r
        r_t = zeros(M,1);
        for i = 1:M
            r_t(i) = norm(u_hat - Tx(i,:)'); % 目标到发射机的距离
        end
        B_r = 2 * kron(eye(N), diag(r_t));
        
        % 2. 计算 B_theta (方位角)
        B_theta = zeros(N);
        for j = 1:N
            vj = [cos(AOA_theta(j)); sin(AOA_theta(j)); 0];
            B_theta(j,j) = vj' * (u_hat - Rx(j,:)');
        end
        
        % 3. 计算 B_phi (俯仰角)
        B_phi = zeros(N);
        for j = 1:N
            r_j = norm(u_hat - Rx(j,:)');
            B_phi(j,j) = cos(AOA_phi(j)) * r_j;
        end
        
        % 4. 构造 B1
        B1 = blkdiag(B_r, B_theta, B_phi);
        
        % 5. 计算 Q_m (考虑不同的角度噪声)
        Q_r = sigma_r^2 * eye(M*N);
        Q_theta = sigma_azimuth^2 * eye(N);
        Q_phi = sigma_elevation^2 * eye(N);
        Q_m = blkdiag(Q_r, Q_theta, Q_phi);
        
        % 6. 计算 W1
        inv_B1 = inv(B1);
        W1 = inv_B1' * inv(Q_m) * inv_B1;
        
        % 7. 添加小的正则化项以增强数值稳定性
        W1 = W1 + 1e-8 * eye(size(W1));
    end
    
    % 第二阶段：误差修正
    h2 = [];
    G2 = [];
    
    for j = 1:N
        term = r_hat(j)^2 - norm(u_hat - Rx(j,:)')^2;
        h2 = [h2; term];
        G2_row = [-2*(u_hat - Rx(j,:)')'];
        G2 = [G2; G2_row];
    end
    
    h2 = [zeros(3,1); h2];
    G2 = [-eye(3); G2];
    
    % 构建加权矩阵
    B_psi = 2 * diag(r_hat);
    B2 = blkdiag(eye(3), B_psi);
    inv_B2 = blkdiag(eye(3), diag(1./(2*r_hat)));
    cov_Psi = inv(G1' * W1 * G1);
    W2 = inv_B2' * inv(cov_Psi) * inv_B2;
    
    % 求解
    delta_u = inv(G2'*W2*G2 + 1e-6*eye(3)) * (G2'*W2*h2);
    
    u_est = u_hat - delta_u(1:3);
end

function u_est = classic_weighted_ls(BR_meas, AOA_theta, AOA_phi, Tx, Rx, sigma_r, sigma_azimuth, sigma_elevation)
    % 经典加权最小二乘方法
    M = size(Tx,1);
    N = size(Rx,1);
    
    % 构建方程组 Ax = b
    A = [];
    b = [];
    W_diag = []; % 权重对角元素
    
    % BR测量部分
    for i = 1:M
        for j = 1:N
            d_ij = norm(Tx(i,:) - Rx(j,:));
            r_i = norm(Tx(i,:)); % 原点到发射机距离
            r_j = norm(Rx(j,:)); % 原点到接收机距离
            
            % 线性化约束：(x-xt)'(x-xt)/|x-xt| + (x-xr)'(x-xr)/|x-xr| = BR + d_ij
            % 第一次迭代用原点作为线性化点
            dx_t = -Tx(i,:)';
            dx_r = -Rx(j,:)';
            
            A_row = (dx_t/r_i + dx_r/r_j)';
            b_row = BR_meas(i,j) + d_ij - r_i - r_j + (dx_t'*dx_t)/r_i + (dx_r'*dx_r)/r_j;
            
            A = [A; A_row];
            b = [b; b_row];
            W_diag = [W_diag; 1/sigma_r^2];
        end
    end
    
    % 方位角约束
    for j = 1:N
        % tan(theta) = (y-yr)/(x-xr) => y*cos(theta) - x*sin(theta) = yr*cos(theta) - xr*sin(theta)
        A_row = [sin(AOA_theta(j)), -cos(AOA_theta(j)), 0];
        b_row = Rx(j,1)*sin(AOA_theta(j)) - Rx(j,2)*cos(AOA_theta(j));
        
        A = [A; A_row];
        b = [b; b_row];
        W_diag = [W_diag; 1/sigma_azimuth^2];
    end
    
    % 俯仰角约束
    for j = 1:N
        % tan(phi) = z/sqrt((x-xr)^2+(y-yr)^2)
        dx = -Rx(j,1);
        dy = -Rx(j,2);
        dz = -Rx(j,3);
        r_xy = sqrt(dx^2 + dy^2);
        
        % 线性化: z*cos(phi) - (x-xr)*sin(phi)*cos(theta) - (y-yr)*sin(phi)*sin(theta) = 0
        A_row = [sin(AOA_phi(j))*cos(AOA_theta(j)), 
                 sin(AOA_phi(j))*sin(AOA_theta(j)), 
                -cos(AOA_phi(j))];
        b_row = Rx(j,1)*sin(AOA_phi(j))*cos(AOA_theta(j)) + ...
                Rx(j,2)*sin(AOA_phi(j))*sin(AOA_theta(j)) - ...
                Rx(j,3)*cos(AOA_phi(j));
        
        A = [A; A_row];
        b = [b; b_row];
        W_diag = [W_diag; 1/sigma_elevation^2];
    end
    
    % 构建权重矩阵
    W = diag(W_diag);
    
    % 加权最小二乘求解
    u_init = (A'*W*A)\(A'*W*b);
    
    % 迭代优化
    u_est = u_init;
    
    for iter = 1:10
        A = [];
        b = [];
        W_diag = [];
        
        % 更新BR约束
        for i = 1:M
            for j = 1:N
                d_ij = norm(Tx(i,:) - Rx(j,:));
                
                % 计算到当前估计位置的向量
                r_i_vec = u_est - Tx(i,:)';
                r_j_vec = u_est - Rx(j,:)';
                r_i = norm(r_i_vec);
                r_j = norm(r_j_vec);
                
                % 更新线性化约束
                A_row = (r_i_vec/r_i + r_j_vec/r_j)';
                b_row = BR_meas(i,j) + d_ij - r_i - r_j + r_i_vec'*r_i_vec/r_i + r_j_vec'*r_j_vec/r_j;
                
                A = [A; A_row];
                b = [b; b_row];
                
                % BR残差权重（考虑几何因素）
                z_factor = abs(r_i_vec(3)/r_i + r_j_vec(3)/r_j);
                geom_weight = 1 + z_factor;  % 增强Z敏感度的测量
                W_diag = [W_diag; geom_weight/sigma_r^2];
            end
        end
        
        % 更新方位角约束
        for j = 1:N
            dx = u_est(1) - Rx(j,1);
            dy = u_est(2) - Rx(j,2);
            theta_est = atan2(dy, dx);
            
            % 线性化约束
            A_row = [-dy/(dx^2+dy^2), dx/(dx^2+dy^2), 0];
            angle_diff = mod(AOA_theta(j) - theta_est + pi, 2*pi) - pi;
            b_row = angle_diff - A_row*u_est;
            
            A = [A; A_row];
            b = [b; b_row];
            W_diag = [W_diag; 1/sigma_azimuth^2];
        end
        
        % 更新俯仰角约束
        for j = 1:N
            dx = u_est(1) - Rx(j,1);
            dy = u_est(2) - Rx(j,2);
            dz = u_est(3) - Rx(j,3);
            r = norm([dx; dy; dz]);
            r_xy = sqrt(dx^2 + dy^2);
            phi_est = asin(dz/r);
            
            % 俯仰角导数
            dphi_dx = -dx*dz/(r^3*r_xy);
            dphi_dy = -dy*dz/(r^3*r_xy);
            dphi_dz = r_xy/(r^3);
            
            A_row = [dphi_dx, dphi_dy, dphi_dz];
            angle_diff = mod(AOA_phi(j) - phi_est + pi, 2*pi) - pi;
            b_row = angle_diff - A_row*u_est;
            
            A = [A; A_row];
            b = [b; b_row];
            W_diag = [W_diag; 1/sigma_elevation^2];
        end
        
        % 更新权重矩阵
        W = diag(W_diag);
        
        % 计算更新量
        delta_u = (A'*W*A)\(A'*W*b);
        
        % 更新估计值（添加步长控制）
        step_size = 0.8;
        if iter > 5
            step_size = 0.5; % 后期减小步长提高稳定性
        end
        u_est = u_est + step_size * delta_u;
    end
end

function res = br_aoa_residuals(u, Tx, Rx, BR_meas, AOA_theta, AOA_phi, sigma_r, sigma_azimuth, sigma_elevation)
    M = size(Tx,1);
    N = size(Rx,1);
    
    % BR残差
    res_br = zeros(M*N,1);
    idx = 1;
    for i = 1:M
        for j = 1:N
            d_ij = norm(Tx(i,:)-Rx(j,:));
            r_i = norm(u - Tx(i,:)');
            r_j = norm(u - Rx(j,:)');
            BR_true = r_i + r_j - d_ij;
            res_br(idx) = (BR_meas(i,j)-BR_true)/sigma_r;
            idx = idx + 1;
        end
    end
    
    % 方位角残差
    res_azimuth = zeros(N,1);
    for j = 1:N
        dx = u(1) - Rx(j,1);
        dy = u(2) - Rx(j,2);
        theta_true = atan2(dy, dx);
        angle_diff = mod(AOA_theta(j) - theta_true + pi, 2*pi) - pi;
        res_azimuth(j) = angle_diff/sigma_azimuth;
    end
    
    % 俯仰角残差
    res_elevation = zeros(N,1);
    for j = 1:N
        r_j = u - Rx(j,:)';
        phi_true = asin(r_j(3)/norm(r_j));
        angle_diff = mod(AOA_phi(j) - phi_true + pi, 2*pi) - pi;
        res_elevation(j) = angle_diff/sigma_elevation;
    end
    
    % 合并所有残差
    res = [res_br; res_azimuth; res_elevation];
end