%% データを読み込む
% filename ファイル名
% plot データをプロットする 1(真)で表示 0(偽)で非表示
% 初期高度(気圧の原点)
function data = load_data(filename,p,h0)
    % データのインポート
    raw = csvread(filename,1,0);
    data.t      = raw(:,1)';
    
    data.press  = raw(:,2)';
    data.temp   = raw(:,3)';
    data.mode   = raw(:,4)';
    
    % 機体系座標 加速度
    data.ax0     = raw(:,5)';
    data.ay0     = raw(:,6)';
    data.az0     = raw(:,7)';
    
    % 機体系座標　角速度
    data.avx0     = raw(:,8)';
    data.avy0     = raw(:,9)';
    data.avz0     = raw(:,10)';
    
    % 機体の向き 四元数
    % data.quat = quaternion(raw(:,11),raw(:,12),raw(:,13),raw(:,14));
    data.q0     = raw(:,11)';
    data.q1     = raw(:,12)';
    data.q2     = raw(:,13)';
    data.q3     = raw(:,14)';
    
    % 地上座標系 加速度
    data.ax     = raw(:,15)';
    data.ay     = raw(:,16)';
    data.az     = raw(:,17)';
    
    % 地上座標系 速度
    data.vx     = raw(:,18)';
    data.vy     = raw(:,19)';
    data.vz     = raw(:,20)';
    
    % 地上座標系 位置
    data.x      = raw(:,21)';
    data.y      = raw(:,22)';
    data.z      = raw(:,23)';
    
    % 高度(気圧から変換)
    data.press_h = (((1013.25./data.press).^(1./5.257)-1).*(data.temp+273.15)./0.0065)-h0;
    
    clearvars raw ;
    
    if p  == true
        figure
        ax1 = subplot(3,2,1);
        plot(ax1,data.t,data.ax0, data.t,data.ay0, data.t,data.az0)
        xlabel(ax1,'時間[s]')
        ylabel(ax1,'加速度[m/s^2]')
        title(ax1,'機体座標系の加速度') 
        
        ax2 = subplot(3,2,3);
        plot(ax2,data.t,data.q0, data.t,data.q1, data.t,data.q2, data.t,data.q3)
        xlabel(ax2,'時間[s]')
        ylabel(ax2,'四元数')
        title(ax2,'機体の向き') 
        
        ax3 = subplot(3,2,5);
        plot(ax3,data.t,data.y, data.t,data.press_h)
        xlabel(ax3,'時間[s]')
        ylabel(ax3,'高度[m]')
        title(ax3,'高度') 
        ylim([-10 inf])
        
        
        ax4 = subplot(3,2,2);
        plot(ax4,data.t,data.ax, data.t,data.ay, data.t,data.az)
        xlabel(ax4,'時間[s]')
        ylabel(ax4,'加速度[m/s^2]')
        title(ax4,'加速度') 
        
        ax5 = subplot(3,2,4);
        plot(ax5,data.t,data.vx, data.t,data.vy, data.t,data.vz)
        xlabel(ax5,'時間[s]')
        ylabel(ax5,'加速度[m/s]')
        title(ax5,'速度') 
        
        ax6 = subplot(3,2,6);
        plot(ax6,data.t,data.x, data.t,data.y, data.t,data.z)
        xlabel(ax6,'時間[s]')
        ylabel(ax6,'位置[m]')
        title(ax6,'位置') 
    end
    
end
