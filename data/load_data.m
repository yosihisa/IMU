%% �f�[�^��ǂݍ���
% filename �t�@�C����
% plot �f�[�^���v���b�g���� 1(�^)�ŕ\�� 0(�U)�Ŕ�\��
% �������x(�C���̌��_)
function data = load_data(filename,p,h0)
    % �f�[�^�̃C���|�[�g
    raw = csvread(filename,1,0);
    data.t      = raw(:,1)';
    
    data.press  = raw(:,2)';
    data.temp   = raw(:,3)';
    data.mode   = raw(:,4)';
    
    % �@�̌n���W �����x
    data.ax0     = raw(:,5)';
    data.ay0     = raw(:,6)';
    data.az0     = raw(:,7)';
    
    % �@�̌n���W�@�p���x
    data.avx0     = raw(:,8)';
    data.avy0     = raw(:,9)';
    data.avz0     = raw(:,10)';
    
    % �@�̂̌��� �l����
    % data.quat = quaternion(raw(:,11),raw(:,12),raw(:,13),raw(:,14));
    data.q0     = raw(:,11)';
    data.q1     = raw(:,12)';
    data.q2     = raw(:,13)';
    data.q3     = raw(:,14)';
    
    % �n����W�n �����x
    data.ax     = raw(:,15)';
    data.ay     = raw(:,16)';
    data.az     = raw(:,17)';
    
    % �n����W�n ���x
    data.vx     = raw(:,18)';
    data.vy     = raw(:,19)';
    data.vz     = raw(:,20)';
    
    % �n����W�n �ʒu
    data.x      = raw(:,21)';
    data.y      = raw(:,22)';
    data.z      = raw(:,23)';
    
    % ���x(�C������ϊ�)
    data.press_h = (((1013.25./data.press).^(1./5.257)-1).*(data.temp+273.15)./0.0065)-h0;
    
    clearvars raw ;
    
    if p  == true
        figure
        ax1 = subplot(3,2,1);
        plot(ax1,data.t,data.ax0, data.t,data.ay0, data.t,data.az0)
        xlabel(ax1,'����[s]')
        ylabel(ax1,'�����x[m/s^2]')
        title(ax1,'�@�̍��W�n�̉����x') 
        
        ax2 = subplot(3,2,3);
        plot(ax2,data.t,data.q0, data.t,data.q1, data.t,data.q2, data.t,data.q3)
        xlabel(ax2,'����[s]')
        ylabel(ax2,'�l����')
        title(ax2,'�@�̂̌���') 
        
        ax3 = subplot(3,2,5);
        plot(ax3,data.t,data.y, data.t,data.press_h)
        xlabel(ax3,'����[s]')
        ylabel(ax3,'���x[m]')
        title(ax3,'���x') 
        ylim([-10 inf])
        
        
        ax4 = subplot(3,2,2);
        plot(ax4,data.t,data.ax, data.t,data.ay, data.t,data.az)
        xlabel(ax4,'����[s]')
        ylabel(ax4,'�����x[m/s^2]')
        title(ax4,'�����x') 
        
        ax5 = subplot(3,2,4);
        plot(ax5,data.t,data.vx, data.t,data.vy, data.t,data.vz)
        xlabel(ax5,'����[s]')
        ylabel(ax5,'�����x[m/s]')
        title(ax5,'���x') 
        
        ax6 = subplot(3,2,6);
        plot(ax6,data.t,data.x, data.t,data.y, data.t,data.z)
        xlabel(ax6,'����[s]')
        ylabel(ax6,'�ʒu[m]')
        title(ax6,'�ʒu') 
    end
    
end
