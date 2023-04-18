%************ In the name of GOD*******************%
%***************** MHA 991122 *********************%
%**************** Parallel Navigation ********************%
% This guidance law is similar to pure pursuit,
% except that the missile heading leads the LOS by a fixed angle
% When the fixed lead angle is zero, deviated pursuit becomes pure pursuit
clc, close all


V_T = 300;
alphaT = 0;% LOS Angle
alphaT_rad = (alphaT/180)*pi;

R0 = 3000;
theta0 = -30;
theta0_rad = (theta0/180)*pi;

xt0 = R0*cos(theta0_rad);
yt0 = R0*sin(theta0_rad);
V_T_hor = V_T*cos(alphaT_rad);
V_T_ver = V_T*sin(alphaT_rad);

Mu = 2;
V_M = Mu*V_T;

xm0 = 0;
ym0 = 0;

xt = xt0;
yt = yt0;
xm = xm0;
ym = ym0;

delta = 150;% Lead Angle
delta_rad = (delta*(pi/180));
alphaM_rad = atan3((yt-ym),(xt-xm)) + (delta/180)*pi;
V_M_hor = V_M*cos(alphaM_rad);%Vm_Horizontal
V_M_ver = V_M*sin(alphaM_rad);%Vm_Vertical
delT = 1;
K = R0*(((1+cos(alphaT_rad - theta0_rad))^Mu)/...
    ((sin(alphaT_rad - theta0_rad))^(Mu-1)));
theta_rad = theta0_rad;
R = R0;

imgframe = 0;
figHandle = figure;
set(figHandle,'WindowStyle','docked');
hold on;
% f1 = subplot(2,3,1);
l1 = plot(0,0,'*');

flag = 0;
%% 
for t = 0:delT:50

    xt = xt + V_T_hor*delT;
    yt = yt + V_T_ver*delT;
    %%
    d_R = (V_T*cos(alphaT_rad - theta_rad) - ...
            V_M*cos( delta_rad));%alphaM_rad - theta_rad +
    R2 = R + d_R * delT; 
%     R2 = sqrt((yt-ym)^2 + (xt-xm)^2);
%     
%     if abs(R2) > abs(R)
%         flag = flag+1;
%     end
%     if flag == 2 
%         break;
%     end
%     
    R = R2;
    %%
    d_theta_rad = (V_T*sin(alphaT_rad - theta_rad) - V_M*sin(delta_rad))/R;
    theta_rad = theta_rad + d_theta_rad * delT; % atan3((yt-ym),(xt-xm));
%     alphaM_rad = theta_rad + delta_rad;
    
    V_M_hor = V_M*cos(alphaM_rad);
    V_M_ver = V_M*sin(alphaM_rad);    
    xm = xm + V_M_hor*delT;
    ym = ym + V_M_ver*delT;
    
%     am_hor = (Vm_hor2 - Vm_hor)/delT;
%     am_ver = (Vm_ver2 - Vm_ver)/delT;
%     Vm_hor = Vm_hor2;
%     Vm_ver = Vm_ver2;

    %% 1st state 
    % V_M, V_T, gamma, LOS are Constant  
    % nz = a / g ; nz_m= V_M * gamm
    a_lat_s1 =0;
    %% 2nd state  V-T monovr
    % gamma, V-M are cte but d_V_T is cte
    V_T = V_T + delT ;
    d_V_T = 1 ;
    a_lat_s2 = -d_V_T * sin(theta_rad) - d_theta_rad*V_T*cos(theta_rad);
    d_V_T = V_T + delT ; 
    %% 3rd state V-M monovr
    % gamma, V-T are cte but d_V_M is cte
    V_M = V_M + delT ;
    d_V_M = 1 ;
    a_lat_s3 = -d_V_M * tan(delta);
    
    %% 4th state gamma monovr
    % V_M, V-T are cte but d_gamma is cte
    
    a_lat_s4 = d_theta_rad * V_T* cos(theta_rad);
    
%%
    
    
    %     am = norm([am_hor am_ver],2);

%     if t == 4
    pause(0.1);
%     end

    f1 = figure(1);
    set(f1,'WindowStyle','docked');
%     f1 = subplot(2,3,1);
    box on;
    hold on;
%     delete(l1);
    plot(xt,yt,'r*');    
    plot(xm,ym,'g*');
    l1 = plot([xm xt], [ym yt], 'b');
    xlabel('X-coordinate (meters)');
    ylabel('Y-coordinate (meters)');
        
    f2 = figure(2);
    set(f2,'WindowStyle','docked');
%     f2 = subplot(2,3,2);
    box on;
    hold on;
    plot(t,alphaM_rad,'*b');
    xlabel('Time');
    ylabel('LOS Distance (meters)');
   
    f3 = figure(3);
    set(f3,'WindowStyle','docked');
%     f3 = subplot(2,3,3);
    box on;
    hold on;
    plot(t,theta_rad*(180/pi),'+b');
    plot(t,alphaT_rad - theta_rad,'*g');%alphaM_rad,
    xlabel('Time');
    ylabel('theta (degrees)');
     
    f4 = figure(4);
    set(f4,'WindowStyle','docked');   
%     f4 = subplot(2,3,4);
    box on;
    hold on;
    plot(t,a_lat_s1,'*r');
    plot(t,a_lat_s2,'*g');
    plot(t,a_lat_s3,'*c');
    plot(t,a_lat_s4,'*b');
    xlabel('Time');
    ylabel('Lateral Acceleration (m/s^{2})');

    f5 = figure(5);
    set(f5,'WindowStyle','docked');  
%     f5 = subplot(2,3,5);
    box on;
    hold on;
    plot(t,V_T*sin(alphaT_rad - theta_rad) - V_M*sin((delta/180)*pi),'*r');%
    plot(t,V_T*cos(alphaT_rad - theta_rad) - V_M*cos((delta/180)*pi),'*b');% 
    xlabel('Time');
    ylabel(' V_{R}(blue) (m/s), V_{theta}(red) (m/s)');
    
    f6 = figure(6);
    set(f6,'WindowStyle','docked');  
%     f6 = subplot(2,3,6);
    box on;
    hold on;
   
    xlabel('V_{theta} (m/s)');
    ylabel('V_R (m/s)');
    
    
    f7 = figure(7);
    set(f7,'WindowStyle','docked');  
    box on;
    hold on;
    g = 9.81;
    plot(t,(a_lat_s1)/g,'*r');
    plot(t,a_lat_s2/g,'*g');
    plot(t,a_lat_s3/g,'*c');
    plot(t,a_lat_s4/g,'*b');
    legend('1st state V_M, V_T, gamma, LOS are Constant',' 2nd state  V-T monovr gamma, V-M are cte but d_V_T is cte','3rd state V-M monovr gamma, V-T are cte but d_V_M is cte','4th state gamma monovr V_M, V-T are cte but d_gamma is cte')
    xlabel('Time');
    ylabel('n_zm');
    
    
    imgframe = imgframe+1;
    images1(imgframe) = getframe(figHandle);
end

