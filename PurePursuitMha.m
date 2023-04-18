%************ In the name of GOD*******************%
%***************** MHA 991122 *********************%
%**************** Pure Pursuit ********************%
clc, close all

Vt = 300;
lambdaT = 0;% LOS Angle
lambdaT_rad = (lambdaT/180)*pi;

R0 = 3000;
theta0 = 90;
theta0_rad = (theta0/180)*pi;

xt0 = R0*cos(theta0_rad);
yt0 = R0*sin(theta0_rad);
Vt_hor = Vt*cos(lambdaT_rad);
Vt_ver = Vt*sin(lambdaT_rad);

Mu = 2;
Vm = Mu*Vt;

xm0 = 0;
ym0 = 0;

xt = xt0;
yt = yt0;
xm = xm0;
ym = ym0;

delta = 0;% Lead Angle

% delta = 0;% Lead Angle
delta_rad = (delta*(pi/180));
alphaM_rad = atan3((yt-ym),(xt-xm)) + (delta/180)*pi;
Vm_hor = Vm*cos(alphaM_rad);%Vm_Horizontal
Vm_ver = Vm*sin(alphaM_rad);%Vm_Vertical


delT = 0.1;
A = R0*(((1+cos(lambdaT_rad - theta0_rad))^Mu)/...
    ((sin(lambdaT_rad - theta0_rad))^(Mu-1)));
theta_rad = theta0_rad;
R = R0;

imgframe = 0;
figHandle = figure;
set(figHandle,'WindowStyle','docked');
hold on;
% f1 = subplot(2,3,1);
l1 = plot(0,0,'*');

flag = 0;

for t = 0:delT:50

    xt = xt + Vt_hor*delT;
    yt = yt + Vt_ver*delT;
    %%
    del_R = (Vt*cos(lambdaT_rad - theta_rad) - ...
            Vm*cos( delta_rad));%alphaM_rad - theta_rad +
    R2 = R + del_R * delT; 
%     R2 = sqrt((yt-ym)^2 + (xt-xm)^2);
    
    if abs(R2) > abs(R)
   
        break;
    end
    
    R = R2;
    %%
    
    del_theta_rad = (-Vt*sin(theta_rad) + ...
                    Vm*sin(delta_rad))/R;%alphaM_rad - theta_rad + 
    theta_rad = theta_rad + del_theta_rad * delT; % atan3((yt-ym),(xt-xm));
    alphaM_rad = theta_rad + delta_rad;
    
    Vm_hor = Vm*cos(alphaM_rad);
    Vm_ver = Vm*sin(alphaM_rad);    
    xm = xm + Vm_hor*delT;
    ym = ym + Vm_ver*delT;
    
%     am_hor = (Vm_hor2 - Vm_hor)/delT;
%     am_ver = (Vm_ver2 - Vm_ver)/delT;
%     Vm_hor = Vm_hor2;
%     Vm_ver = Vm_ver2;
%     
    am = ((Vm*Vt)*((sin(lambdaT_rad - theta_rad))^2)...
        /(A*((tan((lambdaT_rad - theta_rad)/2))^Mu))); 
%     am = norm([am_hor am_ver],2);

%     if t == 4
%     pause(0.1);
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
    plot(t,R,'*b');
    xlabel('Time');
    ylabel('LOS Distance (meters)');
   
    f3 = figure(3);
    set(f3,'WindowStyle','docked');
%     f3 = subplot(2,3,3);
    box on;
    hold on;
    plot(t,theta_rad*(180/pi),'+b');
    plot(t,delta_rad,'*g');%alphaM_rad,
    xlabel('Time');
    ylabel('theta (degrees)');
    legend('theta','delta') 
    
    f4 = figure(4);
    set(f4,'WindowStyle','docked');   
%     f4 = subplot(2,3,4);
    box on;
    hold on;
    plot(t,am,'*r');
    xlabel('Time');
    ylabel('Lateral Acceleration (m/s^{2})');

    f5 = figure(5);
    set(f5,'WindowStyle','docked');  
%     f5 = subplot(2,3,5);
    box on;
    hold on;
    plot(t,Vt*sin(lambdaT_rad - theta_rad) - Vm*sin((delta/180)*pi),'*r');%
    plot(t,Vt*cos(lambdaT_rad - theta_rad) - Vm*cos((delta/180)*pi),'*b');% 
    xlabel('Time');
    ylabel(' V_{R}(blue) (m/s), V_{theta}(red) (m/s)');
    
  
    f6 = figure(6);
    set(f6,'WindowStyle','docked');  
    gamma_dot_M1 =  -Vt*(((1+cos(lambdaT_rad + theta0_rad))^(1.3))/(A*(sin(lambdaT_rad + theta0_rad))^((1.3)-2)));
    n_z_m1 = gamma_dot_M1*Vm/9.81; 
    gamma_dot_M2 =  -Vt*(((1+cos(lambdaT_rad + theta0_rad))^(2))/((sin(lambdaT_rad + theta0_rad))^((2)-2)));
    n_z_m2 = gamma_dot_M2*Vm/9.81;
    gamma_dot_M5 =  -Vt*(((1+cos(lambdaT_rad + theta0_rad))^(10))/((sin(lambdaT_rad + theta0_rad))^((10)-2)));
    n_z_m_inf = gamma_dot_M1*Vm/9.81;
    
    box on;
    hold on;
    plot(t,n_z_m1,'*r');%
    plot(t,n_z_m2,'*b');%
    plot(t,n_z_m_inf,'*g');%
    xlabel('time');
    ylabel('N_z_m_a_x');
    legend('n_z_m1','n_z_m2','n_z_m_inf')
    imgframe = imgframe+1;
    images1(imgframe) = getframe(figHandle);
end

