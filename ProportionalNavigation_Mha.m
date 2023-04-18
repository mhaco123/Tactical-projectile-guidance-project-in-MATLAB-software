%************ In the name of GOD*******************%
%***************** MHA 991122 *********************%
%**************** Proportional Naviagtion *************%

%% d_d_gammma = C d_d_lambda
% d_gamma = (1- k')d_lambda

clc, close all



V_T = 300;
alphaT = 0;% LOS Angle
alphaT_rad = (alphaT/180)*pi;

R0 = 3000;
theta0 = 90;
theta0_rad = (theta0/180)*pi;

xt0 = R0*cos(theta0_rad);
yt0 = R0*sin(theta0_rad);
V_T_hor = V_T*cos(alphaT_rad);
V_T_ver = V_T*sin(alphaT_rad);

Mu = 1.3;
V_M = Mu*V_T;

xm0 = 0;
ym0 = 0;

xt = xt0;
yt = yt0;
xm = xm0;
ym = ym0;
%%

lambda = atan3((yt-ym),(xt-xm));

%%
delta = 30;% Lead Angle

% delta = 0;% Lead Angle
delta_rad = (delta*(pi/180));

alphaM_rad = atan3((yt-ym),(xt-xm)) + (delta/180)*pi;
V_M_hor = V_M*cos(alphaM_rad);%Vm_Horizontal
V__ver = V_M*sin(alphaM_rad);%Vm_Vertical

delT = 0.1;
K = R0*(((1+cos(alphaT_rad - theta0_rad))^Mu)/...
    ((sin(alphaT_rad - theta0_rad))^(Mu-1)));
theta_rad = theta0_rad;
R = R0;

imgframe = 0;
figHandle = figure;
set(figHandle,'WindowStyle','docked');
hold on;
l1 = plot(0,0,'*');

% flag = 0;
% 
% for t = 0:delT:50
% 
%     xt = xt + V_T_hor*delT;
%     yt = yt + V_T_ver*delT;
%     %%
%     del_R = (V_T*cos(alphaT_rad - theta_rad) - V_M*cos( delta_rad));%alphaM_rad - theta_rad +
%     R2 = R + del_R * delT; 
% %     R2 = sqrt((yt-ym)^2 + (xt-xm)^2);
%     
%     if abs(R2) < abs(R)
%         flag = flag+0.1;
%     end
%     if flag == 2 
%         break;
%     end
%     
%     R = R2;
%     %%
%     del_theta_rad = 0;%alphaM_rad - theta_rad + 
%     theta_rad = theta_rad + del_theta_rad * delT; % atan3((yt-ym),(xt-xm));
%     alphaM_rad = theta_rad + delta_rad;
% %     lambda = 
%     V_M_hor = V_M*cos(alphaM_rad);
%     Vm_ver = V_M*sin(alphaM_rad);    
%     xm = xm + V_M_hor*delT;
%     ym = ym + Vm_ver*delT;
%     
% %     am_hor = (Vm_hor2 - Vm_hor)/delT;
% %     am_ver = (Vm_ver2 - Vm_ver)/delT;
% %     Vm_hor = Vm_hor2;
% %     Vm_ver = Vm_ver2;
%     
%     am = ((V_M*V_T)*((sin(alphaT_rad - theta_rad))^2)...
%         /(K*((tan((alphaT_rad - theta_rad)/2))^Mu))); 
% %     am = norm([am_hor am_ver],2);
% 
% %     if t == 4
%     pause(0.1);
% %     end
% %%
% %Calculate commanded acceleration
%     
% 
% 
% %%
%     f1 = figure(1);
%     set(f1,'WindowStyle','docked');
% %     f1 = subplot(2,3,1);
%     box on;
%     hold on;
% %     delete(l1);
%     plot(xt,yt,'r*');    
%     plot(xm,ym,'g*');
%     l1 = plot([xm xt], [ym yt], 'b');
%     xlabel('X-coordinate (meters)');
%     ylabel('Y-coordinate (meters)');
%         
%     f2 = figure(2);
%     set(f2,'WindowStyle','docked');
% %     f2 = subplot(2,3,2);
%     box on;
%     hold on;
%     plot(t,R,'*b');
%     xlabel('Time');
%     ylabel('LOS Distance (meters)');
%    
%     f3 = figure(3);
%     set(f3,'WindowStyle','docked');
% %     f3 = subplot(2,3,3);
%     box on;
%     hold on;
%     plot(t,theta_rad*(180/pi),'+b');
%     plot(t,alphaT_rad - theta_rad,'*g');%alphaM_rad,
%     xlabel('Time');
%     ylabel('theta (degrees)');
%      
%     f4 = figure(4);
%     set(f4,'WindowStyle','docked');   
% %     f4 = subplot(2,3,4);
%     box on;
%     hold on;
%     plot(t,am,'*r');
%     xlabel('Time');
%     ylabel('Lateral Acceleration (m/s^{2})');
% 
%     f5 = figure(5);
%     set(f5,'WindowStyle','docked');  
% %     f5 = subplot(2,3,5);
%     box on;
%     hold on;
%     plot(t,V_T*sin(alphaT_rad - theta_rad) - V_M*sin((delta/180)*pi),'*r');%
%     plot(t,V_T*cos(alphaT_rad - theta_rad) - V_M*cos((delta/180)*pi),'*b');% 
%     xlabel('Time');
%     ylabel(' V_{R}(blue) (m/s), V_{theta}(red) (m/s)');
%     
%     f6 = figure(6);
%     set(f6,'WindowStyle','docked');  
% %     f6 = subplot(2,3,6);
%     box on;
%     hold on;
%     plot(V_T*sin(alphaT_rad - theta_rad) - V_M*sin((delta/180)*pi),V_T*cos(alphaT_rad - theta_rad) - V_M*cos((delta/180)*pi),'k*'); % 
%     xlabel('V_{theta} (m/s)');
%     ylabel('V_R (m/s)');
%     
%     imgframe = imgframe+1;
%     images1(imgframe) = getframe(figHandle);
% end

    lambda = atan3((yt-ym),(xt-xm));    

for lambda0 = 0:0.05:pi
    k_prime_1 = 1 - (cos(lambda0)/(sqrt((1.1)^2 - (sin(lambda0)^2))));
    
    
    k_prime_2 = 1 - (cos(lambda0)/(sqrt((2)^2 - (sin(lambda0)^2))));
  
    
    k_prime_3 = 1 - (cos(lambda0)/(sqrt((3)^2 - (sin(lambda0)^2))));
    
    k_prime_10 = 1 - (cos(lambda0)/(sqrt((10)^2 - (sin(lambda0)^2))));

    f9 = figure(9);
    set(f9,'WindowStyle','docked');
    box on;
    hold on;
    plot(lambda0,k_prime_1,'-*g');
    plot(lambda0,k_prime_2,'--*c');
    plot(lambda0,k_prime_3,'--*b');
    plot(lambda0,k_prime_10,'-*k');
    xlabel(' lambda0(rad)');
    ylabel('K-prime');
    legend('K-1','K-2','K-3','K-infinitive')
    
end
