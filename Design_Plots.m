
%DesignProblem01('Controller','diagnostics',true,'datafile','data.mat')
load('data.mat','processdata')
t= processdata.t;
w=processdata.w_01in1;

%%
processl1 = 1.5;
processl2 = 1.0;
processl3 = 0.2;
m = 1;
% - Principal moments of inertia
J1 = (m/12)*(processl2^2+processl3^2);
J2 = (m/12)*(processl3^2+processl1^2);
J3 = (m/12)*(processl1^2+processl2^2);

w3=0;
A= [0 (J2-J3)*w3/J1 0; (J3-J1)*w3/J2 0 0; 0 0 0];


B = [1/J1 0; 0 1/J2;0 0];
K = [1 0 0; 0 .2 0];

M = A - B*K;
[V,F] = eig(M)
w0=[.0001;.0001;.000001];
for i=1:length(t)
    wm(:,i)=expm(M*t(i))*w0;
end


%% 
subplot(3,1,1);
plot(t,w(1,:),'r-.',t,wm(1,:),'b-.','linewidth',2)
grid on
ylabel(' \omega 1 ')
xlabel('Time')
legend('Real \omega','Linearized \omega Prediction','location','southeast')
title('Angular Velocity v.s. Time')

subplot(3,1,2);
plot(t,w(2,:),'r-',t,wm(2,:),'b-.','linewidth',2)
grid on
ylabel(' \omega 2 ')
xlabel('Time')
legend('Real \omega','Linearized \omega Prediction','location','northeast')
subplot(3,1,3);
plot(t,w(3,:),'r-',t,wm(3,:),'b-.','linewidth',2)
grid on
ylabel(' \omega 3 ')
xlabel('Time')

legend('Real \omega','Linearized \omega Prediction','location','northeast')

