%% Carga de parámetros
clc,clear;
run('.\param.m');
%% Parametros de simulación
% Simulación de cinemática inversa
q0 = homeConfiguration(robot);
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1];
endEffector = 'tool';
qInitial = q0; % Use home configuration as the initial guess
%% Planificador de trayectorias interpolador articular
%Consigna clasica
% waypoints = [0.011265502281722 0.032831860285015   -1.078889510983337;
%     0.011265502281722 0.032831860285015   -1.078889510983337;
%     0.4 0.4 -0.8;
%     0.4 -0.4  -0.8;
%     -0.25 -0.25  -0.6;
%     -0.4 0.3  -0.6];
% t = 0:0.0001:2.1;
% timepoints = [0 0.1 0.6 1.1 1.6 2.1];
%Consigna real
waypoints = [0.011265502281722 0.032831860285015   -1.078889510983337;
    0.011265502281722 0.032831860285015   -1.078889510983337;
    0 0.55 -0.4;
    0.55 0  -0.4;
    0.55 0 -0.8;
    0.3 0.3 -0.85;
    -0.3 -0.3 -0.85;
    0.011265502281722 0.032831860285015   -1.078889510983337];
t = 0:0.0001:3.1;
timepoints = [0 0.1 0.6 1.1 1.6 2.1 2.6 3.1];
%Consigna extrema
% waypoints = [0.011265502281722 0.032831860285015   -1.078889510983337;
%     0.011265502281722 0.032831860285015   -1.078889510983337;
%     0 0.55 -0.2;
%     0.55 0 -0.2;
%    0 -0.55 -0.2;
%    0.011265502281722 0.032831860285015   -1.078889510983337];
% t = 0:0.0001:2.1;
% timepoints = [0 0.1 0.6 1.1 1.6 2.1];
%%
qSol=qInitial;
qsols = zeros(4, 4);
qsols(1,:)=qSol';
for i = 2:length(waypoints(:,1))
    qSol = ik(endEffector,trvec2tform(waypoints(i,:)),weights,qInitial);
    % Store the configuration
    qsols(i,:) = qSol';
    % Start from prior solution
    qInitial = qSol;
end
[q22,q2p,q2pp] =quinticpolytraj(qsols',timepoints,t);
q2 = timeseries(q22',t');
punt_art = zeros(length(q22(1,:)),3);

for i=1:length(q22(1,:))
    transform = getTransform(robot,q22(:,i),'tool','base_link');
    punt_art(i,1) = transform(1,4);
    punt_art(i,2) = transform(2,4);
    punt_art(i,3) = transform(3,4);
end
%% Simulación
out = sim('Trabajo_final_PID',t(end));
%% Animación
tout=out.tout;
qr=zeros(3,length(out.poseData(1,4,:)));
for i=1:length(out.poseData(1,4,:))
    qr(:,i)=[out.poseData(1,4,i);out.poseData(2,4,i);out.poseData(3,4,i)];
end

salto = round(length(out.jointData)/60);
figure(1)
show(robot,out.jointData(1,:)','Frames','off');
axis([-1.5 1.5 -1.5 1.5 -1.3 0.3])
hold on
pause 
plot3(punt_art(:,1),punt_art(:,2),punt_art(:,3))
plot3(qr(1,:),qr(2,:),qr(3,:))
fps = 30;
r = rateControl(fps);
for i = 1:salto:length(out.jointData)
    t_act = tout(i,1);
    titulo = 'Tiempo actual: ';
    titulo = strcat(titulo,num2str(t_act));
    show(robot,out.jointData(i,:)','PreservePlot',false,'Frames','off');
    axis([-1.5 1.5 -1.5 1.5 -1.3 0.3])
    drawnow
    title(titulo)
    waitfor(r);
end
hold off
pause
close all
%% Gráficas
figure(1)
subplot(4,1,1);
plot(out.torque1_real.Time,out.torque1_real.Data,out.torque_carga1.Time,...
    out.torque_carga1.Data);
title('Torques de articulación 1')
xlabel('t(s)')
ylabel('T (N m)')
legend('Torque entregado','Torque de carga')
grid on
xlim([0,2.1])

subplot(4,1,2);
plot(out.torque2_real.Time,out.torque2_real.Data,out.torque_carga2.Time,...
    out.torque_carga2.Data);
title('Torques de articulación 2')
xlabel('t(s)')
ylabel('T (N m)')
legend('Torque entregado','Torque de carga')
grid on
xlim([0,2.1])

subplot(4,1,3);
plot(out.torque3_real.Time,out.torque3_real.Data,out.torque_carga3.Time,...
    out.torque_carga3.Data);
title('Torques de articulación 3')
xlabel('t(s)')
ylabel('T (N m)')
legend('Torque entregado','Torque de carga')
grid on
xlim([0,2.1])

subplot(4,1,4);
plot(out.torque4_real.Time,out.torque4_real.Data,out.torque_carga4.Time,...
    out.torque_carga4.Data);
title('Torques de articulación 4')
xlabel('t(s)')
ylabel('T (N m)')
legend('Torque entregado','Torque de carga')
grid on
xlim([0,2.1])

figure(2)
subplot(4,1,1);
plot(out.consignas_vel1.Time,out.consignas_vel1.Data,out.tout,...
    out.velocidades(:,1));
title('Velocidades de articulación 1')
xlabel('t(s)')
ylabel('\omega (rad/s)')
legend('Consignas de velocidad','Velocidad real')
grid on
xlim([0,2.1])

subplot(4,1,2);
plot(out.consignas_vel2.Time,out.consignas_vel2.Data,out.tout,...
    out.velocidades(:,2));
title('Velocidades de articulación 2')
xlabel('t(s)')
ylabel('\omega (rad/s)')
legend('Consignas de velocidad','Velocidad real')
grid on
xlim([0,2.1])

subplot(4,1,3);
plot(out.consignas_vel3.Time,out.consignas_vel3.Data,out.tout,...
    out.velocidades(:,3));
title('Velocidades de articulación 3')
xlabel('t(s)')
ylabel('\omega (rad/s)')
legend('Consignas de velocidad','Velocidad real')
grid on
xlim([0,2.1])

subplot(4,1,4);
plot(out.consignas_vel4.Time,out.consignas_vel4.Data,out.tout,...
    out.velocidades(:,4));
title('Velocidades de articulación 4')
xlabel('t(s)')
ylabel('\omega (rad/s)')
legend('Consignas de velocidad','Velocidad real')
grid on
xlim([0,2.1])
