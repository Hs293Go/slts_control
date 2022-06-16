close all;
%% load data
load('SingleDroneTest')
%% payload position 
figure
subplot(2,1,1);
plot(FlightData.time,FlightData.x_p_inertial(:,1),'-.r','LineWidth',1.5);
hold on
plot(FlightData.time,FlightData.x_p_inertial(:,2),'-g','LineWidth',1);
plot(FlightData.time,FlightData.x_p_inertial(:,3),'-k','LineWidth',0.8);
legend({'$x_{p,x}$','$x_{p,y}$','$x_{p,z}$'},'Interpreter','latex')
ylabel('Position (m)','Interpreter','latex')
hold on
grid on;
xlabel('Time (s)');
title('Payload Position');

subplot(2,1,2);
plot(FlightData.time,FlightData.v_p_inertial(:,1),'-.r','LineWidth',1.5);
hold on
plot(FlightData.time,FlightData.v_p_inertial(:,2),'-g','LineWidth',1);
plot(FlightData.time,FlightData.v_p_inertial(:,3),'-k','LineWidth',0.8);
legend({'$v_{p,x}$','$v_{p,y}$','$v_{p,z}$'},'Interpreter','latex')
ylabel('Velocity (m/s)','Interpreter','latex')
hold on
grid on;
xlabel('Time (s)');
title('Payload Velocity');

%% path following error
ev =figure;
subplot(2,1,1);
plot(FlightData.time,FlightData.e_v ,'LineWidth',2);
ylabel('Velocity Error $||e_{v,i}||$(m/s)','Interpreter','latex')
hold on
grid on;
xlabel('Time (s)');
title('Path Velocity Error');

subplot(2,1,2)
plot(FlightData.time,FlightData.e_p,'b','LineWidth',2);
hold on
grid on;
xlabel('Time (s)');
ylabel('Position Error $||e_{x,i}||$ (m)',...
        'Interpreter','latex');
title('Path Position Error');

%% plot the disturbance estimation results
figure
subplot(2,1,1)
plot(FlightData.time,FlightData.delta_p_bot_error ,'-b','LineWidth',2);
hold on
plot(FlightData.time,FlightData.delta_q_bot(:,1),'-.r','LineWidth',1.5);
plot(FlightData.time,FlightData.delta_q_bot(:,2),'-g','LineWidth',1);
plot(FlightData.time,FlightData.delta_q_bot(:,3),'-k','LineWidth',0.8);
ylabel('Magnitude $(N)$','Interpreter','latex')
xlabel('Time (s)');title('Estimated Disturbance')
legend({'$||\tilde{\Delta}_{\bot}||$','$\hat{\Delta}_{\bot,x}$','$\hat{\Delta}_{\bot,y}$','$\hat{\Delta}_{\bot,z}$'},'Interpreter','latex')
grid on
title('The Estimated Disturbances on The Payload')

subplot(2,1,2)

plot(FlightData.time,FlightData.delta_T_error ,'-b','LineWidth',2);
hold on
plot(FlightData.time,FlightData.delta_T(:,1),'-.r','LineWidth',1.5);
plot(FlightData.time,FlightData.delta_T(:,2),'-g','LineWidth',1);
plot(FlightData.time,FlightData.delta_T(:,3),'-k','LineWidth',0.8);

ylabel('Magnitude $(N)$','Interpreter','latex')
xlabel('Time (s)');title('Estimated Disturbance')
legend({'$||\tilde{\Delta}_{T}||$','$\hat{\Delta}_{T,x}$','$\hat{\Delta}_{T,y}$','$\hat{\Delta}_{T,z}$'},'Interpreter','latex')
grid on
title('The Estimated Disturbances on The Quadrotor')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
plot(FlightData.time ,FlightData.r_tilde(:,1),'-b','LineWidth',1);
hold on
plot(FlightData.time ,FlightData.r_tilde(:,2),'-.r','LineWidth',2);
ylabel('Position (m)','Interpreter','latex')

title('Quadrotor Relative Position Error');
legend({'$\tilde{r}_{x}$','$\tilde{r}_{y}$'},'Interpreter','latex')
xlabel('Time (s)')
grid on
