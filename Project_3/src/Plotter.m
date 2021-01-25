close all force;
% Data fetching from the simulation
t = out.tout;
t_total  = out.tout(end);
P_desired = out.P_desired;
P_actual = out.P_actual;
force = out.force;
q = out.q;
q_desired = out.q_desired;
mean_track_error = mean(P_desired - P_actual);
L1= out.L1;
L2= out.L2;


str = ["X", "Y", "Z"];
theta = {'$\theta_1$', '$\theta_2$', '$\theta_3$'};


% Controller force 
figure;
for k = 1:3
    
    subplot(3,1,k)
    plot(t, force(:,k))
    xlabel("Time [sec]")
    ylabel([str(k),newline,"Force [N]"]);
    title("Controller Force Output")
    
end

% Actual and Desired end effector position
figure;
for k = 1:3
    
    subplot(3,1,k)
    hold on;
    plot(t, P_actual(:,k))
    plot(t, P_desired(:,k))
    hold off;
    xlabel("Time [sec]")
    ylabel([str(k), newline, "Position [m]"]);
    title("End Effector Position")
    legend("Actual", "Desired")
    
end

% Actual and Desired joint angle
figure;
for k = 1:3
    
    subplot(3,1,k)
    hold on;
    plot(t, q(:,k))
    plot(t, q_desired(:, k))
    hold off;
    xlabel("Time [sec]")
    ylabel([theta{k}, ' [rad]'], "interpreter", "latex");
    title("Joint Angles")
    legend("Actual", "Desired")
    
end

% Desired - Actual
figure;
for k = 1:3
    
    subplot(3,1,k)
    plot(t, P_desired(:,k) - P_actual(:, k))
    xlabel("Time [sec]")
    ylabel([str(k), " Position [m]"]);
    title(['Deviation from the Desired Position', newline, '$P_{desired} - P_{actual}$'], "interpreter", "latex")
    legend("Actual", "Desired")
    
end


pause(2);
%% Animation Part
figure;
set(gcf,'WindowState','Maximized')
subplot(2,2,1:2)

title('Isometric')  
hold on;
grid on;
axis([-0.2 0.2 -0.2 0.2 -0.2 0.2])
xlabel('X');
ylabel('Y');
zlabel('Z');
mArrow3([0 0 0],[0.07 0 0],'color','red','facealpha',0.5);
text(0.08, 0 ,0,'X');
mArrow3([0 0 0],[0 0.07 0],'color','green','facealpha',0.5);
text(0,0.08 ,0,'Y');
mArrow3([0 0 0],[0 0 0.07],'color','blue','facealpha',0.5);
text(0 ,0 ,0.08,'Z');
camroll(-90);
view(-65,9);


history_ee = zeros(size(q, 1), 3);
desired_ee = zeros(size(P_desired,1),3);
for i = 1:size(q,1)
Base = [0;L2;-L1];
Shoulder = Base + [L1*sin(q(i,1));L1*sin(q(i,2));L1*cos(q(i,1))*cos(q(i,2))];
EndEffector = Base + [sin(q(i,1))*(L1*cos(q(i,2))+L2*sin(q(i,3)));L1*sin(q(i,2))-L2*cos(q(i,3));(L1*cos(q(i,2))+L2*sin(q(i,3)))*cos(q(i,1))];
history_ee(i, :) = EndEffector';
desired_ee(i,:) = P_desired(i,:);
if i > 1
    history = [history_ee(i - 1, :)', history_ee(i, :)'];
    desired = [desired_ee(i - 1, :)', desired_ee(i, :)'];
    line(history(1, :), history(2, :),history(3,:), 'color', 'r', 'linewidth', 0.1);
    line(desired(1, :), desired(2, :),desired(3,:), 'color', 'k', 'linewidth', 0.1);

end

L_XY = [Base, Shoulder, EndEffector];
plot_line = line(L_XY(1, :), L_XY(2, :),L_XY(3,:), 'color', [109/255,  117/255, 144/255], 'linewidth', 3);
J1 = plot3(Shoulder(1), Shoulder(2), Shoulder(3), 'ko', 'MarkerSize', 5, 'linewidth', 5);
pause(0.001);
if i ~= size(q, 1)
                 delete(plot_line); delete(J1);
end
end
pause(1);
subplot(2,2,3)
title('Top View')  
hold on;
grid on;
rotate3d off;
axis([-0.2 0.2 -0.2 0.2 -0.2 0.2])
xlabel('X');
ylabel('Y');
zlabel('Z');
mArrow3([0 0 0],[0.07 0 0],'color','red','facealpha',0.5);
text(0.08, 0 ,0,'X');
mArrow3([0 0 0],[0 0.07 0],'color','green','facealpha',0.5);
text(0,0.08 ,0,'Y');
mArrow3([0 0 0],[0 0 0.07],'color','blue','facealpha',0.5);
text(0 ,0 ,0.08,'Z');
view(90,0) 
set(gca,'ydir','reverse')
camroll(-90)
history_ee = zeros(size(q, 1), 3);
desired_ee = zeros(size(P_desired,1),3);
for i = 1:size(q,1)
Base = [0;L2;-L1];
Shoulder = Base + [L1*sin(q(i,1));L1*sin(q(i,2));L1*cos(q(i,1))*cos(q(i,2))];
EndEffector = Base + [sin(q(i,1))*(L1*cos(q(i,2))+L2*sin(q(i,3)));L1*sin(q(i,2))-L2*cos(q(i,3));(L1*cos(q(i,2))+L2*sin(q(i,3)))*cos(q(i,1))];
history_ee(i, :) = EndEffector';
desired_ee(i,:) = P_desired(i,:);
if i > 1
    history = [history_ee(i - 1, :)', history_ee(i, :)'];
    desired = [desired_ee(i - 1, :)', desired_ee(i, :)'];
    line(history(1, :), history(2, :),history(3,:), 'color', 'r', 'linewidth', 0.1);
    line(desired(1, :), desired(2, :),desired(3,:), 'color', 'k', 'linewidth', 0.1);

end

L_XY = [Base, Shoulder, EndEffector];
plot_line = line(L_XY(1, :), L_XY(2, :),L_XY(3,:), 'color', [109/255,  117/255, 144/255], 'linewidth', 3);
J1 = plot3(Shoulder(1), Shoulder(2), Shoulder(3), 'ko', 'MarkerSize', 5, 'linewidth', 5);
pause(0.001);
if i ~= size(q, 1)
                 delete(plot_line); delete(J1);
end
end
pause(1);
subplot(2,2,4);
title('Side View')
rotate3d off;
hold on;
grid on;
axis([-0.2 0.2 -0.2 0.2 -0.2 0.2])
xlabel('X');
ylabel('Y');
zlabel('Z');
mArrow3([0 0 0],[0.07 0 0],'color','red','facealpha',0.5);
text(0.08, 0 ,0,'X');
mArrow3([0 0 0],[0 0.07 0],'color','green','facealpha',0.5);
text(0,0.08 ,0,'Y');
mArrow3([0 0 0],[0 0 0.07],'color','blue','facealpha',0.5);
text(0 ,0 ,0.08,'Z');
view(0,0) 
camroll(-90)

history_ee = zeros(size(q, 1), 3);
desired_ee = zeros(size(P_desired,1),3);
for i = 1:size(q,1)
Base = [0;L2;-L1];
Shoulder = Base + [L1*sin(q(i,1));L1*sin(q(i,2));L1*cos(q(i,1))*cos(q(i,2))];
EndEffector = Base + [sin(q(i,1))*(L1*cos(q(i,2))+L2*sin(q(i,3)));L1*sin(q(i,2))-L2*cos(q(i,3));(L1*cos(q(i,2))+L2*sin(q(i,3)))*cos(q(i,1))];
history_ee(i, :) = EndEffector';
desired_ee(i,:) = P_desired(i,:);
if i > 1
    history = [history_ee(i - 1, :)', history_ee(i, :)'];
    desired = [desired_ee(i - 1, :)', desired_ee(i, :)'];
    line(history(1, :), history(2, :),history(3,:), 'color', 'r', 'linewidth', 0.1);
    line(desired(1, :), desired(2, :),desired(3,:), 'color', 'k', 'linewidth', 0.1);

end

L_XY = [Base, Shoulder, EndEffector];
plot_line = line(L_XY(1, :), L_XY(2, :),L_XY(3,:), 'color', [109/255,  117/255, 144/255], 'linewidth', 3);
J1 = plot3(Shoulder(1), Shoulder(2), Shoulder(3), 'ko', 'MarkerSize', 5, 'linewidth', 5);
pause(0.001);
if i ~= size(q, 1)
                 delete(plot_line); delete(J1);
end
end
