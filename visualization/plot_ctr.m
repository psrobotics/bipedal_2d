function [] = plot_ctr(x_sol, f_sol, low_ctr,...
                       body_p, ctr_p, name)

size_arr = size(x_sol);
len_m = size_arr(2)-1;

dt = ctr_p.dt;
t_s_l = (1:(len_m+1))*dt;
t_s = (1:len_m)*dt;

rg = 5:len_m-100;

%% body state
figure(2); 
set(gcf, 'Position', [100, 100, 1920, 1080/1.5]);

subplot(4,1,1);
plot(t_s_l(rg), x_sol(4,rg),"LineWidth",1.5);
hold on;
plot(t_s_l(rg), x_sol(6,rg),"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Body's XZ Position")
xlabel("s")
ylabel("m")
legend("X", "Z");
grid on  

subplot(4,1,2);
plot(t_s_l(rg), x_sol(2,rg),"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Body's Pitch")
xlabel("s")
ylabel("rad")
grid on  

subplot(4,1,3);
plot(t_s_l(rg), x_sol(4+6,rg),"LineWidth",1.5);
hold on;
plot(t_s_l(rg), x_sol(6+6,rg),"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Body's XZ Velocity")
xlabel("s")
ylabel("m/s")
legend("X", "Z");
grid on  

subplot(4,1,4);
plot(t_s_l(rg), x_sol(2+6,rg),"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Body's Pitch Velocity")
xlabel("s")
ylabel("rad/s")
grid on  

resolution = 600;
print(2, name+'_state.png', '-dpng', ['-r', num2str(resolution)]);

%% fpos, jpos
figure(3); 
set(gcf, 'Position', [100, 100, 1920, 1080/1.5]);

subplot(4,1,1);
plot(t_s(rg), low_ctr.fp_s(1,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.fp_s(3,rg),"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Foot Placement Points X")
xlabel("s")
ylabel("m")
legend("leg1", "leg2");
grid on  

subplot(4,1,2);
plot(t_s(rg), low_ctr.fp_s(2,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.fp_s(4,rg),"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Foot Placement Points Z")
xlabel("s")
ylabel("m")
legend("leg1", "leg2");
grid on  

subplot(4,1,3);
plot(t_s(rg), low_ctr.rad_s(1,rg)/pi*180,"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.rad_s(2,rg)/pi*180,"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.rad_s(3,rg)/pi*180,"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.rad_s(4,rg)/pi*180,"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Joint Position")
xlabel("s")
ylabel("deg")
legend("q1", "q2", "q3", "q4");
grid on  

subplot(4,1,4);
plot(t_s(rg), low_ctr.drad_s(1,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.drad_s(2,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.drad_s(3,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.drad_s(4,rg),"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Joint Velocity")
xlabel("s")
ylabel("rad/s")
legend("dq1", "dq2", "dq3", "dq4");
grid on 

resolution = 600;
print(3, name+'_ffp.png', '-dpng', ['-r', num2str(resolution)]);

%% leg force, torque
figure(4); 
set(gcf, 'Position', [100, 100, 1920, 1080/3]);

subplot(2,1,1);
plot(t_s(rg), f_sol(1,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), f_sol(3,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), f_sol(4,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), f_sol(6,rg),"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Leg Force")
xlabel("s")
ylabel("N")
legend("fx1", "fz1", "fx2", "fz2");
grid on 

subplot(2,1,2);
plot(t_s(rg), low_ctr.tor_s(1,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.tor_s(2,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.tor_s(3,rg),"LineWidth",1.5);
hold on;
plot(t_s(rg), low_ctr.tor_s(4,rg),"LineWidth",1.5);
hold on;
xlim([t_s_l(rg(1)),t_s_l(rg(end))]);
title("Joint Torque")
xlabel("s")
ylabel("N*m")
legend("j1", "j2", "j3", "j4");
grid on 

resolution = 600;
print(4, name+'_tor.png', '-dpng', ['-r', num2str(resolution)]);

end