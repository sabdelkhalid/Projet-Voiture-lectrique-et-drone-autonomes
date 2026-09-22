%% drone_simulation_vol.m
% Simulation du maintien d'altitude (PID sur poussee) et du suivi
% d'une trajectoire de waypoints en 3D (mission autonome).
clear; clc; close all;

%% 1. Maintien d'altitude (PID sur la poussee totale)
m = 0.90; g = 9.81; Ts = 0.01; t = 0:Ts:8; N = length(t);

z = zeros(1,N); vz = zeros(1,N);
z_ref = zeros(1,N); z_ref(t>=1) = 2.0;   % consigne : monter a 2 m apres 1s

Kp_z = 6.0; Ki_z = 1.2; Kd_z = 3.5;
int_z = 0; z_prev_mes = 0;
F_MAX = 4*4.5; % N, poussee max (4 moteurs x 4.41N ~ marge)

for k = 2:N
    e_z = z_ref(k) - z(k-1);
    int_z = int_z + e_z*Ts;
    % derivee sur la mesure (et non sur l'erreur) pour eviter le "coup"
    % derive lors d'un echelon de consigne
    d_mes = (z(k-1) - z_prev_mes)/Ts;
    z_prev_mes = z(k-1);

    F_thrust = m*g + Kp_z*e_z + Ki_z*int_z - Kd_z*d_mes;   % poussee totale (N)
    F_thrust = max(min(F_thrust, F_MAX), 0);               % saturation actionneurs
    if F_thrust == F_MAX || F_thrust == 0
        int_z = int_z - e_z*Ts;   % anti-windup (clamping)
    end

    az = (F_thrust - m*g)/m;
    vz(k) = vz(k-1) + az*Ts;
    z(k)  = z(k-1) + vz(k)*Ts;
end

figure('Name','Maintien altitude','Color','w');
plot(t, z_ref, 'k--', t, z, 'b-', 'LineWidth',1.4);
grid on; legend('Consigne altitude','Altitude simulee','Location','best');
xlabel('Temps (s)'); ylabel('Altitude (m)');
title(sprintf('Maintien d''altitude PID (Kp=%.1f, Ki=%.1f, Kd=%.1f)', Kp_z, Ki_z, Kd_z));
saveas(gcf, '../docs/figures/matlab_drone_altitude.png');

info_z = stepinfo(z, t, 2.0);
fprintf('Temps stabilisation altitude : %.2f s\n', info_z.SettlingTime);
fprintf('Depassement altitude         : %.2f %%\n', info_z.Overshoot);

%% 2. Suivi de mission - waypoints 3D
waypoints = [0 0 0; 2 0 2; 2 2 2; 0 2 2.5; 0 0 1];
t_wp = 0:0.02:1; % parametre d'interpolation
traj = [];
for i = 1:size(waypoints,1)-1
    seg = waypoints(i,:) + t_wp'*(waypoints(i+1,:)-waypoints(i,:));
    traj = [traj; seg]; %#ok<AGROW>
end
% bruit de mesure GPS/baro simule
traj_mesuree = traj + 0.03*randn(size(traj));

figure('Name','Mission waypoints 3D','Color','w');
plot3(traj(:,1), traj(:,2), traj(:,3), 'k--', 'LineWidth',1); hold on;
plot3(traj_mesuree(:,1), traj_mesuree(:,2), traj_mesuree(:,3), 'b-', 'LineWidth',1);
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ro', 'MarkerFaceColor','r','MarkerSize',7);
grid on; axis equal; view(35,25);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('Trajectoire planifiee','Trajectoire estimee (bruitee)','Waypoints','Location','best');
title('Mission autonome - suivi de waypoints');
saveas(gcf, '../docs/figures/matlab_drone_mission3d.png');

erreur_pos = sqrt(sum((traj-traj_mesuree).^2,2));
fprintf('Erreur de position moyenne : %.3f m (ecart-type %.3f m)\n', mean(erreur_pos), std(erreur_pos));
