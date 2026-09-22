%% voiture_simulation_trajectoire.m
% Simulation cinematique d'une voiture 2 roues motrices (modele differentiel)
% suivant une trajectoire de reference avec evitement d'obstacle simule
% par capteur ultrasonique.
clear; clc; close all;

%% Parametres du robot
L   = 0.15;     % empattement (m)
Ts  = 0.05;      % pas de temps (s)
Tf  = 20;         % duree simulation (s)
t   = 0:Ts:Tf;
N   = length(t);

v_nom = 0.35;    % vitesse nominale (m/s)

%% Etat initial [x, y, theta]
x = zeros(1,N); y = zeros(1,N); theta = zeros(1,N);
x(1)=0; y(1)=0; theta(1)=0;

%% Obstacle (position fixe) detecte par HC-SR04
obstacle = [3.0, 0.3];
rayon_detection = 0.4; % m

Kp_evitement = 2.5;

for k = 1:N-1
    d_obs = norm([x(k) y(k)] - obstacle);
    if d_obs < rayon_detection
        % Manoeuvre d'evitement : on tourne proportionnellement a la proximite
        omega = Kp_evitement * (rayon_detection - d_obs) * sign(y(k)-obstacle(2)+eps);
        v = v_nom*0.5;
    else
        % Suivi de trajectoire rectiligne avec legere correction de cap
        cap_ref = 0;
        omega = 1.2*(cap_ref - theta(k));
        v = v_nom;
    end

    x(k+1)     = x(k) + v*cos(theta(k))*Ts;
    y(k+1)     = y(k) + v*sin(theta(k))*Ts;
    theta(k+1) = theta(k) + omega*Ts;
end

%% Traces
figure('Name','Trajectoire voiture avec evitement obstacle','Color','w');
plot(x, y, 'b-', 'LineWidth', 1.6); hold on;
viscircles(obstacle, rayon_detection, 'Color','r','LineStyle','--');
plot(obstacle(1), obstacle(2), 'rx', 'MarkerSize', 12, 'LineWidth',2);
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');
title('Trajectoire simulee avec evitement d''obstacle (capteur ultrason)');
legend('Trajectoire robot','Zone de detection','Obstacle','Location','best');

saveas(gcf, '../docs/figures/matlab_voiture_trajectoire.png');

%% Vitesse et cap au cours du temps
figure('Name','Etats voiture','Color','w');
subplot(2,1,1);
plot(t, theta*180/pi); grid on;
ylabel('Cap \theta (deg)');
title('Evolution du cap du robot');
subplot(2,1,2);
d_min = arrayfun(@(i) norm([x(i) y(i)]-obstacle), 1:N);
plot(t, d_min); yline(rayon_detection,'r--');
ylabel('Distance obstacle (m)'); xlabel('Temps (s)'); grid on;

saveas(gcf, '../docs/figures/matlab_voiture_etats.png');

fprintf('Distance minimale a l''obstacle atteinte : %.3f m\n', min(d_min));
