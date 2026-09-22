%% voiture_pid_tuning.m
% Identification du moteur CC (1er ordre) et reglage du correcteur PI
% de vitesse par placement de poles. Trace la reponse indicielle en
% boucle ouverte puis en boucle fermee (avec/sans correcteur).
clear; clc; close all;

%% 1. Modele du moteur CC identifie experimentalement
% Fonction de transfert Omega(s)/V(s) = K / (tau.s + 1)
K   = 25;      % gain statique (tr/min par volt)
tau = 0.15;    % constante de temps (s)
G_moteur = tf(K, [tau 1]);

fprintf('Fonction de transfert moteur : K=%.1f, tau=%.2f s\n', K, tau);

%% 2. Boucle ouverte - reponse a un echelon de 6V
figure('Name','Reponse moteur - boucle ouverte','Color','w');
step(6*G_moteur, 1.5);
grid on; title('Vitesse moteur en boucle ouverte (echelon 6V)');
ylabel('Vitesse (tr/min)'); xlabel('Temps (s)');
saveas(gcf, '../docs/figures/matlab_voiture_bo.png');

%% 3. Correcteur PI (Kp, Ki) regle par placement de poles
Kp = 0.45;
Ki = 3.0;
C_PI = pid(Kp, Ki);

sys_BF = feedback(C_PI*G_moteur, 1);

figure('Name','Reponse moteur - boucle fermee PI','Color','w');
step(20*sys_BF, 1.0);   % consigne 20 tr/min
grid on; title(sprintf('Asservissement vitesse PI (Kp=%.2f, Ki=%.2f)', Kp, Ki));
ylabel('Vitesse (tr/min)'); xlabel('Temps (s)');
saveas(gcf, '../docs/figures/matlab_voiture_bf_pi.png');

info = stepinfo(20*sys_BF);
fprintf('Temps de montee   : %.3f s\n', info.RiseTime);
fprintf('Depassement       : %.2f %%\n', info.Overshoot);
fprintf('Temps stabilisation (2%%) : %.3f s\n', info.SettlingTime);

%% 4. Suivi de ligne - boucle d'asservissement de cap (PID discret)
% Simulation simplifiee : erreur de position (capteurs IR) -> PWM differentiel
Ts = 0.02;             % periode d'echantillonnage (s)
t  = 0:Ts:5;
N  = length(t);

Kp_l = 40; Ki_l = 0.5; Kd_l = 12;

% Trajectoire de ligne simulee : virage sinusoidal
erreur_ref = 0; % on veut ramener l'erreur capteur a 0
e   = zeros(1,N); u = zeros(1,N); integ = 0; prev_e = 0;
pos_ligne = 0.6*sin(2*pi*0.2*t) + 0.05*randn(1,N); % position ligne (bruitee)
robot_pos = zeros(1,N);

for k = 2:N
    e(k) = pos_ligne(k) - robot_pos(k-1);
    integ = integ + e(k)*Ts;
    deriv = (e(k) - prev_e)/Ts;
    u(k) = Kp_l*e(k) + Ki_l*integ + Kd_l*deriv;
    u(k) = max(min(u(k), 255), -255);     % saturation PWM
    robot_pos(k) = robot_pos(k-1) + 0.15*u(k)*Ts/50; % reponse simplifiee du robot
    prev_e = e(k);
end

figure('Name','Suivi de ligne - PID','Color','w');
subplot(2,1,1);
plot(t, pos_ligne, 'b--', t, robot_pos, 'r-', 'LineWidth',1.3);
legend('Position ligne (consigne)','Position robot (mesure)');
ylabel('Position laterale (u.a.)'); grid on;
title(sprintf('Suivi de ligne PID (Kp=%d, Ki=%.1f, Kd=%d)', Kp_l, Ki_l, Kd_l));

subplot(2,1,2);
plot(t, u, 'k'); grid on;
ylabel('Commande PWM differentiel'); xlabel('Temps (s)');

saveas(gcf, '../docs/figures/matlab_voiture_suivi_ligne.png');

rmse = sqrt(mean((pos_ligne-robot_pos).^2));
fprintf('RMSE suivi de ligne : %.4f (u.a.)\n', rmse);
