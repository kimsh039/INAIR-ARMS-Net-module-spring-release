%% Drone folding-wing spring preliminary simulation
% Geometry is derived from the Fusion 360 measurements taken on 2026-07-30.
% This is a planar 2-DOF model: main cam/wing and release latch.

clear; clc; close all;

%% Measured geometry [SI units]
pivotCam   = [0, 0];
pivotLatch = [48.00, 4.48] * 1e-3;
rCam0      = [-10.24, -10.56] * 1e-3;
rLatch0    = [-5.12, -3.68] * 1e-3;

lockedLengthMeasured = 54.339e-3;
openLengthEstimated  = 35.6e-3;

%% Preliminary mechanical parameters
% Replace these with measured values after the first printed prototype.
inertiaCam   = 3.5e-4;  % kg*m^2, wing + cam + motor preliminary estimate
inertiaLatch = 3.0e-6;  % kg*m^2
dampingCam   = 8.0e-4;  % N*m*s/rad
dampingLatch = 8.0e-5;  % N*m*s/rad

movingMassCam = 42e-3;  % kg
massLatch     = 0.785e-3;
rComCam0      = [-70, 0] * 1e-3;
rComLatch0    = [-3.5, -1.5] * 1e-3;
g = 9.80665;

freeLength     = 31e-3;
initialTension = 1.5;   % N
springRates    = [0.25, 0.30, 0.35] * 1e3; % N/m

camStop   = deg2rad(78);
latchStop = deg2rad(-70);
tEnd = 0.50;

%% Check that reconstructed locked geometry matches Fusion
pCamLocked   = pivotCam + rCam0;
pLatchLocked = pivotLatch + rLatch0;
lockedLengthModel = norm(pLatchLocked - pCamLocked);

fprintf('Fusion locked length      : %.3f mm\n', lockedLengthMeasured*1e3);
fprintf('Reconstructed locked length: %.3f mm\n', lockedLengthModel*1e3);
fprintf('Estimated open length      : %.3f mm\n', openLengthEstimated*1e3);
fprintf('Required spring travel     : %.3f mm\n\n', ...
    (lockedLengthMeasured-openLengthEstimated)*1e3);

%% Parameter sweep
results = struct([]);
colors = lines(numel(springRates));
figure('Color','w','Name','Drone spring preliminary sweep');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

for i = 1:numel(springRates)
    par = struct( ...
        'pivotCam',pivotCam,'pivotLatch',pivotLatch, ...
        'rCam0',rCam0,'rLatch0',rLatch0, ...
        'inertiaCam',inertiaCam,'inertiaLatch',inertiaLatch, ...
        'dampingCam',dampingCam,'dampingLatch',dampingLatch, ...
        'movingMassCam',movingMassCam,'massLatch',massLatch, ...
        'rComCam0',rComCam0,'rComLatch0',rComLatch0,'g',g, ...
        'freeLength',freeLength,'initialTension',initialTension, ...
        'springRate',springRates(i), ...
        'camStop',camStop,'latchStop',latchStop);

    x0 = [0; 0; 0; 0]; % cam angle/rate, latch angle/rate
    opts = odeset('RelTol',1e-7,'AbsTol',1e-9,'MaxStep',2e-4);
    [t,x] = ode45(@(t,x) dynamics(t,x,par),[0 tEnd],x0,opts);

    n = numel(t);
    springLength = zeros(n,1);
    springForce  = zeros(n,1);
    for j = 1:n
        [springLength(j),springForce(j)] = springState(x(j,:),par);
    end

    camAngleDeg = rad2deg(x(:,1));
    latchAngleDeg = rad2deg(x(:,3));
    idx90 = find(camAngleDeg >= 0.9*rad2deg(camStop),1,'first');
    if isempty(idx90)
        deployTime90 = NaN;
    else
        deployTime90 = t(idx90);
    end

    results(i).springRate_N_per_mm = springRates(i)/1e3;
    results(i).time_s = t;
    results(i).state = x;
    results(i).springLength_m = springLength;
    results(i).springForce_N = springForce;
    results(i).deployTime90_s = deployTime90;
    results(i).peakCamSpeed_deg_s = max(abs(rad2deg(x(:,2))));

    nexttile(1); hold on; grid on;
    plot(t,camAngleDeg,'LineWidth',1.5,'Color',colors(i,:));
    nexttile(2); hold on; grid on;
    plot(t,latchAngleDeg,'LineWidth',1.5,'Color',colors(i,:));
    nexttile(3); hold on; grid on;
    plot(t,springForce,'LineWidth',1.5,'Color',colors(i,:));
    nexttile(4); hold on; grid on;
    plot(t,springLength*1e3,'LineWidth',1.5,'Color',colors(i,:));
end

labels = compose('k = %.2f N/mm',springRates/1e3);
nexttile(1); xlabel('Time [s]'); ylabel('Cam angle [deg]');
title('Wing/cam deployment'); legend(labels,'Location','best');
nexttile(2); xlabel('Time [s]'); ylabel('Latch angle [deg]');
title('Latch rotation'); legend(labels,'Location','best');
nexttile(3); xlabel('Time [s]'); ylabel('Spring force [N]');
title('Extension-spring force'); legend(labels,'Location','best');
nexttile(4); xlabel('Time [s]'); ylabel('Spring length [mm]');
title('Spring attachment distance'); legend(labels,'Location','best');

fprintf(' k [N/mm]   90%% deploy [ms]   peak cam speed [deg/s]   locked force [N]\n');
for i = 1:numel(results)
    lockedForce = initialTension + springRates(i) * ...
        max(0,lockedLengthMeasured-freeLength);
    fprintf('   %.2f          %8.2f              %8.1f              %6.2f\n', ...
        results(i).springRate_N_per_mm, ...
        results(i).deployTime90_s*1e3, ...
        results(i).peakCamSpeed_deg_s,lockedForce);
end

outDir = fileparts(mfilename('fullpath'));
save(fullfile(outDir,'drone_spring_preliminary_results.mat'), ...
    'results','lockedLengthMeasured','openLengthEstimated', ...
    'freeLength','initialTension','springRates');
exportgraphics(gcf,fullfile(outDir,'drone_spring_preliminary_plot.png'), ...
    'Resolution',180);

%% Local functions
function dx = dynamics(~,x,p)
thetaCam = x(1);
omegaCam = x(2);
thetaLatch = x(3);
omegaLatch = x(4);

rCam = rotate2(p.rCam0,thetaCam);
rLatch = rotate2(p.rLatch0,thetaLatch);
pCam = p.pivotCam + rCam;
pLatch = p.pivotLatch + rLatch;
d = pLatch-pCam;
L = norm(d);
u = d/max(L,eps);
Fmag = p.initialTension + p.springRate*max(0,L-p.freeLength);

forceCam = Fmag*u;
forceLatch = -forceCam;
tauSpringCam = cross2(rCam,forceCam);
tauSpringLatch = cross2(rLatch,forceLatch);

rComCam = rotate2(p.rComCam0,thetaCam);
rComLatch = rotate2(p.rComLatch0,thetaLatch);
tauGravityCam = cross2(rComCam,[0,-p.movingMassCam*p.g]);
tauGravityLatch = cross2(rComLatch,[0,-p.massLatch*p.g]);

alphaCam = (tauSpringCam+tauGravityCam-p.dampingCam*omegaCam) ...
    / p.inertiaCam;
alphaLatch = (tauSpringLatch+tauGravityLatch-p.dampingLatch*omegaLatch) ...
    / p.inertiaLatch;

% Simple hard-stop approximation for preliminary sizing.
if thetaCam >= p.camStop && omegaCam > 0
    omegaCam = 0;
    alphaCam = min(alphaCam,0);
end
if thetaLatch <= p.latchStop && omegaLatch < 0
    omegaLatch = 0;
    alphaLatch = max(alphaLatch,0);
end

dx = [omegaCam;alphaCam;omegaLatch;alphaLatch];
end

function [L,F] = springState(x,p)
rCam = rotate2(p.rCam0,x(1));
rLatch = rotate2(p.rLatch0,x(3));
L = norm((p.pivotLatch+rLatch)-(p.pivotCam+rCam));
F = p.initialTension+p.springRate*max(0,L-p.freeLength);
end

function r = rotate2(r0,theta)
c = cos(theta); s = sin(theta);
r = [c*r0(1)-s*r0(2), s*r0(1)+c*r0(2)];
end

function z = cross2(a,b)
z = a(1)*b(2)-a(2)*b(1);
end
