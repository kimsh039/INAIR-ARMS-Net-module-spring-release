function drone_spring_sfun(block)
% Level-2 MATLAB S-Function for the planar drone spring mechanism.
setup(block);
end

function setup(block)
block.NumDialogPrms = 0;

block.NumInputPorts = 1;
block.InputPort(1).Dimensions = 1;
block.InputPort(1).DatatypeID = 0;
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.NumOutputPorts = 3;
block.OutputPort(1).Dimensions = 4; % [cam angle; cam speed; latch angle; latch speed]
block.OutputPort(2).Dimensions = 1; % spring force [N]
block.OutputPort(3).Dimensions = 1; % spring length [mm]
for i = 1:3
    block.OutputPort(i).DatatypeID = 0;
    block.OutputPort(i).Complexity = 'Real';
end

block.NumContStates = 4;
block.SampleTimes = [0 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('InitializeConditions',@initializeConditions);
block.RegBlockMethod('Outputs',@outputs);
block.RegBlockMethod('Derivatives',@derivatives);
end

function initializeConditions(block)
block.ContStates.Data = zeros(4,1);
end

function outputs(block)
x = block.ContStates.Data;
kNmm = block.InputPort(1).Data;
[~,forceN,lengthMm] = mechanism(x,kNmm);
block.OutputPort(1).Data = x;
block.OutputPort(2).Data = forceN;
block.OutputPort(3).Data = lengthMm;
end

function derivatives(block)
x = block.ContStates.Data;
kNmm = block.InputPort(1).Data;
[dx,~,~] = mechanism(x,kNmm);
block.Derivatives.Data = dx;
end

function [dx,forceN,lengthMm] = mechanism(x,kNmm)
thetaCam = x(1);
omegaCam = x(2);
thetaLatch = x(3);
omegaLatch = x(4);

pivotLatch = [48.00e-3,4.48e-3];
rCam0 = [-10.24e-3,-10.56e-3];
rLatch0 = [-5.12e-3,-3.68e-3];

c1 = cos(thetaCam); s1 = sin(thetaCam);
rCam = [c1*rCam0(1)-s1*rCam0(2), ...
        s1*rCam0(1)+c1*rCam0(2)];
c2 = cos(thetaLatch); s2 = sin(thetaLatch);
rLatch = [c2*rLatch0(1)-s2*rLatch0(2), ...
          s2*rLatch0(1)+c2*rLatch0(2)];

d = (pivotLatch+rLatch)-rCam;
lengthM = hypot(d(1),d(2));
direction = d/max(lengthM,eps);

freeLengthM = 31e-3;
initialTensionN = 1.5;
forceN = initialTensionN+(kNmm*1e3)*max(0,lengthM-freeLengthM);
lengthMm = lengthM*1e3;

forceCam = forceN*direction;
forceLatch = -forceCam;
tauCam = cross2(rCam,forceCam);
tauLatch = cross2(rLatch,forceLatch);

rComCam0 = [-70e-3,0];
rComLatch0 = [-3.5e-3,-1.5e-3];
rComCam = [c1*rComCam0(1)-s1*rComCam0(2), ...
           s1*rComCam0(1)+c1*rComCam0(2)];
rComLatch = [c2*rComLatch0(1)-s2*rComLatch0(2), ...
             s2*rComLatch0(1)+c2*rComLatch0(2)];

tauGravityCam = rComCam(1)*(-42e-3*9.80665);
tauGravityLatch = rComLatch(1)*(-0.785e-3*9.80665);

alphaCam = (tauCam+tauGravityCam-8e-4*omegaCam)/3.5e-4;
alphaLatch = (tauLatch+tauGravityLatch-8e-5*omegaLatch)/3e-6;

dThetaCam = omegaCam;
dThetaLatch = omegaLatch;
if thetaCam >= deg2rad(78) && omegaCam > 0
    dThetaCam = 0;
    alphaCam = min(alphaCam,0);
end
if thetaLatch <= deg2rad(-70) && omegaLatch < 0
    dThetaLatch = 0;
    alphaLatch = max(alphaLatch,0);
end

dx = [dThetaCam;alphaCam;dThetaLatch;alphaLatch];
end

function value = cross2(a,b)
value = a(1)*b(2)-a(2)*b(1);
end
