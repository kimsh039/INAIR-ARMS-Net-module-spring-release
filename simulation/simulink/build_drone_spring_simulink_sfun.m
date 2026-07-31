%% Build and verify the drone spring model with a Level-2 S-Function
clearvars -except results

outDir = fileparts(mfilename('fullpath'));
addpath(outDir);
modelName = 'drone_spring_2dof_sfun';
modelFile = fullfile(outDir,[modelName '.slx']);

if bdIsLoaded(modelName), close_system(modelName,0); end
if exist(modelFile,'file'), delete(modelFile); end
new_system(modelName);

add_block('simulink/Sources/Constant',[modelName '/Spring rate N_per_mm'], ...
    'Value','0.25','Position',[80 180 175 215]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [modelName '/Drone spring mechanism'], ...
    'FunctionName','drone_spring_sfun', ...
    'Position',[265 125 430 270]);

add_block('simulink/Sinks/To Workspace',[modelName '/State output'], ...
    'VariableName','simState','SaveFormat','Timeseries', ...
    'Position',[535 120 650 155]);
add_block('simulink/Sinks/To Workspace',[modelName '/Spring force output'], ...
    'VariableName','simSpringForce','SaveFormat','Timeseries', ...
    'Position',[535 185 665 220]);
add_block('simulink/Sinks/To Workspace',[modelName '/Spring length output'], ...
    'VariableName','simSpringLength','SaveFormat','Timeseries', ...
    'Position',[535 250 665 285]);
add_block('simulink/Sinks/Scope',[modelName '/State scope'], ...
    'Position',[735 120 785 160]);

add_line(modelName,'Spring rate N_per_mm/1','Drone spring mechanism/1', ...
    'autorouting','on');
add_line(modelName,'Drone spring mechanism/1','State output/1', ...
    'autorouting','on');
add_line(modelName,'Drone spring mechanism/1','State scope/1', ...
    'autorouting','on');
add_line(modelName,'Drone spring mechanism/2','Spring force output/1', ...
    'autorouting','on');
add_line(modelName,'Drone spring mechanism/3','Spring length output/1', ...
    'autorouting','on');

set_param(modelName,'StopTime','0.5','Solver','ode45','MaxStep','2e-4');
Simulink.BlockDiagram.arrangeSystem(modelName);
save_system(modelName,modelFile);

simOut = sim(modelName,'ReturnWorkspaceOutputs','on');
simState = simOut.get('simState');
simSpringForce = simOut.get('simSpringForce');
simSpringLength = simOut.get('simSpringLength');

figure('Color','w','Name','S-Function Simulink verification');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
nexttile;
plot(simState.Time,rad2deg(simState.Data(:,1)),'LineWidth',1.5);
grid on; ylabel('Cam [deg]');
title('Level-2 S-Function verification, k=0.25 N/mm');
nexttile;
plot(simSpringForce.Time,simSpringForce.Data,'LineWidth',1.5);
grid on; ylabel('Force [N]');
nexttile;
plot(simSpringLength.Time,simSpringLength.Data,'LineWidth',1.5);
grid on; ylabel('Length [mm]'); xlabel('Time [s]');

exportgraphics(gcf,fullfile(outDir, ...
    'drone_spring_simulink_sfun_verification.png'),'Resolution',180);
save(fullfile(outDir,'drone_spring_simulink_sfun_results.mat'), ...
    'simState','simSpringForce','simSpringLength');

fprintf('\nVerified Simulink model: %s\n',modelFile);
fprintf('Final cam angle: %.2f deg\n',rad2deg(simState.Data(end,1)));
fprintf('Peak spring force: %.2f N\n',max(simSpringForce.Data));
fprintf('Minimum spring length: %.2f mm\n',min(simSpringLength.Data));

open_system(modelName);
