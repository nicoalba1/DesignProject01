function func = ControllerUIN
% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [actuators,data] = initControlSystem(sensors,references,parameters,data)
actuators.tau1 = 0;
actuators.tau2 = 0;
data.K= [1 0 0; 0 .2 0];
end

%
% STEP #1: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors,references,parameters,data)
x=[sensors.w1;sensors.w2; sensors.w3];
u= -data.K*x;
actuators.tau1 = u(1);
actuators.tau2 = u(2);
end


