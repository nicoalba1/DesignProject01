function DesignProblem01(controller,varargin)
% DesignProblem01   run simulation of spacecraft
%
%   DesignProblem01('FunctionName') uses the controller defined by
%       the function 'FunctionName' - for example, in a file with
%       the name FunctionName.m - in the simulation.
%
%   DesignProblem01('FunctionName','P1',V1,'P2','V2',...) optionally
%       defines a number of parameter values:
%
%           'team' : a name (e.g., 'FirstName LastName') that, if defined,
%                    will appear on the figure window
%
%           'datafile' : a filename (e.g., 'data.mat') where, if defined,
%                        data will be logged and saved
%
%           'moviefile' : a filename (e.g., 'movie.mp4') where, if defined,
%                         a movie of the simulation will be saved
%
%           'snapshotfile' : a filename (e.g., 'snap.pdf') where, if
%                            defined, a PDF with a snapshot of the last
%                            frame of the simulation will be saved
%
%           'controllerdatalog' : a cell array (e.g., {'y','xhat'}) with
%                                 the names of fields in controller.data -
%                                 if 'datafile' is defined (so data is
%                                 logged), then values in these fields will
%                                 also be logged and saved
%
%           'diagnostics' : a flag - true or false (default) that, if true,
%                           will show plots of state and actuator values
%
%           'tStop' : the time at which the simulation will stop (a
%                     positive number) - defaults value is 30
%
%           'disturbance' : a flag - true or false (default) that, if true,
%                           will add an unknown disturbance to each torque
%
%   Regardless of how the function is called, it will clear the current
%   figure and will show the simulation. To quit, type 'q' when this figure
%   is in the foreground.

% Parse the arguments
% - Create input parser
p = inputParser;
% - Parameter names must be specified in full
p.PartialMatching = false;
% - This argument is required, and must be first
addRequired(p,'controller',@ischar);
% - These parameters are optional, and can be in any order
addParameter(p,'team',[],@ischar);
addParameter(p,'datafile',[],@ischar);
addParameter(p,'moviefile',[],@ischar);
addParameter(p,'snapshotfile',[],@ischar);
addParameter(p,'controllerdatatolog',[],@iscell);
addParameter(p,'diagnostics',false,@islogical);
addParameter(p,'tStop',30,@(x) isscalar(x) && isnumeric(x) && (x>0));
addParameter(p,'disturbance',false,@islogical);
% - Apply input parser
parse(p,controller,varargin{:});
% - Extract parameters
process = p.Results;
% - Check that the 'controller' function exists
if (exist(process.controller,'file')~=2)
    error('Controller ''%s'' does not exist.',process.controller);
end

% Setup the simulation
[process,controller] = SetupSimulation(process);

% Run the simulation
RunSimulation(process,controller);


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT WILL CHANGE FOR DIFFERENT PROCESSES
%

function [process,controller] = SetupSimulation(process)

% DEFINE CONSTANTS

% Constants related to simulation.
% - State time.
process.tStart = 0;
% - Time step.
process.tStep = 1/50;
% - Names of things to log in datafile, if desired
process.processdatatolog = {'t','w_01in1'};

% Constants related to physical properties.
% - Mass
process.m = 1;
% - Dimensions
process.l1 = 1.5;
process.l2 = 1.0;
process.l3 = 0.2;
% - Principal moments of inertia
process.J1 = (process.m/12)*(process.l2^2+process.l3^2);
process.J2 = (process.m/12)*(process.l3^2+process.l1^2);
process.J3 = (process.m/12)*(process.l1^2+process.l2^2);
% - Moment of inertia matrix
process.J_in1 = diag([process.J1,process.J2,process.J3]);
% - Maximum torque
process.tauMax = 1;
% - Disturbance torque
if process.disturbance
    process.d_tau1 = 0.5*(-1+2*rand);
    process.d_tau2 = 0.5*(-1+2*rand);
end

% DEFINE VARIABLES

% Time
process.t = 0;
% Orientation (ZYX Euler Angles)
process.theta = zeros(3,1);
% Angular velocity
process.w_01in1 = 5*randn(3,1);

% DEFINE CONTROLLER

% Functions
% - get handles to user-defined functions 'init' and 'run'
controller = eval(process.controller);
controller.name = process.controller;
% Parameters
% - define a list of constants that will be passed to the controller
names = {'tStep','J1','J2','J3'};
% - loop to create a structure with only these constants
controller.parameters = struct;
for i=1:length(names)
    controller.parameters.(names{i}) = process.(names{i});
end
% Storage
controller.data = struct;
% References
controller.references = struct('wbar1',0,'wbar2',0,'wbar3',0);
% Sensors
controller.sensors = GetSensors(process);
% Actuators
controller.running = true;
tic
try
    [controller.actuators,controller.data] = ...
        controller.init(controller.sensors, ...
                           controller.references, ...
                           controller.parameters, ...
                           controller.data);
catch exception
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name,getReport(exception));
	controller.actuators = ZeroActuators();
    controller.running = false;
end
if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'did not return a structure ''actuators'' with the right\n' ...
             'format. Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name);
    controller.actuators = ZeroActuators();
    controller.running = false;
end
controller.tComputation = toc;

end


function sensors = GetSensors(process)
sensors.t = process.t;
sensors.w1 = process.w_01in1(1);
sensors.w2 = process.w_01in1(2);
sensors.w3 = process.w_01in1(3);
% Add noise
%   (nothing)
end

function [t,x] = Get_TandX_From_Process(process)
t = process.t;
x = [process.theta; process.w_01in1];
end

function u = GetInput(process,actuators)
% Copy input from actuators
u = [actuators.tau1; actuators.tau2];

% Bound input
for i=1:length(u)
    if (u(i) < -process.tauMax)
        u(i) = -process.tauMax;
    elseif (u(i) > process.tauMax)
        u(i) = process.tauMax;
    end
end

% Add disturbance
if process.disturbance
    u(1) = u(1) + process.d_tau1;
    u(2) = u(2) + process.d_tau2;
end
end

function process = Get_Process_From_TandX(t,x,process)
process.t = t;
process.theta = x(1:3,1);
process.w_01in1 = x(4:6,1);
end

function xdot = GetXDot(t,x,u,process)
% unpack x and u
theta = x(1:3,1);
w_01in1 = x(4:6,1);
tau_in1 = [u(1:2,1);0];
% compute rates of change
thetadot = GetAngularRates_ZYX(theta,w_01in1);
w_01in1dot = process.J_in1\(tau_in1-wedge(w_01in1)*process.J_in1*w_01in1);
% pack xdot
xdot = [thetadot; w_01in1dot];
end

function iscorrect = CheckActuators(actuators)
iscorrect = false;
if all(isfield(actuators,{'tau1','tau2'}))&&(length(fieldnames(actuators))==2)
    if isnumeric(actuators.tau1) && isnumeric(actuators.tau2)
        if isscalar(actuators.tau1) && isscalar(actuators.tau2)
            iscorrect = true;
        end
    end
end
end

function actuators = ZeroActuators()
actuators = struct('tau1',0,'tau2',0);
end

function fig = UpdateFigure(process,controller,fig)
if (isempty(fig))
    % CREATE FIGURE

    % Clear the current figure.
    clf;

    % Create an axis for text (it's important this is in the back,
    % so you can rotate the view and other stuff!)
    fig.text.axis = axes('position',[0 0 1 1]);
    axis([0 1 0 1]);
    hold on;
    axis off;
    fs = 14;
    if (controller.running)
        status = 'ON';
        color = 'g';
    else
        status = 'OFF';
        color = 'r';
    end
    fig.text.status=text(0.05,0.975,...
        sprintf('CONTROLLER: %s',status),...
        'fontweight','bold','fontsize',fs,...
        'color',color,'verticalalignment','top');
    fig.text.time=text(0.05,0.12,...
        sprintf('t = %6.2f / %6.2f\n',process.t,process.tStop),...
        'fontsize',fs,'verticalalignment','top','fontname','monaco');
    fig.text.teamname=text(0.05,0.06,...
        sprintf('%s',process.team),...
        'fontsize',fs,'verticalalignment','top','fontweight','bold');

    % Create axes for diagnostic plots
    if process.diagnostics
        fig.x.axis = axes('position',[0.55,0.6,0.4,0.35],'fontsize',fs);
        axis([0,process.tStop,-15,15]);
        hold on;
        for i=1:3
            fig.x.w(i) = plot(nan,nan,'linewidth',2);
        end
        fig.x.legend = legend('w_1','w_2','w_3');
        xlabel('time');

        fig.u.axis = axes('position',[0.55,0.1,0.4,0.35],'fontsize',fs);
        delta = 0.1*process.tauMax;
        axis([0,process.tStop,-process.tauMax-delta,process.tauMax+delta]);
        hold on;
        fig.u.tau1 = plot(nan,nan,'linewidth',3);
        fig.u.tau2 = plot(nan,nan,'linewidth',3);
        fig.u.umin = plot([0 process.tStop],-process.tauMax*[1 1],...
                          'linewidth',1,'linestyle','--','color','k');
        fig.u.umax = plot([0 process.tStop],process.tauMax*[1 1],...
                          'linewidth',1,'linestyle','--','color','k');
        fig.u.legend = legend('\tau_1','\tau_2');
        xlabel('time');
    end

    % Create an axis for the view from frame 0.
    if process.diagnostics
        fig.view0.axis = axes('position',[0 0.02 0.5 1]);
    else
        fig.view0.axis = axes('position',[0 0 1 1]);
    end
    set(gcf,'renderer','opengl');
    set(gcf,'color','w');
    axis equal;
    axis(1.0*[-1 1 -1 1 -1 1]);
    axis manual;
    hold on;
    axis off;
    view([37.5,20]);
    box on;
    set(gca,'projection','perspective');
    set(gca,'clipping','on','clippingstyle','3dbox');
    lighting gouraud
    fig.view0.light = light('position',[0;0;2],'style','local');

    pFrame = [0 1 0 0;
              0 0 1 0;
              0 0 0 1];
    fig.geom.pFrame0_in0 = pFrame;
    fig.geom.pFrame1_in1 = pFrame;
    fig.geom.pSpacecraft_in1 = [0.5*process.l1*[-1 1 1 -1 -1 1 1 -1];
                                0.5*process.l2*[-1 -1 1 1 -1 -1 1 1];
                                0.5*process.l3*[-1 -1 -1 -1 1 1 1 1]];
	fig.geom.fSpacecraft = [1 2 3 4;
                            5 6 7 8;
                            1 2 6 5;
                            2 3 7 6;
                            3 4 8 7;
                            4 1 5 8];

    R_1in0 = R_ZYX(process.theta);
    fig.geom.pFrame1_in0 = Transform(zeros(3,1),R_1in0,fig.geom.pFrame1_in1);
    fig.geom.pSpacecraft_in0 = Transform(zeros(3,1),R_1in0,fig.geom.pSpacecraft_in1);

    fig.view0.frame0 = DrawFrame([],fig.geom.pFrame0_in0);
    fig.view0.frame1 = DrawFrame([],fig.geom.pFrame1_in0);
    fig.view0.spacecraft = DrawMesh([],fig.geom.pSpacecraft_in0,fig.geom.fSpacecraft,'y',0.9);

    % Make the figure respond to key commands.
    set(gcf,'KeyPressFcn',@onkeypress);
else
    % UPDATE FIGURE

    set(fig.text.time,'string',sprintf('t = %6.2f / %6.2f\n',process.t,process.tStop));
    if (controller.running)
        status = 'ON';
        color = 'g';
    else
        status = 'OFF';
        color = 'r';
    end
    set(fig.text.status,'string',sprintf('CONTROLLER: %s',status),'color',color);

    if process.diagnostics
        t = [get(fig.x.w(1),'xdata') process.t];
        for i=1:3
            w = [get(fig.x.w(i),'ydata') process.w_01in1(i,1)];
            set(fig.x.w(i),'xdata',t,'ydata',w);
        end
        tau1 = [get(fig.u.tau1,'ydata') controller.actuators.tau1];
        tau2 = [get(fig.u.tau2,'ydata') controller.actuators.tau2];
        set(fig.u.tau1,'xdata',t,'ydata',tau1);
        set(fig.u.tau2,'xdata',t,'ydata',tau2);
    end


    R_1in0 = R_ZYX(process.theta);
    fig.geom.pFrame1_in0 = Transform(zeros(3,1),R_1in0,fig.geom.pFrame1_in1);
    fig.geom.pSpacecraft_in0 = Transform(zeros(3,1),R_1in0,fig.geom.pSpacecraft_in1);

    fig.view0.frame1 = DrawFrame(fig.view0.frame1,fig.geom.pFrame1_in0);
    fig.view0.spacecraft = DrawMesh(fig.view0.spacecraft,fig.geom.pSpacecraft_in0);

end
drawnow;
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT (I HOPE) WILL REMAIN THE SAME FOR ALL PROCESSES
%

function RunSimulation(process,controller)

% START-UP

% Create empty figure.
fig = [];

% Flag to stop simulation on keypress.
global done
done = false;

% Start making movie, if necessary.
if (~isempty(process.moviefile))
    myV = VideoWriter(process.moviefile,'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 1/process.tStep;
    open(myV);
end

% LOOP

% Loop until break.
tStart = tic;
while (1)

    % Update figure (create one if fig is empty).
    fig = UpdateFigure(process,controller,fig);

    % Update data.
    if (~isempty(process.datafile) && controller.running)
        [process,controller] = UpdateDatalog(process,controller);
    end

    % If making a movie, store the current figure as a frame.
    if (~isempty(process.moviefile))
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end

    % Stop if time has reached its maximum.
    if ((process.t>=process.tStop)||done)
        break;
    end

    % Update process (integrate equations of motion).
    [process,controller] = UpdateProcess(process,controller);

    % Wait if necessary, to stay real-time.
    while (toc(tStart)<process.t-process.tStart)
        % Do nothing
    end

end

% SHUT-DOWN

% Close and save the movie, if necessary.
if (~isempty(process.moviefile))
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

% Save the data.
if (~isempty(process.datafile))
    processdata = process.log.process; %#ok<NASGU>
    controllerdata = process.log.controller; %#ok<NASGU>
    save(process.datafile,'processdata','controllerdata');
end

% Save the snapshot, if necessary.
if (~isempty(process.snapshotfile))
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',process.snapshotfile);
end

end


function [process,controller] = UpdateDatalog(process,controller)
% Create data log if it does not already exist.
if (~isfield(process,'log'))
    process.log = struct('process',struct,...
                         'controller',struct('tComputation',[],...
                                             'sensors',struct,...
                                             'actuators',struct,...
                                             'data',struct),...
                         'count',0);
end
% Increment log count.
process.log.count = process.log.count+1;
% Write data to log.
for i=1:length(process.processdatatolog)
    name = process.processdatatolog{i};
    process.log.process.(name)(:,process.log.count) = process.(name);
end
process.log.controller.tComputation(:,process.log.count) = ...
    controller.tComputation;
names = fieldnames(controller.sensors);
for i=1:length(names)
    name = names{i};
    process.log.controller.sensors.(name)(:,process.log.count) = ...
        controller.sensors.(name);
end
names = fieldnames(controller.actuators);
for i=1:length(names)
    name = names{i};
    process.log.controller.actuators.(name)(:,process.log.count) = ...
        controller.actuators.(name);
end
for i=1:length(process.controllerdatatolog)
    name = process.controllerdatatolog{i};
    try
        process.log.controller.data.(name)(:,process.log.count) = ...
            controller.data.(name);
    catch exception
        warning(['Saving element ''%s'' of data for controller\n',...
                 '     ''%s''',...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],...
                 name,controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
        return
    end
end
end


function [process,controller] = UpdateProcess(process,controller)
% Integrate equations of motion
[t0,x] = Get_TandX_From_Process(process);
u = GetInput(process,controller.actuators);
[t,x] = ode45(@(t,x) GetXDot(t,x,u,process),[t0 t0+process.tStep],x);
process = Get_Process_From_TandX(t(end),x(end,:)',process);

% Get sensor values
controller.sensors = GetSensors(process);

% Get actuator values (run controller)
if (controller.running)
    tic
    try
        [controller.actuators,controller.data] = ...
            controller.run(controller.sensors, ...
                              controller.references, ...
                              controller.parameters, ...
                              controller.data);
    catch exception
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'did not return a structure ''actuators'' with the right\n' ...
                 'format. Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name);
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    controller.tComputation = toc;
else
    controller.tComputation = 0;
end

end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% HELPER FUNCTIONS
%

function R = RX(h)
R = [1 0 0;
     0 cos(h) -sin(h);
     0 sin(h) cos(h)];
end

function R = RY(h)
R = [cos(h) 0 sin(h);
     0 1 0;
     -sin(h) 0 cos(h)];
end

function R = RZ(h)
R = [cos(h) -sin(h) 0;
     sin(h) cos(h) 0;
     0 0 1];
end

function R = R_ZYX(theta)
R = RZ(theta(1))*RY(theta(2))*RX(theta(3));
end

function thetadot = GetAngularRates_ZYX(theta,w)
c2 = cos(theta(2));
s2 = sin(theta(2));
c3 = cos(theta(3));
s3 = sin(theta(3));
A = [   -s2     0       1;
        c2*s3   c3      0;
        c2*c3   -s3     0];
thetadot = A\w;
end

function wHat = wedge(w)
wHat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
end

function p_inj = Transform(o_kinj,R_kinj,p_ink)
p_inj = zeros(size(p_ink));
for i=1:size(p_ink,2)
    p_inj(:,i) = o_kinj + R_kinj*p_ink(:,i);
end
end

function onkeypress(src,event)
global done
if event.Character == 'q'
    done = true;
end
end

function mesh = DrawMesh(mesh,p,f,color,alpha)
if isempty(mesh)
    mesh = patch('Vertices',p','Faces',f,...
                 'FaceColor',color,'FaceAlpha',alpha,'EdgeAlpha',alpha);
else
    set(mesh,'vertices',p');
end
end

function frame = DrawFrame(frame,p)
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',3);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',3);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',3);
else
    set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
    set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
    set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
