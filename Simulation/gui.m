function AntiDroneDashboard
% HIGH ALTITUDE ANTI-DRONE DETECTION DASHBOARD
% MATLAB Online compatible dashboard-style simulation
%
% Demonstrates:
% - Moving aerial target
% - Radar-style visualization
% - Altitude control
% - Detection status
% - Live tracking
% - Environmental conditions
% - Performance estimation
%
% Simulation only: detection and tracking.

%% ============================================================
% INITIAL PARAMETERS
% =============================================================

altitude = 3000;          % Initial altitude in metres
timeValue = 0;
angle = 0;

droneX = 1000;
droneY = 500;

maxRadarRange = 5000;

isRunning = false;

rangeHistory = [];
trackHistory = [];
timeHistory = [];

%% ============================================================
% MAIN WINDOW
% =============================================================

fig = uifigure( ...
    'Name','High Altitude Anti-Drone Detection System', ...
    'Position',[50 50 1400 800], ...
    'Color',[0.05 0.07 0.10]);

%% ============================================================
% TITLE
% =============================================================

titleLabel = uilabel(fig, ...
    'Text','HIGH ALTITUDE ANTI-DRONE DETECTION & TRACKING SYSTEM', ...
    'Position',[350 755 750 35], ...
    'FontSize',20, ...
    'FontWeight','bold', ...
    'FontColor',[0.2 0.9 0.8], ...
    'HorizontalAlignment','center');

subtitleLabel = uilabel(fig, ...
    'Text','MATLAB Real-Time Simulation Dashboard', ...
    'Position',[500 725 500 25], ...
    'FontSize',13, ...
    'FontColor',[0.75 0.8 0.85], ...
    'HorizontalAlignment','center');

%% ============================================================
% RADAR PANEL
% =============================================================

radarPanel = uipanel(fig, ...
    'Title','RADAR / SENSOR VIEW', ...
    'Position',[20 350 600 350], ...
    'BackgroundColor',[0.07 0.09 0.13], ...
    'ForegroundColor',[0.2 0.9 0.8], ...
    'FontSize',13, ...
    'FontWeight','bold');

radarAx = polaraxes(radarPanel, ...
    'Position',[0.08 0.05 0.85 0.85]);

radarAx.ThetaZeroLocation = 'top';
radarAx.ThetaDir = 'clockwise';
radarAx.RLim = [0 maxRadarRange];

title(radarAx,'LIVE RADAR DISPLAY', ...
    'Color',[0.2 0.9 0.8], ...
    'FontSize',13);

hold(radarAx,'on');

% Radar rings
thetaRing = linspace(0,2*pi,360);

for r = 1000:1000:maxRadarRange
    polarplot(radarAx,thetaRing,...
        r*ones(size(thetaRing)),...
        'LineStyle','--',...
        'Color',[0.25 0.4 0.4]);
end

%% Radar sweep

sweepLine = polarplot(radarAx,...
    [0 0],[0 maxRadarRange],...
    'LineWidth',2,...
    'Color',[0.2 0.9 0.8]);

%% Drone marker

droneMarker = polarscatter(radarAx,...
    0,0,100,...
    'filled');

%% ============================================================
% LIVE TRACKING PANEL
% =============================================================

trackPanel = uipanel(fig, ...
    'Title','LIVE TARGET TRACKING', ...
    'Position',[640 350 740 350], ...
    'BackgroundColor',[0.07 0.09 0.13], ...
    'ForegroundColor',[0.2 0.9 0.8], ...
    'FontSize',13, ...
    'FontWeight','bold');

trackAx = uiaxes(trackPanel, ...
    'Position',[50 50 640 250]);

trackAx.BackgroundColor = [0.03 0.04 0.06];

xlabel(trackAx,'Time (s)');
ylabel(trackAx,'Range (m)');
title(trackAx,'Drone Range Tracking');

grid(trackAx,'on');

hold(trackAx,'on');

trueLine = plot(trackAx,...
    NaN,NaN,...
    'LineWidth',2);

measuredLine = plot(trackAx,...
    NaN,NaN,...
    '.',...
    'MarkerSize',10);

trackedLine = plot(trackAx,...
    NaN,NaN,...
    'LineWidth',2);

legend(trackAx,...
    {'True Range','Measurements','Tracked Range'},...
    'Location','northwest');

%% ============================================================
% CONTROL PANEL
% =============================================================

controlPanel = uipanel(fig,...
    'Title','SIMULATION CONTROL',...
    'Position',[20 40 400 280],...
    'BackgroundColor',[0.07 0.09 0.13],...
    'ForegroundColor',[0.2 0.9 0.8],...
    'FontSize',13,...
    'FontWeight','bold');

%% Altitude label

altitudeLabel = uilabel(controlPanel,...
    'Text','Altitude: 3.0 km',...
    'Position',[30 200 330 30],...
    'FontSize',16,...
    'FontWeight','bold',...
    'FontColor',[0.9 0.9 0.9]);

%% Altitude slider

altitudeSlider = uislider(controlPanel,...
    'Position',[40 180 320 3],...
    'Limits',[1000 8000],...
    'Value',3000);

altitudeSlider.ValueChangedFcn = @altitudeChanged;

%% Slider labels

uilabel(controlPanel,...
    'Text','1 km',...
    'Position',[30 145 50 25],...
    'FontColor','white');

uilabel(controlPanel,...
    'Text','8 km',...
    'Position',[325 145 50 25],...
    'FontColor','white');

%% Start button

startButton = uibutton(controlPanel,...
    'push',...
    'Text','▶ START',...
    'Position',[30 85 100 40],...
    'FontSize',13,...
    'FontWeight','bold',...
    'ButtonPushedFcn',@startSimulation);

%% Stop button

stopButton = uibutton(controlPanel,...
    'push',...
    'Text','⏸ STOP',...
    'Position',[145 85 100 40],...
    'FontSize',13,...
    'FontWeight','bold',...
    'ButtonPushedFcn',@stopSimulation);

%% Reset button

resetButton = uibutton(controlPanel,...
    'push',...
    'Text','↻ RESET',...
    'Position',[260 85 100 40],...
    'FontSize',13,...
    'FontWeight','bold',...
    'ButtonPushedFcn',@resetSimulation);

%% ============================================================
% STATUS PANEL
% =============================================================

statusPanel = uipanel(fig,...
    'Title','SYSTEM STATUS',...
    'Position',[440 40 450 280],...
    'BackgroundColor',[0.07 0.09 0.13],...
    'ForegroundColor',[0.2 0.9 0.8],...
    'FontSize',13,...
    'FontWeight','bold');

%% Detection status

uilabel(statusPanel,...
    'Text','DETECTION STATUS',...
    'Position',[30 215 180 25],...
    'FontSize',12,...
    'FontWeight','bold',...
    'FontColor',[0.7 0.75 0.8]);

detectionStatus = uilabel(statusPanel,...
    'Text','● DETECTED',...
    'Position',[210 210 200 35],...
    'FontSize',18,...
    'FontWeight','bold',...
    'FontColor',[0.2 1 0.4]);

%% Range

uilabel(statusPanel,...
    'Text','Target Range:',...
    'Position',[30 170 150 25],...
    'FontColor','white');

rangeValue = uilabel(statusPanel,...
    'Text','0 m',...
    'Position',[210 170 180 25],...
    'FontSize',14,...
    'FontWeight','bold',...
    'FontColor','white');

%% Detection probability

uilabel(statusPanel,...
    'Text','Detection Probability:',...
    'Position',[30 135 170 25],...
    'FontColor','white');

probabilityValue = uilabel(statusPanel,...
    'Text','0 %',...
    'Position',[210 135 180 25],...
    'FontSize',14,...
    'FontWeight','bold',...
    'FontColor','white');

%% Tracking error

uilabel(statusPanel,...
    'Text','Tracking Error:',...
    'Position',[30 100 150 25],...
    'FontColor','white');

errorValue = uilabel(statusPanel,...
    'Text','0 m',...
    'Position',[210 100 180 25],...
    'FontSize',14,...
    'FontWeight','bold',...
    'FontColor','white');

%% Performance

uilabel(statusPanel,...
    'Text','System Performance:',...
    'Position',[30 65 170 25],...
    'FontColor','white');

performanceValue = uilabel(statusPanel,...
    'Text','0 %',...
    'Position',[210 65 180 25],...
    'FontSize',14,...
    'FontWeight','bold',...
    'FontColor','white');

%% ============================================================
% ENVIRONMENT PANEL
% =============================================================

environmentPanel = uipanel(fig,...
    'Title','HIGH-ALTITUDE ENVIRONMENT',...
    'Position',[910 40 470 280],...
    'BackgroundColor',[0.07 0.09 0.13],...
    'ForegroundColor',[0.2 0.9 0.8],...
    'FontSize',13,...
    'FontWeight','bold');

%% Temperature

uilabel(environmentPanel,...
    'Text','Temperature',...
    'Position',[30 205 160 25],...
    'FontColor','white');

temperatureValue = uilabel(environmentPanel,...
    'Text','-4.5 °C',...
    'Position',[250 205 170 25],...
    'FontSize',16,...
    'FontWeight','bold',...
    'FontColor','white');

%% Pressure

uilabel(environmentPanel,...
    'Text','Atmospheric Pressure',...
    'Position',[30 160 180 25],...
    'FontColor','white');

pressureValue = uilabel(environmentPanel,...
    'Text','70 kPa',...
    'Position',[250 160 170 25],...
    'FontSize',16,...
    'FontWeight','bold',...
    'FontColor','white');

%% Altitude

uilabel(environmentPanel,...
    'Text','Current Altitude',...
    'Position',[30 115 180 25],...
    'FontColor','white');

currentAltitudeValue = uilabel(environmentPanel,...
    'Text','3.00 km',...
    'Position',[250 115 170 25],...
    'FontSize',16,...
    'FontWeight','bold',...
    'FontColor','white');

%% System mode

uilabel(environmentPanel,...
    'Text','Operating Mode',...
    'Position',[30 70 180 25],...
    'FontColor','white');

modeValue = uilabel(environmentPanel,...
    'Text','HIGH ALTITUDE TRACKING',...
    'Position',[200 70 230 25],...
    'FontSize',12,...
    'FontWeight','bold',...
    'FontColor',[0.2 0.9 0.8]);

%% ============================================================
% TIMER
% =============================================================

simulationTimer = timer(...
    'ExecutionMode','fixedRate',...
    'Period',0.15,...
    'TimerFcn',@updateSimulation);

%% ============================================================
% CALLBACK: ALTITUDE
% =============================================================

function altitudeChanged(src,~)

    altitude = src.Value;

    altitudeLabel.Text = ...
        sprintf('Altitude: %.2f km',altitude/1000);

    currentAltitudeValue.Text = ...
        sprintf('%.2f km',altitude/1000);

end

%% ============================================================
% CALLBACK: START
% =============================================================

function startSimulation(~,~)

    if ~isRunning

        isRunning = true;

        start(simulationTimer);

    end

end

%% ============================================================
% CALLBACK: STOP
% =============================================================

function stopSimulation(~,~)

    isRunning = false;

    if strcmp(simulationTimer.Running,'on')
        stop(simulationTimer);
    end

end

%% ============================================================
% CALLBACK: RESET
% =============================================================

function resetSimulation(~,~)

    stopSimulation();

    timeValue = 0;
    angle = 0;

    droneX = 1000;
    droneY = 500;

    rangeHistory = [];
    trackHistory = [];
    timeHistory = [];

    trueLine.XData = [];
    trueLine.YData = [];

    measuredLine.XData = [];
    measuredLine.YData = [];

    trackedLine.XData = [];
    trackedLine.YData = [];

    altitudeSlider.Value = 3000;

    altitude = 3000;

    altitudeLabel.Text = 'Altitude: 3.00 km';

    detectionStatus.Text = '● READY';

    rangeValue.Text = '0 m';

    probabilityValue.Text = '0 %';

    errorValue.Text = '0 m';

    performanceValue.Text = '0 %';

end

%% ============================================================
% MAIN SIMULATION UPDATE
% =============================================================

function updateSimulation(~,~)

    %% Time

    timeValue = timeValue + 0.15;

    %% Drone motion

    droneX = 1000 + 12*timeValue + ...
        150*sin(0.12*timeValue);

    droneY = 500 + 8*timeValue + ...
        120*cos(0.10*timeValue);

    %% Range

    currentRange = sqrt( ...
        droneX^2 + ...
        droneY^2 + ...
        altitude^2);

    %% Bearing

    bearing = atan2(droneX,droneY);

    if bearing < 0
        bearing = bearing + 2*pi;
    end

    %% Radar sweep

    angle = angle + 0.15;

    if angle > 2*pi
        angle = 0;
    end

    sweepLine.ThetaData = [angle angle];
    sweepLine.RData = [0 maxRadarRange];

    %% Detection probability

    rangeFactor = max(0,1-currentRange/maxRadarRange);

    pressure = ...
        101.325*(1-2.25577e-5*altitude)^5.25588;

    pressureFactor = pressure/101.325;

    environmentFactor = ...
        0.85 + 0.15*pressureFactor;

    detectionProbability = ...
        (0.25 + 0.75*rangeFactor*environmentFactor);

    detectionProbability = ...
        min(max(detectionProbability,0),1);

    %% Detection decision

    detected = detectionProbability > 0.35;

    %% Radar target

    if detected

        radarRange = min(currentRange,maxRadarRange);

        droneMarker.ThetaData = bearing;
        droneMarker.RData = radarRange;

        detectionStatus.Text = '● DETECTED';
        detectionStatus.FontColor = [0.2 1 0.4];

    else

        droneMarker.ThetaData = bearing;
        droneMarker.RData = min(currentRange,maxRadarRange);

        detectionStatus.Text = '● LOW CONFIDENCE';
        detectionStatus.FontColor = [1 0.7 0.2];

    end

    %% Measurement noise

    measurement = currentRange + 30*randn;

    %% Simple tracking filter

    if isempty(trackHistory)

        trackedRange = measurement;

    else

        trackedRange = ...
            0.25*measurement + ...
            0.75*trackHistory(end);

    end

    %% Tracking error

    currentError = abs(trackedRange-currentRange);

    %% Save data

    timeHistory(end+1) = timeValue;

    rangeHistory(end+1) = currentRange;

    trackHistory(end+1) = trackedRange;

    %% Update tracking plot

    trueLine.XData = timeHistory;
    trueLine.YData = rangeHistory;

    measuredLine.XData = timeHistory;
    measuredLine.YData = ...
        rangeHistory + 30*randn(size(rangeHistory));

    trackedLine.XData = timeHistory;
    trackedLine.YData = trackHistory;

    xlim(trackAx,[max(0,timeValue-30) max(30,timeValue)]);

    %% Update values

    rangeValue.Text = ...
        sprintf('%.0f m',currentRange);

    probabilityValue.Text = ...
        sprintf('%.1f %%',100*detectionProbability);

    errorValue.Text = ...
        sprintf('%.1f m',currentError);

    %% Performance

    performance = ...
        100*detectionProbability;

    performanceValue.Text = ...
        sprintf('%.1f %%',performance);

    %% Environment

    temperature = ...
        15 - 6.5*(altitude/1000);

    temperatureValue.Text = ...
        sprintf('%.1f °C',temperature);

    pressureValue.Text = ...
        sprintf('%.1f kPa',pressure);

    currentAltitudeValue.Text = ...
        sprintf('%.2f km',altitude/1000);

    %% Update display

    drawnow limitrate;

end

%% ============================================================
% CLOSE FUNCTION
% =============================================================

fig.CloseRequestFcn = @closeDashboard;

function closeDashboard(~,~)

    try
        stop(simulationTimer);
        delete(simulationTimer);
    catch
    end

    delete(fig);

end

end