function main
% MAIN - Integrated Anti-Drone Radar Simulation
% Combines the original radar simulation with the dashboard UI.
%
% Interpretation of requested layout:
%   Page 1 : Original Figures 1-4 in one page
%   Page 2 : Original Figures 7-11 in one page
%   Figure 13: Converted into the GUI-style live PPI dashboard
%
% Original Figure 6 (measurement-noise plot) and Figure 12
% (real-time radar status display) are removed.
%
% NOTE:
% The request listed "7,8,8,10,11" and also said to eliminate Figure 7.
% This version interprets the repeated "8" as "9" and therefore keeps
% Figures 7-11 together, which is the internally consistent layout.

%% ================================================================
%  PARAMETERS AND SIMULATION
% ================================================================

clc;
close all;

radarMaxRange = 1000;
protectedRadius = 300;
approachRadius = 500;

dt = 0.1;
simulationTime = 30;
time = 0:dt:simulationTime;

% Initial drone position
x = 500;
y = -400;
z = 100;

% Drone velocity
vx = -8;
vy = 10;
vz = 0;

%% Drone motion
X = zeros(size(time));
Y = zeros(size(time));
Z = zeros(size(time));

for k = 1:length(time)
    X(k) = x;
    Y(k) = y;
    Z(k) = z;

    x = x + vx*dt;
    y = y + vy*dt;
    z = z + vz*dt;
end

%% Radar quantities
R = sqrt(X.^2 + Y.^2 + Z.^2);
Azimuth = atan2d(Y,X);

HorizontalRange = sqrt(X.^2 + Y.^2);
Elevation = atan2d(Z,HorizontalRange);

%% Radar measurement noise
rangeNoise = 5;
azimuthNoise = 1;
elevationNoise = 1;

rng('default');
Rmeasured = R + rangeNoise*randn(size(R));
AzMeasured = Azimuth + azimuthNoise*randn(size(Azimuth));
ElMeasured = Elevation + elevationNoise*randn(size(Elevation));

%% Convert radar measurements to Cartesian coordinates
xMeasured = Rmeasured .* cosd(ElMeasured) .* cosd(AzMeasured);
yMeasured = Rmeasured .* cosd(ElMeasured) .* sind(AzMeasured);
zMeasured = Rmeasured .* sind(ElMeasured); %#ok<NASGU>

%% ================================================================
% KALMAN FILTER
% ================================================================

% State = [X position; Y position; X velocity; Y velocity]
state = [xMeasured(1); yMeasured(1); 0; 0];

F = [1 0 dt 0;
     0 1 0 dt;
     0 0 1  0;
     0 0 0  1];

H = [1 0 0 0;
     0 1 0 0];

P = eye(4)*100;

Rk = [25 0;
      0 25];

Q = eye(4);

Xestimated = zeros(size(time));
Yestimated = zeros(size(time));
Vxestimated = zeros(size(time));
Vyestimated = zeros(size(time));

for k = 1:length(time)

    statePredicted = F*state;
    PPredicted = F*P*F' + Q;

    measurement = [xMeasured(k); yMeasured(k)];

    K = PPredicted*H'/(H*PPredicted*H' + Rk);

    state = statePredicted + K*(measurement - H*statePredicted);

    P = (eye(4)-K*H)*PPredicted;

    Xestimated(k) = state(1);
    Yestimated(k) = state(2);

    Vxestimated(k) = state(3);
    Vyestimated(k) = state(4);
end

%% Protected-zone quantities
estimatedRange = sqrt(Xestimated.^2 + Yestimated.^2);
estimatedAzimuth = atan2d(Yestimated,Xestimated);

insideProtectedZone = estimatedRange <= protectedRadius;
intrusionIndex = find(insideProtectedZone,1);

if isempty(intrusionIndex)
    intrusionTime = NaN;
    intrusionRange = NaN;
else
    intrusionTime = time(intrusionIndex);
    intrusionRange = estimatedRange(intrusionIndex);
end

%% ================================================================
% PAGE 1 - ORIGINAL FIGURES 1-4
% ================================================================

page1 = figure( ...
    'Name','Anti-Drone Radar - Basic Parameters', ...
    'NumberTitle','off', ...
    'Color','w', ...
    'Position',[50 80 1200 750]);

tl1 = tiledlayout(page1,2,2,'TileSpacing','compact','Padding','compact');

% Figure 1 - Drone trajectory
ax1 = nexttile(tl1);
plot3(ax1,X,Y,Z,'LineWidth',2);
grid(ax1,'on');
xlabel(ax1,'X (m)');
ylabel(ax1,'Y (m)');
zlabel(ax1,'Altitude (m)');
title(ax1,'1. Simulated Drone Trajectory');
axis(ax1,'equal');

% Figure 2 - Range
ax2 = nexttile(tl1);
plot(ax2,time,R,'LineWidth',2);
grid(ax2,'on');
xlabel(ax2,'Time (s)');
ylabel(ax2,'Range (m)');
title(ax2,'2. Drone Range from Radar');

% Figure 3 - Azimuth
ax3 = nexttile(tl1);
plot(ax3,time,Azimuth,'LineWidth',2);
grid(ax3,'on');
xlabel(ax3,'Time (s)');
ylabel(ax3,'Azimuth (degrees)');
title(ax3,'3. Drone Azimuth');

% Figure 4 - Elevation
ax4 = nexttile(tl1);
plot(ax4,time,Elevation,'LineWidth',2);
grid(ax4,'on');
xlabel(ax4,'Time (s)');
ylabel(ax4,'Elevation (degrees)');
title(ax4,'4. Drone Elevation');

%% ================================================================
% PAGE 2 - ORIGINAL FIGURES 7-11
% Figure 6 removed.
% Figure 7-11 are placed on one page.
% ================================================================

page2 = figure( ...
    'Name','Anti-Drone Radar - Tracking Analysis', ...
    'NumberTitle','off', ...
    'Color','w', ...
    'Position',[80 40 1300 850]);

tl2 = tiledlayout(page2,3,2,'TileSpacing','compact','Padding','compact');

% Figure 7 - True / Radar / Kalman + protected zone
ax7 = nexttile(tl2);
hold(ax7,'on');
grid(ax7,'on');
axis(ax7,'equal');

plot(ax7,X,Y,'LineWidth',2);
plot(ax7,xMeasured,yMeasured,'.');
plot(ax7,Xestimated,Yestimated,'LineWidth',2);

thetaZone = linspace(0,2*pi,360);
plot(ax7,protectedRadius*cos(thetaZone), ...
    protectedRadius*sin(thetaZone),'--','LineWidth',2);

plot(ax7,0,0,'+','MarkerSize',12,'LineWidth',3);

if ~isempty(intrusionIndex)
    plot(ax7,Xestimated(intrusionIndex),Yestimated(intrusionIndex), ...
        'o','MarkerSize',12,'LineWidth',3);
    text(ax7,Xestimated(intrusionIndex),Yestimated(intrusionIndex), ...
        '  ALERT: Zone Entry','FontSize',10,'FontWeight','bold');
end

xlabel(ax7,'X Position (m)');
ylabel(ax7,'Y Position (m)');
title(ax7,'7. Drone Tracking and Protected Zone');
legend(ax7,{'True Drone','Radar Measurements','Kalman Estimate', ...
    'Protected Zone','Radar'},'Location','best');

% Figure 8 - True vs measured vs estimated
ax8 = nexttile(tl2);
plot(ax8,X,Y,'LineWidth',2);
hold(ax8,'on');
plot(ax8,xMeasured,yMeasured,'.');
plot(ax8,Xestimated,Yestimated,'LineWidth',2);
grid(ax8,'on');
xlabel(ax8,'X Position (m)');
ylabel(ax8,'Y Position (m)');
legend(ax8,{'True Drone','Radar Measurements','Kalman Estimate'}, ...
    'Location','best');
title(ax8,'8. Drone Tracking using Kalman Filter');
axis(ax8,'equal');

% Figure 9 - Velocity
ax9 = nexttile(tl2);
plot(ax9,time,Vxestimated,'LineWidth',2);
hold(ax9,'on');
plot(ax9,time,Vyestimated,'LineWidth',2);
grid(ax9,'on');
xlabel(ax9,'Time (s)');
ylabel(ax9,'Velocity (m/s)');
legend(ax9,{'Estimated Vx','Estimated Vy'},'Location','best');
title(ax9,'9. Estimated Drone Velocity');

% Figure 10 - Radar + Kalman tracking
ax10 = polaraxes(tl2);
ax10.Layout.Tile = 4;
hold(ax10,'on');
rlim(ax10,[0 radarMaxRange]);
title(ax10,'10. RADAR - DRONE TRACKING');

% Figure 11 - Radar tracking history
ax11 = polaraxes(tl2);
ax11.Layout.Tile = 5;
hold(ax11,'on');
rlim(ax11,[0 radarMaxRange]);
title(ax11,'11. RADAR - KALMAN TRACKING');

% Plot Figure 10 final state
polarplot(ax10,deg2rad(AzMeasured),Rmeasured,'.');
polarplot(ax10,estimatedAzimuth*pi/180,estimatedRange,'x','LineWidth',1.5);

% Plot Figure 11 history
polarplot(ax11,estimatedAzimuth*pi/180,estimatedRange,'LineWidth',2);

% Fifth tile intentionally left as a compact information summary
axInfo = nexttile(tl2);
axis(axInfo,'off');

if isempty(intrusionIndex)
    intrusionText = 'No protected-zone intrusion';
else
    intrusionText = sprintf(['Protected-zone intrusion detected\n' ...
        'Time: %.2f s\nRange: %.2f m'],intrusionTime,intrusionRange);
end

text(axInfo,0.05,0.85,'TRACKING SUMMARY', ...
    'FontSize',15,'FontWeight','bold');
text(axInfo,0.05,0.65,sprintf('Radar range: %.0f m',radarMaxRange), ...
    'FontSize',12);
text(axInfo,0.05,0.52,sprintf('Protected radius: %.0f m',protectedRadius), ...
    'FontSize',12);
text(axInfo,0.05,0.39,sprintf('Approach radius: %.0f m',approachRadius), ...
    'FontSize',12);
text(axInfo,0.05,0.23,intrusionText,'FontSize',12,'FontWeight','bold');

%% ================================================================
% FIGURE 13 - UI-BASED LIVE ANTI-DRONE RADAR DASHBOARD
% Figure 12 is removed.
% This uses the UI structure/features from the supplied GUI script,
% but displays the actual Main simulation/Kalman data.
% ================================================================

createDashboard( ...
    time,R,Azimuth,Rmeasured,AzMeasured, ...
    Xestimated,Yestimated,estimatedRange,estimatedAzimuth, ...
    Vxestimated,Vyestimated, ...
    radarMaxRange,protectedRadius,approachRadius, ...
    dt,intrusionIndex);

end


%% =================================================================
% DASHBOARD FUNCTION
% =================================================================
function createDashboard(time,R,Azimuth,Rmeasured,AzMeasured, ...
    Xestimated,Yestimated,estimatedRange,estimatedAzimuth, ...
    Vxestimated,Vyestimated, ...
    radarMaxRange,protectedRadius,approachRadius,dt,intrusionIndex)

fig = uifigure( ...
    'Name','High Altitude Anti-Drone Detection & Tracking System', ...
    'Position',[50 40 1450 850], ...
    'Color',[0.05 0.07 0.10]);

% ---------------------------------------------------------------
% Title
% ---------------------------------------------------------------
uilabel(fig, ...
    'Text','HIGH ALTITUDE ANTI-DRONE DETECTION & TRACKING SYSTEM', ...
    'Position',[350 800 750 35], ...
    'FontSize',20, ...
    'FontWeight','bold', ...
    'FontColor',[0.2 0.9 0.8], ...
    'HorizontalAlignment','center');

uilabel(fig, ...
    'Text','MATLAB Radar Detection • Measurement • Kalman Tracking • PPI', ...
    'Position',[450 775 550 25], ...
    'FontSize',13, ...
    'FontColor',[0.75 0.8 0.85], ...
    'HorizontalAlignment','center');

% ---------------------------------------------------------------
% Radar panel
% ---------------------------------------------------------------
radarPanel = uipanel(fig, ...
    'Title','RADAR / SENSOR VIEW', ...
    'Position',[20 380 610 370], ...
    'BackgroundColor',[0.07 0.09 0.13], ...
    'ForegroundColor',[0.2 0.9 0.8], ...
    'FontSize',13, ...
    'FontWeight','bold');

radarAx = polaraxes(radarPanel, ...
    'Position',[0.07 0.06 0.88 0.86]);

radarAx.ThetaZeroLocation = 'top';
radarAx.ThetaDir = 'clockwise';
radarAx.RLim = [0 radarMaxRange];

hold(radarAx,'on');

thetaRing = linspace(-pi,pi,360);

for rr = [250 500 750 1000]
    if rr <= radarMaxRange
        polarplot(radarAx,thetaRing, ...
            rr*ones(size(thetaRing)), ...
            '--','Color',[0.25 0.4 0.4]);
    end
end

polarplot(radarAx,thetaRing, ...
    protectedRadius*ones(size(thetaRing)), ...
    '--','LineWidth',2);

sweepLine = polarplot(radarAx,[0 0],[0 radarMaxRange], ...
    'LineWidth',2,'Color',[0.2 0.9 0.8]);

measurementMarker = polarscatter(radarAx,0,0,60,'filled');
trackMarker = polarscatter(radarAx,0,0,80,'x','LineWidth',2);

title(radarAx,'LIVE RADAR PPI', ...
    'Color',[0.2 0.9 0.8],'FontSize',13);

% ---------------------------------------------------------------
% Live tracking panel
% ---------------------------------------------------------------
trackPanel = uipanel(fig, ...
    'Title','LIVE TARGET TRACKING', ...
    'Position',[650 380 780 370], ...
    'BackgroundColor',[0.07 0.09 0.13], ...
    'ForegroundColor',[0.2 0.9 0.8], ...
    'FontSize',13, ...
    'FontWeight','bold');

trackAx = uiaxes(trackPanel, ...
    'Position',[45 55 690 270]);

trackAx.BackgroundColor = [0.03 0.04 0.06];
xlabel(trackAx,'Time (s)');
ylabel(trackAx,'Range (m)');
title(trackAx,'True Range / Radar Measurement / Kalman Track');
grid(trackAx,'on');
hold(trackAx,'on');

trueLine = plot(trackAx,NaN,NaN,'LineWidth',2);
measuredLine = plot(trackAx,NaN,NaN,'.','MarkerSize',8);
trackedLine = plot(trackAx,NaN,NaN,'LineWidth',2);

legend(trackAx, ...
    {'True Range','Radar Measurement','Kalman Tracked Range'}, ...
    'Location','northwest');

% ---------------------------------------------------------------
% Control panel
% ---------------------------------------------------------------
controlPanel = uipanel(fig, ...
    'Title','SIMULATION CONTROL', ...
    'Position',[20 40 410 300], ...
    'BackgroundColor',[0.07 0.09 0.13], ...
    'ForegroundColor',[0.2 0.9 0.8], ...
    'FontSize',13, ...
    'FontWeight','bold');

uilabel(controlPanel, ...
    'Text','Simulation Playback', ...
    'Position',[30 230 220 25], ...
    'FontSize',14, ...
    'FontWeight','bold', ...
    'FontColor','white');

playSlider = uislider(controlPanel, ...
    'Position',[35 200 335 3], ...
    'Limits',[1 length(time)], ...
    'Value',1);

uilabel(controlPanel,'Text','Start', ...
    'Position',[30 165 50 20],'FontColor','white');

uilabel(controlPanel,'Text','End', ...
    'Position',[350 165 40 20],'FontColor','white');

startButton = uibutton(controlPanel,'push', ...
    'Text','▶ START', ...
    'Position',[30 95 105 40], ...
    'FontSize',13, ...
    'FontWeight','bold');

stopButton = uibutton(controlPanel,'push', ...
    'Text','⏸ STOP', ...
    'Position',[150 95 105 40], ...
    'FontSize',13, ...
    'FontWeight','bold');

resetButton = uibutton(controlPanel,'push', ...
    'Text','↻ RESET', ...
    'Position',[270 95 105 40], ...
    'FontSize',13, ...
    'FontWeight','bold');

timeDisplay = uilabel(controlPanel, ...
    'Text','Time: 0.0 s', ...
    'Position',[30 45 330 25], ...
    'FontSize',13, ...
    'FontWeight','bold', ...
    'FontColor',[0.2 0.9 0.8]);

% ---------------------------------------------------------------
% Status panel
% ---------------------------------------------------------------
statusPanel = uipanel(fig, ...
    'Title','SYSTEM STATUS', ...
    'Position',[450 40 470 300], ...
    'BackgroundColor',[0.07 0.09 0.13], ...
    'ForegroundColor',[0.2 0.9 0.8], ...
    'FontSize',13, ...
    'FontWeight','bold');

uilabel(statusPanel,'Text','DETECTION STATUS', ...
    'Position',[30 235 180 25], ...
    'FontSize',12,'FontWeight','bold','FontColor',[0.7 0.75 0.8]);

detectionStatus = uilabel(statusPanel, ...
    'Text','● READY', ...
    'Position',[215 230 210 35], ...
    'FontSize',18,'FontWeight','bold', ...
    'FontColor',[0.2 1 0.4]);

uilabel(statusPanel,'Text','Target Range:', ...
    'Position',[30 190 150 25],'FontColor','white');

rangeValue = uilabel(statusPanel,'Text','0 m', ...
    'Position',[220 190 200 25], ...
    'FontSize',14,'FontWeight','bold','FontColor','white');

uilabel(statusPanel,'Text','Detection Probability:', ...
    'Position',[30 150 180 25],'FontColor','white');

probabilityValue = uilabel(statusPanel,'Text','0 %', ...
    'Position',[220 150 200 25], ...
    'FontSize',14,'FontWeight','bold','FontColor','white');

uilabel(statusPanel,'Text','Tracking Error:', ...
    'Position',[30 110 150 25],'FontColor','white');

errorValue = uilabel(statusPanel,'Text','0 m', ...
    'Position',[220 110 200 25], ...
    'FontSize',14,'FontWeight','bold','FontColor','white');

uilabel(statusPanel,'Text','Target Speed:', ...
    'Position',[30 70 150 25],'FontColor','white');

speedValue = uilabel(statusPanel,'Text','0 m/s', ...
    'Position',[220 70 200 25], ...
    'FontSize',14,'FontWeight','bold','FontColor','white');

% ---------------------------------------------------------------
% Environment / radar information panel
% ---------------------------------------------------------------
environmentPanel = uipanel(fig, ...
    'Title','RADAR / ENVIRONMENT INFORMATION', ...
    'Position',[940 40 490 300], ...
    'BackgroundColor',[0.07 0.09 0.13], ...
    'ForegroundColor',[0.2 0.9 0.8], ...
    'FontSize',13, ...
    'FontWeight','bold');

uilabel(environmentPanel,'Text','Radar Maximum Range', ...
    'Position',[30 235 190 25],'FontColor','white');

radarRangeValue = uilabel(environmentPanel, ...
    'Text',sprintf('%.0f m',radarMaxRange), ...
    'Position',[280 235 170 25], ...
    'FontSize',15,'FontWeight','bold','FontColor','white');

uilabel(environmentPanel,'Text','Protected Zone', ...
    'Position',[30 195 190 25],'FontColor','white');

protectedValue = uilabel(environmentPanel, ...
    'Text',sprintf('%.0f m',protectedRadius), ...
    'Position',[280 195 170 25], ...
    'FontSize',15,'FontWeight','bold','FontColor','white');

uilabel(environmentPanel,'Text','Approach Zone', ...
    'Position',[30 155 190 25],'FontColor','white');

approachValue = uilabel(environmentPanel, ...
    'Text',sprintf('%.0f m',approachRadius), ...
    'Position',[280 155 170 25], ...
    'FontSize',15,'FontWeight','bold','FontColor','white');

uilabel(environmentPanel,'Text','Current Azimuth', ...
    'Position',[30 115 190 25],'FontColor','white');

azimuthValue = uilabel(environmentPanel, ...
    'Text','0°', ...
    'Position',[280 115 170 25], ...
    'FontSize',15,'FontWeight','bold','FontColor','white');

uilabel(environmentPanel,'Text','Operating Mode', ...
    'Position',[30 75 190 25],'FontColor','white');

modeValue = uilabel(environmentPanel, ...
    'Text','KALMAN TRACKING', ...
    'Position',[250 75 200 25], ...
    'FontSize',12,'FontWeight','bold', ...
    'FontColor',[0.2 0.9 0.8]);

if ~isempty(intrusionIndex)
    intrusionInfo = sprintf('Zone entry at %.2f s',time(intrusionIndex));
else
    intrusionInfo = 'No zone entry in simulation';
end

uilabel(environmentPanel, ...
    'Text',intrusionInfo, ...
    'Position',[30 35 420 25], ...
    'FontSize',11, ...
    'FontWeight','bold', ...
    'FontColor',[1 0.75 0.2]);

% ---------------------------------------------------------------
% Playback callbacks
% ---------------------------------------------------------------
isRunning = false;
currentIndex = 1;
timerObj = timer( ...
    'ExecutionMode','fixedRate', ...
    'Period',dt, ...
    'TimerFcn',@updateDashboard);

playSlider.ValueChangedFcn = @sliderChanged;
startButton.ButtonPushedFcn = @startSimulation;
stopButton.ButtonPushedFcn = @stopSimulation;
resetButton.ButtonPushedFcn = @resetSimulation;
fig.CloseRequestFcn = @closeDashboard;

updateDashboard();

    function startSimulation(~,~)
        if ~isRunning
            isRunning = true;
            if strcmp(timerObj.Running,'off')
                start(timerObj);
            end
        end
    end

    function stopSimulation(~,~)
        isRunning = false;
        if strcmp(timerObj.Running,'on')
            stop(timerObj);
        end
    end

    function resetSimulation(~,~)
        stopSimulation();
        currentIndex = 1;
        playSlider.Value = 1;
        updateDashboard();
    end

    function sliderChanged(src,~)
        stopSimulation();
        currentIndex = max(1,min(length(time),round(src.Value)));
        updateDashboard();
    end

    function updateDashboard(~,~)

        if currentIndex > length(time)
            currentIndex = 1;
            isRunning = false;
            if strcmp(timerObj.Running,'on')
                stop(timerObj);
            end
        end

        k = currentIndex;

        % Current radar data
        currentMeasuredRange = Rmeasured(k);
        currentMeasuredAzimuth = deg2rad(AzMeasured(k));

        currentRange = estimatedRange(k);
        currentAzimuth = deg2rad(estimatedAzimuth(k));

        currentSpeed = sqrt(Vxestimated(k)^2 + Vyestimated(k)^2);

        % Simple confidence metric based on range
        rangeFactor = max(0,1-currentRange/radarMaxRange);
        detectionProbability = min(max(0.25 + 0.75*rangeFactor,0),1);

        % Tracking error
        currentError = abs(currentRange - R(k));

        % Status
        if currentRange <= protectedRadius
            status = '● INTRUSION';
            statusColor = [1 0.25 0.25];
        elseif currentRange <= approachRadius
            status = '● APPROACHING';
            statusColor = [1 0.7 0.2];
        else
            status = '● DETECTED';
            statusColor = [0.2 1 0.4];
        end

        % Radar sweep
        sweepAngle = deg2rad(mod(time(k)*60,360));

        sweepLine.ThetaData = [sweepAngle sweepAngle];
        sweepLine.RData = [0 radarMaxRange];

        measurementMarker.ThetaData = currentMeasuredAzimuth;
        measurementMarker.RData = min(abs(currentMeasuredRange),radarMaxRange);

        trackMarker.ThetaData = currentAzimuth;
        trackMarker.RData = min(abs(currentRange),radarMaxRange);

        % Tracking plot
        trueLine.XData = time(1:k);
        trueLine.YData = R(1:k);

        measuredLine.XData = time(1:k);
        measuredLine.YData = Rmeasured(1:k);

        trackedLine.XData = time(1:k);
        trackedLine.YData = estimatedRange(1:k);

        xlim(trackAx,[max(0,time(k)-10) max(10,time(k))]);

        % Status values
        detectionStatus.Text = status;
        detectionStatus.FontColor = statusColor;

        rangeValue.Text = sprintf('%.1f m',currentRange);
        probabilityValue.Text = sprintf('%.1f %%',100*detectionProbability);
        errorValue.Text = sprintf('%.1f m',currentError);
        speedValue.Text = sprintf('%.1f m/s',currentSpeed);

        azimuthValue.Text = sprintf('%.1f°',estimatedAzimuth(k));
        timeDisplay.Text = sprintf('Time: %.1f s',time(k));

        % PPI title
        title(radarAx,sprintf( ...
            'LIVE RADAR PPI  |  T001  |  %.1f m  |  %.1f°', ...
            currentRange,estimatedAzimuth(k)), ...
            'Color',[0.2 0.9 0.8], ...
            'FontSize',13);

        drawnow limitrate;

        if isRunning
            currentIndex = currentIndex + 1;
            if currentIndex <= length(time)
                playSlider.Value = currentIndex;
            end
        end
    end

    function closeDashboard(~,~)
        try
            stop(timerObj);
            delete(timerObj);
        catch
        end
        delete(fig);
    end
end
