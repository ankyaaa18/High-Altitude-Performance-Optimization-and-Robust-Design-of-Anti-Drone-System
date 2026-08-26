run('gui.m');
clear;
clc;
close all;

%% Radar and protected-zone parameters

radarMaxRange = 1000;       % Maximum radar detection range (m)

protectedRadius = 300;      % Protected zone radius (m)
%% Simulation parameters
dt = 0.1;          % Time step (seconds)
simulationTime = 30;
time = 0:dt:simulationTime;

%% Initial drone position
x = 500;           % meters
y = -400;          % meters
z = 100;           % meters

%% Drone velocity
vx = -8;           % m/s
vy = 10;           % m/s
vz = 0;            % m/s

%% Store position
X = zeros(size(time));
Y = zeros(size(time));
Z = zeros(size(time));

%% Simulate drone movement
for k = 1:length(time)

    X(k) = x;
    Y(k) = y;
    Z(k) = z;

    % Update position
    x = x + vx*dt;
    y = y + vy*dt;
    z = z + vz*dt;

end

%% Plot drone trajectory

figure;

plot3(X,Y,Z,'LineWidth',2);

grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Altitude (m)');

title('Simulated Drone Trajectory');

axis equal;

%% Calculate radar range

R = sqrt(X.^2 + Y.^2 + Z.^2);

figure;

plot(time,R,'LineWidth',2);

grid on;

xlabel('Time (s)');
ylabel('Range (m)');

title('Drone Range from Radar');

%% Calculate azimuth

Azimuth = atan2d(Y,X);

figure;

plot(time,Azimuth,'LineWidth',2);

grid on;

xlabel('Time (s)');
ylabel('Azimuth (degrees)');

title('Drone Azimuth');

%% Calculate elevation

HorizontalRange = sqrt(X.^2 + Y.^2);

Elevation = atan2d(Z,HorizontalRange);

figure;

plot(time,Elevation,'LineWidth',2);

grid on;

xlabel('Time (s)');
ylabel('Elevation (degrees)');

title('Drone Elevation');

%% Real-time radar display

figure;

ax = polaraxes;

rlim(ax,[0 1000]);

title(ax,'RADAR - DRONE DETECTION');

hold(ax,'on');
for k = 1:length(time)

    currentRange = R(k);
    currentAzimuth = deg2rad(Azimuth(k));

    if currentRange <= radarMaxRange

        p = polarplot(ax,...
            currentAzimuth,...
            currentRange,...
            'o',...
            'MarkerSize',8,...
            'LineWidth',2);

        drawnow;

        pause(dt);

        if k < length(time)
            delete(p);
        end

    else

        pause(dt);

    end

end

%% Radar measurement noise

rangeNoise = 5;       % meters
azimuthNoise = 1;     % degrees
elevationNoise = 1;   % degrees

Rmeasured = R + rangeNoise*randn(size(R));

AzMeasured = Azimuth + ...
    azimuthNoise*randn(size(Azimuth));

ElMeasured = Elevation + ...
    elevationNoise*randn(size(Elevation));

figure;

plot(time,R,'LineWidth',2);
hold on;

plot(time,Rmeasured,'.');

grid on;

xlabel('Time (s)');
ylabel('Range (m)');

legend('True Range','Radar Measurement');

title('Radar Measurement Noise');

%% Convert radar measurements to Cartesian coordinates

xMeasured = Rmeasured .* ...
    cosd(ElMeasured) .* ...
    cosd(AzMeasured);

yMeasured = Rmeasured .* ...
    cosd(ElMeasured) .* ...
    sind(AzMeasured);

zMeasured = Rmeasured .* ...
    sind(ElMeasured);

%% ==============================
%  KALMAN FILTER INITIALIZATION
%  ==============================

% State vector:
% [X position
%  Y position
%  X velocity
%  Y velocity]

state = [xMeasured(1);
    yMeasured(1);
    0;
    0];

% State transition matrix
F = [1 0 dt 0;
    0 1 0 dt;
    0 0 1  0;
    0 0 0  1];

% Measurement matrix
% Radar measures X and Y position
H = [1 0 0 0;
    0 1 0 0];

% Initial uncertainty
P = eye(4) * 100;

% Measurement noise
Rk = [25 0;
    0 25];

% Process noise
Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

%% Store Kalman estimates

Xestimated = zeros(size(time));
Yestimated = zeros(size(time));

Vxestimated = zeros(size(time));
Vyestimated = zeros(size(time));

%% ==============================
%  KALMAN FILTER LOOP
%  ==============================

for k = 1:length(time)

    %% --------------------------
    % Prediction
    % --------------------------

    statePredicted = F * state;

    PPredicted = F * P * F' + Q;


    %% --------------------------
    % Measurement
    % --------------------------

    measurement = [xMeasured(k);
        yMeasured(k)];


    %% --------------------------
    % Kalman Gain
    % --------------------------

    K = PPredicted * H' / ...
        (H * PPredicted * H' + Rk);


    %% --------------------------
    % Correction
    % --------------------------

    state = statePredicted + ...
        K * (measurement - H * statePredicted);


    %% --------------------------
    % Update uncertainty
    % --------------------------

    P = (eye(4) - K * H) * PPredicted;


    %% --------------------------
    % Store results
    % --------------------------

    Xestimated(k) = state(1);
    Yestimated(k) = state(2);

    Vxestimated(k) = state(3);
    Vyestimated(k) = state(4);

end

%% ==========================================
%  PHASE D - PROTECTED ZONE
%  ==========================================

% Protected zone size
protectedRadius = 300;

% Approach zone
approachRadius = 500;

% Calculate filtered target range
estimatedRange = sqrt( ...
    Xestimated.^2 + ...
    Yestimated.^2);

% Calculate filtered target azimuth
estimatedAzimuth = atan2d( ...
    Yestimated, ...
    Xestimated);

%% Find first protected-zone intrusion

intrusionIndex = find( ...
    estimatedRange <= protectedRadius, 1); 

%% ==========================================
% TRUE / RADAR / KALMAN + PROTECTED ZONE
% ==========================================

figure;

hold on;
grid on;
axis equal;

% True drone trajectory
plot(X,Y,...
    'LineWidth',2);

% Radar measurements
plot(xMeasured,...
    yMeasured,...
    '.');

% Kalman estimate
plot(Xestimated,...
    Yestimated,...
    'LineWidth',2);

% Draw protected zone
thetaZone = linspace(0,2*pi,360);

zoneX = protectedRadius*cos(thetaZone);
zoneY = protectedRadius*sin(thetaZone);

plot(zoneX,...
    zoneY,...
    '--',...
    'LineWidth',2);

% Radar location
plot(0,0,...
    '+',...
    'MarkerSize',12,...
    'LineWidth',3);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('Drone Tracking and Protected Zone');

legend('True Drone',...
    'Radar Measurements',...
    'Kalman Estimate',...
    'Protected Zone',...
    'Radar');

%% ==============================
%  TRUE VS MEASURED VS ESTIMATED
%  ==============================

figure;

plot(X,Y,'LineWidth',2);
hold on;

plot(xMeasured,...
    yMeasured,...
    '.');

plot(Xestimated,...
    Yestimated,...
    'LineWidth',2);

grid on;

xlabel('X Position (m)');
ylabel('Y Position (m)');

legend('True Drone',...
    'Radar Measurements',...
    'Kalman Estimate');

title('Drone Tracking using Kalman Filter');

axis equal;

%% Velocity estimate

figure;

plot(time,Vxestimated,'LineWidth',2);
hold on;

plot(time,Vyestimated,'LineWidth',2);

grid on;

xlabel('Time (s)');
ylabel('Velocity (m/s)');

legend('Estimated Vx',...
    'Estimated Vy');

title('Estimated Drone Velocity');

%% Mark intrusion point

if ~isempty(intrusionIndex)

    plot(Xestimated(intrusionIndex),...
        Yestimated(intrusionIndex),...
        'o',...
        'MarkerSize',12,...
        'LineWidth',3);

    text(Xestimated(intrusionIndex),...
        Yestimated(intrusionIndex),...
        '  ALERT: Zone Entry',...
        'FontSize',11,...
        'FontWeight','bold');

end
%% ==============================
%  RADAR + KALMAN TRACKING
%  ==============================

figure;

ax = polaraxes;

hold(ax,'on');

rlim(ax,[0 radarMaxRange]);

title(ax,'RADAR - DRONE TRACKING');

for k = 1:length(time)

    %% Radar measurement

    measuredRange = Rmeasured(k);

    measuredAzimuth = ...
        deg2rad(AzMeasured(k));


    %% Kalman estimate

currentEstimatedRange = ...
    sqrt(Xestimated(k)^2 + ...
    Yestimated(k)^2);

currentEstimatedAzimuth = ...
    atan2(Yestimated(k),...
    Xestimated(k));


    %% Plot radar measurement

    p1 = polarplot(ax,...
        measuredAzimuth,...
        measuredRange,...
        'o',...
        'MarkerSize',7,...
        'LineWidth',2);


    %% Plot Kalman estimate

p2 = polarplot(ax,...
    currentEstimatedAzimuth,...
    currentEstimatedRange,...
        'x',...
        'MarkerSize',9,...
        'LineWidth',2);


    %% Update display

    drawnow;

    pause(dt);


    %% Remove previous markers

    if k < length(time)

        delete(p1);
        delete(p2);

    end

end

%% Radar tracking with history

figure;

ax = polaraxes;

hold(ax,'on');

rlim(ax,[0 radarMaxRange]);

title(ax,'RADAR - KALMAN TRACKING');

for k = 1:length(time)

    measuredRange = Rmeasured(k);

    measuredAzimuth = ...
        deg2rad(AzMeasured(k));


currentEstimatedRange = ...
    sqrt(Xestimated(k)^2 + ...
    Yestimated(k)^2);

currentEstimatedAzimuth = ...
    atan2(Yestimated(k),...
    Xestimated(k));


    %% Current radar measurement

    p1 = polarplot(ax,...
        measuredAzimuth,...
        measuredRange,...
        'o',...
        'MarkerSize',6,...
        'LineWidth',1);


    %% Kalman track history

    trackRange = sqrt(...
        Xestimated(1:k).^2 + ...
        Yestimated(1:k).^2);

    trackAzimuth = atan2(...
        Yestimated(1:k),...
        Xestimated(1:k));

    p2 = polarplot(ax,...
        trackAzimuth,...
        trackRange,...
        'LineWidth',2);


    drawnow;

    pause(dt);

    delete(p1);

    if k < length(time)
        delete(p2);
    end

end

%% Protected zone detection

insideProtectedZone = ...
    estimatedRange <= protectedRadius;

%% Find first protected-zone intrusion

intrusionIndex = find(insideProtectedZone,1);

if isempty(intrusionIndex)

    disp('No protected-zone intrusion detected.');

else

    intrusionTime = time(intrusionIndex);
    intrusionRange = estimatedRange(intrusionIndex);

    fprintf('\n');
    fprintf('====================================\n');
    fprintf('       ⚠ DRONE INTRUSION ALERT\n');
    fprintf('====================================\n');
    fprintf('Time  : %.2f seconds\n',intrusionTime);
    fprintf('Range : %.2f meters\n',intrusionRange);
    fprintf('====================================\n');

end


%% ==========================================
%  REAL-TIME RADAR STATUS DISPLAY
%  ==========================================
alertTriggered = false;
figure;

ax = polaraxes;

hold(ax,'on');

rlim(ax,[0 radarMaxRange]);

title(ax,'RADAR - DRONE DETECTION SYSTEM');

%% Protected zone on radar

thetaCircle = linspace(0,2*pi,360);

polarplot(ax,...
    thetaCircle,...
    protectedRadius*ones(size(thetaCircle)),...
    '--',...
    'LineWidth',2);

%% ==========================================
%  REAL-TIME RADAR + ALERT
%  ==========================================

for k = 1:length(time)

    %% Current radar measurement

    measuredRange = Rmeasured(k);

    measuredAzimuth = ...
        deg2rad(AzMeasured(k));


    %% Current Kalman estimate

    currentRange = estimatedRange(k);

    currentAzimuth = ...
        deg2rad(estimatedAzimuth(k));


    %% Determine target status

    if currentRange <= protectedRadius

        status = "⚠ INTRUSION";

    elseif currentRange <= approachRadius

        status = "APPROACHING";

    else

        status = "NORMAL";

    end

    %% Intrusion alert

    if currentRange <= protectedRadius

        if ~alertTriggered

            fprintf('\n');
            fprintf('====================================\n');
            fprintf('       DRONE INTRUSION ALERT\n');
            fprintf('====================================\n');
            fprintf('Target  : T001\n');
            fprintf('Range   : %.1f m\n', currentRange);
            fprintf('Azimuth : %.1f degrees\n', estimatedAzimuth(k));
            fprintf('====================================\n');

            alertTriggered = true;

        end

    end
    %% Plot radar measurement

    p1 = polarplot(ax,...
        measuredAzimuth,...
        measuredRange,...
        'o',...
        'MarkerSize',6,...
        'LineWidth',1);


    %% Plot Kalman estimate

    p2 = polarplot(ax,...
        currentAzimuth,...
        currentRange,...
        'x',...
        'MarkerSize',10,...
        'LineWidth',2);


    %% Display information

    title(ax,...
        sprintf(['RADAR - DRONE DETECTION SYSTEM\n' ...
        'Range: %.1f m   Azimuth: %.1f°   Status: %s'],...
        currentRange,...
        estimatedAzimuth(k),...
        status));


    drawnow;

    pause(dt);


    %% Remove current markers

    delete(p1);
    delete(p2);

end

%% ==========================================
% PHASE E - RADAR PPI
% ==========================================

alertTriggered = false;

figure('Name','Anti-Drone Radar System');

ax = polaraxes;

hold(ax,'on');

rlim(ax,[0 radarMaxRange]);

thetalim(ax,[-180 180]);

title(ax,'ANTI-DRONE RADAR SYSTEM');


%% Range rings

ringRanges = [250 500 750 1000];

thetaRing = linspace(-pi,pi,360);

for r = ringRanges

    polarplot(ax,...
        thetaRing,...
        r*ones(size(thetaRing)),...
        '--',...
        'LineWidth',1);

end


%% Protected zone

polarplot(ax,...
    thetaRing,...
    protectedRadius*ones(size(thetaRing)),...
    '--',...
    'LineWidth',2);


%% Radar center

polarplot(ax,...
    0,...
    0,...
    '+',...
    'MarkerSize',12,...
    'LineWidth',3);


%% Sweep

sweepAngle = 0;

sweepSpeed = deg2rad(6);


%% Main radar loop

for k = 1:length(time)

    %% Radar measurement

    measuredRange = Rmeasured(k);

    measuredAzimuth = ...
        deg2rad(AzMeasured(k));


    %% Kalman estimate

    currentRange = estimatedRange(k);

    currentAzimuth = ...
        deg2rad(estimatedAzimuth(k));


    %% Velocity

    currentVelocity = sqrt( ...
        Vxestimated(k)^2 + ...
        Vyestimated(k)^2);


    %% Status

    if currentRange <= protectedRadius

        status = "INTRUSION";

    elseif currentRange <= approachRadius

        status = "APPROACHING";

    else

        status = "NORMAL";

    end


    %% Radar sweep

    sweepAngle = sweepAngle + sweepSpeed;

    if sweepAngle > pi

        sweepAngle = sweepAngle - 2*pi;

    end


    sweepLine = polarplot(ax,...
        [sweepAngle sweepAngle],...
        [0 radarMaxRange],...
        'LineWidth',2);


    %% Raw radar measurement

    measurementPlot = polarplot(ax,...
        measuredAzimuth,...
        measuredRange,...
        'o',...
        'MarkerSize',6,...
        'LineWidth',1);


    %% Kalman target

    targetPlot = polarplot(ax,...
        currentAzimuth,...
        currentRange,...
        'x',...
        'MarkerSize',10,...
        'LineWidth',3);


    %% Kalman track history

    trackRange = sqrt( ...
        Xestimated(1:k).^2 + ...
        Yestimated(1:k).^2);

    trackAzimuth = atan2( ...
        Yestimated(1:k),...
        Xestimated(1:k));

    trackPlot = polarplot(ax,...
        trackAzimuth,...
        trackRange,...
        'LineWidth',2);


    %% Display information

    title(ax,...
        sprintf(['ANTI-DRONE RADAR SYSTEM\n' ...
        'TARGET: T001   RANGE: %.1f m   ' ...
        'AZIMUTH: %.1f°   SPEED: %.1f m/s   STATUS: %s'],...
        currentRange,...
        estimatedAzimuth(k),...
        currentVelocity,...
        status));


    %% Alert

    if currentRange <= protectedRadius

        if ~alertTriggered

            fprintf('\n');
            fprintf('====================================\n');
            fprintf('       DRONE INTRUSION ALERT\n');
            fprintf('====================================\n');
            fprintf('Target  : T001\n');
            fprintf('Range   : %.1f m\n',currentRange);
            fprintf('Azimuth : %.1f degrees\n',estimatedAzimuth(k));
            fprintf('Speed   : %.1f m/s\n',currentVelocity);
            fprintf('====================================\n');

            alertTriggered = true;

        end

    end


    %% Update

    drawnow;

    pause(dt);


    %% Delete temporary graphics

    delete(sweepLine);
    delete(measurementPlot);
    delete(targetPlot);
    delete(trackPlot);

end
