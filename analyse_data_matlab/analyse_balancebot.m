% analyze_balancebot.m
% --------------------
% Expects a CSV with header: time,angle,output
% Columns: time (s), angle (deg), output (PWM)

%% 1) Load data
T = readtable('dummy_log.csv');

t      = T.time;    % time in seconds
angle  = T.angle;   % tilt angle in degrees
output = T.output;  % PID output command

%% 2) Plot raw data
figure('Name','Balance Bot Data','NumberTitle','off');
subplot(2,1,1);
plot(t, angle);
xlabel('Time (s)');
ylabel('Tilt Angle (°)');
title('Tilt Angle vs. Time');
grid on;

subplot(2,1,2);
plot(t, output);
xlabel('Time (s)');
ylabel('Control Output (PWM)');
title('PID Output vs. Time');
grid on;

%% 3) Compute step–response metrics
%    Identify the first “step” event by a quick change in angle
%    Here we assume the bot was at 0° then tipped—adjust indices as needed.

% Find indices where |angle| first exceeds 5% of peak
peak = max(abs(angle));
thr5 = 0.05 * peak;
ind_start = find(abs(angle) > thr5, 1, 'first');

% Rise time: time to go from 10% to 90% of peak magnitude
thr10 = 0.10 * peak;
thr90 = 0.90 * peak;
t10 = t(find(abs(angle)>=thr10,1,'first'));
t90 = t(find(abs(angle)>=thr90,1,'first'));
rise_time = t90 - t10;

% Overshoot: max(angle) beyond steady-state (0°)
overshoot = (max(angle) - 0) / peak * 100;

% Settling time: time after which angle stays within ±5% band of final value
settle_inds = find(abs(angle) <= thr5);
if ~isempty(settle_inds)
    settling_time = t(settle_inds(end));
else
    settling_time = NaN;
end

%% 4) Display metrics
fprintf('\nPerformance Metrics:\n');
fprintf('  Peak response (deg)    : %.2f°\n', peak);
fprintf('  Rise time (10→90%%)     : %.3f s\n', rise_time);
fprintf('  Overshoot              : %.1f %%\n', overshoot);
fprintf('  Settling time (±5%%)    : %.3f s\n\n', settling_time);

%% 5) Optional: overlay the 5/10/90% bands on the first plot
subplot(2,1,1);
hold on;
yline( thr5, '--r','±5%','LabelHorizontalAlignment','left');
yline(-thr5, '--r','LabelHorizontalAlignment','left');
yline( thr10,'--g','10%','LabelHorizontalAlignment','left');
yline( thr90,'--g','90%','LabelHorizontalAlignment','left');
hold off;