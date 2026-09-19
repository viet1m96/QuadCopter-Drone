clear;
clc;

csvFile = "/home/vietht-hl/FromMx/Quadcopter Drone/telemetry.csv";
D = csvRead(csvFile, ",", ".", "double", [], [], [], 1);

validRows = find(sum(isnan(D), "c") == 0);
D = D(validRows, :);

if size(D, "c") <> 20 then
    error("Telemetry CSV must contain exactly 20 columns.");
end

if size(D, "r") < 2 then
    error("Telemetry CSV does not contain enough valid samples.");
end

t = (D(:,1) - D(1,1)) / 1e6;
dt = diff(t);
sampleRate = 1 / mean(dt);
duration = t($);
longGaps = find(dt > 0.080);

mprintf("Valid samples: %d\n", size(D, "r"));
mprintf("Log duration: %.3f s\n", duration);
mprintf("Average sample rate: %.2f Hz\n", sampleRate);
mprintf("Gaps longer than 80 ms: %d\n", size(longGaps, "*"));

scf(1);
clf();

subplot(3,1,1);
plot(t, D(:,7));
xgrid();
xtitle("Roll Command", "Time (s)", "Normalized input");

subplot(3,1,2);
plot(t, D(:,10));
xgrid();
xtitle("Measured Roll", "Time (s)", "Angle (deg)");

subplot(3,1,3);
plot(t, D(:,18));
xgrid();
xtitle("Roll Correction", "Time (s)", "Correction");

scf(2);
clf();

subplot(3,1,1);
plot(t, D(:,8));
xgrid();
xtitle("Pitch Command", "Time (s)", "Normalized input");

subplot(3,1,2);
plot(t, D(:,11));
xgrid();
xtitle("Measured Pitch", "Time (s)", "Angle (deg)");

subplot(3,1,3);
plot(t, D(:,19));
xgrid();
xtitle("Pitch Correction", "Time (s)", "Correction");

scf(3);
clf();

subplot(3,1,1);
plot(t, D(:,9));
xgrid();
xtitle("Yaw Command", "Time (s)", "Normalized input");

subplot(3,1,2);
plot(t, D(:,14));
xgrid();
xtitle("Measured Yaw Rate", "Time (s)", "Angular velocity (deg/s)");

subplot(3,1,3);
plot(t, D(:,20));
xgrid();
xtitle("Yaw Correction", "Time (s)", "Correction");

scf(4);
clf();

plot(t, D(:,2:6));
xgrid();
legend(["Throttle"; "Motor FL"; "Motor FR"; "Motor RR"; "Motor RL"], "in_upper_right");
xtitle("Throttle and Motor Outputs", "Time (s)", "Normalized output");

scf(5);
clf();

subplot(2,1,1);
plot(t, D(:,12:14));
xgrid();
legend(["Gyro X"; "Gyro Y"; "Gyro Z"], "in_upper_right");
xtitle("Raw Gyroscope", "Time (s)", "Angular velocity (deg/s)");

subplot(2,1,2);
plot(t, D(:,15:17));
xgrid();
legend(["Accel X"; "Accel Y"; "Accel Z"], "in_upper_right");
xtitle("Raw Accelerometer", "Time (s)", "Acceleration (g)");
