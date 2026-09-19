clear;
clc;

csvFile = get_absolute_file_path("plot_sensor_telemetry.sce") + "telemetry.csv";
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

f = scf(1);
f.figure_name = "Quadcopter Telemetry";
f.figure_size = [1400, 900];
clf();

subplot(3,2,1);
plot(t, D(:,10));
xgrid();
xtitle("Measured Roll", "Time (s)", "Angle (deg)");

subplot(3,2,2);
plot(t, D(:,11));
xgrid();
xtitle("Measured Pitch", "Time (s)", "Angle (deg)");

subplot(3,2,3);
plot(t, D(:,14));
xgrid();
xtitle("Yaw Rate", "Time (s)", "Angular velocity (deg/s)");

subplot(3,2,4);
plot(t, D(:,3:6));
xgrid();
legend(["Motor FL"; "Motor FR"; "Motor RR"; "Motor RL"], "in_upper_right");
xtitle("Motor Throttle", "Time (s)", "Normalized throttle");

subplot(3,2,5);
plot(t, D(:,12:14));
xgrid();
legend(["Gyro X"; "Gyro Y"; "Gyro Z"], "in_upper_right");
xtitle("Gyroscope Data", "Time (s)", "Angular velocity (deg/s)");

subplot(3,2,6);
plot(t, D(:,15:17));
xgrid();
legend(["Accel X"; "Accel Y"; "Accel Z"], "in_upper_right");
xtitle("Accelerometer Data", "Time (s)", "Acceleration (g)");
