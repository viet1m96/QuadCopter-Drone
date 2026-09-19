clear;
clc;

scriptDir = get_absolute_file_path("plot_roll_pitch.sce");
csvFile = scriptDir + "telemetry.csv";

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
rollDeg = D(:,10);
pitchDeg = D(:,11);

scf(1);
clf();

subplot(3,1,1);
plot(t, [rollDeg pitchDeg]);
xgrid();
legend(["Roll"; "Pitch"], "in_upper_right");
xtitle("Measured Roll and Pitch", "Time (s)", "Angle (deg)");

subplot(3,1,2);
plot(t, rollDeg);
xgrid();
xtitle("Measured Roll", "Time (s)", "Roll (deg)");

subplot(3,1,3);
plot(t, pitchDeg);
xgrid();
xtitle("Measured Pitch", "Time (s)", "Pitch (deg)");
