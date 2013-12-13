clearvars -except

[torque, position] = textread('armTorque.txt');

for i = 1:5
    torque = smooth(torque);
end

hold on
plot(1:length(torque), 0)
plotyy(1:length(torque), torque, 1:length(position), position)