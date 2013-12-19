clearvars -except

[pos, cur] = textread('rsrSpeed.txt');

for i = 1:2
    cur = smooth(cur);
end

pos = pos(170:length(pos));
cur = cur(170:length(cur));
vel = zeros(length(pos));
for i = 2:length(pos)
    vel(i) = (pos(i) - pos(i-1))/.05;
end

%plotyy(1:length(pos), pos, 1:length(cur), cur)
starts = zeros(10, 1);
ends = zeros(10, 1);
j = 1;
for i = 1:10
    while(abs(vel(j)) < (11-i)*.1)
        starts(i) = j + 1;
        j = j + 1;
    end
    ends(i) = j + 20; 
    j = j + 21;
end

hold on
sums = zeros(8, 1);
aves = zeros(8, 1);
for i = 1:8
    %plot(vel(starts(i):ends(i)))
    for j = starts(i):ends(i)
        sums(i) = sums(i) + cur(j);
    end
    aves(i) = sums(i)/20;
end

k = polyfit(1:-.1:.3, transpose(aves), 1);
y = k(1)*(1:-.1:0) + k(2);
plot((1:-.1:0), y);
plot(1:-.1:.3, aves, '.', 'color', 'r')

%{
timeStep = .005;
accel = 5;
velStep = accel*timeStep;
sums = zeros(10);
aves = zeros(10);
starts = zeros(10);
j = 1;
for i = 10:-1:1
    v = i*.1;
    j = j + floor((floor(v/velStep))/10);
    starts(i) = j;
    for j = j:j+20
        sums(i) = sums(i) + cur(j);
    end
    j = j + floor(v/velStep);
    aves(i) = sums(i)/200;
end

plot(starts, aves)
%}  