clearvars -except

joints = ['RHY';'RHR';'RHP';'RKP';'RAP';'RAR';'LHY';'LHR';'LHP';'LKP';'LAP';'LAR';'RSP';'RSR';'RSY';'REP';'RWY';'RWR';'RWP';'LSP';'LSR';'LSY';'LEP';'LWY';'LWR';'LWP';'NKY';'NK1';'NK2';'WST';'RF1';'RF2';'RF3';'RF4';'RF5';'LF1';'LF2';'LF3';'LF4';'LF5'];
          %1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23    24    25    26    27    28    29    30    31    32    33    34    35    36    37    38    39    40
joints = cellstr(joints);

timeStep = .005;
accel = 5;
velStep = accel*timeStep;
positions = [0];
DIRECTION = -1;
for i = 10:-1:1
    % Accelerate
    v = 0;
    V = i*.1*DIRECTION;
    for j = 1:abs(V)/velStep
        v = DIRECTION*(abs(v) + velStep);
        pos = positions(length(positions)) + v*timeStep;
        positions = [positions, pos];
    end
    
    % Constant velocity
    posStep = V*timeStep;
    for i = 1:200
        pos = positions(length(positions)) + posStep;
        positions = [positions, pos];
    end
    
    % Decelerate
    v = V;
    for j = 1:abs(V)/velStep
        v = DIRECTION*(abs(v) - velStep);
        pos = positions(length(positions)) + v*timeStep;
        positions = [positions, pos];
    end
    DIRECTION = DIRECTION*-1;
end

data = zeros(length(positions), length(joints));
data(:,13) = -1.569;
data(:,20) = -1.569;
data(:,14) = positions;

fileName = 'rsrSpeed.traj';
saveTraj(fileName, data, joints);