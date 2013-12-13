clearvars -except

joints = ['RHY';'RHR';'RHP';'RKP';'RAP';'RAR';'LHY';'LHR';'LHP';'LKP';'LAP';'LAR';'RSP';'RSR';'RSY';'REP';'RWY';'RWR';'RWP';'LSP';'LSR';'LSY';'LEP';'LWY';'LWR';'LWP';'NKY';'NK1';'NK2';'WST';'RF1';'RF2';'RF3';'RF4';'RF5';'LF1';'LF2';'LF3';'LF4';'LF5'];
          %1     2     3     4     5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21    22    23    24    25    26    27    28    29    30    31    32    33    34    35    36    37    38    39    40
joints = cellstr(joints);

V = .3;
timeStep = .005;
posStep = V*timeStep;
%{
% Arms to horizontal, start position: home
fileName = 'armsToHorizontal.traj';

rsp = 0:-posStep:-1.57;
lsp = 0:-posStep:-1.57;

data = zeros(length(rsp), length(joints));
data(:,13) = rsp;
data(:,20) = lsp;

saveTraj(fileName, data, joints);
%}

%
% Arms Sinusoidal, start position: armsToHorizontal
fileName = 'armssinusoidal.traj';

i = 0:2512;
A = .5;
omega = 1;
rsr = A*cos(omega*i*timeStep)-.5;
lsr = -A*cos(omega*i*timeStep)+.5;

data = zeros(length(rsr), length(joints));
data(:,14) = rsr;
data(:,21) = lsr;
data(:,13) = -1.569;
data(:,20) = -1.569;

saveTraj(fileName, data, joints);
%