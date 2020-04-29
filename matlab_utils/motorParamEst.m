%% STEP 0

if (exist('board', 'var'))
    clearvars -except board;
else
    clear;
    board = arduino('/dev/cu.usbmodemFA131', 'Mega2560', 'Libraries', 'rotaryEncoder');
end

%% STEP 1

inputs = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
inputsPWM = inputs./12;

expInput = [];
expOutput = [];

FORW = 1;
BACK = 0;
direction = 0;
timeEach = 6;
fbEach = 2;

encoder = rotaryEncoder(board, 'D2', 'D3', 260);

for i = 1:length(inputsPWM)
    writePWMDutyCycle(board, 'D6', inputsPWM(i));
    
    for j = 1:fbEach
        if mod(j, 2) == 0
            fprintf('... doing input equal to %d V in backward direction.\n', inputs(i));
            writePWMDutyCycle(board, 'D5', BACK);
            direction = -1;
        else
            fprintf('... doing input equal to %d V in forward direction.\n', inputs(i));
            writePWMDutyCycle(board, 'D5', FORW);
            direction = 1;
        end
        
        tic
        while(toc < timeEach)
            speed = (readSpeed(encoder)/60)*(2*pi);
            input = direction * inputs(i);
            
            expOutput = [expOutput, speed];
            expInput = [expInput, input];
            
            pause(0.001)
        end
    end
end

writePWMDutyCycle(board, 'D6', 0);

%% STEP 2

totalTime = timeEach*fbEach*length(inputs);
dt = totalTime/length(expInput);
t = 0:dt:totalTime - dt;

s = tf('s');
K = 0.4;
J = 0.5;
R = 0.8;

motorModel = K/(J*R*s + K^2);
expOutputModel = lsim(motorModel, expInput, t);

%% STEP 3

% Execute Simulink parameter estimation using:
%
%   input:      [t', expInput']
%   output:     [t', expOutput']
%   paramters:  L, R, J in [0, inf]

%% STEP 4

motorModelFitted = K/(J*R*s + K^2); % Paramters are automatically adjusted by Simulink
expOutputModelFitted = lsim(motorModelFitted, expInput, t);

plot(t, expInput, t, expOutput, t, expOutputModel, t, expOutputModelFitted, 'linewidth', 2);
legend('input (V)', 'motor (omega)', 'motor model (omega)', 'motor model fitted (omega)');
    







