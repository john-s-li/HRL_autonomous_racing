%% vehicleParams.m

% Load the relevant vehicle parameters (BARC params)
function p = vehicleParams()

    p = struct;
    
    p.g = 9.81;
    p.m  = 1.98;
    p.lf = 0.125;
    p.lr = 0.125;
    p.Iz = 0.024;
    p.Df = 0.8 * p.m * p.g / 2.0;
    p.Cf = 1.25;
    p.Bf = 1.0;
    p.Dr = 0.8 * p.m * p.g / 2.0;
    p.Cr = 1.25;
    p.Br = 1.0;
    
    % Length and Width of Car (Based on Traxxas 1:10 RC Car)
    p.l = 0.568;
    p.w = 0.296;
    
end