Ts = 0.1;
% The ego car has rectangular shaper with a length of 5 meters and width of
% 2 meters. The model has four states:
%
% * |xPos| - Global horizontal position of the car center
% * |yPos| - Global vertical position of the car center
% * |theta| - Heading angle of the car (0 when facing east, counterclockwise positive)
% * |V| - Speed of the car (positve)
%
% There are two manipulated variables:
%
% * |throttle| - Throttle (positive when accelerating, negative when braking)
% * |delta| - Steering angle change (counterclockwise positive)

%#codegen

% Define continuous-time linear model from Jacobian of the nonlinear model.
carLength = 5;
theta = x(3);
V = x(4);
delta = u(2);
A = [ 0, 0, -V*sin(theta), cos(theta);
       0, 0,  V*cos(theta), sin(theta);
       0, 0, 0,             tan(delta)/carLength;
       0, 0, 0,             0];
B = [0  , 0;
      0  , 0;
      0  , (V*(tan(delta)^2 + 1))/carLength;
      0.5, 0];
C= eye(4);
D = zeros(4,2);
% Generate discrete-time model using ZOH.
[Ad, Bd] = adasblocks_utilDicretizeModel(A,B,Ts);
Cd = C;
Dd = D;
% Nominal conditions for discrete-time plant
X = x;
U = u;
Y = x;
DX = Ad*x+Bd*u-x;