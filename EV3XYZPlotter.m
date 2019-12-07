%ToDO:
%1. Velocity profile is good but did not checked for end co-ordinates from
%   encoder, so not getting same radius every time. 
%2. Triangle test
%3. PID controller overkill??
%4. Position the shape as defined in the project
%5. ??

clc
clear all
if exist('ev3ShellHandle')
   clear ev3ShellHandle
end

%Axis as defined in the project
%                   x
%                   x
%                   x
%                   x
%                   x
%                   x
%yyyyyyyyyyyyyyyyyyyy

%Square Parameter
l = 50; % in mm

%Circle Parameter
r = l/2; % in mm
centercircle = [25 25]; %in mm from 0,0

%Traingle Parameter
init = [0 0];
mid = [50 25];
final = [0 50];

%initializing variables
global speed;
global xm;
global xdegree;
global ym;
global ydegree;
global zm;
global limitSwitch;

speed = 20; % default motor speed
xPort = 'A'; % ports in ev3
yPort = 'B'; % ports in ev3
zPort = 'C'; % ports in ev3
xdegree = 2.04545; % degree per mm
ydegree = 3; % degree per mm

%making connection to EV3
ev3ShellHandle = legoev3('USB');

%if wifi exists
%ev3ShellHandle = legoev3('WiFi','ip address','hardware id'); 

%initialize motor port
xm = motor(ev3ShellHandle, xPort);
stop(xm); %if motor was moving then stop
xm.Speed = speed; %set speed

ym = motor(ev3ShellHandle, yPort);
stop(ym); %if motor was moving then stop
ym.Speed = speed; %set speed

zm = motor(ev3ShellHandle, zPort);
stop(zm); %if motor was moving then stop
zm.Speed = speed/10; %set speed

%limit switch
limitSwitch = touchSensor(ev3ShellHandle);

%explicitly reset the initial motor degrees
resetRotation(xm);
resetRotation(ym);
resetRotation(zm);

%Perform Performing Mechatronic systems task
home(l) %home -> always run this function to home y. Make sure limit switch is connected
engagePen(zm, 1) % pen down
drawQuadrilateral(l,l)
drawCircle(centercircle, r);
drawTriangle(t_init, t_mid, t_final)
engagePen(zm, -1) % pen up

% terminate the connection
clear ev3ShellHandle

%move pen up or down
function engagePen(zm, direction)
%   Move pen up or down
%   Parameter
%       zm : otor to move
%       direction: 1 for up and -1 for down
    global speed;
    start(xm , direction * (speed/2));
    changed = 1;
    i = 0;
    oldValue = readRotation(zm);
    while changed
        if readRotation(zm) == oldValue
            i = i + 1;
        end
        if i >= 1
            changed = 0;
            stop(zm);
        end
        oldValue = readRotation(zm);
    end
    zm.Speed = speed;
end

%home the y axis
function home(l)
%   Home Y axis
%   Parameter
%       l : max lenth before it stops if it didn't hit limit switch
    global ym;
    global ydegree;
    global limitSwitch;
    global speed;
    maxdegree = l * ydegree;
    ym.Speed = -speed;
    start(ym);
    while ~readTouch(limitSwitch)
        if readRotation(ym) > maxdegree
            stop(ym);
            disp('Maximum distance traveled for homing. If everything is fine then Please start again.')
            break;
        end
    end
    stop(ym);
    ym.Speed = speed;
    sleep(0.1) % settling time for percussion measure
    resetRotation(ym)
end

%Draw Quadrilateral
function drawQuadrilateral(l,b)
%   Draw Quadrilateral
%   Parameters
%       l : lenth of one side 
%       b : breadth of other side
    global xm;
    global xdegree;
    global ym;
    global ydegree;
    global speed;
    x = l * xdegree; %total degree to turn
    y = b * ydegree; %total degree to turn
    moveTo(xm, x, speed); % y is 0
    moveTo(ym, y, speed); % x is 0
    moveTo(xm, 0, speed); % y is 50
    moveTo(ym, 0, speed); % x is 0
end

%Draw Circle
function drawCircle(centercircle, r)
%   Draw Circle
%   Parameters
%       centercircle : center of the circle 
%            type : array of x and y cordinates  
%            example : centercircle = [x,y];
%       r : radius of circle
    global xm;
    global xdegree;
    global ym;
    global ydegree;
    global speed;
    x0 = centercircle(1) * xdegree;
    y0 = centercircle(2) * ydegree;
    rx = r * xdegree; % xdegree 
    ry = r * ydegree; % ydegree
    degree = 0:1:360;
    x =  x0 + rx*(cos((pi/180) * degree));
    y =  y0 + ry*(sin((pi/180) * degree));
    if length(x) ~= length(y)
        disp('WARNING!!! Length of x and y is not equal.')
    end
    Vx = -10/rx .* x;
    Vy = 10/ry .* y;
    start(xm, 0);
    start(ym, 0);
    for i = 1:length(Vx)
        xm.Speed = Vx(i);
        ym.Speed = Vy(i);
    end
    stop(xm);
    stop(ym);
    xm.Speed  = speed;
    ym.Speed  = speed;
end

%Draw Triangle
function drawTriangle(init, mid, final)
%   Draw Traingle
%   Parameters
%       init : initial cordinate of triangle
%       mid : middle cordinate of triangle
%       final : final cordinate of triangle
    global xm;
    global xdegree;
    global ym;
    global ydegree;
    global speed;
    %triangle first line
    ym.Speed = (double(double(xm.Speed))/2); % set speed y = mx + b where b = 0 and m = 2
    start(xm);
    start(ym);
    while readRotation(xm) < mid(1) * xdegree
    end
    stop(xm);
    stop(ym);

    %triangle second line
    xm.Speed = -1 * xm.Speed;
    start(xm);
    start(ym);
    while readRotation(xm) > final(1) * xdegree
    end
    stop(xm);
    stop(ym);
    xm.Speed = speed;
    ym.Speed = speed;
end

%move one motor in a straight line
function moveTo(motor, degree, speed)  
%   linear movement
%   one motor at a time
%   Parameters
%       motor : motor to turn on
%       degrees : turn motor specified degrees
%       speed : move motor at specified speed
    if readRotation(motor) < degree  %move forward
        motor.Speed = speed; %set speed and direction
        start(motor);
        while readRotation(motor) < degree
        end
    elseif readRotation(motor) > degree %move backward
        motor.Speed = -speed; %set speed and direction
        start(motor);
        while readRotation(motor) > degree
        end
    end
    stop(motor);
    motor.Speed = speed;
end

function stop()
    %for app if designed
    global xm
    global ym
    global zm
    stop(xm)
    stop(ym)
    stop(zm)
end

function objg = getGlobalObject()
    global xm;
    global xdegree;
    global ym;
    global ydegree;
    global speed;
    global zm;
    
    objg.xm = xm;
    objg.xdegree = xdegree;
    objg.ym = ym;
    objg.ydegree = ydegree;
    objg.zm = zm;
    objg.speed = speed;
end