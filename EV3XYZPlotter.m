clear all
if exist('ev3ShellHandle')
   clear ev3ShellHandle
end


%Square Parameter
l = 50; % in mm

%Circle Parameter
r = l/2; % in mm
centercircle = [25 25]; %in mm from 0,0

%Traingle Parameter
init = [0 0];
mid = [25 50];
final = [50 0];

% Axis as defined in the project
%y         |          
%y         |          
%y         |          
%y---------|----------
%y         |          
%y         |          
%y         |          
%yxxxxxxxxxxxxxxxxxxxxx

%initializing variables

global xm;
global ym;
global zm;
global limitSwitch;
global speed;
global smallGear;
global bigGear;

xPort = 'A'; % ports in ev3
yPort = 'B'; % ports in ev3
zPort = 'C'; % ports in ev3
speed = 30; % default motor speed
smallGear = 8;
bigGear = 40;

ev3ShellHandle = legoev3('USB'); %making connection to EV3

%if wifi exists
%ev3ShellHandle = legoev3('WiFi','ip address','hardware id'); 

%initialize motor port
xm = motor(ev3ShellHandle, xPort);
xm.Speed = speed; %set speed
ym = motor(ev3ShellHandle, yPort);
ym.Speed = speed; %set speed
zm = motor(ev3ShellHandle, zPort);
zm.Speed = speed; %set speed
limitSwitch = touchSensor(ev3ShellHandle); %limit switch

%explicitly reset the initial motor degrees
resetRotation(xm);
resetRotation(ym);
resetRotation(zm);

%Perform Performing Mechatronic systems task
home(l) %home -> always run this function to home x. Make sure limit switch is connected
%engagePen(zm, -1) % pen down
drawQuadrilateral(l,l)
drawTriangle(init, mid, final)
drawCircle(centercircle, r);
%engagePen(zm, 1) % pen up

%terminate the connection
clear ev3ShellHandle

%move pen up or down
function engagePen(zm, direction)
%   Move pen up or down
%   Parameter
%       zm : otor to move
%       direction: 1 for up and -1 for down
    global speed;
    start(zm , direction * (30));
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
    global xm;
    global ym;
    global limitSwitch;
    global speed;
    xm.Speed = -speed;
    start(xm);
    while ~readTouch(limitSwitch)
%        disp(distancetraveled(xm))
%         if distancetraveled(xm) > l
%             stop(xm);
%             disp('Maximum distance traveled for homing. If everything is fine then Please start again.')
%             break;
%         end
    end
    stop(xm);
    xm.Speed = speed;
    pause(1) % settling time for percussion measure
    resetRotation(xm)
    resetRotation(ym)
end

%Draw Quadrilateral
function drawQuadrilateral(l,b)
%   Draw Quadrilateral
%   Parameters
%       l : lenth of one side 
%       b : breadth of other side
    global xm;
    global ym;
    moveTo(xm, ym, 0, b)
    moveTo(xm, ym, l, b)
    moveTo(xm, ym, l, 0)
    moveTo(xm, ym, 0, 0)
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
    global ym;
    global speed;
    x0 = centercircle(1);
    y0 = centercircle(2);
    degree = 0:10:360;
    x = x0 + r*(cos((pi/180) * degree));
    y = y0 + r*(sin((pi/180) * degree));
    for i = 1:length(x)
        moveTo(xm, ym, x(i), y(i))
    end
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
    global ym;
    moveTo(xm, ym, 25, 50)
    moveTo(xm, ym, 50, 0)
    %moveToPid(xm, ym, 0, 0)
end

%Move motor1 and motor2 simultaneously
function moveTo(motor1, motor2, position1, position2)
%   linear movement
%   one motor at a time
%   Parameters
%       motor1 : first motor to turn on
%       motor1 : second motor to turn on
%       position1 : position for motor1 to run
%       position2 : position for motor2 to run
    mindistance = 2; %min of 5 distance to run otherwise exit the loop
    stability = 0;   % if more than 9 stability in sequence then exit the loop
    pid1 = PID(2, 3, 0, 0.02);
    pid2 = PID(2, 3, 0, 0.02);
    start(motor1,0);
    start(motor2,0);
    %time1 = clock;
    while true
        %time2 = clock;
        %difftime = etime(time2,time1);
        %time1 = clock;
        %disp(difftime)
        breakLoop = false;
        measured1 = distancetraveled(motor1);
        measured2 = distancetraveled(motor2);
        out1 = pid1.cal(position1, measured1);
        out2 = pid2.cal(position2, measured2);
        if out1>=-mindistance && out1 <=  mindistance  && out2 >=  -mindistance && out2 <=  mindistance
            stability = stability + 1;
        end
        motor1.Speed = out1;
        motor2.Speed = out2;
        if measured1 == position1 && measured2 == position2
            stability = stability + 1;
%         else 
%             stability = 0;
        end
        if stability == 2
            breakLoop = true;
        end
        if breakLoop
            break;
        end
    end
    stop(motor1)
    stop(motor2)
end

%Get absolute distance travelled
function distance = distancetraveled(motor)
    global xm;
    global ym;
    global zm;
    rxMotor = 20;
    ryMotor = 23; %29
    rzMotor = 1;
    
    global smallGear;
    global bigGear;
    theta = (smallGear/bigGear) * double(readRotation(motor));
    if isequal(xm,motor)
        distance = 2 * pi * rxMotor * theta/360; 
    elseif isequal(ym,motor)
        distance = 2 * pi * ryMotor * theta/360; 
    elseif isequal(zm,motor)
        distance = 2 * pi * rzMotor * theta/360; 
    end
end

function stopMotors()
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

