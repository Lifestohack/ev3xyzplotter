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
global minDis;

xPort = 'A'; % ports in ev3
yPort = 'B'; % ports in ev3
zPort = 'C'; % ports in ev3
speed = 50; % default motor speed
smallGear = 8;
bigGear = 40;
minDis = 5;
ev3ShellHandle = legoev3('USB'); %making connection to EV3

%if wifi exists
%ev3ShellHandle = legoev3('WiFi','ip address','hardware id'); 

%initialize motor port
xm = motor(ev3ShellHandle, xPort);
stop(xm);
xm.Speed = speed; %set speed
ym = motor(ev3ShellHandle, yPort);
stop(ym);
ym.Speed = speed; %set speed
zm = motor(ev3ShellHandle, zPort);
stop(zm);
zm.Speed = speed; %set speed
limitSwitch = touchSensor(ev3ShellHandle); %limit switch

%explicitly reset the initial motor degrees
resetRotation(xm);
resetRotation(ym);
resetRotation(zm);
%moveTo(xm,zm,0,10)
%home(l)
%moveTo(xm,ym,50,0)

%Perform Performing Mechatronic systems task
home(l) %home -> always run this function to home x. Make sure limit switch is connected
moveTo(xm,zm,0,5)
drawQuadrilateral(l,l)
measured1 = distancetraveled(xm);
measured2 = distancetraveled(ym);
drawTriangle(init, mid, final)
drawCircle(centercircle, r);
engagePen(zm, -1) % pen up
engagePen(zm, -1) % pen up
%terminate the connection
clear ev3ShellHandle

%move pen up or down
function engagePen(zm, direction)
%   Move pen up or down
%   Parameter
%       zm : otor to move
%       direction: 1 for up and -1 for down
    global speed;
    start(zm , direction * (50));
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
    %resetRotation(zm)
    moveTo(xm, ym, 15, 0)
    resetRotation(xm)
    resetRotation(ym)
    %resetRotation(zm)
end

%Draw Quadrilateral
function drawQuadrilateral(l,b)
%   Draw Quadrilateral
%   Parameters
%       l : lenth of one side 
%       b : breadth of other side
    global xm;
    global ym;
    moveTo(xm, ym, 0, 10)
    moveTo(xm, ym, 0, 20)
    moveTo(xm, ym, 0, 30)
    moveTo(xm, ym, 0, 40)
    moveTo(xm, ym, 0, 50)
    
    moveTo(xm, ym, 10, 50)
    moveTo(xm, ym, 20, 50)
    moveTo(xm, ym, 30, 50)
    moveTo(xm, ym, 40, 50)
    moveTo(xm, ym, 50, 50)
    
    moveTo(xm, ym, 50, 40)
    moveTo(xm, ym, 50, 30)
    moveTo(xm, ym, 50, 20)
    moveTo(xm, ym, 50, 10)
    moveTo(xm, ym, 50, 0)
    
    moveTo(xm, ym, 40, 0)
    moveTo(xm, ym, 30, 0)
    moveTo(xm, ym, 20, 0)
    moveTo(xm, ym, 10, 0)
    moveTo(xm, ym, 0, 0)
    
    measured1 = distancetraveled(xm)
    measured2 = distancetraveled(ym)
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
    
    moveTo(xm, ym, 2.5, 5)
    moveTo(xm, ym, 5, 10)
    moveTo(xm, ym, 7.5, 15)
    moveTo(xm, ym, 10, 20)
    moveTo(xm, ym, 12.5, 25)
    moveTo(xm, ym, 15, 30)
    moveTo(xm, ym, 17.5, 35)
    moveTo(xm, ym, 20, 40)
    moveTo(xm, ym, 22.5, 45)
    moveTo(xm, ym, 25, 50)             
                    
    moveTo(xm, ym, 27.5, 45)
    moveTo(xm, ym, 30, 40)
    moveTo(xm, ym, 32.5, 35)
    moveTo(xm, ym, 35, 30)
    moveTo(xm, ym, 37.5, 25)
    moveTo(xm, ym, 40, 20)
    moveTo(xm, ym, 42.5, 15)
    moveTo(xm, ym, 45, 10)
    moveTo(xm, ym, 47.5, 5)
    moveTo(xm, ym, 50, 0)
    
    measured1 = distancetraveled(xm)
    measured2 = distancetraveled(ym)
end

function [xdot, ydot] = segmentize(x,y)
    global xm;
    global ym;
    global minDis;
    initalX = distancetraveled(xm);
    initalY = distancetraveled(ym);
    if initalX < x
        xPrime = initalX:minDis:x;
    else
        xPrime = initalX:-minDis:x;
    end
    if initalY < y
        yPrime = initalY:minDis:y;
    else
        yPrime = initalY:-minDis:y;
    end
    if length(yPrime) ~= length(xPrime)
        if length(xPrime) == 1
            xPrime = ones(1,length(yPrime),'uint32') * xPrime
        end
        if length(yPrime) == 1
            yPrime = ones(1,length(xPrime),'uint32') * yPrime
        end
    end
    xdot = xPrime;
    ydot = yPrime;
end

function moveTo(motor1, motor2, position1, position2)
%     [x,y] = segmentize(position1,position2);
%     for i = 2:length(x)
%         waitTill(motor1, motor2, x(i), y(i));
%         dis = distancetraveled(motor2);
%     end
    waitTill(motor1, motor2, position1, position2);
end

%Move motor1 and motor2 simultaneously
function waitTill(motor1, motor2, position1, position2)
%   linear movement
%   one motor at a time
%   Parameters
%       motor1 : first motor to turn on
%       motor1 : second motor to turn on
%       position1 : position for motor1 to run
%       position2 : position for motor2 to run
    mindistance = 2; %min of 5 distance to run otherwise exit the loop
    stability = 0;   % if more than 9 stability in sequence then exit the loop
    pid1 = PID(1.71, 0.03622, 0, 0.02);
    pid2 = PID(1.063, 0.020786, 0, 0.02);

    start(motor1,0);
    start(motor2,0);
%     time1 = clock;
    breakLoop = false;
    fileID = fopen('ymotor.txt','w');
    while true
%         time2 = clock;
%         difftime = etime(time2,time1);
%         time1 = clock;
%         disp(difftime);
        measured1 = distancetraveled(motor1);
        measured2 = distancetraveled(motor2);
        out1 = pid1.cal(position1, measured1);
        out2 = pid2.cal(position2, measured2);
        fprintf(fileID,'%6.2f,%6.2f,%6.2f\n',position2,measured2,out2);
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
        if stability >= 30
            breakLoop = true;
        end
        if breakLoop
            break;
        end
    end
    stop(motor1)
    stop(motor2)
    fclose(fileID);
end

%Get absolute distance travelled
function distance = distancetraveled(motor)
    global xm;
    global ym;
    global zm;
    rxMotor = 19.5;
    ryMotor = 27;
    rzMotor = 50;
    
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

