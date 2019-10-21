% takes as input current position of end effector, and the end effect
% desired position

h = 0.01; %needs to be fine-tuned to avoid the solution from blowing up
[currentRobotPos, TT] = fanuc_m900_fk(0, 0, 0, 0, 0, 0); %returns the robot position and transformation matrices in 3D (for each joint with respect to base frame)
th = [0 0 0 0 0 0];
EETarget = [1.9 0 -0.585]; 
counster = 1;

while (abs(EEPos - EETarget) > 0.001)
    compute_dth(currentRobotPos, EETarget, TT);  %returns a vector of angle changes
    th = th + h*dth;   %computes new angles
    th1 = th(1); th2 = th(2); th3 = th(3); th4 = th(4); th5 = th(5); th6 = th(6);
    [currentRobotPos, TT] = fanuc_m900_fk(th1, th2, th3, th4, th5, th6); %finds new orientation for new EE position with the new angle
    outputRobPositions(:,:,counter) = currentRobotPos;
    counter = counter + 1;
end

outputRobPositions; %this is a 3D matrix of the robot's outputted positions for the plot


