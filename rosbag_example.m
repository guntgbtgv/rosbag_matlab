%% Read .bag file 
clear all

bag = rosbag('./2024-01-07-17-38-49.bag');   %sliding

bag.MessageList   % prints out message
%% Select specific topic & data within the topic
 

% imu data
bagselectImu = select(bag,"Topic","/imu")  
msgsImu = readMessages(bagselectImu,"DataFormat","struct");    % For some reason, readMessages should be used... I don't know why. 

% joint position, joint velocity, joint torque data
bagselectJs = select(bag,"Topic","/2DRRbounder/joint_states")
msgsJs = readMessages(bagselectJs,"DataFormat","struct");

% ground truth data 
bagselectGz = select(bag,"Topic", "/gazebo/link_states")
msgsGz = readMessages(bagselectGz,"DataFormat","struct");

% tf data
bagselectTf = select(bag,"Topic", "/tf")
msgsTf = readMessages(bagselectTf,"DataFormat","struct");

start = bag.StartTime
size(msgsImu)

%% Eextract specific data from cell format as double format 

% imu data
tsImu = zeros(length(msgsImu),1);
acc = zeros(length(msgsImu),3);
gyro = zeros(length(msgsImu),3);

for i = 1:length(msgsImu)
    tsImu(i) = double(msgsImu{i}.Header.Stamp.Sec) + double(msgsImu{i}.Header.Stamp.Nsec)*1e-9;
    acc(i,:) = [double(msgsImu{i}.LinearAcceleration.X), double(msgsImu{i}.LinearAcceleration.Y), double(msgsImu{i}.LinearAcceleration.Z)];
    gyro(i,:) = [double(msgsImu{i}.AngularVelocity.X), double(msgsImu{i}.AngularVelocity.Y), double(msgsImu{i}.AngularVelocity.Z)];  
end

% joint_states data
numJs = 9;
baseNum = 7;
foot1Num = 10;
foot2Num = 13;
tsJs = zeros(length(msgsJs),1);
jointPos = zeros(length(msgsJs),numJs); %[elbow hip knee shoulder pitch roll y yaw z]
jointVel = zeros(length(msgsJs),numJs);
jointEff = zeros(length(msgsJs),numJs);
for i = 1:length(msgsJs)
    tsJs(i,:) = double(msgsJs{i}.Header.Stamp.Sec) + double(msgsJs{i}.Header.Stamp.Nsec)*1e-9;
    jointPos(i,:) = msgsJs{i}.Position;
    jointVel(i,:) = msgsJs{i}.Velocity;
    jointEff(i,:) = msgsJs{i}.Effort;
end

% Ground truth from Gazebo

% since there are multiple datapoints in a single timestep, use only one
% data
tsGz = bagselectGz.MessageList.Time;
tsGz_ = bagselectGz.MessageList.Time;
ts_isSame = tsGz(1:33293) - tsGz_(2:33294);
ff = find(ts_isSame);
tsGz = tsGz(ff);


basePos = zeros(length(msgsGz),3);
baseVel = zeros(length(msgsGz),3);
baseQuat = zeros(length(msgsGz),4); % [W X Y Z]
baseAxang = zeros(length(msgsGz),4); % [phi norm(phi)]

for i=1:length(msgsGz)
    %link0
    basePos(i,:) = [double(msgsGz{i}.Pose(baseNum).Position.X), double(msgsGz{i}.Pose(baseNum).Position.Y), double(msgsGz{i}.Pose(baseNum).Position.Z)]; 
    baseVel(i,:) = [double(msgsGz{i}.Twist(baseNum).Linear.X), double(msgsGz{i}.Twist(baseNum).Linear.Y), double(msgsGz{i}.Twist(baseNum).Linear.Z)];
    baseQuat(i,:) = [double(msgsGz{i}.Pose(baseNum).Orientation.W), double(msgsGz{i}.Pose(baseNum).Orientation.X), double(msgsGz{i}.Pose(baseNum).Orientation.Y), double(msgsGz{i}.Pose(baseNum).Orientation.Z)];
    baseAxang(i,:) = quat2axang(baseQuat(i,:));
end

basePos = basePos(ff,:);
baseVel = baseVel(ff,:);
baseQuat = baseQuat(ff,:); 
baseAxang = baseAxang(ff,:);



foot1Pos = zeros(length(msgsGz),3);
foot2Pos = zeros(length(msgsGz),3);
foot1Vel = zeros(length(msgsGz),3);
foot2Vel = zeros(length(msgsGz),3);
foot1Quat = zeros(length(msgsGz),4); % [W X Y Z]
foot2Quat = zeros(length(msgsGz),4); % [W X Y Z]
for i=1:length(msgsGz)
    foot1Pos(i,:) = [double(msgsGz{i}.Pose(foot1Num).Position.X), double(msgsGz{i}.Pose(foot1Num).Position.Y), double(msgsGz{i}.Pose(foot1Num).Position.Z)]; 
    foot2Pos(i,:) = [double(msgsGz{i}.Pose(foot2Num).Position.X), double(msgsGz{i}.Pose(foot2Num).Position.Y), double(msgsGz{i}.Pose(foot2Num).Position.Z)]; 
    foot1Vel(i,:) = [double(msgsGz{i}.Twist(foot1Num).Linear.X), double(msgsGz{i}.Twist(foot1Num).Linear.Y), double(msgsGz{i}.Twist(foot1Num).Linear.Z)];
    foot2Vel(i,:) = [double(msgsGz{i}.Twist(foot2Num).Linear.X), double(msgsGz{i}.Twist(foot2Num).Linear.Y), double(msgsGz{i}.Twist(foot2Num).Linear.Z)];
    foot1Quat(i,:) = [double(msgsGz{i}.Pose(foot1Num).Orientation.W), double(msgsGz{i}.Pose(foot1Num).Orientation.X), double(msgsGz{i}.Pose(foot1Num).Orientation.Y), double(msgsGz{i}.Pose(foot1Num).Orientation.Z)];
    foot2Quat(i,:) = [double(msgsGz{i}.Pose(foot2Num).Orientation.W), double(msgsGz{i}.Pose(foot2Num).Orientation.X), double(msgsGz{i}.Pose(foot2Num).Orientation.Y), double(msgsGz{i}.Pose(foot2Num).Orientation.Z)];
end

foot1Pos = foot1Pos(ff,:);
foot2Pos = foot2Pos(ff,:);
foot1Vel = foot1Vel(ff,:);
foot2Vel = foot2Vel(ff,:);
foot1Quat = foot1Quat(ff,:);
foot2Quat = foot2Quat(ff,:);

%% Plot ground truth state  

figure 
hold on; grid on
plot(tsGz,basePos,'.')
plot(tsGz_selected,test,'.')
legend('X','Y','Z','X_selected','Y_selected','Z_selected')
xlabel('time (sec)'),ylabel('Position(m)')

figure 
hold on; grid on
plot(tsGz,baseVel,'.')
legend('X','Y','Z')
xlabel('time (sec)'),ylabel('Velocity(m/s)')

figure 
hold on; grid on
plot(tsGz,baseQuat)
legend('W','X','Y','Z')
xlabel('time (sec)'),ylabel('Quaternion')

figure 
hold on; grid on
plot(tsGz,baseAxang(:,1:3).*baseAxang(:,4))
legend('X','Y','Z','norm')
xlabel('time (sec)'),ylabel('Angle axis')
%% Plot imu 

figure
hold on; grid on
plot(tsImu, acc)
legend('X','Y','Z')
xlabel('time (sec)'),ylabel('Acceleration(m/s^2)')

figure
hold on; grid on
plot(tsImu, gyro)
legend('X','Y','Z')
xlabel('time (sec)'),ylabel('Base angular Velocity (rad/s)')

%% Plot joint states
figure
hold on; grid on
plot(tsJs,jointPos(:,1:4))
plot(tsJs,jointPos(:,5:end),':.')
legend('elbow','hip','knee','shoulder','pitch','roll','y','yaw','z')
xlabel('time (sec)'),ylabel('Joint angle(rad)')

figure
hold on; grid on
plot(tsJs,jointVel(:,1:4))
plot(tsJs,jointVel(:,5:end),':.')
legend('elbow','hip','knee','shoulder','pitch','roll','y','yaw','z')
xlabel('time (sec)'),ylabel('Joint angular velocity(rad/s)')

figure
hold on; grid on
plot(tsJs,jointEff(:,1:4))
plot(tsJs,jointEff(:,5:end),':.')
legend('elbow','hip','knee','shoulder','pitch','y','z')
xlabel('time (sec)'),ylabel('Joint torque(Nm)')

% xlabel('time (sec)'),ylabel('Ground Reaction Force (N)')
% legend('GRF_z','GRF_y','desired GRF_y')
% title('Foreleg GRF in Sloped terrain')

%% Save

clear ans i start numJs baseNum foot1Num foot2Num bag tsGz_ ff 


%save('Standing2DRR_data.mat')
save('Sliding2DRR_data.mat')

