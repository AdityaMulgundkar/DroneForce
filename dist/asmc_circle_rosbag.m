!bag = rosbag('2023-01-23-20-07-46.bag');
!bag = rosbag('2023-01-23-22-03-24.bag');
!bag = rosbag('2023-01-23-21-50-31.bag');
!bag = rosbag('2023-01-23-22-26-17.bag');
!bag = rosbag('2023-01-23-22-29-20.bag');
!bag = rosbag('2023-01-23-23-32-22.bag');
!bag = rosbag('2023-01-23-23-37-30.bag');
!bag = rosbag('2023-01-23-23-43-13.bag');
!bag = rosbag('2023-01-24-11-07-31.bag');
!bag = rosbag('2023-01-24-11-48-22.bag');
!bag = rosbag('2023-01-24-12-26-50.bag');
!bag = rosbag('2023-01-24-13-03-39.bag');
!bag = rosbag('2023-01-24-13-13-01.bag');

!PID
!bag = rosbag('2023-01-24-15-40-53.bag');
!bag = rosbag('2023-01-24-18-34-05.bag');
!bag = rosbag('2023-01-26-15-47-44.bag');

!PID-circle-8.jpg
!bag = rosbag('2023-01-26-17-09-26.bag');
bag = rosbag('2023-01-26-17-29-03.bag');

bSel = select(bag,'Topic','/mavros/local_position/pose');

msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs{1};

axis square;
xPoints = cellfun(@(m) double(m.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Position.Y),msgStructs);
plot(xPoints,yPoints);

hold on;


bSel = select(bag,'Topic','/desired_position');

msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs{1};

axis square;
xPoints = cellfun(@(m) double(m.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Position.Y),msgStructs);
plot(xPoints,yPoints);

hold off;

xlim([-6, 6]);
ylim([-6, 6]);