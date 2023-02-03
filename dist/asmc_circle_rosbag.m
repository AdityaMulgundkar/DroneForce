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
!bag = rosbag('2023-01-30-08-05-38.bag');

!PID-circle-wf
!bag = rosbag('2023-01-30-08-31-08.bag');
!bag = rosbag('2023-01-30-08-49-35.bag');

!PID-circle-f-2
!bag = rosbag('2023-01-30-09-22-42.bag');

!ASMC-circle-f-1
!bag = rosbag('2023-01-30-09-38-32.bag');

!bag = rosbag('2023-01-30-16-43-40.bag');

!bag = rosbag('2023-01-31-13-01-21.bag');
!bag = rosbag('2023-01-31-20-27-10.bag');

!ASMC-circle-3
!bag = rosbag('2023-02-03-15-56-27.bag');

!ASMC-circle-4
!bag = rosbag('2023-02-03-16-28-50.bag');

!ASMC-circle-5-f
!bag = rosbag('2023-02-03-17-12-17.bag');

!ASMC-circle-6-f
bag = rosbag('2023-02-03-17-37-30.bag');

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