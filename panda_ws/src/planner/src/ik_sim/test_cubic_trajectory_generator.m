% test_cubic_trajectory_generator
wayPoints       = [0.4 -0.2 0.3; 0.3 0.2 0.4]
wayPointVels    = [0   0    0  ; 0   0   0  ]
trajectory = cubic_trajectory_generator(wayPoints,wayPointVels)

% figure
% hold on
% plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);
% % %%
% % figure
% % 
% % plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);
% % hold on
% plot3(wayPoints(:,1)',wayPoints(:,2)',wayPoints(:,3)','LineStyle','none','Marker','.','MarkerSize',20,'MarkerFaceColor','none')