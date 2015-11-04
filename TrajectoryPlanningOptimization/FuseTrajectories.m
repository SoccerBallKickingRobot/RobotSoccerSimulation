function traj = FuseTrajectories(traj1, traj2)

traj = traj1;

if (traj1.y(:,end) == traj2.y(:,1))
    traj.x = [traj1.x, traj1.x(end) + traj2.x];
    traj.y = [traj1.y, traj2.y];
else
    error('Trajectories do not line up. Cannot fuse them.');
end