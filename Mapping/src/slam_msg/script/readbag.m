
%%
bag = rosbag('/mnt/DataBlock/thomas/YQ/thomas_2017-03-01-10-35-31.bag');
bag = bag.select('Topic', {'/rtk_gps/pose'});
msg = readMessages(bag);

%%
poses = [];
for i=1:length(msg)
    % pos{i} = [pos; msg{i}.Position.X msg{i}.Position.Y msg{i}.Position.Z msg{i}.Orientation.W msg{i}.Orientation.X msg{i}.Orientation.Y msg{i}.Orientation.Z];
    poses{i} = [quat2rotm([msg{i}.Orientation.W msg{i}.Orientation.X msg{i}.Orientation.Y msg{i}.Orientation.Z]) [msg{i}.Position.X-msg{1}.Position.X msg{i}.Position.Y-msg{1}.Position.Y msg{i}.Position.Z-msg{1}.Position.Z]';0 0 0 1];
end

%%
drawPoses(poses(1:10:end));

