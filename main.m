

S_3, Pose_3 = Initialization();

Pose = []
for i = 3:end
S_i+1, Pose_i+1 = ProcessFrame(S_i, Frame_i, frame_i+1)
Pose append Pose_new