# Drop in Sanitization for retargeting

Current issues:
1. Wrists phase through each other in retargeting/no collision checking implemented
2. Replay is pure retargeting, no IK solving, somehow has branch flipping causing torque and velocity spikes
3. Hand drift over time so over the course of the video the wrist rotation drifts causing the hands to end up inverted by the end

Suggested solution:
An offline sanitization pipeline. drop in: npz arrays inputted, sanitize by extracting wrist and elbow poses and hand poses, resolving IK based on g1_wuji_description's and applying collision checks, outputs the same format npz arrays that can be dropped in to the existing replay pipeline. I think the cleanest solution is to start from shoulder to elbow, then take the target elbow positions as starting position and solve for wrist pose