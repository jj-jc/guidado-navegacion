function [poseMapa] = apolo2map(poseApolo)
poseMapa = [8+poseApolo(2), 8-poseApolo(1), poseApolo(3)];
end