function [poseApolo] = map2apolo(poseMapa)
poseApolo = [8-poseMapa(2), -8+poseMapa(1), poseMapa(3)];
end