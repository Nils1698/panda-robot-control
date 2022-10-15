function  [color] = threeColorSigmoidTransition(i)
% if i == 1 return pink
% if i == around 0.5 return yellow
% if i == 0 return cyan

p = [255, 27, 141]/255;
y = [255, 218, 0]/255;
c = [27, 179, 255]/255;

steepness = 20

% plot(x,1./(1+exp(-(x-2/3)*steepness)))
% plot(x,1./(1+exp(-(x-1/3)*steepness)) -1./(1+exp(-(x-2/3)*steepness)))
% plot(x,1-1./(1+exp(-(x-1/3)*steepness)))

color = p * 1./(1+exp(-(i-2/3)*steepness)) + ...
        y * (1./(1+exp(-(i-1/3)*steepness)) -1./(1+exp(-(i-2/3)*steepness))) + ...
        c * (1-1./(1+exp(-(i-1/3)*steepness)))
end