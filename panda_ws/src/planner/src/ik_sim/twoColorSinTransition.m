function  [color] = twoColorSinTransition(i)
% if i == 1 return pink
% if i == around 0.5 return yellow
% if i == 0 return cyan
theme = "pan"; % "bi"; % 
if theme == "pan"
    p = [255, 27, 141]/255;
    y = [255, 218, 0]/255;
    c = [27, 179, 255]/255;
    colortop = p;
    colormid = y;
    colorlow = c;
elseif theme == "bi"
    r = [208, 0, 112]/255;
    v = [140, 71, 153]/255;
    b = [0, 50, 160]/255;
    colortop = r;
    colormid = v;
    colorlow = b;
    
end

%     p = [255, 27, 141]/255;
%     y = [255, 218, 0]/255;
%     c = [27, 179, 255]/255;

% steepness = 2

% plot(x,1./(1+exp(-(x-2/3)*steepness)))
% plot(x,1./(1+exp(-(x-1/3)*steepness)) -1./(1+exp(-(x-2/3)*steepness)))
% plot(x,1-1./(1+exp(-(x-1/3)*steepness)))

% color = p * .99 * (sin(max(i,0.5)*pi+pi/2).^2) + ... 
%         y * .99 * (sin(i*pi).^2) + ...
%         c * .99 * (sin(min(i,0.5)*pi+pi/2).^2)


color = colortop * .9 * (sin((i)*pi/2).^2) + colorlow * .9 * (sin((i)*pi/2-pi/2).^2);
% 
%         colormid * .99 * (sin(i*pi).^2) + ...

end