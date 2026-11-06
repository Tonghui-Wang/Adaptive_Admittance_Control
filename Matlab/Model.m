function [robot]=Model(a, alpha, d, theta, name)
%Modified-DH model. a and d are metres, angles are radians.

% l=[];
for i=1:6
    l(i)=RevoluteMDH('d',d(i), 'a',a(i), 'alpha',alpha(i), 'offset',theta(i));
end

robot=SerialLink(l);
robot.name=name;
end
