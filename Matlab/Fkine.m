function [p]=Fkine(q)

a1=150;
a2=750;
a3=155;
d1=253;
d4=800;
d6=154;

s1=sind(q(1));
s2=sind(q(2));
s23=sind(q(2)+q(3));
s4=sind(q(4));
s5=sind(q(5));
s6=sind(q(6));
c1=cosd(q(1));
c2=cosd(q(2));
c23=cosd(q(2)+q(3));
c4=cosd(q(4));
c5=cosd(q(5));
c6=cosd(q(6));

nx=c6*(c1*c23*c5 + s5*(-c1*c4*s23 + s1*s4)) + s6*(c1*s23*s4 + c4*s1);
ox=c6*(c1*s23*s4 + c4*s1) - s6*(c1*c23*c5 + s5*(-c1*c4*s23 + s1*s4));
ax=c1*c23*s5 - c5*(-c1*c4*s23 + s1*s4);
px=c1*(a1 - a2*s2 - a3*s23 + c23*d4) - d6*(-c1*c23*s5 + c5*(-c1*c4*s23 + s1*s4));
ny=-c6*(-c23*c5*s1 + s5*(c1*s4 + c4*s1*s23)) + s6*(-c1*c4 + s1*s23*s4);
oy=c6*(-c1*c4 + s1*s23*s4) + s6*(-c23*c5*s1 + s5*(c1*s4 + c4*s1*s23));
ay=c23*s1*s5 + c5*(c1*s4 + c4*s1*s23);
py=d6*(c23*s1*s5 + c5*(c1*s4 + c4*s1*s23)) + s1*(a1 - a2*s2 - a3*s23 + c23*d4);
nz=-c23*s4*s6 + c6*(c23*c4*s5 + c5*s23);
oz=-c23*c6*s4 - s6*(c23*c4*s5 + c5*s23);
az=-c23*c4*c5 + s23*s5;
pz=a2*c2 + a3*c23 + d1 + d4*s23 + d6*(-c23*c4*c5 + s23*s5);

T=[nx,ox,ax,px;
    ny,oy,ay,py;
    nz,oz,az,pz;
    0,0,0,1];

p=[T(1:3,4)',rad2deg(tform2eul(T,'ZYX'))];
end
