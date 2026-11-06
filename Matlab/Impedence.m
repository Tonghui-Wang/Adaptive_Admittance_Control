function a = Impedence(p, v, pRef, vRef, aRef, normal, force, forceStar, penetration, mN, bN)
%Cartesian selective admittance. Rows are 1x3.
%normal points out of the material. When force is below forceStar the
%tool accelerates into the surface; extra force makes it yield outward.
%Tangential axes track the planned path and are not adapted.
c = Params();
n = normal(:).';
projector = eye(3) - (n.' * n);
errorT = (p - pRef) * projector;
velocityErrorT = (v - vRef) * projector;
accelT = (aRef * projector) + (-c.kT * errorT - c.bT * velocityErrorT) / c.mT;
speedN = v * n.';
if penetration <= 1e-6
    accelN = bN * (-c.approach - speedN) / mN;
else
    accelN = ((force - forceStar) - bN * speedN) / mN;
end
a = accelT + accelN * n;
end
