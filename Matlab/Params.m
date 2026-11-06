function c = Params()
%Shared scenario constants. Keep these aligned with Python/control.py.
c.dt = 0.01;
c.fStar = 10;
c.fRamp = 0.4;
c.kSoft = 800;
c.kStiff = 8000;
c.kHat0 = 2000;
c.approach = 0.04;
c.adaptEvery = 25;
c.horizon = 40;
c.particles = 12;
c.iters = 8;
c.mBounds = [0.4, 6];
c.bBounds = [20, 400];
c.vmax = [0.35, 30];
c.rate = [0.35, 40];
c.lambda = 0.05;
c.nullGain = 1.2;
c.qdotLimit = 1.5;
c.mT = 1.5;
c.bT = 70;
c.kT = 350;
end
