quad = Quad();

[xs,us] = quad.trim();
sys = quad.linearize(xs, us);
systransformed = sys*inv(quad.T);
[sysx, sysy, sysz, sysyaw] = quad.decompose(sys, xs, us);