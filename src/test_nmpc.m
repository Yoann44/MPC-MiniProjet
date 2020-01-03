close all
clear all
clc

quad = Quad();
CTRL = ctrlNMPC(quad);

sim = quad.sim(CTRL)
quad.plot(sim)