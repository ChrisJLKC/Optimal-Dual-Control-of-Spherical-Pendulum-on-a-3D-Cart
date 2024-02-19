clear all;
close all;
clc;
%% Running Pendulum that is Controlled.
[Mx, My, xm, ym, zm, t] = CalcPendControlled();
AniPendulumOnCart(Mx, My, xm, ym, zm, t);
