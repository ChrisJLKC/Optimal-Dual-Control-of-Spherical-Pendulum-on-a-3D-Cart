clear all;
close all;
clc;

[Mx, My, xm, ym, zm, t] = CalcPendControlled();
AniPendulumOnCart(Mx, My, xm, ym, zm, t);
