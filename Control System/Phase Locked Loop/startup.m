clear all
clc

%% VCO

Kvco = 100; % Hz/V
fo = 90; % Hz

%% Loop filter

Kp = 50;
Ki = 100;

t1 = tf([Kp Ki],[1 0])
bode(t1)