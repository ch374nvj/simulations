clear all
clc

%% VCO

Kvco = 100; % Hz/V
fo = 90; % Hz

%% Loop filter

Kp = 1;
Ki = 1e+4;
fc = 300;
Wc = 2*pi*fc;
tau = 1/Wc;
%%
t1 = tf([Kp Ki],[1 Wc 0])
bp = bodeplot(t1)
bp.FrequencyUnit = "Hz";