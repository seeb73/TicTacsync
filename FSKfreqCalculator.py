#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from math import pi,sin
import numpy as np
import matplotlib.pyplot as plt

# this computes f1 and f2 FSK frequencies so each signal
# STARTS and ENDS at y=0 on the DAC output
# in order to alleviate signal discontinuities (audio clics)
# at symbol transitions. A plot is produced to verify junctions are indeed smooth

N_SYMBOLS_SAMD21 = 35
f_sig1 = 600 # Hz, lower frequency
symbol_duration = 0.5/N_SYMBOLS_SAMD21 # in second
# symbol_duration = 10e-3 # second
ra, rb = (7, 4) # desired_ f2/f1 ratio expressed as two integers:
# the time one signal does 'rb' complete cycles, the other does
# 'ra' complete cycles
f2_f1_ratio = float(ra)/rb # float ratio

f_sample = 96e3 # Hz
sampling_period = 1/f_sample

NSamples = round(symbol_duration/sampling_period)
mode = 1
sig1_period = f_sample / f_sig1
mode_period = NSamples / mode
while (mode_period > sig1_period):
    mode_period = round(NSamples / mode)
    # print(mode, mode_period, sig1_period)
    mode += 1

mode -= 1
eff_f1 = (1/symbol_duration)*mode

msg = '\neffective f1: %.2f Hz with %i samples'
print(msg%(eff_f1, NSamples))

print('checks for f1:')
print('  %i sig1 cycles takes %.3f ms'%(mode, 1e3*(1/eff_f1)*mode))
print('  symbol ends after:  %.3f ms'%(1e3*NSamples/f_sample))
print('  audio sample each %.3f ms'%(1e3/f_sample))

goal_f2 = eff_f1 * f2_f1_ratio

mode = 1
sig2_period = f_sample / goal_f2
mode_period = NSamples / mode
while (mode_period > sig2_period):
    mode_period = round(NSamples / mode)
    # print(mode, mode_period, sig2_period)
    mode += 1

eff_f2 = (1/symbol_duration)*mode
msg = '\neffective f2: %.2f Hz with %i samples'
print(msg%(eff_f2, NSamples))

print('checks for f2:')
print('  %i sig2 cycles takes %.3f ms'%(mode, 1e3*(1/eff_f2)*mode))
print('  symbol ends after:  %.3f ms'%(1e3*NSamples/f_sample))
print('  audio sample each %.3f ms'%(1e3/f_sample))

print('''\nCut&Paste in Arduino code ppssyncYears.ino :
    
#define N_SYMBOLS %i
#define DAC_OUT_FREQ %i // in Hertz      
#define N_SAMPLES_WAVETABLE %i // %.3f ms each of %i symbols
#define f1 %.2f // for BFSK, in Hertz 
#define f2 %.2f 
'''%(N_SYMBOLS_SAMD21, f_sample, NSamples, 1e3*(1/eff_f2)*mode, N_SYMBOLS_SAMD21, eff_f1, eff_f2))

print('''\nCut&Paste in postprod code dualsoundsync.py :

F1 = %.2f # Hertz 
F2 = %.2f # Hz , both from FSKfreqCalculator output
SYMBOL_LENGTH = %.3f # ms, from FSKfreqCalculator.py
N_SYMBOLS_SAMD21 = %i # including sync pulse
    '''%(eff_f1, eff_f2, 1e3*(1/eff_f2)*mode, N_SYMBOLS_SAMD21))

isamples = np.arange(NSamples)
w1 = 2*np.pi*eff_f1/f_sample
s1 = np.sin(w1*isamples)
w2 = 2*np.pi*eff_f2/f_sample
s2 = np.sin(w2*isamples)

some_points = 400 # for plotting

time = 1e3*symbol_duration * isamples/NSamples

plt.hlines(0,0,20, color='black', linewidth=0.5)
plt.plot(time[:some_points], s1[:some_points], '.', color='blue', markersize=1)
plt.plot(time[:some_points], s2[:some_points], '.', color='red', markersize=1)
plt.plot(time[-some_points:], s1[-some_points:], '.', color='blue', markersize=1)
plt.plot(time[-some_points:], s2[-some_points:], '.', color='red', markersize=1)
plt.xlabel("Time (ms)")
plt.show()
