function [sb] = simulateSignal(fmcw, ranges, velocities, AddRandomTargets,  RayleighAmp)
%% simulateSignal() Summary
% Full RX baseband signal simulation of gaussian Radar targets


%Initialize Targets -------------------------------------------------------

% Received Signal Amplitude
amp = ones(1,length(ranges)+AddRandomTargets);
if AddRandomTargets >0
    for n = 1:AddRandomTargets
        if RayleighAmp
            amp(n) = random('Rayleigh', 0.5);
        end
    end
    %initialize random Velocities and Ranges for targets
    randRange = rand(1,AddRandomTargets)*fmcw.rangeBins(end-1);  % randRange > dR
    ranges = [ranges, randRange];
    randVel = (rand(1,AddRandomTargets)-0.5)*2*fmcw.velBins(end-1);
    velocities = [velocities, randVel];
end


%% Calculate Frequency Shifts ------------------------------------

% RD Frequency shifts
% DOPPLER f_dopp = -f0* 2* v_target/c0 
doppler_targets = -fmcw.f0 * 2/fmcw.c0 * velocities;
% RANGE f_R = 2*Bw*R/(t_chirp *c0)
% f_beat = f_dopp - f_R
beat_targets = doppler_targets - fmcw.sweepBw/fmcw.chirpTime * 2/fmcw.c0 *ranges;
% PHASE phi = 2*f0*R/c0
phase_targets = fmcw.f0*2/fmcw.c0*ranges;


%% Model Frequency Shifted Signal -------------------------------
dopplerIdxMat = meshgrid(0:fmcw.L-1, 1:fmcw.K); %Matrix 0:L in K lines
rangeIdxMat = transpose(meshgrid(0:fmcw.K-1, 1:fmcw.L)); %Matrix 0:K Samples(time) in L (Chirps) columns

timeSignal = zeros(fmcw.K, fmcw.L);
for target = 1:length(ranges)
    f_beat = beat_targets(target);
    f_doppler = doppler_targets(target);
    phase = phase_targets(target);
    amplitude = amp(target);

    % add signals for Frequency shifts (Range, Doppler and Phase shift)
    rangeMat   = exp(1j*2*pi * f_beat/fmcw.fs * rangeIdxMat); % t = over K Sample values(time) add f_R, to each Chirp (L lines)
    dopplerMat = exp(1j*2*pi * f_doppler*fmcw.chirpInterval* dopplerIdxMat); % t = over L Chirps add f_D, to each Sample (lines)
    phaseMat   = exp(1j*2*pi * phase); %phase difference due to Range

    timeSignal = timeSignal + amplitude*(dopplerMat.*rangeMat.*phaseMat);
end

% Return Simulated Baseband Signal
sb = real(timeSignal);
end

