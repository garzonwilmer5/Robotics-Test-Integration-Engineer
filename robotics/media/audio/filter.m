%load audio
[y,Fs] = audioread("track2_orig.wav");
soundsc(y,Fs);
%% Bode Diagram
T = 1/Fs;                     % Sample time
L = numel(y);                     % Length of signal
NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(y,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);
% Plot single-sided amplitude spectrum.
plot(f,10*log10(2*abs(Y(1:NFFT/2+1)))) 
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('Y in db')
grid on
%% low pass filter
x = lowpass(y,1300,Fs);
%%
soundsc(y,Fs);
%%
soundsc(x,Fs);
filename = "track2.wav";
audiowrite(filename,y,Fs);