%%                                  Problem4: Simulate the baseband digital communication system designed in Problem 3
%%
clc;
close all;
clear all;
nbits=6;                    %No of bits taken from problem 3.
levels=(2^nbits)-1;         %Quantizer levels
L=100;
fs=8*10e3;                  %given sampling rate
nsamp=4;                    %times of sampling rate
Ts=1/fs;
%%                                  Generation of Quantizer samples
quant_samples = randi(levels,1,100);
ts=0:Ts:99*Ts;
figure; grid on;
subplot(3,1,1);
stem(ts,quant_samples,'Linewidth',1);
axis([0 100*Ts 0 65]);
title('Random 100 Quantized samples');
ylabel('Amplitude--->');
xlabel('Time--->');

%%                                  Conversion of quantizer alphabets into PCM sequence
                
codes=[];
for i=1:100
    temp=de2bi(quant_samples(i),6);
    codes=[codes,temp];
end
%%                                 Conversion of PCM sequence as rectangular pulse
        
s1=ones(1,4);
s0=zeros(1,4);
pcm_seq=[];   
for i=1:L*nbits
     if (codes(i)==0)
         pcm_seq=[pcm_seq,s0];
     elseif (codes(i)==1)
         pcm_seq=[pcm_seq,s1];
     end
end

%%                                        Plotting PCM Sequence

Ts=1/(4*6*fs);
ts=0:Ts:((nsamp*nbits*L)-1)*Ts;
subplot(3,1,2);grid on;
plot(ts,pcm_seq,'Linewidth',1);
axis([0 (nsamp*nbits*L)*Ts -1 2]);
title('PCM Signal:');
ylabel('Amplitude--->');
xlabel('Time--->');

%%                                        Conversion into 4-PAM

 pam_4=[];
 sym00=-3*s1;
 sym01=-1*s1;
 sym10=1*s1;
 sym11=3*s1;
 for i=1:2:nbits*L
     if (codes(i)==0 && codes(i+1)==0)          %Comparing with original 600-bit sequence
         pam_4=[pam_4,sym00,sym00];
     elseif (codes(i)==0 && codes(i+1)==1)
         pam_4=[pam_4,sym01,sym01];
     elseif (codes(i)==1 && codes(i+1)==0)
         pam_4=[pam_4,sym10,sym10];
     elseif (codes(i)==1 && codes(i+1)==1)
         pam_4=[pam_4,sym11,sym11];
     end
 end
%%                                          Plotting of 4-PAM

subplot(3,1,3); grid on;
plot(ts,pam_4,'Linewidth',1);
axis([0 2400*Ts -4 4]);
title('4-PAM Signal:');
ylabel('Amplitude--->');
xlabel('Time--->');

%%                                            Addition of AWGN

ak=[-3,-1,1,3]; %alphabets of PAM4 symbols 
% Every symbol of PAM is equiprobable
Es=1/4*sum(ak.^2); %energy of PAM signal
snr=6;
snr_mag=10^(snr/10);
sigma = sqrt(Es/snr_mag); %we know that SNR=E_s/variance of noise
mean_awgn=0;
noise_awgn = mean_awgn+sigma*randn(1,2400); %generating random gaussian noise of sigma variance
rxedSig = pam_4+noise_awgn;

%%                                          Plotting Received Signal with added AWGN noise

figure; grid on;
subplot(3,1,1);
plot(ts,rxedSig,'Linewidth',1);
axis([0 2400*Ts -5 5]);
title('Received 4-PAM Signal with AWGN noise added:');
ylabel('Amplitude--->');
xlabel('Time--->');


%%                                             Passing the received signal through Matched Filter
            
gt=ones(1,8);
Eg=sum(gt.^2);
filter_out=filter(gt/Eg,1,rxedSig);
subplot(3,1,2);
plot(ts,filter_out,'Linewidth',1);
axis([0 2400*Ts -4 4]);
title('Output of Matched filter with sampled output from Matched filter');
ylabel('Amplitude--->');
xlabel('Time--->');
hold on;
%%                                             Sampling the Matched filter output

k=1;
for i=8:8:numel(filter_out)
    z(k)=filter_out(i);             %Sampled output after matched filtering
    k=k+1;
end

stem((7*Ts:8*Ts:2400*Ts),z,'red');              %Plot filtered output                 
hold off;
%%                                                  Decision Making (Maximum-Likelihood Criteria)

%Decoding 4PAM from the received signal
ak=[-3,-1,1,3];          %PAM alphabets
decoded_pam=[];
for j=1:numel(z)
    for i=1:numel(ak)
        dist(i)=(z(j)-ak(i))^2;
    end
    [Y,index]=min(dist);
    decoded_pam=[decoded_pam,repelem(ak(index),8)];
end

% Plotting Decoded PAM                          

subplot(3,1,3);
grid on;
plot(ts,decoded_pam,'Linewidth',1);
axis([0 2400*Ts -4 4]);
title('Decoded 4-PAM Signal:');
ylabel('Amplitude--->');
xlabel('Time--->');
%%                                                      Decoded PCM Signal at Output
output_pcm=[];
for i=8:8:numel(decoded_pam)
    if(decoded_pam(i)==-3)
        output_pcm=[output_pcm,s0,s0];
    elseif(decoded_pam(i)==-1)
        output_pcm=[output_pcm,s0,s1];
    elseif(decoded_pam(i)==1)
         output_pcm=[output_pcm,s1,s0];
    elseif(decoded_pam(i)==3)
         output_pcm=[output_pcm,s1,s1];
    end
end
%Plotting decoded PCM Signal
figure;
subplot(3,1,1);grid on;
plot(ts,output_pcm,'LineWidth',1);
axis([0 2400*Ts -1 2]);
title('Decoded PCM output:');
ylabel('Amplitude--->');
xlabel('Time--->');

%%                                                          Reconstructed Quantized Output
q_sample_decoded=[];
for i=1:nbits:(nbits*L)
    pcm_sample=output_pcm(1,i:i+5);
    Qsample=bi2de(pcm_sample);
    q_sample_decoded=[q_sample_decoded,Qsample];
end
display(q_sample_decoded);
               % Plotting reconstructed quantizer output
subplot(3,1,2);
Ts=1/fs;
ts=0:Ts:99*Ts;
stem(ts,q_sample_decoded,'Linewidth',1);
axis([0 100*Ts 0 65]);
title('Quantized samples reconstructed at output');
ylabel('Amplitude--->');
xlabel('Time--->');


%%                                                          Comparison of transmitted and decoded PAM (BER for 4-PAM)

%Symbol error rate of 4-PAM
count_error_symbols=0;
for i=8:8:2400
    error=pam_4(i)-decoded_pam(i);
    if (error~=0)
        count_error_symbols=count_error_symbols+1;
    end
end
count_symbols=(nbits*L)/2;      % 1 symbol for every 2-bit
symbol_error_rate=count_error_symbols/count_symbols;
display(symbol_error_rate,'Symbol Error Rate for 4-PAM:');


%%                                                       Symbol Error rate and Bit Error Rate of PCM (BER and SER for PCM)
%Bit Error Rate
No_of_bit_errors=0;
temp=0;
error_pcm=output_pcm-pcm_seq;
e_pcm=nnz(error_pcm);
total_pcm_bits=length(pcm_seq);
pcm_ber=e_pcm/total_pcm_bits;
display(pcm_ber,'Bit Error Rate for PCM is');
temp=0;

%Symbol Error Rate
for j=4:4:2400
    error=pcm_seq(j)-output_pcm(j);
        if(error~=0)
            temp=temp+1;
        else
            continue
        end
end

count_sym_error=temp;
total_pcm_symbols=100;   %100 symbols of 6 bits each
pcm_ser=count_sym_error/total_pcm_symbols;
display(pcm_ser,'Symbol Error Rate for PCM is');


%%                                                          Calculation of SQNR
q=(63-0)/63;                     %step size = Vp-p/2^n-1, where n is 6
Qnoise_power=q^2/12;
Qsamples_power = std(q_sample_decoded)^2;                       %Power=sigma^2
SQNR=Qsamples_power/Qnoise_power;
SQNR_dB=10*log10(SQNR);
display(SQNR_dB,'SQNR in dB is');





