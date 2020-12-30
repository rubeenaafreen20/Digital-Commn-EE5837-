%%                              Generating Random Binary data
clc;
clear all;
num_bit = 100000;
M=2;                                        %for bpsk
Es=1;
bits = randi([0 1],num_bit,1);
symbolBinary=bits';               %Generated bit

%%                          Mapping to bpsk constellation
signalSpace=2*bits-1;
%%                          Addition of AWGN
SNRdB=-5:10;
SNR=10.^(SNRdB/10);
for k=1:length(SNRdB)
receivedSignal=awgn(complex(signalSpace),SNRdB(k));
error=0;
%%                           Decoding
for c=1:1:num_bit
    if receivedSignal(c)>0
        decoded_bit=1;
    else
        decoded_bit=0;
    end
    if (decoded_bit==0 && symbolBinary(c)==1)||(decoded_bit==1 &&symbolBinary(c)==0) 
        error=error+1;
    end
end
%%                          Error Calculation
error=error/num_bit; %Calculate error/bit
m(k)=error;
end
display(m);
figure(1) 
%plot start
semilogy(SNRdB,m,'o','linewidth',2.5),grid on,hold on;
title(' curve for Bit Error Rate verses  SNR for Binary PSK modulation');
xlabel(' SNR(dB)');
ylabel('BER');
axis([-5 10 10^-5 1]);