clc;
clear all;
num_bit= 100000;
bits = transpose(randi([0 1],num_bit,1));
Binary = reshape(bits,4,(num_bit/4));
symbolBinary=transpose(Binary);
signalSpace = zeros((num_bit/4),1);
outputSignal=zeros((num_bit/4),1);
%receivedSignal = zeros((num_bit/4),1);
decoded=zeros((num_bit/4),1);
M=16;
%%                  Mapping to QAM points
for i=1:(num_bit/4)
    temp=bi2de(symbolBinary(i,:));
    signalSpace(i)=encode_func_qam(temp);                   %function encode used
end
%%                          Addition of AWGN
SNRdB=-5:10;
SNR=10.^(SNRdB/10);
for k=1:length(SNRdB)
receivedSignal=awgn(complex(signalSpace),SNRdB(k));
error=0;
%%                          Decode and find error
for j=1:(num_bit/4)
decoded=de2bi(decode_func_qam(receivedSignal(j)),4);
     for c=1:4
        if (decoded(:,c)==0  && symbolBinary(j,c)==1)||(decoded(:,c)==1  && symbolBinary(j,c)==0)
            error=error+1;
        end
    end
end
%%                          Calculation of BER
error=error/num_bit;
m(k)=error;
end
display(m);
figure(1) 
%plot start
semilogy(SNRdB,m,'o','linewidth',2.5),grid on,hold on;
title(' curve for Bit Error Rate verses  SNR for 16-QAM modulation');
xlabel(' SNR(dB)');
ylabel('BER');
axis([-5 10 10^-5 1]);   
