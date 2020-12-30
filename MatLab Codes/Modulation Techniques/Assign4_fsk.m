clc;
clear all;
num_bit = 100000;
bits = transpose(randi([0 1],num_bit,1));
Binary = reshape(bits,2,(num_bit/2));
symbolBinary=transpose(Binary);
signalSpace = zeros((num_bit/2),4);
receivedSignal = zeros((num_bit/2),1);
decoded_bit=zeros((num_bit/2),2);
M=4;
k=log2(M);
sym=eye(M);
%%                  Mapping to FSK points
for i=1:(num_bit/2)
    temp=bi2de(symbolBinary(i,:));
    if (temp==0)
        signalSpace(i,:)=sym(1,:);
    elseif (temp==1)
        signalSpace(i,:)=sym(2,:);
    elseif (temp==2)
        signalSpace(i,:)=sym(3,:);
    elseif (temp==3)
        signalSpace(i,:)=sym(4,:);
    end
end
%%                          Addition of AWGN
SNRdB=-5:10;
SNR=10.^(SNRdB/10);
for k=1:length(SNRdB)
receivedSignal=awgn(complex(signalSpace),SNRdB(k));
error=0;
for j=1:1:(num_bit/2)
    [val,index]=max(receivedSignal(j,:));
    decoded=de2bi((index-1),2);
    for c=1:2
        if (decoded(:,c)==0  && symbolBinary(j,c)==1)||(decoded(:,c)==1  && symbolBinary(j,c)==0)
            error=error+1;
        end
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
title(' curve for Bit Error Rate verses  SNR for 4-FSK modulation');
xlabel(' SNR(dB)');
ylabel('BER');
axis([-5 10 10^-5 1]);