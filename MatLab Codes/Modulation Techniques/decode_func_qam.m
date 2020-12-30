function decoded_sig = decode_func_qam(sig)
    if imag(sig)>2
        if real(sig)<-2
            decoded_sig=0;
        elseif (real(sig)>-2) && (real(sig)<0)
            decoded_sig=1;
        elseif (real(sig)>0) && (real(sig)<2)
            decoded_sig=3;
        else
            decoded_sig=2;
        end
        
    elseif (imag(sig)>0) && (imag(sig)<2)
        if real(sig)<-2
            decoded_sig=4;
        elseif (real(sig)>-2 && real(sig)<0)
            decoded_sig=5;
        elseif (real(sig)>0 && real(sig)<2)
            decoded_sig=7;
        else
            decoded_sig=6;
        end
        
     elseif (imag(sig)<0 && imag(sig)>-2)
        if real(sig)<-2
            decoded_sig=12;
        elseif (real(sig)>-2 && real(sig)<0)
            decoded_sig=13;
        elseif (real(sig)>0 && real(sig)<2)
            decoded_sig=15;
        else
            decoded_sig=14;
        end
       
     elseif imag(sig)< -2
        if real(sig)<-2
            decoded_sig=8;
        elseif (real(sig)>-2 && real(sig)<0)
            decoded_sig=9;
        elseif (real(sig)>0 && real(sig)<2)
            decoded_sig=11;
        else
            decoded_sig=10;
        end
    end
end