function encoded_sig = encode_func_qam(temp)
    if (temp==0)
        encoded_sig=complex(-3,3);
    elseif (temp==1)
        encoded_sig=complex(-1,3);
    elseif (temp==3)
        encoded_sig=complex(1,3);
    elseif (temp==2)
        encoded_sig=complex(3,3);
    elseif (temp==4)
        encoded_sig=complex(-3,1);
    elseif (temp==5)
        encoded_sig=complex(-1,1);
    elseif (temp==7)
        encoded_sig=complex(1,1);
    elseif (temp==6)
        encoded_sig=complex(3,1);
    elseif (temp==8)
        encoded_sig=complex(-3,-3);
    elseif (temp==9)
        encoded_sig=complex(-1,-3);
    elseif (temp==11)
        encoded_sig=complex(1,-3);
    elseif (temp==10)
        encoded_sig=complex(3,-3);
    elseif (temp==12)
        encoded_sig=complex(-3,-1);
    elseif (temp==13)
        encoded_sig=complex(-1,-1);
    elseif (temp==15)
        encoded_sig=complex(1,-1);
    elseif (temp==14)
        encoded_sig=complex(3,-1);    
    end
end