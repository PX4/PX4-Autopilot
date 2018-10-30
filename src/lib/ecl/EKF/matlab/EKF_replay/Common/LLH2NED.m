function posNED = LLH2NED(LLH,refLLH)

radius = 6378137;
flattening = 1/298.257223563;
e = sqrt(flattening*(2-flattening));
Rm = radius*(1-e^2)/(1-e^2*sin(refLLH(1)*pi/180)^2)^(3/2);
Rn = radius/(1-e^2*sin(refLLH(1)*pi/180)^2)^(1/2);
posN = (LLH(1,:)-refLLH(1))*pi/180.*(Rm+LLH(3,:));
posE = (LLH(2,:)-refLLH(2))*pi/180.*(Rn+LLH(3,:))*cos(refLLH(1)*pi/180);
posD = -(LLH(3,:)-refLLH(3));
posNED = [posN;posE;posD];
end