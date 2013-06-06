function [x_aposteriori,P_aposteriori]=positionKalmanFilter1D(A,B,C,x_aposteriori_k,P_aposteriori_k,u,z,gps_update,Q,R,thresh,decay)
%prediction
    x_apriori=A*x_aposteriori_k+B*u;
    P_apriori=A*P_aposteriori_k*A'+Q;
    if abs(u)<thresh
        x_apriori(2)=decay*x_apriori(2);
    end
    %update
    if gps_update==1
        y=z-C*x_apriori;
        S=C*P_apriori*C'+R;
        K=(P_apriori*C')/S;
        x_aposteriori=x_apriori+K*y;
        P_aposteriori=(eye(size(P_apriori))-K*C)*P_apriori;
    else
        x_aposteriori=x_apriori;
        P_aposteriori=P_apriori;
    end
end
