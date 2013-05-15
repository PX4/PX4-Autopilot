function  [x_aposteriori,P_aposteriori]=positionKalmanFilter1D_dT(dT,x_aposteriori_k,P_aposteriori_k,u,z,gps_update,Q,R,thresh,decay)
    %dynamics
    A = [1 dT -0.5*dT*dT;
         0 1 -dT;
         0 0 1];
    B = [0.5*dT*dT; dT; 0];
    C = [1 0 0];
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

