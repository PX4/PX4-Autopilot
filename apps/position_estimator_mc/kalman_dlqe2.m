function [x_aposteriori] = kalman_dlqe2(dt,k1,k2,k3,x_aposteriori_k,z)
    st = 1/2*dt^2;
    A = [1,dt,st;
        0,1,dt;
        0,0,1];
    C=[1,0,0];
    K = [k1;k2;k3];
    x_aposteriori=A*x_aposteriori_k+K*(z-C*A*x_aposteriori_k);      
end