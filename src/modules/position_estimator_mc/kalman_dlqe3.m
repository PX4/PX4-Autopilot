function [x_aposteriori] = kalman_dlqe3(dt,k1,k2,k3,x_aposteriori_k,z,posUpdate,addNoise,sigma)
    st = 1/2*dt^2;
    A = [1,dt,st;
        0,1,dt;
        0,0,1];
    C=[1,0,0];
    K = [k1;k2;k3];
    if addNoise==1
        noise = sigma*randn(1,1);
        z = z + noise;
    end
    if(posUpdate)
        x_aposteriori=A*x_aposteriori_k+K*(z-C*A*x_aposteriori_k);
    else
        x_aposteriori=A*x_aposteriori_k;
    end
end