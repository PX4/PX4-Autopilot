function [x_aposteriori] = kalman_dlqe1(A,C,K,x_aposteriori_k,z)
    x_aposteriori=A*x_aposteriori_k+K*(z-C*A*x_aposteriori_k);      
end