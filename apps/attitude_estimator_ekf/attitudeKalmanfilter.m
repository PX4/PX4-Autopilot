function [Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
%#codegen


%Extended Attitude Kalmanfilter
%   
    %state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]'
    %measurement vector z has the following entries [ax,ay,az||mx,my,mz||wmx,wmy,wmz]'
    %knownConst has the following entries [PrvaA,PrvarM,PrvarWO,PrvarW||MsvarA,MsvarM,MsvarW]
    %
    %[x_aposteriori,P_aposteriori] = AttKalman(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
    %
    %Example.... 
    %
    % $Author: Tobias Naegeli $    $Date: 2012 $    $Revision: 1 $
   

    %%define the matrices
    acc_ProcessNoise=knownConst(1);
    mag_ProcessNoise=knownConst(2); 
    ratesOffset_ProcessNoise=knownConst(3);
    rates_ProcessNoise=knownConst(4);
   
    
    acc_MeasurementNoise=knownConst(5);
    mag_MeasurementNoise=knownConst(6);
    rates_MeasurementNoise=knownConst(7);

     %process noise covariance matrix
     Q = [      eye(3)*acc_ProcessNoise,    zeros(3),                   zeros(3),                           zeros(3);
                zeros(3),                   eye(3)*mag_ProcessNoise,    zeros(3),                           zeros(3);
                zeros(3),                   zeros(3),                   eye(3)*ratesOffset_ProcessNoise,    zeros(3);
                zeros(3),                   zeros(3),                   zeros(3),                           eye(3)*rates_ProcessNoise];
    
     %measurement noise covariance matrix
     R = [   eye(3)*acc_MeasurementNoise,       zeros(3),                       zeros(3);
                 zeros(3),                          eye(3)*mag_MeasurementNoise,    zeros(3);
                 zeros(3),                          zeros(3),                       eye(3)*rates_MeasurementNoise];
    

    %observation matrix
    H_k=[   eye(3),     zeros(3),   zeros(3),   zeros(3);
            zeros(3),   eye(3),     zeros(3),   zeros(3);
            zeros(3),   zeros(3),   eye(3),     eye(3)];
        
    %compute A(t,w)
    
    %x_aposteriori_k[10,11,12] should be [p,q,r]
    %R_temp=[1,-r, q
    %        r, 1, -p
    %       -q, p, 1]

    R_temp=[1,-dt*x_aposteriori_k(12),dt*x_aposteriori_k(11);
        dt*x_aposteriori_k(12),1,-dt*x_aposteriori_k(10);
        -dt*x_aposteriori_k(11), dt*x_aposteriori_k(10),1];
    
    %strange, should not be transposed
    A_pred=[R_temp',     zeros(3),   zeros(3),   zeros(3);
        zeros(3),   R_temp',     zeros(3),   zeros(3);
        zeros(3),   zeros(3),   eye(3),     zeros(3);
        zeros(3),   zeros(3),   zeros(3),   eye(3)];
    
    %%prediction step
    x_apriori=A_pred*x_aposteriori_k;

    %linearization
    acc_temp_mat=[0,              dt*x_aposteriori_k(3),    -dt*x_aposteriori_k(2);
        -dt*x_aposteriori_k(3), 0,                  dt*x_aposteriori_k(1);
        dt*x_aposteriori_k(2), -dt*x_aposteriori_k(1),    0];
    
    mag_temp_mat=[0,              dt*x_aposteriori_k(6),    -dt*x_aposteriori_k(5);
        -dt*x_aposteriori_k(6), 0,                  dt*x_aposteriori_k(4);
        dt*x_aposteriori_k(5), -dt*x_aposteriori_k(4),    0];
    
    A_lin=[R_temp',     zeros(3),   zeros(3),   acc_temp_mat';
        zeros(3),   R_temp',     zeros(3),   mag_temp_mat';
        zeros(3),   zeros(3),   eye(3),     zeros(3);
        zeros(3),   zeros(3),   zeros(3),   eye(3)];
    
    
    P_apriori=A_lin*P_aposteriori_k*A_lin'+Q;
    
    
    %%update step

    y_k=z_k-H_k*x_apriori;
    S_k=H_k*P_apriori*H_k'+R;
    K_k=(P_apriori*H_k'/(S_k));

    
    x_aposteriori=x_apriori+K_k*y_k;
    P_aposteriori=(eye(12)-K_k*H_k)*P_apriori;


    %%Rotation matrix generation
    
    earth_z=x_aposteriori(1:3)/norm(x_aposteriori(1:3));
    earth_x=cross(earth_z,x_aposteriori(4:6)/norm(x_aposteriori(4:6)));
    earth_y=cross(earth_x,earth_z);
    
    Rot_matrix=[earth_x,earth_y,earth_z];
    






