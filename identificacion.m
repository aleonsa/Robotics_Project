function [Theta] = identificacion(q,qdot,qrddot,s,dt,Theta_anterior)
    
    Gamma = 0.01*eye(8);

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    
    qdot1 = qdot(1);
    qdot2 = qdot(2);
    qdot3 = qdot(3);

    qrddot1 = qrddot(1);
    qrddot2 = qrddot(2);
    qrddot3 = qrddot(3);

    % Constantes útiles
%     s1 = sin(q1);
%     c1 = cos(q1);
    s2 = sin(q2);
    c2 = cos(q2);
    s3 = sin(q3);
    c3 = cos(q3);
    s23 = sin(q2 + q3);
    c23 = cos(q2 + q3);
    
    % Gravedad
    g0 = 9.81;  % Aceleración debida a la gravedad (m/s^2)

    Y = zeros(length(q1)*3,8);

    Y(1,1) = c2^2*qrddot1-2*c2*s2*qdot1*qdot2;
    Y(1,2) = c2*c23*qrddot1-(c2*s23*(qdot2+qdot3)+s2*c23*qdot2)*qdot1;
    Y(1,3) = s23^2*qrddot1+c23*s23*(qdot2+2*qdot3)*qdot1+s23*c23*qdot1*qdot2;
    Y(1,4) = qdot1;
    Y(1,5) = 0;
    Y(1,6) = 0;
    Y(1,7) = 0;
    Y(1,8) = 0;
    Y(2,1) = qrddot2+c2*s2*qdot1^2;
    Y(2,2) = 2*c3*qrddot2+c3*qrddot3+0.5*(s2*c23+c2*s23)*qdot1^2-2*s3*qdot2*qdot3-s3*qdot3^2;
    Y(2,3) = qrddot2+qrddot3-s23*c23*qdot1^2;
    Y(2,4) = 0;
    Y(2,5) = qdot2;
    Y(2,6) = 0;
    Y(2,7) = g0*c2;
    Y(2,8) = g0*c23;
    Y(3,1) = 0;
    Y(3,2) = c2*qrddot2+0.5*c2*s23*qdot1^2+s3*qdot2^2;
    Y(3,3) = qrddot2+qrddot3-c23*s23*qdot1^2;
    Y(3,4) = 0;
    Y(3,5) = 0;
    Y(3,6) = qdot3;
    Y(3,7) = 0;
    Y(3,8) = g0*c23;

    theta_error_dot = -Gamma*Y'*s;

    Theta = Theta_anterior + theta_error_dot * dt; % Actualización con integración explícita
end

