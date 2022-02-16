function [ofs,gain,rotM]=ellipsoid_fit(XYZ,varargin)

% Fit an (non)rotated ellipsoid or sphere to a set of xyz data points

% XYZ: N(rows) x 3(cols), matrix of N data points (x,y,z)

% optional flag f, default to 0 (fitting of rotated ellipsoid)

x=XYZ(:,1); y=XYZ(:,2); z=XYZ(:,3); if nargin>1, f=varargin{1}; else f=0; end;

if f==0, D=[x.*x, y.*y, z.*z, 2*x.*y,2*x.*z,2*y.*z, 2*x,2*y,2*z]; % any axes (rotated ellipsoid)

elseif f==1, D=[x.*x, y.*y, z.*z, 2*x,2*y,2*z]; % XYZ axes (non-rotated ellipsoid)

elseif f==2, D=[x.*x+y.*y, z.*z, 2*x,2*y,2*z]; % and radius x=y

elseif f==3, D=[x.*x+z.*z, y.*y, 2*x,2*y,2*z]; % and radius x=z

elseif f==4, D=[y.*y+z.*z, x.*x, 2*x,2*y,2*z]; % and radius y=z

elseif f==5, D=[x.*x+y.*y+z.*z, 2*x,2*y,2*z]; % and radius x=y=z (sphere)

end; 

v = (D'*D)\(D'*ones(length(x),1)); % least square fitting

if f==0, % rotated ellipsoid

 A = [ v(1) v(4) v(5) v(7); v(4) v(2) v(6) v(8); v(5) v(6) v(3) v(9); v(7) v(8) v(9) -1 ];

 ofs=-A(1:3,1:3)\[v(7);v(8);v(9)]; % offset is center of ellipsoid

 Tmtx=eye(4); Tmtx(4,1:3)=ofs'; AT=Tmtx*A*Tmtx'; % ellipsoid translated to (0,0,0)

 [rotM ev]=eig(AT(1:3,1:3)/-AT(4,4)); % eigenvectors (rotation) and eigenvalues (gain)

 gain=sqrt(1./diag(ev)); % gain is radius of the ellipsoid

else % non-rotated ellipsoid

 if f==1, v = [ v(1) v(2) v(3) 0 0 0 v(4) v(5) v(6) ];

 elseif f==2, v = [ v(1) v(1) v(2) 0 0 0 v(3) v(4) v(5) ];

 elseif f==3, v = [ v(1) v(2) v(1) 0 0 0 v(3) v(4) v(5) ];

 elseif f==4, v = [ v(2) v(1) v(1) 0 0 0 v(3) v(4) v(5) ];

 elseif f==5, v = [ v(1) v(1) v(1) 0 0 0 v(2) v(3) v(4) ]; % sphere

 end;

 ofs=-(v(1:3).\v(7:9))'; % offset is center of ellipsoid

 rotM=eye(3); % eigenvectors (rotation), identity = no rotation

 g=1+(v(7)^2/v(1)+v(8)^2/v(2)+v(9)^2/v(3));

 gain=(sqrt(g./v(1:3)))'; % find radii of the ellipsoid (scale)

end; 
