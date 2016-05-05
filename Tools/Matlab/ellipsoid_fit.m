% Copyright (c) 2009, Yury Petrov
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in
%       the documentation and/or other materials provided with the distribution
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%

function [ center, radii, evecs, v ] = ellipsoid_fit( X, flag, equals )
%
% Fit an ellispoid/sphere to a set of xyz data points:
%
%   [center, radii, evecs, pars ] = ellipsoid_fit( X )
%   [center, radii, evecs, pars ] = ellipsoid_fit( [x y z] );
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, 1 );
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, 2, 'xz' );
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, 3 );
%
% Parameters:
% * X, [x y z]   - Cartesian data, n x 3 matrix or three n x 1 vectors
% * flag         - 0 fits an arbitrary ellipsoid (default),
%                - 1 fits an ellipsoid with its axes along [x y z] axes
%                - 2 followed by, say, 'xy' fits as 1 but also x_rad = y_rad
%                - 3 fits a sphere
%
% Output:
% * center    -  ellispoid center coordinates [xc; yc; zc]
% * ax        -  ellipsoid radii [a; b; c]
% * evecs     -  ellipsoid radii directions as columns of the 3x3 matrix
% * v         -  the 9 parameters describing the ellipsoid algebraically: 
%                Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
%
% Author:
% Yury Petrov, Northeastern University, Boston, MA
%

error( nargchk( 1, 3, nargin ) );  % check input arguments
if nargin == 1
    flag = 0;  % default to a free ellipsoid
end
if flag == 2 && nargin == 2
    equals = 'xy';
end
    
if size( X, 2 ) ~= 3
    error( 'Input data must have three columns!' );
else
    x = X( :, 1 );
    y = X( :, 2 );
    z = X( :, 3 );
end

% need nine or more data points
if length( x ) < 9 && flag == 0 
   error( 'Must have at least 9 points to fit a unique ellipsoid' );
end
if length( x ) < 6 && flag == 1
   error( 'Must have at least 6 points to fit a unique oriented ellipsoid' );
end
if length( x ) < 5 && flag == 2
   error( 'Must have at least 5 points to fit a unique oriented ellipsoid with two axes equal' );
end
if length( x ) < 3 && flag == 3
   error( 'Must have at least 4 points to fit a unique sphere' );
end

if flag == 0
    % fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
    D = [ x .* x, ...
          y .* y, ...
          z .* z, ...
      2 * x .* y, ...
      2 * x .* z, ...
      2 * y .* z, ...
      2 * x, ...
      2 * y, ... 
      2 * z ];  % ndatapoints x 9 ellipsoid parameters
elseif flag == 1
    % fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1
    D = [ x .* x, ...
          y .* y, ...
          z .* z, ...
      2 * x, ...
      2 * y, ... 
      2 * z ];  % ndatapoints x 6 ellipsoid parameters
elseif flag == 2
    % fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1,
    % where A = B or B = C or A = C
    if strcmp( equals, 'yz' ) || strcmp( equals, 'zy' )
        D = [ y .* y + z .* z, ...
            x .* x, ...
            2 * x, ...
            2 * y, ...
            2 * z ];
    elseif strcmp( equals, 'xz' ) || strcmp( equals, 'zx' )
        D = [ x .* x + z .* z, ...
            y .* y, ...
            2 * x, ...
            2 * y, ...
            2 * z ];
    else
        D = [ x .* x + y .* y, ...
            z .* z, ...
            2 * x, ...
            2 * y, ...
            2 * z ];
    end
else
    % fit sphere in the form A(x^2 + y^2 + z^2) + 2Gx + 2Hy + 2Iz = 1
    D = [ x .* x + y .* y + z .* z, ...
      2 * x, ...
      2 * y, ... 
      2 * z ];  % ndatapoints x 4 sphere parameters
end

% solve the normal system of equations
v = ( D' * D ) \ ( D' * ones( size( x, 1 ), 1 ) );

% find the ellipsoid parameters
if flag == 0
    % form the algebraic form of the ellipsoid
    A = [ v(1) v(4) v(5) v(7); ...
          v(4) v(2) v(6) v(8); ...
          v(5) v(6) v(3) v(9); ...
          v(7) v(8) v(9) -1 ];
    % find the center of the ellipsoid
    center = -A( 1:3, 1:3 ) \ [ v(7); v(8); v(9) ];
    % form the corresponding translation matrix
    T = eye( 4 );
    T( 4, 1:3 ) = center';
    % translate to the center
    R = T * A * T';
    % solve the eigenproblem
    [ evecs evals ] = eig( R( 1:3, 1:3 ) / -R( 4, 4 ) );
    radii = sqrt( 1 ./ diag( evals ) );
else
    if flag == 1
        v = [ v(1) v(2) v(3) 0 0 0 v(4) v(5) v(6) ];
    elseif flag == 2
        if strcmp( equals, 'xz' ) || strcmp( equals, 'zx' )
            v = [ v(1) v(2) v(1) 0 0 0 v(3) v(4) v(5) ];
        elseif strcmp( equals, 'yz' ) || strcmp( equals, 'zy' )
            v = [ v(2) v(1) v(1) 0 0 0 v(3) v(4) v(5) ];
        else % xy
            v = [ v(1) v(1) v(2) 0 0 0 v(3) v(4) v(5) ];
        end
    else
        v = [ v(1) v(1) v(1) 0 0 0 v(2) v(3) v(4) ];
    end
    center = ( -v( 7:9 ) ./ v( 1:3 ) )';
    gam = 1 + ( v(7)^2 / v(1) + v(8)^2 / v(2) + v(9)^2 / v(3) );
    radii = ( sqrt( gam ./ v( 1:3 ) ) )';
    evecs = eye( 3 );
end


