function Rx = RotationX( theta )
%Rx = RotationX( theta ) 3D rotation around the X axis

s=sin(theta); c=cos(theta);

Rx = [1 , 0 , 0 ;
      0 , c , -s;
      0 , s , c ];

end

