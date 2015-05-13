function Rz = RotationZ( theta )
%Rz = RotationZ( theta ) 3D rotation around the Z axis

s=sin(theta); c=cos(theta);

Rz = [c , -s, 0 ;
      s , c , 0 ;
      0 , 0 , 1 ];

end

