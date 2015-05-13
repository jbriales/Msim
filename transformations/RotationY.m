function Ry = RotationY( theta )
%Ry = RotationY( theta ) 3D rotation around the Y axis

s=sin(theta); c=cos(theta);

Ry = [c , 0 , s ;
      0 , 1 , 0;
      -s, 0 , c ];

end

