function CreateSoccerBall(xc, yc, zc, r)
n = 10;
[x,y,z] = sphere(n);

C(:,:,1) = 0.8*ones(n + 1);
C(:,:,2) = 0.8*ones(n + 1);
C(:,:,3) = 0.8*ones(n + 1);
surf(r*x + xc, r*y + yc, r*z + zc, C);
%shading interp