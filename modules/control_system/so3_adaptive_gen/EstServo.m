function EstServo(t,p,u)
p = p/180*pi;
Ts = 0.005;
A = [-10 10; 0 -5]; 
B = [0; 1]; 
C = [1 0];
D = 0;
K = zeros(2,1);
m = idss(A,B,C,D,K,'Ts',0);

m.Structure.A.Free(2,1) = false;

m.Structure.B.Free(1,1) = false;
m.Structure.B.Free(2,1) = false;

m.Structure.C.Free(1,1) = false;
m.Structure.C.Free(1,2) = false;


opt = ssestOptions;
%opt.Display = 'on';
%opt.InitialState = 'backcast';
%opt.EnforceStability = true;
data = iddata(p,u,Ts);
sysEst = ssest(data,m,opt)
idssdata(sysEst)
figure
compare(data,m,sysEst);
A = sysEst.A;
kq = -A(1,1);
wc = -A(2,2)
fc = -A(2,2) / (2*pi)
wp = -A(1,2)/A(2,2)
fn = sqrt(A(1,2))/(2*pi)
eta = (wc + kq)/(2*sqrt(A(1,2)))
fd = fn*sqrt(1-eta*eta)
end