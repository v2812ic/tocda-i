S=122	%Superficie alar


m=101000	%Masa del avión
E=200000	%Empuje (Proveniente de la función calcularmotor)

vx=0	%Velocidad inicial en el eje x
vy=0	%Velocidad inicial en el eje y
x=0     %Distancia inicial en el eje x
y=0     %Distancia inicial en el eje y
R=0	    %Distancia en el plano horizontal
h=0	    %Altura inicial
beta=0

t=1     %Duración del intervalo
T=0	    %Tiempo de vuelo

while T<=10
	if vx==0
		alpha=5
	else
		alpha=5-atand(vy/vx)
	end
	if alpha<=10
		cl=0.1*alpha+0.5
	else
		cl=-(0.009*alpha^2)+0.298*alpha-0.59
	end
	cd=0.00036*alpha^2+0.00036*alpha+0.029
	if h<=11000
		rho=1.225*(1-0.000022558*h)^4.2559
	else
		rho=0.3639*exp(-0.00015769*(h-11000))
	end

	v0=sqrt(vx^2+vy^2)
	L=1/2*rho*v0^2*S*cl
	D=1/2*rho*v0^2*S*cd
	Fx=E-D-m*9.8*sin(beta)
	Fy=L-m*9.8*cos(beta)
    %Aceleración, velocidad y espacio en eje x avión
	ax=Fx/m
	vx=vx+ax*t
	x=1/2*ax*t^2+vx*t+x

    %Aceleración, velocidad y espacio en eje y avión
    disp(h<=0 && Fy<0)

    if h<=0 && Fy<0
        ay=0
        vy=0
        y=0
    else
        ay=Fy/m
        vy=vy+ay*t
	    y=1/2*ay*t^2+vy*t+y
    end
	R=x*cos(beta)-y*sin(beta)
	h=x*sin(beta)+y*cos(beta)
	T=T+t
    disp(T)
end
