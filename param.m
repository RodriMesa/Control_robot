clc,clear
%% Robot
robot = importrobot('.\Modelos urdf brazo\Robot\urdf\Robot.urdf');
%robot = importrobot('..\Simulacion dinamica brazo\Modelos urdf brazo\Brazo\urdf\Brazo.urdf');
robot.DataFormat = 'column';
robot.Gravity=[0 0 -9.81];
robot.Bodies{1,1}.Joint.PositionLimits=[-180,180];
robot.Bodies{1,2}.Joint.PositionLimits=[-180,180];
robot.Bodies{1,3}.Joint.PositionLimits=[-180,180];
robot.Bodies{1,4}.Joint.PositionLimits=[-180,180];
%% Diseño de controladores PID
omega_b = 200; %rad/s
n = 2.5;
%% Diseño de observador
polo = 4*omega_b;
%% Prueba inercia
Jl=3.1*10^-6;
%% Tiempos de muestreo
Ts1 = (omega_b*25/(2*pi))^(-1); %% Tiempo de muestreo para lazo PID
Ts2 = (polo*25/(2*pi))^(-1);    %% Tiempo de muestreo para observador
Ts3 = Ts2;                   %% Tiempo de muestreo para lazo de torque
%% Información de sensores
int_encoder = 2*pi/10000;
%% Motor 1
%Parametros Físicos
r1=71.2;                %Relación de transmisión
L_1=0.25;               %Inductancia [H]
R_1=2;                  %Resistencia [ohm]
%J_1=25/100^2/1000;      %Inercia de actuador [Kg m^2]
J_1=Jl;
b_1=0;                  %Coeficiente de fricción viscosa [N s/rad]
%Control PID
ba_1=J_1 * omega_b * n;
Ksa_1=J_1 *n *omega_b^2;
Ksia_1=J_1*omega_b^3;
%Observador de estado
Ke_tita_1 = 3*polo;
Ke_omega_1 = 3*(polo^2);
Ke_zeta_1 = polo^3;
%Valores iniciales
omega0_1=0;
tita0_1=0;
%% Motor 2
%Parametros Físicos
r2=99.5;            %Relación de transmisión
L_2=0.25;           %Inductancia [H]
R_2=2;              %Resistencia [ohm]
%J_2=25/100^2/1000;     %Inercia de actuador [Kg m^2]
J_2=Jl;
b_2=0;              %Coeficiente de fricción viscosa [N s/rad]
%Control PID
ba_2=J_2 * omega_b * n;
Ksa_2=J_2 *n *omega_b^2;
Ksia_2=J_2*omega_b^3;
%Observador de estado
Ke_tita_2 = 3*polo;
Ke_omega_2 = 3*(polo^2);
Ke_zeta_2= polo^3;
%Valores iniciales
omega0_2=0;
tita0_2=0;
%% Motor 3
%Parametros Físicos
r3=171;             %Relación de transmisión
L_3=0.25;           %Inductancia [H]
R_3=2;              %Resistencia [ohm]
%J_3=25/100^2/1000;     %Inercia de actuador [Kg m^2]
J_3=Jl/3;
b_3=0;              %Coeficiente de fricción viscosa [N s/rad]
%Control PID
ba_3=J_3 * omega_b * n;
Ksa_3=J_3 *n *omega_b^2;
Ksia_3=J_3*omega_b^3;
%Observador de estado
Ke_tita_3 = 3*polo;
Ke_omega_3 = 3*(polo^2);
Ke_zeta_3 = polo^3;
%Valores iniciales
omega0_3=0;
tita0_3=0;
%% Motor 4
%Parametros Físicos
r4=298;             %Relación de transmisión
L_4=0.25;           %Inductancia [H]
R_4=2;              %Resistencia [ohm]
%J_4=25/100^2/1000;%Inercia de actuador [Kg m^2]
J_4=Jl/5;
b_4=0;              %Coeficiente de fricción viscosa [N s/rad]
%Control PID
ba_4=J_4 * omega_b * n;
Ksa_4=J_4 *n *omega_b^2;
Ksia_4=J_4*omega_b^3;
%Observador de estado
Ke_tita_4 = 3*polo;
Ke_omega_4 = 3*(polo^2);
Ke_zeta_4 = polo^3;
%Valores iniciales
omega0_4=0;
tita0_4=0;
%% Observador 1
A1 = [0 1;
    0 0];
B1 = [0;1/J_1];
C1 = [1 0];
D1=0;
%% Observador 2
A2 = [0 1;
    0 0];
B2 = [0;1/J_2];
C2 = [1 0];
D2=0;
%% Observador 3
A3 = [0 1;
    0 0];
B3 = [0;1/J_3];
C3 = [1 0];
D3=0;
%% Observador 4
A4 = [0 1;
    0 0];
B4 = [0;1/J_4];
C4 = [1 0];
D4=0;