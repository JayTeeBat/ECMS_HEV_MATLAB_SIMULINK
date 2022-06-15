function [u]=parallelhybrid_ECMS(x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This files calculates torque split for simulink model. 
% Given inputs from the driving cycle and controller this script should return
% The torques on the electric motor and combustion engine.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Input data from Simulink Model
P_em=x(1);   % Power request from E_machine
lambda=x(2);  % Equivalence Factor
SOC_t=x(3);  % Battery SOC

%% H2 Fuel and Air Parameters
H_l= 120e6; % Lower Heating Value of H2 [J/kg]
roha_air=1.18; % Air Density [kg/m3] 

%% Fuel Cell Configuration

eta_dcdc = 0.98; % DCDC efficiency

%% Battery Configuration
Q_0=6.5; % Battery charging capacity [Ah]
U_oc=300; % Open circuit voltage [V]
I_max= 200;% Maximum dis-/charging current [A]
I_min=-200; % Maximum dis-/charging current [A]
R_i= 0.65; % Inner resistance [ohm]
    
%% TotalPowertrain Electrical Power
P_total_max=90.8; % Total powertrain power[kW]

%-----------------------Cost Vector Calculations--------------------------%
%% Battery Model

I=linspace(I_min,I_max,50000);

P_bat=(U_oc*I)-(R_i*I.^2);   % Power of Battery


%% Fuel Cell Model

P_DCDC=P_em-P_bat;  % DCDC required power

x=((p_me_0*V_d)/(4*pi));  % First Part to calculate Fuel consumption

y=J_e*dw_ice; % Second Part to calculate Fuel consumption

costVector=(w_ice/(e*H_l))*(T_ice+x+y); % Fuel Consumption Cost Vector for given t_vec


%% Hamiltonian Calculation [Torque Split]

P_H2=H_l*costVector; % Fuel Power
P_H2(costVector<0)=0; % Constraints
P_batrev=I*U_oc;  % Electro-chemical Power for Battery

%%% If condition for torque split
if 0
    T_ice=0;
    T_em=0;
    u=[T_ice;T_em];
else
    H=P_H2 + lambda*P_batrev; % Hamiltonian update

    H((T_ice+(J_e*dw_ice))>T_ice_max)=inf;
    H(T_em>T_em_max | T_em<-T_em_max)=Inf;
    H(Power_electric_machine<-P_em_max*1000 | Power_electric_machine>P_em_max*1000)=Inf;
    [q,i]=min(H);
    P_DCDC=P_DCDC(i);
    P_bat=P_bat(i);
    u=[P_DCDC;P_bat];
end
   


