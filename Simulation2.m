clc;

% Importent changebale varibalse 
Antall_Vinkler = 200;               % Number of Drone positions
NrAntenne      = 32;                % Number of antennas in Subarray
SNR            = 20;                % Signal to Noise ratio
std            = 0.05;              % Position Uncertenty 
UpSample       = 10000;             % Interprolation 

std_per_dim    = std / sqrt(3); 
A(NrAntenne)   = Antenne_Object();
% Lager en tilfeldig Amplitude feil [0, 1] og fasefeil [0, 2pi]
Cal_Offset       = rand(NrAntenne, 2); 
Cal_Offset(:, 2) = Cal_Offset(:, 2) * 2*pi; 
% Cal_Offset(:, 1) = 1; Cal_Offset(:, 2) = 0;
Drone          = [0, 10, 0];

% Place the Antennas  
Teller1 = 0; Teller2 = 0;  Polarisation = 0;
for i = 1:NrAntenne
    A(i).A_Pos        = [(Teller1)*0.11, 0, +Teller2*0.11];
    A(i).Polarisation = Polarisation;
    if Polarisation == 0; Polarisation = 1; else; Polarisation = 0; end
    

    Teller1 = Teller1 + 1;
    if Teller1 == 4
        Teller1 = 0;
        Teller2 = Teller2 + 1;
    end
    
    if mod(Teller2, 2) == 0 && Teller1 == 0; Polarisation = 0; elseif mod(Teller2, 2) ~= 0 && Teller1 == 0;  Polarisation = 1; end

end



Bits = randi([0, 1], A(1).N*4, 1);
Tx = qammod(Bits, 16, InputType="bit").';     % Sendt Signal'

Rx    = zeros(NrAntenne, 2000);
H_FFT = zeros(NrAntenne, 2000);
hTime = zeros(NrAntenne, A(1).N * A(1).NN);

peaks = zeros(2, 1);
dPhase       = zeros(NrAntenne, Antall_Vinkler);
dAmp         = zeros(NrAntenne, Antall_Vinkler);
dPhase_E     = zeros(NrAntenne, Antall_Vinkler);
dAmp_E       = zeros(NrAntenne, Antall_Vinkler);


AntennePattern   = zeros(NrAntenne, Antall_Vinkler, 5);
AntennePattern_E = zeros(NrAntenne, Antall_Vinkler, 5);


theta = deg2rad(linspace(20, 160, UpSample));
AntennePatternUpSampled = zeros(NrAntenne, UpSample, 2); 


%_______________Sim________________________________________________________
Vinkler = linspace(20, 160, Antall_Vinkler);
for V = 1:Antall_Vinkler % Looper over flere vinkler
    v     = deg2rad(Vinkler(V)); 
    Drone = [10*cos(v), 10*sin(v), 0];
    Drone_Error = std_per_dim * randn(size(Drone));


for i = 1:NrAntenne
    A(i).D_Pos   = Drone;
    A(i).D_Pos_E = Drone + Drone_Error;
    A(i) = A(i).Geometry(1);
    A(i) = A(i).AntenneVinkelOffset;
  % A(i) = A(i).KalibreringsOffset(1, 0);
    A(i) = A(i).KalibreringsOffset(Cal_Offset(i, 1), Cal_Offset(i, 2));
    A(i) = A(i).FinnDelaySpread;
    A(i) = A(i).LagKanal;
    [A(i), Rx(i, :)]                 = A(i).SendDataGjennomKanal(Tx, SNR);
    [A(i), H_FFT(i, :), hTime(i, :)] = A(i).EstimerKanal(Rx(i, :));
    [A(i), Phasor, ErrorPhasor]      = A(i).FasitPhasor;
    
    peaks(i)         = max(hTime(i, :));

    AntennePattern(i, V, 1) = A(i).AZ;
    AntennePattern(i, V, 2) = A(i).EL;
    AntennePattern(i, V, 3) = peaks(i)/Phasor;

   
    AntennePattern_E(i, V, 1) = A(i).AZ_E;
    AntennePattern_E(i, V, 2) = A(i).EL_E;
    AntennePattern_E(i, V, 3) = peaks(i)/ErrorPhasor; 
    
end

end




%_________________________DataProcessing___________________________________
    % Perfekt phase
% dPhase(:, :) = (angle(AntennePattern(:, :, 3) ./ AntennePattern(1, :, 3)) );
dPhase(:, :)   = (angle(AntennePattern(:, : , 3) ./ AntennePattern(1, :, 3) ));
dP             = wrapTo2Pi(mean(dPhase, 2));
ActualCalibration1 = wrapTo2Pi(Cal_Offset(:, 2) - Cal_Offset(1, 2));

    % Perfekt Amplitude  
dAmp(: ,:) = abs(AntennePattern(:, :, 3)./AntennePattern(1, :, 3));
dA         = (mean(dAmp, 2));
ActualCalibration2 = abs(Cal_Offset(:, 1)) / Cal_Offset(1, 1);

%__________________________________________________________________________
    % Ikke Perfekt phase
dPhase_E(:, :) = (angle( (AntennePattern_E(:, :, 3)) ./ ...
                         (AntennePattern_E(1, :, 3)) ) );
dP_E           = wrapTo2Pi(mean(dPhase_E, 2));

    % Ikke Perfekt Amplitude  
dAmp_E(: ,:) = abs(AntennePattern_E(:, :, 3)./AntennePattern_E(1, :, 3));
dA_E         = mean(dAmp_E, 2);

%__________________________________________________________________________
    % Upsampled Data

for i = 1:NrAntenne

    PhaseUp       = griddedInterpolant(AntennePattern(i, :, 1), ...
        unwrap(angle(AntennePattern(i, :, 3))), 'spline', 'spline'); 
    AmpUp         = griddedInterpolant(AntennePattern(i, :, 1), ...
        abs(AntennePattern(i, :, 3)), 'pchip');

    AntennePatternUpSampled(i, :, 1) = AmpUp(theta); 
    AntennePatternUpSampled(i, :, 2) = PhaseUp(theta);

end

dPhase_U = AntennePatternUpSampled(:, :, 2) - AntennePatternUpSampled(1, :, 2);
dP_U     = wrapTo2Pi(mean(dPhase_U, 2));

dAmp_U   = AntennePatternUpSampled(:, :, 1) ./ AntennePatternUpSampled(1, :, 1);
dA_U     = mean(dAmp_U, 2);

%_________________PLOTS____________________________________________________
% The plot is activated by chanching the if statment


if 1
figure(1); % Har perfekt posisjons data
    for i = 1:NrAntenne
        subplot(2, 1, 1);
        plot((AntennePattern(i, :, 1)), abs(AntennePattern(i, :, 3))); hold on;
        subplot(2, 1, 2);
        plot(AntennePattern(i, :, 1), unwrap((angle(AntennePattern(i, :, 3)) ))); hold on;
    end
    title('Phase')
    ylabel('$\angle \left( G(\theta, \psi)\cdot \alpha_{e}e^{j \varphi} \right)$', ...
        Rotation=0, Interpreter='latex');
    xlabel('\theta Azimtut (rad) ');
    hold off;
    subplot(2, 1, 1);
    title('Magnetude')
    ylabel('$|G(\theta, \psi)\cdot \alpha_{e}e^{j \varphi}|$', ...
        Rotation=0, Interpreter='latex');
    xlabel(' \theta Azimtut (rad) ')
    hold off;
end


if 0
figure(2); % Har dårlig posisjonsdata. 
    for i = 1:1:NrAntenne
        subplot(2, 1, 1);
        plot(AntennePattern_E(i, :, 1), abs(AntennePattern_E(i, :, 3))); hold on;
        subplot(2, 1, 2);
        %plot(AntennePattern_E(i, :, 1), movmean( unwrap(angle(AntennePattern_E(i, : , 3))), 100, 2)); hold on;
        plot(AntennePattern_E(i, :, 1),  unwrap(angle(AntennePattern_E(i, : , 3))) ); hold on;
    end
    title('Phase (rad)')
    ylabel('$\angle \left( G(\theta, \psi)\cdot \alpha_{e}e^{j \varphi} \right)$', ...
        Rotation=0, Interpreter='latex');
    xlabel('\theta Azimtut (rad) ');
    hold off;
    subplot(2, 1, 1);
    title('Magnetude')
    ylabel('$|G(\theta, \psi)\cdot \alpha_{e}e^{j \varphi}|$', ...
        Rotation=0, Interpreter='latex');
    xlabel(' \theta Azimtut (rad) ')
    hold off;
end

if 1  % Up sampler  
figure(3); 
    subplot(2, 1, 1);
    plot(theta, AntennePatternUpSampled(:, :, 1));
    title('Magnetude Interpolation');
    ylabel('$|G(\theta, \psi)\cdot \alpha_{e}e^{j \varphi}|$', ...
        Rotation=0, Interpreter='latex');
    xlabel(' \theta Azimtut (rad) '); hold off;

    subplot(2, 1, 2);
    plot(theta, AntennePatternUpSampled(:, :, 2));
    title('Phase Interpolation');
    ylabel('$\angle \left( G(\theta, \psi)\cdot \alpha_{e}e^{j \varphi} \right)$', ...
        Rotation=0, Interpreter='latex');
    xlabel('\theta Azimtut (rad) '); hold off;
end

%__________Stolpe
if 0
figure(4);
    x = 1:NrAntenne;
    subplot(2, 1, 1);
        bar(x, wrapTo2Pi(dP), ...
            DisplayName='Simulation Result (perfect position)'); hold on;
        bar(x, wrapTo2Pi(ActualCalibration1), 0.2, ...
            DisplayName='Actuall Offset');
        title('Phase offset (rad)');
        xlabel('Antenna Nr:');
        ylabel('\phi offset', Rotation=0);
        grid on; legend; hold off;

    subplot(2, 1 ,2)
        bar(x, dA, DisplayName='Simulation Result (perfect position)'); hold on;
        bar(x, ActualCalibration2, 0.2,  DisplayName='Actuall Offset');
        title('Amplitude Offset');
        xlabel('Antenna Nr');
        ylabel('Amp offset', Rotation=0);
        grid on; legend; hold off;
end


if 1
figure(5);
    x = 1:NrAntenne;
    subplot(2, 1, 1);
        bar(x, wrapTo2Pi(dP_E), ...
            DisplayName='Simulation Result (non perfect position)'); hold on;
        bar(x, wrapTo2Pi(ActualCalibration1), 0.2, ...
            DisplayName='Actuall Offset');
        title('Phase offset');
        xlabel('Antenna Nr:');
        ylabel('\phi offset', Rotation=0);
        grid on; hold off; %legend;

    subplot(2, 1 ,2)
        bar(x, dA_E, ...
            DisplayName='Simulation Result (non perfect position)'); hold on;
        bar(x, ActualCalibration2, 0.2,  ...
            DisplayName='Actuall Offset');
        title('Amplitude Offset');
        xlabel('Antenna Nr');
        ylabel('Amp offset', Rotation=0);
        grid on; legend; hold off;
end


if 1
figure(6);
    x = 1:NrAntenne;
    subplot(2, 1, 1);
        bar(x, (dP_U), ...
            DisplayName='Simulation Result (non perfect position)'); hold on;
        bar(x, wrapTo2Pi(ActualCalibration1), 0.2, ...
            DisplayName='Actuall Offset');
        title('Phase offset');
        xlabel('Antenna Nr:');
        ylabel('\phi offset', Rotation=0);
        grid on; hold off; % legend;

    subplot(2, 1 ,2)
        bar(x, dA_U, ...
            DisplayName='Simulation Result (non perfect position)'); hold on;
        bar(x, ActualCalibration2, 0.2,  DisplayName='Actuall Offset');
        title('Amplitude Offset');
        xlabel('Antenna Nr');
        ylabel('Amp offset', Rotation=0);
        grid on; legend; hold off;
end

if 0
figure(7);
    x = 1:NrAntenne;
    subplot(2, 1, 1);
        bar(x, wrapTo2Pi(dP), 1,  ...
            DisplayName='Simulation Result (No Position Uncertainty)'); hold on;
        bar(x, wrapTo2Pi(ActualCalibration1), 0.2, ...
            DisplayName='Actuall Offset');
        bar(x, wrapTo2Pi(dP_E), 0.5, ...
            DisplayName='Simulation Result (Widt Position Uncertainty)'); hold on;
        title('Phase offset');
        xlabel('Antenna Nr:');
        ylabel('\phi offset', Rotation=0);
        grid on; legend; hold off;

    subplot(2, 1 ,2)
        bar(x, dA_E, DisplayName='Simulation Result'); hold on;
        bar(x, ActualCalibration2, 0.2,  DisplayName='Actuall Offset');
        title('Amplitude Offset');
        xlabel('Antenna Nr');
        ylabel('Amp offset', Rotation=0);
        grid on; legend; hold off;
end





