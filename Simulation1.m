clc;
Antall_Vinkler = 100;
NrAntenne      = 32;
SNR            = 20;

A(NrAntenne)   = Antenne_Object();
% Lager en tilfeldig Amplitude feil [0, 1] og fasefeil [0, 2pi]
Cal_Offset     = rand(NrAntenne, 2); 
Cal_Offset(:, 2) = Cal_Offset(:, 2) * 2*pi; 
Drone          = [0, 10, 0];

% Place the Antennas and alsselect the right polarisation. 
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
    
    if mod(Teller2, 2) == 0 && Teller1 == 0; 
        Polarisation = 0; 
    elseif mod(Teller2, 2) ~= 0 && Teller1 == 0;  
        Polarisation = 1; 
    end

end



Bits = randi([0, 1], A(1).N*4, 1);
Tx = qammod(Bits, 16, InputType="bit").';     % Sendt Signal'

Rx    = zeros(NrAntenne, 2000);
H_FFT = zeros(NrAntenne, 2000);
hTime = zeros(NrAntenne, A(1).N * A(1).NN);

peaks = zeros(2, 1);
dPhase_Fasit = zeros(NrAntenne, Antall_Vinkler);
dAmp__Fasit  = zeros(NrAntenne, Antall_Vinkler);
dPhase       = zeros(NrAntenne, Antall_Vinkler);
PhaseError   = zeros(NrAntenne, Antall_Vinkler);
dAmp         = zeros(NrAntenne, Antall_Vinkler);
AmpError     = zeros(NrAntenne, Antall_Vinkler);



Vinkler = linspace(10, 170, Antall_Vinkler);
for V = 1:Antall_Vinkler % Looper over flere vinkler
    v     = deg2rad(Vinkler(V)); 
    Drone = [10*cos(v), 10*sin(v), 0];



for i = 1:NrAntenne
    A(i).D_Pos = Drone;
    A(i) = A(i).Geometry(0);
    A(i) = A(i).AntenneVinkelOffset;
  % A(i) = A(i).KalibreringsOffset(1, 0);
    A(i) = A(i).KalibreringsOffset(Cal_Offset(i, 1), Cal_Offset(i, 2));
    A(i) = A(i).FinnDelaySpread;
    A(i) = A(i).LagKanal;
    [A(i), Rx(i, :)] = A(i).SendDataGjennomKanal(Tx, SNR);
    [A(i), H_FFT(i, :), hTime(i, :)] = A(i).EstimerKanal(Rx(i, :));
    [A(i), dPhase_Fasit(i, V), dAmp__Fasit(i, V)] = A(i).FinnFasitOffset(A(1));

        

    peaks(i)         = max(hTime(i, :));
    dPhase(i, V)     = wrapTo2Pi(angle(peaks(i)) - angle(peaks(1)));  
    PhaseError(i, V) = wrapTo2Pi(dPhase_Fasit(i, V) - dPhase(i, V));
    
    dAmp(i, V)       = abs(peaks(i)) ./ abs(peaks(1)); 
    AmpError(i, V)= dAmp(i, V) / dAmp__Fasit(i, V);
    
end

end
    % Phase
% Antenne x + CalibrationValue is Antenne 1
CalibrationValues1 = mean(PhaseError, 2);  
ActualCalibration1 = wrapTo2Pi(Cal_Offset(1, 2) - Cal_Offset(:, 2));

    % Amplitude
% Antenne x / CalibrationValue is Antenne 1
CalibrationValues2 = mean(AmpError, 2);    
ActualCalibration2 = abs(Cal_Offset(:, 1)) / Cal_Offset(1, 1);



if 0
figure(1);  
    subplot(2, 1, 1);
    plot(1:length(hTime(1, :)), (abs(hTime)) ); hold on;
    title('|h(t)|')
    ylabel('Amp', Rotation=0)
    xlabel('dealy [n]');
    xlim([0, 200]);
    % legend
    hold off;
    
    subplot(2, 1, 2);
    % plot(1:length(hTime(1, :)), (angle(hTime) - angle(hTime(1, :))) ); hold on
    plot(1:length(hTime(1, :)), (angle(hTime) ) ); hold on
    title('\angleh(t)')
    ylabel('\angle', Rotation=0);
    xlabel('dealy  [n]');
    xlim([0, 200]);
    % legend;
    hold off;
end

if 1
figure(2);
    x = 1:NrAntenne;
    subplot(2, 1, 1);
        bar(x, wrapTo2Pi(CalibrationValues1), ...
            DisplayName='Simulation Result'); hold on;
        bar(x, wrapTo2Pi(ActualCalibration1), 0.2, ...
            DisplayName='Actuall Offset');
        title('Phase offset');
        xlabel('Antenna Nr:');
        ylabel('\phi offset', Rotation=0);
        grid on; legend; hold off;

    subplot(2, 1, 2);
        bar(x, CalibrationValues2, ...
            DisplayName='Simulation Result'); hold on;
        bar(x, ActualCalibration2, 0.2,  ...
            DisplayName='Actuall Offset');
        title('Amplitude Offset');
        xlabel('Antenna Nr');
        ylabel('Amp offset', Rotation=0);
        grid on; legend; hold off;

    end


