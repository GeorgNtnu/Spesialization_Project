classdef Antenne_Object
    % Object that is used to simulate a N number of antenna element fast
    % and effichent. Comatedabale with EstimerOffsetAntenneDiagram.m and
    % EstimerOffset.m
    properties
        % Parameters 
            c      = 3e8;           % Speed of light           
            fs     = 20e6;          % Sampling frequency
            fc     = 6e9;           % Carrier  frecuency
            N      = 20e2;          % Number of samples 
            NN     = 20;             % FFT upsample
            Height = 15;            % Høyde

        % Unknown Parameter
            lamda;                  % Wavelenght

            A_Pos;                  % Antenne Posisjonen
            D_Pos;                  % Drone   Posisjonen
            ADvec;                  % Vector  from Antenne to Drone
            AZ;                     % Azimtut
            EL;                     % Elevation 
            d;                      % Distanse traveld
            Polarisation            % 0 is flat, 1 is verticaly

                % Antenne Offset as a function of angle
            A_amplitude;            % Amplitude Offset
            A_phase;                % Phase     Offset
            A_offset;               % Combinatain of offset
            C_offset;               % Constant offset (found in calibration) 

            
            Tap1;                   % The first  dleay Tap
            Tap2;                   % The second delay Tap
            H;                      % The channel 
            Rx;                     % The signal passing throug the channel
            RxFFT;                  
            TxFFT;

                % Error Position
            D_Pos_E;                  % Drone   Posisjonen
            ADvec_E;                  % Vector  from Antenne to Drone
            AZ_E;                     % Azimtut
            EL_E;                     % Elevation 
            d_E;                      % Distanse traveld




    end
    % The function the simulation can exexute
    methods
        function obj = Geometry(obj, Error)
            % Calculate the incoming DoA and the distance of the wave
            obj.ADvec = obj.D_Pos - obj.A_Pos;        
            [obj.AZ, EL, obj.d] = cart2sph(obj.ADvec(1), ...
                                           obj.ADvec(2), ...
                                           obj.ADvec(3));
            obj.EL = -EL + pi/2;
            
            if Error
                % Error
                obj.ADvec_E = obj.D_Pos_E - obj.A_Pos;        
                [obj.AZ_E, EL, obj.d_E] = cart2sph(obj.ADvec_E(1), ...
                                                   obj.ADvec_E(2), ...
                                                   obj.ADvec_E(3));
                obj.EL_E = -EL + pi/2;
            end
        end

        function obj = AntenneVinkelOffset(obj) 
           % Generate a DoA dependent offset 
           AZ_Amp   = exp(-((rad2deg(obj.AZ) - 90).^2) / (2 * 15^2));
           AZ_phase = deg2rad(45 * sin(obj.AZ) + 30 * sin(obj.AZ*2));
           EL_phase = deg2rad(45 * sin(obj.EL) + 30 * sin(obj.EL*2));
           EL_Amp   = exp(-((rad2deg(obj.EL) - 90).^2) / (2 * 15^2));

           obj.A_amplitude = AZ_Amp * EL_Amp;
           obj.A_phase     = AZ_phase + EL_phase;
           obj.A_offset = obj.A_amplitude*exp(1j * obj.A_phase);

            % Can be activated to remove the the DoA dependent offset
           %obj.A_offset = obj.A_offset/obj.A_offset;  
        end

        function obj = KalibreringsOffset(obj, Amp, Phase)
            % Combine a phase and amplitude to the constant offset phasor
            obj.C_offset = Amp * exp(1j * Phase);
        end
        
        function obj = FinnDelaySpread(obj)
            % Find the delay spread of the channel based on the position
            FSPL1    = (4 * pi * obj.d * obj.fc / obj.c)^2;
            % Amplitude and phase [Amp, phase]
            obj.Tap1 = [sqrt(1/FSPL1), obj.d/obj.c]; 
            
            d2       = 2 * sqrt(obj.d^2 + obj.Height^2);
            FSPL2    = (4 * pi * d2 * obj.fc / obj.c)^2;
            obj.Tap2 = [sqrt(1/FSPL2), d2/obj.c]; % [Amp, phase]
        end

        function obj = LagKanal(obj) 
            % Generate the channel
            Amp1 = obj.Tap1(1);
            Amp2 = obj.Tap2(1);

            tau1 = obj.Tap1(2);
            tau2 = obj.Tap2(2);
            
            f = linspace(obj.fc-obj.fs/2, obj.fc+obj.fs/2, obj.N);
            obj.H = Amp1 * exp(-1j * 2 * pi * f * tau1) +... 
                    Amp2 * exp(-1j * pi) * exp(-1j * 2 * pi * f * tau2);
        end

        function [obj, Rx]= SendDataGjennomKanal(obj, Tx, SNR)
            % Simulate data being transmited through the channel
            obj.TxFFT = fft(Tx);
            obj.RxFFT = obj.TxFFT.*obj.H;                                  % 1xN 
            obj.Rx    = ifft(obj.RxFFT) .* obj.C_offset .* obj.A_offset;   % Legger til feil som

                % Legger til Noise;
            signalPower = mean(abs(obj.Rx).^2);
            desiredSNR  = 10^(SNR/10);
            noisePower = signalPower / desiredSNR;
            noise      = sqrt(noisePower/2) * (randn(1, obj.N) + 1i*randn(1, obj.N));
            obj.Rx     = obj.Rx + noise;

            % obj.Rx    = awgn(obj.Rx, SNR, "measured");
            obj.RxFFT = fft(obj.Rx);
            Rx        = obj.Rx;
       
        end

        function [obj, H_estimert, h_estimert] = EstimerKanal(obj, ~)
            % Estimate the channel frequency response
            H_estimert = obj.RxFFT./ obj.TxFFT;
            h_estimert = ifft(H_estimert, obj.N*obj.NN);
        end

        function [obj, dPhase_Fasit, dAmp___Fasit] = FinnFasitOffset(obj, obj1)  
            % Calculate the actuall offset between to antenna element
            obj.lamda = (obj.c/obj.fc);
            Phase1 = -2*pi*obj.d / obj.lamda;
            Amp1   = obj.Tap1(1);

            Phase2 = -2*pi*obj1.d ./ obj1.lamda;
            Amp2   = obj1.Tap1(1);
            
            dAmp___Fasit = Amp1 / Amp2;
            dPhase_Fasit = wrapTo2Pi(Phase1 - Phase2);
        end


        function [obj, phasor, ErrorPhasor] = FasitPhasor(obj)
            % Calculate the actual offset phasor. 
            obj.lamda   = obj.c/obj.fc;
            Fase1        = (-2*pi*obj.d / obj.lamda);
            phasor      = obj.Tap1(1) * exp(1j* Fase1);  
            
            Amp_E = (4 * pi * obj.d_E / obj.lamda)^2; 
            Fase2  = (-2*pi*obj.d_E / obj.lamda);
            ErrorPhasor = sqrt(1/Amp_E)* exp(1j* Fase2);
        end

    end
end

