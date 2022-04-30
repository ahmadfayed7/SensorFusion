
## Rubric

# 2D CFAR Implementation README

In this small report, an overview of how the 2D variant of the Constant False Alarm Rate (CFAR) was implemented as well as the rationale behind the choices made for the hyperparameters and how the edges of the joint Doppler frequency and range spectrum were handled.

## Implementation Steps

2D CFAR was implemented following the prompts with following parameters: 8 doppler training cells and 10 range training cells, 4 guard cells for both dimensions. Offset of 6 dB to construct the threshold. For each iteration CUT cell, we take the noise level as the average value of the db2pow(signal) around it's training cells. I further added the noise level with the offset. After comparing the signal value with the calculated threshold, ones that bigger than threshold was normalized to 1, otherwise 0. 
## Selection of Hyperparameters

Due to repeated experimentation and observing the output, the following hyperparameters were chosen:

* `Td = 10`
* `Tr = 8`
* `Gr = 4`
* `Gd = 4`
* `offset = 6`

These were primarily chosen because of the walkthrough video, but also because of repeated experimentation to see how the performance varies when we slowly diverge away from the initial hyperparameters chosen.  The initial hyperparameters chosen were already good enough, so they remain in the final version of this project.

## Dealing with the edge cases

Finally, in a vectorised manner we simply use indexing to suppress all of the edges of the output by simply examining how much the halfway point of the rows and columns would be in the proposed mask and setting those locations in the output mask to 0 accordingly.  By using indexing, we leave the remaining elements intact.


## Radar Specifications {#2}

```{.codeinput}
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
```

## User Defined Range and Velocity of target {#3}

**%TODO** : define the target's initial position and velocity. Note :
Velocity remains contant

```{.codeinput}
R = 110;
v = -20;

max_range = 200;
range_res = 1;
max_velocity = 100;
c = 3e8;
```

## FMCW Waveform Generation {#4}

```{.codeinput}
% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and slope (slope) of the FMCW
% chirp using the requirements above.

%Operating carrier frequency of Radar
fc = 77e9;             %carrier freq Hz
B = c / (2 * range_res);

% The sweep time can be computed based on the time needed for the signal to travel the unambiguous
% maximum range. In general, for an FMCW radar system, the sweep time should be at least
% 5 to 6 times the round trip time. This example uses a factor of 5.5.
Tchirp = 5.5 * 2 * max_range / c;
slope = B / Tchirp;

%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation.
Nd = 128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp.
Nr = 1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t = linspace(0,Nd*t_chirp,Nr*Nd); %total time for samples

%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx = zeros(1, length(t)); %transmitted signal
Rx = zeros(1, length(t)); %received signal
Mix = zeros(1, length(t)); %beat signal


%Similar vectors for range_covered and time delay.
r_t = zeros(1, length(t));
td = zeros(1, length(t));
```

## Signal generation and Moving Target simulation {#5}

Running the radar scenario over the time.

```{.codeinput}
for i = 1 : length(t)
  % *%TODO* :
  %For each time stamp update the Range of the Target for constant velocity.
  r_t(i) = target_range + (target_velocity*t(i));


  % *%TODO* :
  %For each time sample we need update the transmitted and
  %received signal.
    td(i) = 2 * r_t(i) / c;
    Tx(i) = cos(2*pi*(fc*t(i) + (alpha*(t(i)^2)/2)));
    Rx(i) = cos(2*pi*(fc*(t(i) - td(i)) + (alpha*((t(i) - td(i))^2)/2)));

  % *%TODO* :
  %Now by mixing the train_cellsansmit and Receive generate the beat signal
  %This is done by element wise matrix multiplication of train_cellsansmit and
  %Receiver Signal
   Mix(i) = Tx(i) * Rx(i); 
end
```

## RANGE MEASUREMENT {#6}

**%TODO** :

```{.codeinput}
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix_matrix = reshape(Mix, [Nr, Nd]);

% *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
Mix_FFT = fft(Mix_matrix, [], 1) / Nr;

% *%TODO* :
% Take the absolute value of FFT output
Mix_FFT_mag = abs(Mix_FFT);
Mix_FFT_mag = max(Mix_FFT_mag, [], 2);

% *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
Mix_FFT_mag = Mix_FFT_mag(1 : Nr/2);

%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

% *%TODO* :
% plot FFT output
plot(sig_fft);
plot(Mix_FFT_mag);
 xlabel('Range (ft)');
 ylabel('FFT Magnitude');
axis ([0 200 0 1]);
grid;
 
```

![](./images/1.png)

## RANGE DOPPLER RESPONSE {#7}

---

The 2D FFT implementation is already provided here. This will run a
2DFFT on the mixed signal (beat signal) output and generate a range
doppler map.You will implement CFAR on the generated RDM

```{.codeinput}
% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix = reshape(Mix, [Nr, Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix, Nr, Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1 : Nr/2, 1 : Nd);
sig_fft2 = fftshift(sig_fft2);

range_doppler_map = abs(sig_fft2);
range_doppler_map = 10 * log10(range_doppler_map);

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100, 100, Nd);
range_axis = linspace(-200, 200, Nr/2) * ((Nr/2) / 400);

% Additional views of the surface plot
figure ('Name', 'Amplitude and Range From FFT2');
surf(doppler_axis, range_axis, range_doppler_map);
title('Amplitude and Range From FFT2');
xlabel('Speed');
ylabel('Range');
zlabel('Amplitude');
```

![](./images/2.png)

##CFAR implementation {#8}

---

```{.codeinput}
%Slide Window through the complete Range Doppler Map
% *%TODO* :
%Select the number of train_cellsaining Cells in both the dimensions.
Tr = 10;
Td = 8;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under
%test (CUT) for accurate estimation
Gr = 4;
Gd = 4;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 6;
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);

% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for train_cellsaining and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR

RDM_copy = RDM;
for i = Tr + Gr + 1 : Nr/2 - (Gr + Tr)
    for j = Td + Gd + 1 : Nd - (Gd + Td)
        noise_level = 0;
        for p = i - (Tr + Gr) : i + Tr + Gr
            for q = j - (Td + Gd) : j + Td + Gd
                if abs(i - p) > Gr || abs(j - q) > Gd
                    noise_level = noise_level + db2pow(RDM_copy(p, q));
                end
            end
         end
         threshold = pow2db(noise_level / ( ((2 * Tr + 2 * Gr + 1) * (2 * Td + 2 * Gd + 1)) - ((2 * Gr + 1)*(2 * Gd + 1))));
         threshold = threshold + offset;
         if RDM(i, j) < threshold
             RDM(i, j) = 0;
         else
             RDM(i, j) = 1;
         end
     end
end

% *%TODO* :
% The process above will generate a thresholded block, which is smaller
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0.
 RDM(1 : Tr + Gr, :) = 0;
RDM(Nr/2 - (Gr + Tr) + 1 : end, :) = 0;
RDM(:, 1 : Td + Gd) = 0;
RDM(:, Nd - (Gd + Td) + 1 : end) = 0;

% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis, RDM);
colorbar;

```

![](./images/3.png)

</div>
